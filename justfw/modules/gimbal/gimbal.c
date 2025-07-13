//
// Created by Ukua on 2024/1/22.
// 功能：云台模块实现文件，包含云台初始化、模式切换、角度控制、主循环任务等核心逻辑
//

#include "gimbal.h"          // 云台模块头文件
#include "BMI088driver.h"    // BMI088六轴传感器驱动
#include "GM6020.h"          // GM6020电机驱动（假设为云台电机型号）
#include "interface.h"       // 接口定义（如结构体、模式枚举等）
#include <stdio.h>           // 标准输入输出库
#include "cmsis_os.h"        // CMSIS-RTOS操作系统接口（用于任务管理）
#include "user_lib.h"        // 用户自定义库（如PID、数学函数等）

// 全局变量声明
INTF_Gimbal_HandleTypeDef *g_gimbal;           // 云台核心控制句柄（指向云台状态结构体）
extern float INS_angle[3];                     // 外部声明：惯性导航实时欧拉角 [0:Pitch, 1:Roll, 2:Yaw]（弧度）
extern float INS_SUM_angle[3];                 // 外部声明：惯性导航累积欧拉角（用于长时角度测量）
PIDInstance gimbal_yaw_gyro_pid;               // 偏航轴（Yaw）PID控制器实例（速度环）
PIDInstance gimbal_pitch_gyro_pid;             // 俯仰轴（Pitch）PID控制器实例（速度环）
extern bmi088_real_data_t bmi088_real_data;     // 外部声明：BMI088传感器实时数据（含陀螺仪、加速度计值）

// 宏定义：简化传感器数据访问
#define GIMBAL_YAW INS_SUM_angle[2]             // 云台当前偏航角（使用累积角度，抗漂移）
#define GIMBAL_YAW_SPEED bmi088_real_data.gyro[2] // 云台偏航角速度（陀螺仪原始数据，弧度/秒）
#define GIMBAL_PITCH INS_angle[0]               // 云台当前俯仰角（使用实时角度，响应快）

/**
 * @brief 获取当前云台偏航角（根据模式选择数据源）
 * @return 偏航角（弧度）
 */
float Gimbal_GetYaw(){
  switch (g_gimbal->mode) {
  case GIMBAL_MODE_NORMAL:       // 普通模式：直接读取电机反馈角度
    return g_gimbal->motor_yaw->real_angle;
  case GIMBAL_MODE_FOLLOW_GYRO:  // 跟随陀螺仪模式：使用惯性导航累积角度
    return GIMBAL_YAW;
  }
  return 0; // 默认返回0（理论上不会执行到）
}

/**
 * @brief 获取当前云台俯仰角（考虑硬件倒置补偿）
 * @return 俯仰角（弧度，范围[-π, π]）
 */
float Gimbal_GetPitch(){
  // TODO 注意：C板硬件倒置，需通过+π补偿初始角度
  return loop_float_constrain(GIMBAL_PITCH+3.1415f,-PI,PI); // 约束角度范围
}

/**
 * @brief 设置云台偏航轴目标角度（带角度限制）
 * @param self 云台句柄指针
 * @param target_yaw 目标偏航角（弧度）
 */
void Gimbal_SetYaw(struct INTF_Gimbal_Handle *self, float target_yaw) {
    // 限制yaw轴角度：若未设置限制（max=min）则直接赋值，否则截断到[min, max]
    if (self->yaw_limit_max==self->yaw_limit_min) { // 未设置限制
        self->target_yaw = target_yaw;
    }else{
        if (target_yaw>self->yaw_limit_max){
            self->target_yaw = self->yaw_limit_max;
        }else if (target_yaw<self->yaw_limit_min){
            self->target_yaw = self->yaw_limit_min;
        }else{
            self->target_yaw = target_yaw;
        }
    }
}

/**
 * @brief 设置云台俯仰轴目标角度（带角度限制）
 * @param self 云台句柄指针
 * @param target_pitch 目标俯仰角（弧度）
 */
void Gimbal_SetPitch(struct INTF_Gimbal_Handle *self, float target_pitch) {
    // 限制pitch轴角度：逻辑同Yaw轴
    if (self->pitch_limit_max==self->pitch_limit_min) { // 未设置限制
        self->target_pitch = target_pitch;
    }else{
        if (target_pitch>self->pitch_limit_max){
            self->target_pitch = self->pitch_limit_max;
        }else if (target_pitch<self->pitch_limit_min){
            self->target_pitch = self->pitch_limit_min;
        }else{
            self->target_pitch = target_pitch;
        }
    }
}

/**
 * @brief 切换云台工作模式（防止模式切换时电机甩动）
 * @param self 云台句柄指针
 * @param mode 目标模式（GIMBAL_MODE_NORMAL/FOLLOW_GYRO）
 */
void Gimbal_SetMode(struct INTF_Gimbal_Handle *self,Gimbal_ModeTypeDef mode){
    switch (mode) {
        // 模式切换时调整电机控制模式并同步角度，避免突变
        case GIMBAL_MODE_NORMAL:       // 普通模式：电机切换为角度控制，目标角度设为当前电机角度
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_ANGLE);
            self->set_yaw(self,self->motor_yaw->real_angle);
            break;
        case GIMBAL_MODE_FOLLOW_GYRO:  // 跟随陀螺仪模式：电机切换为速度控制，目标角度设为当前惯性角度
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_SPEED);
            self->set_yaw(self,GIMBAL_YAW);
            break;
    }
    self->mode = mode; // 更新当前模式
}

/**
 * @brief 云台主循环任务（RTOS实时任务）
 * @note 包含电机控制、角度反馈、PID计算等核心逻辑
 */
void Gimbal_MainLoop() {
    extern volatile float q0, q1, q2, q3; // 外部声明：四元数（可能用于姿态解算）
    // TODO 注意：C板初始位置倒置，通过设置四元数初始值补偿
    q0= 0.0f; // 四元数初始值（对应初始姿态）
    q1= 1.0f; // 四元数初始值（对应初始姿态）
    osDelay(1000); // 延时1秒，等待电机、陀螺仪初始化完成

    // 等待电机数据更新（防止使用过时数据）
    while (HAL_GetTick() - g_gimbal->motor_yaw->update_time>1000||HAL_GetTick() - g_gimbal->motor_pitch->update_time>1000){
      osDelay(1); // 每1ms检查一次
    }

    while (1) { // 无限循环（RTOS任务主逻辑）
      float a=0; // 临时变量（未使用，可能为调试保留）
        switch (g_gimbal->mode) {
            case GIMBAL_MODE_NORMAL:    // 普通模式：直接设置电机目标角度
              g_gimbal->motor_yaw->set_angle(g_gimbal->motor_yaw,g_gimbal->target_yaw);
              g_gimbal->real_yaw = g_gimbal->motor_yaw->real_angle; // 同步实际角度
              break;
            case GIMBAL_MODE_FOLLOW_GYRO:// 跟随陀螺仪模式：通过PID计算速度控制电机
              g_gimbal->real_yaw = GIMBAL_YAW; // 获取当前惯性角度
              // 计算角度误差（取反并约束范围），输入PID计算速度输出
              g_gimbal->motor_yaw->set_speed(g_gimbal->motor_yaw,PIDCalculate(&gimbal_yaw_gyro_pid,-loop_float_constrain(g_gimbal->target_yaw-g_gimbal->real_yaw,-PI,PI),0));
              break;
        }
        // 俯仰轴控制（所有模式均通过PID速度控制）
        g_gimbal->real_pitch = Gimbal_GetPitch(); // 获取当前俯仰角（带硬件补偿）
        // 计算俯仰角度误差，输入PID计算速度输出
        g_gimbal->motor_pitch->set_speed(g_gimbal->motor_pitch,PIDCalculate(&gimbal_pitch_gyro_pid,-loop_float_constrain(g_gimbal->target_pitch-g_gimbal->real_pitch,-PI,PI),0));
        osDelay(1); // 任务延时1ms（控制周期约1ms）
    }
}

/**
 * @brief 云台模块初始化函数（系统启动时调用）
 */
void Gimbal_Init() {
    // 分配云台句柄内存（通过共享指针接口）
    g_gimbal = pvSharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef));

    g_gimbal->mode=GIMBAL_MODE_NORMAL; // 默认普通模式

    // 分配电机句柄内存（Yaw轴和Pitch轴电机）
    g_gimbal->motor_yaw = pvSharePtr("/motor/gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    g_gimbal->motor_pitch = pvSharePtr("/motor/gimbal_pitch", sizeof(INTF_Motor_HandleTypeDef));

    // 绑定控制函数指针（设置角度、模式等）
    g_gimbal->set_pitch = Gimbal_SetPitch;
    g_gimbal->set_yaw = Gimbal_SetYaw;
    g_gimbal->set_mode = Gimbal_SetMode;

    // 初始化目标角度（默认0弧度）
    g_gimbal->target_yaw=0;
    g_gimbal->target_pitch=0;

    // 设置俯仰轴角度限制（单位：弧度）
    g_gimbal->pitch_limit_max = 0.15f;  // 最大俯仰角（约8.6度）
    g_gimbal->pitch_limit_min = -0.4f;  // 最小俯仰角（约-22.9度）
    // 初始未设置Yaw轴限制（max=min=0）
    g_gimbal->yaw_limit_max = 0;
    g_gimbal->yaw_limit_min = 0;

    // 初始化Yaw轴PID参数（速度环）
    PID_Init_Config_s gimbal_yaw_angle_config = {
        .Kp=40.0f,          // 比例系数（角度误差放大）
        .Ki=120.0f,         // 积分系数（消除稳态误差）
        .Kd=8.0f,           // 微分系数（抑制超调）
        .CoefB=0.05f,       // 比例项滤波系数（可选）
        .CoefA=0.05f,       // 微分项滤波系数（可选）
        .IntegralLimit=200.0f, // 积分限幅（防止积分饱和）
        .MaxOut=30.0f * RPM2RPS, // 最大输出（转换为弧度/秒）
        .Improve=PID_Integral_Limit|PID_Derivative_On_Measurement|PID_ChangingIntegrationRate, // 改进功能（积分限幅、微分测量、变积分率）
    };
    PIDInit(&gimbal_yaw_gyro_pid,&gimbal_yaw_angle_config); // 初始化PID实例

    // 初始化Pitch轴PID参数（速度环）
    PID_Init_Config_s gimbal_pitch_config = {
            .Kp=15.0f,          // 比例系数
            .Ki=1.0f,           // 积分系数
            .Kd=0.0f,           // 微分系数（未启用）
            .MaxOut=5.0f * RPM2RPS, // 最大输出（转换为弧度/秒）
            .DeadBand = 0.0f,   // 死区（无）
            .Improve=PID_Integral_Limit, // 改进功能（仅积分限幅）
            .IntegralLimit=5.0f, // 积分限幅
    };
    PIDInit(&gimbal_pitch_gyro_pid,&gimbal_pitch_config); // 初始化PID实例

    // 创建RTOS任务（主循环任务，优先级5，栈大小256字节）
    xTaskCreate(Gimbal_MainLoop,"Gimbal_MainLoopTask",256,NULL,5,NULL);
}