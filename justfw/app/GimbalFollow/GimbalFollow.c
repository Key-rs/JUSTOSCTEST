#include <stdio.h>
#include "test.h"
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "PID.h"  // 引入新的PID库
#include "GimbalFollow.h"

extern float INS_SUM_angle[3];
#define GIMBAL_REAL_YAW INS_SUM_angle[2]  // YAW角度(rad)
RC_ctrl_t *gimbal_logic_rc_ctrl;
INTF_Motor_HandleTypeDef *gimbal_motor;
extern int g_dr16_is_connected;

static float* angle_table[2];



// 云台控制器初始化（使用库中的PID初始化函数）
void Gimbal_PID_Init(Gimbal_Control *gimbal,
                // PID参数
                float kp, float ki, float kd, float output_limit, float dead_zone,
                // 滤波参数
                float CoefA, float CoefB//变速积分参数
                ) {

    // 初始化时建立映射表（只执行一次）

        angle_table[MOTOR_ANGLE_MODE] = &gimbal->motor_angle;
        angle_table[GYRO_ANGLE_MODE] = &gimbal->gyro_angle;

    // 配置PID初始化参数
    PID_Init_Config_s pid_config = {
        .Kp = kp,
        .Ki = ki,
        .Kd = kd,
        .MaxOut = 30.0f * RPM2RPS,        // 输出限幅
        .IntegralLimit = output_limit, // 积分限幅
        .DeadBand = dead_zone,         // 死区阈值
         .Improve = //PID_Trapezoid_Intergral |   // 启用梯形积分
                   PID_Integral_Limit ,  // 启用积分限幅
                       // PID_Derivative_On_Measurement|
                       //     PID_ChangingIntegrationRate,
                   // PID_DerivativeFilter ,     // 启用微分滤波
                   PID_OutputFilter,          // 启用输出滤波
        // .CoefA = CoefA,                 // 变速积分参数A
        // .CoefB = CoefB,                 // 变速积分参数B
        .Derivative_LPF_RC = 0.1f,    // 微分滤波时间常数
        // .Output_LPF_RC = 0.01f         // 输出滤波时间常数
    };

    // 初始化PID控制器
    PIDInit(&gimbal->pid, &pid_config);

    // 初始化模式与角度参数
    gimbal->current_mode = GYRO_ANGLE_MODE;  // 默认电机模式
    gimbal->prev_mode = GYRO_ANGLE_MODE;
    gimbal->target_yaw = 0.0f;
    gimbal->real_yaw = 0.0f;
    gimbal->motor_angle = 0.0f;
    gimbal->gyro_angle = 0.0f;
    gimbal->output_speed = 0.0f;
    gimbal->mode_switch_target = 0.0f;
}

// 模式切换核心逻辑（解决抖动的关键）
void Gimbal_SwitchMode(Gimbal_Control *gimbal, Gimbal_Mode new_mode) {
    if (gimbal->current_mode == new_mode) return;  // 模式未变化则退出

    // 1. 记录旧模式，更新新模式
    gimbal->prev_mode = gimbal->current_mode;
    gimbal->current_mode = new_mode;

    // 2. 重置PID状态（清除历史误差，避免切换后突变）
    gimbal->pid.Iout = 0.0f;
    gimbal->pid.Last_Err = 0.0f;

    // 3. 同步目标值：将目标值设置为新模式下的当前实际角度（关键！）
    // 确保切换瞬间误差为0，无输出突变
    if (new_mode == MOTOR_ANGLE_MODE) {
        // 切换到电机模式：目标值 = 当前电机角度（已滤波）
        gimbal->mode_switch_target = gimbal->motor_angle;
    } else {
        // 切换到陀螺仪模式：目标值 = 当前陀螺仪角度（已滤波）
        gimbal->mode_switch_target = gimbal->gyro_angle;
    }
    gimbal->target_yaw = gimbal->mode_switch_target;  // 初始目标值同步

    // 4. 重置滤波器历史值（避免滤波延迟导致的角度跳变）
    gimbal->output_filter.prev_output = 0.0f;
}

// 设置目标角度（支持外部控制，如遥控器）
void Gimbal_SetTargetYaw(Gimbal_Control *gimbal, float target) {
    gimbal->mode_switch_target = target;
}

// 云台控制更新（使用库中的PID计算函数）
float Gimbal_Update(Gimbal_Control *gimbal, float motor_feedback, float gyro_feedback) {
    // 1. 独立更新双传感器角度
    gimbal->motor_angle = motor_feedback;
    gimbal->gyro_angle = gyro_feedback;

     gimbal->real_yaw=*angle_table[gimbal->current_mode];
    gimbal->target_yaw = gimbal->mode_switch_target;

    // 3. 计算角度误差
    float error = gimbal->target_yaw - gimbal->real_yaw;
    // 调整误差到最短路径
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    // 4. 使用库中的PID计算函数进行控制计算
    float pid_output = PIDCalculate(&gimbal->pid, gimbal->real_yaw, gimbal->target_yaw);

    // 5. 输出滤波（进一步平滑输出）
    // gimbal->output_speed = LowPassFilter_Apply(&gimbal->output_filter, pid_output);
    gimbal->output_speed=pid_output;
    return gimbal->output_speed;
}

float target_imu_angle,target_motor_angle;
// 主控制逻辑（包含模式切换触发）
int mode_state;  // 0:电机模式, 1:陀螺仪模式（可通过遥控器切换）

void GimbalFollow_MainLoop() {
    vTaskDelay(3000);  // 系统初始化延迟

    // 初始化云台控制器
    Gimbal_Control gimbal;

    Gimbal_PID_Init(&gimbal,
                // 7.5f, 0.0f, 0.525f,  // PID参数
                70.0f, 0.0f, 16.5f,  // PID参数
                // 105.0f, 0.0f, 12.5f,  // PID参数

                // 18.0f, 0.0f, 10.0f,  // PID参数
                0.0f, 0.0f,      // 输出限幅与死区
                0.05f,0.05f);
    LowPassFilter_Init(&gimbal.output_filter, 0.4f);
    // 主控制循环
    while (1) {
        // 1. 获取传感器数据（合并连续调用，减少重复）
        const float motor_feedback = gimbal_motor->real_angle;
        const float gyro_feedback = GIMBAL_REAL_YAW;

        // 2. 遥控器状态检查（合并重复条件判断）
        const bool rc_connected = (g_dr16_is_connected != 0);

        // 3. 模式切换逻辑（简化条件判断，提取公共操作）
        // Gimbal_Mode desired_mode = (rc_connected &&
        //                                 gimbal_logic_rc_ctrl[0].rc.switch_right == 2)
        //                                 ? GYRO_ANGLE_MODE : MOTOR_ANGLE_MODE;

        Gimbal_Mode desired_mode = GYRO_ANGLE_MODE;

        if (desired_mode != gimbal.current_mode) {
            Gimbal_SwitchMode(&gimbal, desired_mode);
            target_imu_angle = GIMBAL_REAL_YAW;
            target_motor_angle = gimbal_motor->real_angle;
            mode_state = (int)desired_mode;
        }

        // 4. 遥控器控制目标角度（提取公共操作，减少嵌套）
        if (rc_connected) {
            if (gimbal.current_mode == MOTOR_ANGLE_MODE)
            {
                // 设置目标角度（使用临时变量提高可读性）
                Gimbal_SetTargetYaw(&gimbal, target_motor_angle);
            }
            else if (gimbal.current_mode == GYRO_ANGLE_MODE)
            {
                // 设置目标角度（使用临时变量提高可读性）
                Gimbal_SetTargetYaw(&gimbal, target_imu_angle);
            }
            int16_t rocker_value = gimbal_logic_rc_ctrl[0].rc.rocker_r_;
            // 根据摇杆值和采样时间计算角度增量
            float delta = -0.06f;  // 最大角速度(rad/s)
            // float delta = speed_factor * gimbal.dt * (rocker_value / 660.0f);  // 摇杆值归一化到[-1,1]
            // if (rocker_value > 150) {  // 死区判断
            //     target_motor_angle += delta;
            //     target_imu_angle += delta;
            // }else if (rocker_value < -150) {
            //     target_motor_angle -= delta;
            //     target_imu_angle -= delta;
            // }
        }
        target_imu_angle +=gimbal_logic_rc_ctrl[0].rc.rocker_r_ /660.0f*  0.05f;
        const float output_speed = Gimbal_Update(&gimbal, motor_feedback, gyro_feedback);
        LowPassFilter_Apply(&gimbal.output_filter, output_speed);
        gimbal_motor->set_speed(gimbal_motor, rc_connected ? output_speed : 0.0f);
         // printf("target_real_out:%.3f,%.3f,%.3f\n",target_imu_angle,GIMBAL_REAL_YAW,output_speed);
        vTaskDelay(10);  // 10ms控制周期（与初始化时的dt一致）
    }
}

// 任务启动函数
void Gimbal_Start() {
    gimbal_motor = pvSharePtr("gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    gimbal_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));

    xTaskCreate(GimbalFollow_MainLoop, "GimbalFollow_MainLoop", 1024, NULL, 240, NULL);
}