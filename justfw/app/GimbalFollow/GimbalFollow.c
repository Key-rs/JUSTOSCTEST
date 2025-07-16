#include <stdio.h>
#include "test.h"
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "tinybus_intf.h"

RC_ctrl_t *gimbal_logic_rc_ctrl;
INTF_Motor_HandleTypeDef *gimbal_motor;
extern int g_dr16_is_connected;
volatile float real_yaw_r;  // 陀螺仪原始数据（已重命名避免冲突）

/**
 * 云台控制模块（带PID死区功能）
 */

// 角度归一化宏（限制在 [-π, π]）
#define NORMALIZE_ANGLE(angle) \
    while((angle) > M_PI) (angle) -= 2.0f * M_PI; \
    while((angle) < -M_PI) (angle) += 2.0f * M_PI

// PID控制器结构体（新增死区参数）
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_limit; // 输出限幅（rad/s）
    float dead_zone;    // 死区阈值（rad）：误差小于此值时不输出
} PID_Controller;

// 一阶低通滤波器结构体
typedef struct {
    float alpha;        // 滤波系数 (0.0~1.0)
    float prev_output;  // 上一次输出值
} LowPassFilter;

// 云台控制参数结构体
typedef struct {
    PID_Controller pid;           // 角度控制PID
    LowPassFilter gyro_filter;    // 陀螺仪滤波
    LowPassFilter output_filter;  // 输出滤波
    float target_yaw;             // 目标偏航角(rad)
    float real_yaw;               // 滤波后的实际偏航角(rad)
    float output_speed;           // 输出速度(rad/s)
    float dt;                     // 采样时间(s)
} Gimbal_Control;

/**
 * 初始化PID控制器（新增死区参数）
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param limit 输出限幅(rad/s)
 * @param dead_zone 死区阈值(rad)：误差小于此值时不输出
 */
void PID_Controller_Init(PID_Controller *pid, float kp, float ki, float kd,
                        float limit, float dead_zone) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = limit;
    pid->dead_zone = dead_zone;  // 初始化死区阈值
}

/**
 * 初始化低通滤波器
 */
void LowPassFilter_Init(LowPassFilter *filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
}

/**
 * 低通滤波计算
 */
float LowPassFilter_Apply(LowPassFilter *filter, float input) {
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    return output;
}

/**
 * 初始化云台控制器（传递死区参数）
 * @param dead_zone 死区阈值(rad)：例如0.05rad≈2.87°（根据需求调整）
 */
void Gimbal_PID_Init(Gimbal_Control *gimbal,
                float kp, float ki, float kd, float output_limit,
                float dead_zone,  // 新增：死区阈值
                float gyro_alpha, float output_alpha, float dt) {
    // 初始化PID时传入死区参数
    PID_Controller_Init(&gimbal->pid, kp, ki, kd, output_limit, dead_zone);
    LowPassFilter_Init(&gimbal->gyro_filter, gyro_alpha);
    LowPassFilter_Init(&gimbal->output_filter, output_alpha);
    gimbal->target_yaw = 0.0f;
    gimbal->real_yaw = 0.0f;
    gimbal->output_speed = 0.0f;
    gimbal->dt = dt;
}

/**
 * 设置云台目标偏航角
 */
void Gimbal_SetTargetYaw(Gimbal_Control *gimbal, float target_yaw) {
    NORMALIZE_ANGLE(target_yaw);
    gimbal->target_yaw = target_yaw;
}

/**
 * 更新云台控制状态（核心：添加死区逻辑）
 */
float Gimbal_Update(Gimbal_Control *gimbal, float gyro_yaw) {
    // 陀螺仪数据滤波
    gimbal->real_yaw = LowPassFilter_Apply(&gimbal->gyro_filter, gyro_yaw);

    // 计算角度误差（最短路径）
    float error = gimbal->target_yaw - gimbal->real_yaw;
    NORMALIZE_ANGLE(error);

    // --------------------------
    // 新增：死区处理逻辑
    // --------------------------
    if (fabs(error) < gimbal->pid.dead_zone) {
        // 误差在死区内：将误差置为0，停止积分累积
        error = 0.0f;
        // 可选：若希望死区内快速归零，可重置积分项（根据需求选择）
        // gimbal->pid.integral = 0.0f;
    }

    // PID计算（仅当误差不在死区时更新积分/微分）
    if (error != 0.0f) {
        // 积分项累积（死区内不累积）
        gimbal->pid.integral += error * gimbal->dt;
        // 积分限幅
        if (gimbal->pid.integral > gimbal->pid.output_limit)
            gimbal->pid.integral = gimbal->pid.output_limit;
        if (gimbal->pid.integral < -gimbal->pid.output_limit)
            gimbal->pid.integral = -gimbal->pid.output_limit;
    }

    // 微分项计算（死区内误差为0，微分为0）
    float derivative = (error - gimbal->pid.prev_error) / gimbal->dt;
    gimbal->pid.prev_error = error;

    // 计算PID输出
    float pid_output = gimbal->pid.kp * error +
                       gimbal->pid.ki * gimbal->pid.integral +
                       gimbal->pid.kd * derivative;

    // 输出限幅
    if (pid_output > gimbal->pid.output_limit)
        pid_output = gimbal->pid.output_limit;
    if (pid_output < -gimbal->pid.output_limit)
        pid_output = -gimbal->pid.output_limit;

    // 输出滤波
    gimbal->output_speed = LowPassFilter_Apply(&gimbal->output_filter, pid_output);

    return gimbal->output_speed;
}

void GimbalFollow_MainLoop()
{
    vTaskDelay(3000);
    Gimbal_Control gimbal;
    Gimbal_PID_Init(&gimbal,
               6.10f, 0.03f, 0.001f,    // PID参数  7 0.02  0.02
               15.0f,                // 最大输出速度5rad/s
               0.005f, 0.8f,          // 滤波系数
               0.8f,0.001f);              // 采样时间1
    while (1)
    {
        float gyro_data = real_yaw_r;  // 读取陀螺仪数据
        float output_speed = Gimbal_Update(&gimbal, gyro_data);
        // Gimbal_SetTargetYaw(&gimbal, 0);  // 设置为90度
        printf("gyro_data:%f,%f\n",real_yaw_r,output_speed);
        // printf("output_speed:%f\n",output_speed);
        if (g_dr16_is_connected)
        {
            Gimbal_SetTargetYaw(&gimbal, gimbal_logic_rc_ctrl[0].rc.rocker_l1/660.0f*PI);  // 设置为90度
            gimbal_motor->set_speed(gimbal_motor,output_speed);
            // Set_speed(test_logic_rc_ctrl[0].rc.rocker_l1/660.0f*20.0f,-test_logic_rc_ctrl[0].rc.rocker_l_/660.0f*20.0f,test_logic_rc_ctrl[0].rc.rocker_r_/660.0f*20.0,0.0f,0);
            // Set_speed(0.0f,0.0f,20.0f,0.0f,0);

            // test_motor->set_torque(test_motor,3.0f);
            // if (g_dr16_is_connected)
            // {
            //     // if(g_dr16_is_connected)
            //     // {
            //     //     // printf("USB_OK");
            //     //     // float anglex = test_logic_rc_ctrl[0].rc.rocker_l_/660.0f*10.0f;
            //     //     // test_motor->set_angle(test_motor, anglex);
            //     //     vTaskDelay(10);
            //     // }
            //     // test_motor->set_speed(test_motor, test_logic_rc_ctrl[0].rc.rocker_l1/660.0f*3.0f);;
            //
            //     printf("%f\n",test_motor->real_speed);

            // }else
            // {
            //     // test_motor->set_speed(test_motor, 0.0f);;
            // }
        }else
        {
            gimbal_motor->set_speed(gimbal_motor,0.0f);

        }
        vTaskDelay(1);
    }
}

void Gimbal_Test_Start()
{
    gimbal_motor = pvSharePtr("gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    gimbal_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(GimbalFollow_MainLoop, "GimbalFollow_MainLoop", 512, NULL, 240, NULL);
}