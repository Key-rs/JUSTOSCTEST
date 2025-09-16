//
// Created by ASUS on 25-7-16.
//

#ifndef GIMBALFOLLOW_H
#define GIMBALFOLLOW_H
#include "intf_gimbal.h"
#include "PID.h"
#include "user_lib.h"

// 模式定义：电机角度模式/陀螺仪角度模式
typedef enum {
    MOTOR_ANGLE_MODE = 0,  // 电机反馈角度PID控制
    GYRO_ANGLE_MODE = 1    // 陀螺仪角度PID控制
} Gimbal_Mode;



typedef struct {
    // 模式相关
    enum Gimbal_Mode current_mode;       // 当前模式
    enum Gimbal_Mode prev_mode;          // 上一次模式（用于检测切换）

    // 角度源（双传感器独立存储）
    float motor_angle;              // 电机反馈角度(rad)
    float gyro_angle;               // 陀螺仪角度(rad)

    // 控制器核心参数（使用库中的PIDInstance）
    PIDInstance pid;                // 角度控制PID
    LowPassFilterConfig motor_filter;     // 电机角度滤波器
    LowPassFilterConfig gyro_filter;      // 陀螺仪滤波器
    LowPassFilterConfig output_filter;    // 输出速度滤波器
    float target_yaw;               // 目标偏航角(rad)
    float real_yaw;                 // 当前模式下的实际角度(rad，已滤波)
    float output_speed;             // 输出速度(rad/s)
    float dt;                       // 采样时间(s)

    // 模式切换平滑过渡参数
    float mode_switch_alpha;        // 切换时目标值过渡系数
    float mode_switch_target;       // 过渡过程中的目标值
} Gimbal_Control;

void Gimbal_SwitchMode(Gimbal_Control *gimbal,Gimbal_Mode new_mode);



#endif //GIMBALFOLLOW_H
