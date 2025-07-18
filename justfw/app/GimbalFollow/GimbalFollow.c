#include <stdio.h>
#include "test.h"
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "tinybus_intf.h"

// 模式定义：电机角度模式/陀螺仪角度模式
typedef enum {
    MOTOR_ANGLE_MODE = 0,  // 电机反馈角度PID控制
    GYRO_ANGLE_MODE = 1    // 陀螺仪角度PID控制
} Gimbal_Mode;

extern float INS_SUM_angle[3];
#define GIMBAL_REAL_YAW INS_SUM_angle[2]  // 陀螺仪角度(rad)
RC_ctrl_t *gimbal_logic_rc_ctrl;
INTF_Motor_HandleTypeDef *gimbal_motor;
extern int g_dr16_is_connected;

// PID控制器结构体（新增死区参数）
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_limit; // 输出限幅（rad/s）
    float dead_zone;    // 死区阈值（rad）
} PID_Controller;

// 一阶低通滤波器结构体
typedef struct {
    float alpha;        // 滤波系数 (0.0~1.0)
    float prev_output;  // 上一次输出值
} LowPassFilter;

// 云台控制参数结构体（扩展双模式支持）
typedef struct {
    // 模式相关
    Gimbal_Mode current_mode;       // 当前模式
    Gimbal_Mode prev_mode;          // 上一次模式（用于检测切换）

    // 角度源（双传感器独立存储）
    float motor_angle;              // 电机反馈角度(rad)
    float gyro_angle;               // 陀螺仪角度(rad)

    // 控制器核心参数
    PID_Controller pid;             // 角度控制PID
    LowPassFilter motor_filter;     // 电机角度滤波器
    LowPassFilter gyro_filter;      // 陀螺仪滤波器
    LowPassFilter output_filter;    // 输出速度滤波器
    float target_yaw;               // 目标偏航角(rad)
    float real_yaw;                 // 当前模式下的实际角度(rad，已滤波)
    float output_speed;             // 输出速度(rad/s)
    float dt;                       // 采样时间(s)

    // 模式切换平滑过渡参数
    float mode_switch_alpha;        // 切换时目标值过渡系数
    float mode_switch_target;               // 过渡过程中的目标值
} Gimbal_Control;


// PID初始化
void PID_Controller_Init(PID_Controller *pid, float kp, float ki, float kd,
                        float limit, float dead_zone) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = limit;
    pid->dead_zone = dead_zone;
}

// 低通滤波器初始化
void LowPassFilter_Init(LowPassFilter *filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
}

// 低通滤波计算
float LowPassFilter_Apply(LowPassFilter *filter, float input) {
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    return output;
}

// 云台控制器初始化（支持双模式参数）
void Gimbal_PID_Init(Gimbal_Control *gimbal,
                // PID参数（双模式可共用或独立，此处共用）
                float kp, float ki, float kd, float output_limit, float dead_zone,
                // 滤波参数
                float motor_alpha, float gyro_alpha, float output_alpha,
                // 时间与过渡参数
                float dt, float switch_alpha) {
    // 初始化PID
    PID_Controller_Init(&gimbal->pid, kp, ki, kd, output_limit, dead_zone);

    // 初始化滤波器（电机角度和陀螺仪独立滤波）
    LowPassFilter_Init(&gimbal->motor_filter, motor_alpha);
    LowPassFilter_Init(&gimbal->gyro_filter, gyro_alpha);
    LowPassFilter_Init(&gimbal->output_filter, output_alpha);

    // 初始化模式与角度参数
    gimbal->current_mode = MOTOR_ANGLE_MODE;  // 默认电机模式
    gimbal->prev_mode = MOTOR_ANGLE_MODE;
    gimbal->target_yaw = 0.0f;
    gimbal->real_yaw = 0.0f;
    gimbal->motor_angle = 0.0f;
    gimbal->gyro_angle = 0.0f;
    gimbal->output_speed = 0.0f;
    gimbal->dt = dt;

    // 模式切换过渡参数（0.1~0.3为宜，值越小过渡越平滑）
    gimbal->mode_switch_alpha = switch_alpha;
    gimbal->mode_switch_target = 0.0f;
}


// 模式切换核心逻辑（解决抖动的关键）
void Gimbal_SwitchMode(Gimbal_Control *gimbal, Gimbal_Mode new_mode) {
    if (gimbal->current_mode == new_mode) return;  // 模式未变化则退出

    // 1. 记录旧模式，更新新模式
    gimbal->prev_mode = gimbal->current_mode;
    gimbal->current_mode = new_mode;

    // 2. 重置PID状态（清除历史误差，避免切换后突变）
    gimbal->pid.integral = 0.0f;
    gimbal->pid.prev_error = 0.0f;

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

// 云台控制更新（根据当前模式选择角度源）
float Gimbal_Update(Gimbal_Control *gimbal, float motor_feedback, float gyro_data) {
    // 1. 独立更新双传感器角度（分别滤波，互不干扰）
    gimbal->motor_angle = LowPassFilter_Apply(&gimbal->motor_filter, motor_feedback);
    gimbal->gyro_angle = LowPassFilter_Apply(&gimbal->gyro_filter, gyro_data);

    // 2. 根据当前模式选择实际角度，并平滑过渡目标值（解决切换时目标跳变）
    if (gimbal->current_mode == MOTOR_ANGLE_MODE) {
        gimbal->real_yaw = gimbal->motor_angle;  // 电机角度作为反馈
    } else {
        gimbal->real_yaw = gimbal->gyro_angle;   // 陀螺仪角度作为反馈
    }

    // 目标值平滑过渡（无论是否切换模式，均用低通滤波处理目标值，避免突变）
    gimbal->target_yaw = gimbal->mode_switch_alpha * gimbal->mode_switch_target +
                        (1 - gimbal->mode_switch_alpha) * gimbal->target_yaw;
    gimbal->target_yaw = gimbal->target_yaw;

    // 3. 计算角度误差（最短路径）
    float error = (gimbal->target_yaw - gimbal->real_yaw);

    // 4. 死区处理
    if (fabs(error) < gimbal->pid.dead_zone) {
        error = 0.0f;
        // gimbal->pid.integral = 0.0f;  // 可选：死区内清零积分（根据需求）
    }

    // 5. PID计算（仅在误差非零时更新积分/微分）
    float pid_output = 0.0f;
    if (error != 0.0f) {
        // 积分项（限幅）
        gimbal->pid.integral += error * gimbal->dt;
        gimbal->pid.integral = fminf(fmaxf(gimbal->pid.integral, -gimbal->pid.output_limit), gimbal->pid.output_limit);

        // 微分项
        float derivative = (error - gimbal->pid.prev_error) / gimbal->dt;
        gimbal->pid.prev_error = error;

        // 总输出
        pid_output = gimbal->pid.kp * error +
                    gimbal->pid.ki * gimbal->pid.integral +
                    gimbal->pid.kd * derivative;
    }

    // 6. 输出限幅与滤波（进一步平滑输出）
    pid_output = fminf(fmaxf(pid_output, -gimbal->pid.output_limit), gimbal->pid.output_limit);
    gimbal->output_speed = LowPassFilter_Apply(&gimbal->output_filter, pid_output);

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
                10.5f, 0.02f, 0.02f,  // PID参数
                15.0f, 0.002,      // 输出限幅与死区
                0.8f, 0.8f, 0.7f,     // 滤波系数
                0.01f, 0.2f);         // 采样时间与过渡系数

    // 主控制循环
    while (1) {
        // 1. 获取传感器数据（合并连续调用，减少重复）
        const float motor_feedback = gimbal_motor->real_angle;
        const float gyro_feedback = GIMBAL_REAL_YAW;

        // 2. 遥控器状态检查（合并重复条件判断）
        const bool rc_connected = (g_dr16_is_connected != 0);

        // 3. 模式切换逻辑（简化条件判断，提取公共操作）
         Gimbal_Mode  desired_mode = (rc_connected &&
                                         gimbal_logic_rc_ctrl[0].rc.switch_right == 2)
                                         ? GYRO_ANGLE_MODE : MOTOR_ANGLE_MODE;

        if (desired_mode != gimbal.current_mode) {
            target_imu_angle = gimbal.gyro_angle;
            target_motor_angle = gimbal.motor_angle;
            Gimbal_SwitchMode(&gimbal, desired_mode);
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
            const int16_t rocker_value = gimbal_logic_rc_ctrl[0].rc.rocker_r_;
            if (rocker_value > 200) target_motor_angle += 0.5f,target_imu_angle+=0.5f;
            else if (rocker_value < -200) target_motor_angle -= 0.5f,target_imu_angle-=0.5f;

        }

        // 5. 更新控制并输出（合并条件判断，减少重复调用）
        const float output_speed = Gimbal_Update(&gimbal, motor_feedback, gyro_feedback);
        gimbal_motor->set_speed(gimbal_motor, rc_connected ? output_speed : 0.0f);

        // 6. 控制周期延时（明确注释周期时间）
        vTaskDelay(10);  // 10ms控制周期，与采样时间dt一致
    }
}

// 任务启动函数
void Gimbal_Test_Start() {
    gimbal_motor = pvSharePtr("gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    gimbal_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(GimbalFollow_MainLoop, "GimbalFollow_MainLoop", 1024, NULL, 240, NULL);
}