//
// Created by Konodoki on 2024/10/9.
// 文件名: HelmChassis.h
// 描述: 底盘运动控制模块头文件，定义底盘相关参数、结构体类型及核心函数接口，用于底盘运动学解算与车轮控制。
//

#ifndef HUMANROBOT_HELMCHASSIS_H
#define HUMANROBOT_HELMCHASSIS_H

#include "HelmMotor.h"  // 依赖电机控制模块头文件

// 数学常量定义
#define M_PI		3.14159265358979323846  // π的浮点表示

// 角度与弧度转换宏（简化单位转换计算）
#define ANGLE2RAD(x) (x / 180.0f * M_PI)  // 角度转弧度（x为角度值）
#define RAD2ANGLE(x) (x / M_PI * 180.0f)  // 弧度转角度（x为弧度值）

// 机器人结构参数（用于运动学解算）
#define DIS_WHEEL2CENTER 0.125       // 车轮中心到底盘中心的距离（单位：米）
#define WHEEL_RADIUS (0.06f)         // 车轮半径（单位：米）
#define BODY_W (0.28f)               // 车身宽度（左右轮间距，单位：米）
#define BODY_W2 (0.14f)              // 车身半宽（BODY_W/2，单位：米）
#define BODY_L (0.28f)               // 车身长度（前后轮间距，单位：米）
#define BODY_L2 (0.14f)              // 车身半长（BODY_L/2，单位：米）
#define ALLOW_360                    // 宏定义标记：允许车轮360°旋转

// 底盘坐标系类型枚举（控制输入的参考系）
typedef enum
{
  GLOBAL_COORDINATE,    // 基于全局坐标系控制（如绝对北方向）
  CHASSIS_COORDINATE,   // 基于机器人自身底盘坐标系控制（以底盘前进方向为X轴）
  LOCK_CHASSIS          // 锁定底盘方向（不允许旋转）
}chassis_cor_t;

// 底盘解算参数结构体（用于传递控制输入与反馈数据）
typedef struct
{
  float vx_input;                  // 输入的X方向线速度（单位：m/s）
  float vy_input;                  // 输入的Y方向线速度（单位：m/s）
  float vw_input;                  // 输入的旋转角速度（单位：rad/s）
  float rb_wheel_rad_actual;       // 右后轮实际角度（反馈用，单位：rad）
  float rf_wheel_rad_actual;       // 右前轮实际角度（反馈用，单位：rad）
  float lf_wheel_rad_actual;       // 左前轮实际角度（反馈用，单位：rad）
  float lb_wheel_rad_actual;       // 左后轮实际角度（反馈用，单位：rad）
  float yaw;                       // 期望的底盘偏航角（单位：rad）
  chassis_cor_t cor;               // 当前控制使用的坐标系类型（枚举值）
}helm_chassis_solve_param_t;

// 单个车轮状态结构体（描述车轮的实时状态与目标值）
typedef struct
{
  float rad_actual;                // 车轮实际角度（来自传感器反馈，单位：rad）
  float rad_target;                // 车轮目标角度（解算后的期望值，单位：rad）
  float VN2X;                      // 轮系线速度方向与底盘X轴的夹角（-180°~180°，单位：rad）
  float v_target;                  // 车轮目标线速度（解算后的期望值，单位：m/s）
}helm_wheel_t;

// 底盘整体状态结构体（包含所有车轮状态及底盘全局信息）
typedef struct
{
  helm_wheel_t rb_wheel;           // 右后轮状态
  helm_wheel_t rf_wheel;           // 右前轮状态
  helm_wheel_t lf_wheel;           // 左前轮状态
  helm_wheel_t lb_wheel;           // 左后轮状态
  float dir_chassis;               // 底盘当前方向（偏航角，单位：rad）
  chassis_cor_t chassis_cor;       // 底盘当前使用的坐标系类型（枚举值）
}helm_chassis_t;

// 声明外部全局底盘实例（实际定义在HelmChassis.c中，供其他文件共享访问）
extern helm_chassis_t helmChassis;

// 函数声明（具体实现见HelmChassis.c）
float convert_to_smaller_arc(float angle);  // 将角度转换为-π~π范围内的等效角度（小弧度表示）

/* 核心功能函数 */
int Helm_Chassis_Init(helm_chassis_t *helm_chassis);  // 初始化底盘结构体（参数校验、变量清零等）
void Helm_Chassis_Ctrl(helm_chassis_solve_param_t *param, helm_chassis_t *chassis);  // 底盘控制主函数（输入参数解算车轮目标值）
void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel_ptr);  // 单个车轮控制函数（根据底盘速度解算车轮目标）
void Angle_Limit(float *);  // 角度限幅函数（限制角度在合理范围内）
void V_Dir2Wheel_Angle(helm_wheel_t *, float);  // 将速度方向转换为车轮目标角度（调整车轮朝向）
void Helm_Evaluation(helm_chassis_t *);  // 底盘状态评估函数（如异常检测、性能统计等）

void HelmChassis_Init();  // 底盘模块初始化（调用底层硬件初始化等）

#endif // HUMANROBOT_HELMCHASSIS_H