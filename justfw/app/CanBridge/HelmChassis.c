#include "HelmChassis.h"
#include "stdint.h"
#include <math.h>
#include <stdio.h>

// 声明底盘全局实例
helm_chassis_t helmChassis;

/* 函数声明 */

/**
 * @brief 将任意角度转换为0~π范围内的劣弧角度（最短路径角度）
 * @param angle 输入角度（弧度制）
 * @return 转换后的劣弧角度（弧度制）
 */
float convert_to_smaller_arc(float angle) {
  // 将角度归一化到0~2π范围
  float normalized_angle = fmod(angle, 2.0f*M_PI);

  // 处理负角度，转换为正值
  if (normalized_angle < 0) {
    normalized_angle += 2.0f*M_PI;
  }

  // 若角度大于π（半圆），取其补角（2π - angle）作为劣弧
  if (normalized_angle > M_PI) {
    normalized_angle = 2.0f*M_PI - normalized_angle;
  }

  return normalized_angle;
}

/**
 * @brief 初始化底盘及四个轮子的参数
 * @param helm_chassis 底盘结构体指针
 * @return 初始化状态（固定返回1表示成功）
 */
int Helm_Chassis_Init(helm_chassis_t *helm_chassis)
{
  // 初始化右后(RB)轮参数：VN2X为安装角度（-45°），初始目标弧度/速度为0
  helm_chassis->rb_wheel.VN2X = -45;
  helm_chassis->rb_wheel.rad_target = 0;
  helm_chassis->rb_wheel.v_target = 0;

  // 初始化右前(RF)轮参数：安装角度45°
  helm_chassis->rf_wheel.VN2X = 45;
  helm_chassis->rf_wheel.rad_target = 0;
  helm_chassis->rf_wheel.v_target = 0;

  // 初始化左前(LF)轮参数：安装角度135°
  helm_chassis->lf_wheel.VN2X = 135;
  helm_chassis->lf_wheel.rad_target = 0;
  helm_chassis->lf_wheel.v_target = 0;

  // 初始化左后(LB)轮参数：安装角度-135°（等效225°）
  helm_chassis->lb_wheel.VN2X = -135;
  helm_chassis->lb_wheel.rad_target = 0;
  helm_chassis->lb_wheel.v_target = 0;

  // 设置底盘初始坐标系为底盘自身坐标系
  helm_chassis->chassis_cor = CHASSIS_COORDINATE;
  return 1;
}

/**
 * @brief 底盘主控制函数（处理坐标系转换和轮子控制）
 * @param param 控制参数结构体（包含输入速度、当前轮实际弧度等）
 * @param chassis 底盘结构体指针
 */
void Helm_Chassis_Ctrl(helm_chassis_solve_param_t *param,helm_chassis_t *chassis)
{
  static float vx;  // 底盘X方向速度（经坐标系转换后）
  static float vy;  // 底盘Y方向速度（经坐标系转换后）

  // 更新底盘状态：当前朝向、各轮实际弧度、当前坐标系
  chassis->dir_chassis = param->yaw;
  chassis->lb_wheel.rad_actual=param->lb_wheel_rad_actual;
  chassis->lf_wheel.rad_actual=param->lf_wheel_rad_actual;
  chassis->rf_wheel.rad_actual=param->rf_wheel_rad_actual;
  chassis->rb_wheel.rad_actual=param->rb_wheel_rad_actual;
  chassis->chassis_cor=param->cor;

  // 根据当前坐标系类型转换速度输入
  switch(chassis->chassis_cor)
  {
    // 底盘自身坐标系：直接使用输入速度
    case CHASSIS_COORDINATE:
      vx = param->vx_input;
      vy = param->vy_input;
      break;
    // 全局坐标系：将输入速度转换为底盘坐标系（考虑底盘朝向）
    case GLOBAL_COORDINATE:
      vx = param->vx_input*cosf(chassis->dir_chassis) + param->vy_input*sinf(chassis->dir_chassis);
      vy = param->vy_input*cosf(chassis->dir_chassis) - param->vx_input*sinf(chassis->dir_chassis);
      break;
    // 锁定模式：速度置零
    case LOCK_CHASSIS:
      vx = 0;
      vy = 0;
      break;
  }

  // 当输入速度极小时（视为静止），所有轮子目标速度置零
  if(fabsf(vx) <= 0.001f && fabsf(vy) <= 0.001f && fabsf(param->vw_input) <= 0.1f)
  {
    chassis->rb_wheel.v_target = 0;
    chassis->rf_wheel.v_target = 0;
    chassis->lf_wheel.v_target = 0;
    chassis->lb_wheel.v_target = 0;
  }
  // 否则调用轮子控制函数计算各轮目标参数
  else
  {
    Helm_Wheel_Ctrl(vx, vy, param->vw_input, &chassis->rb_wheel);
    Helm_Wheel_Ctrl(vx, vy, param->vw_input, &chassis->rf_wheel);
    Helm_Wheel_Ctrl(vx, vy, param->vw_input, &chassis->lf_wheel);
    Helm_Wheel_Ctrl(vx, vy, param->vw_input, &chassis->lb_wheel);
  }
}

/**
 * @brief 单个轮子控制函数（计算目标速度和角度）
 * @param vx 底盘X方向速度
 * @param vy 底盘Y方向速度
 * @param wz 底盘旋转角速度
 * @param wheel_ptr 轮子结构体指针
 */
void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel_ptr)
{
  // 条件编译：当不允许360°旋转时的处理逻辑
#ifndef ALLOW_360
  // 禁止360°旋转时，通过叠加旋转速度计算轮子实际速度分量
  vx=vx + wz * DIS_WHEEL2CENTER*cosf(ANGLE2RAD(wheel_ptr->VN2X));
  vy=vy + wz * DIS_WHEEL2CENTER*sinf(ANGLE2RAD(wheel_ptr->VN2X));

  // 计算轮子目标方向（弧度）和线速度
  float m_r=atan2(vy,vx);
  float v_t=sqrtf(vx*vx+vy*vy);

  // 角度调整：负角度转换为正方向（通过反转速度实现）
  if(m_r<0){
    m_r+=M_PI;
    v_t*=-1;
  }

  wheel_ptr->rad_target=m_r;
  wheel_ptr->v_target=v_t;
  return;
#endif // ALLOW_360

  // 允许360°旋转时的处理逻辑
  float vx_wheel = 0;  // 轮子X方向速度分量
  float vy_wheel = 0;  // 轮子Y方向速度分量
  float VN = 0;        // 旋转带来的线速度（wz * 轮心到中心距离）
  float v_target = 0;  // 轮子目标线速度
  float diraction = 0; // 轮子目标方向（角度制）

  // 计算旋转带来的线速度分量
  VN = wz * DIS_WHEEL2CENTER;

  // 叠加底盘速度和旋转速度，得到轮子实际速度分量
  vx_wheel = vx + VN*cosf(ANGLE2RAD(wheel_ptr->VN2X));
  vy_wheel = vy + VN*sinf(ANGLE2RAD(wheel_ptr->VN2X));

  // 计算轮子目标线速度（合速度大小）
  v_target = sqrtf(vx_wheel*vx_wheel + vy_wheel*vy_wheel);
  wheel_ptr->v_target = v_target;

  // 计算轮子目标方向（弧度转角度）
  diraction = atan2f(vy_wheel, vx_wheel);
  diraction = RAD2ANGLE(diraction);  // 轮系角度与底盘可能相差180°，需注意机械结构

  // 角度限制（-180°~180°）并转换为轮子目标角度
  Angle_Limit(&diraction);
  V_Dir2Wheel_Angle(wheel_ptr, diraction);
}

/**
 * @brief 将目标方向转换为轮子实际目标角度（处理角度劣弧问题）
 * @param wheel 轮子结构体指针
 * @param dir 目标方向（角度制）
 */
void V_Dir2Wheel_Angle(helm_wheel_t *wheel, float dir)
{
  float err_angle;          // 当前角度与目标方向的误差
  float now_angle = RAD2ANGLE(wheel->rad_actual);  // 当前轮子角度（弧度转角度）

  // 限制当前角度在-180°~180°范围内
  Angle_Limit(&now_angle);

  // 计算角度误差（目标方向 - 当前角度）
  err_angle = dir - now_angle;
  Angle_Limit(&err_angle);  // 误差角度归一化

  // 若误差超过90°（取劣弧），通过反转速度减少旋转角度
  if(fabsf(err_angle) > 90.0f)
  {
    wheel->v_target = -wheel->v_target;  // 反转速度方向
    // 调整误差角度为180°补角（例如120°误差变为-60°）
    if(err_angle > 0) {
      err_angle = err_angle - 180.0f;
    } else {
      err_angle = err_angle + 180.0f;
    }
  }

  // 计算轮子目标弧度（当前弧度 + 误差弧度）
  wheel->rad_target = wheel->rad_actual + ANGLE2RAD(err_angle);
}

/**
 * @brief 角度限制函数（将角度限制在-180°~180°范围内）
 * @param angle 待调整的角度指针（角度制）
 */
void Angle_Limit(float *angle)
{
  // 递归调整角度：超过180°则减360°，低于-180°则加360°
  if(*angle>180.0f) {
    *angle-=360.0f;
    Angle_Limit(angle);
  } else if(*angle<=-180.0f) {
    *angle+=360.0f;
    Angle_Limit(angle);
  }
}

/**
 * @brief 底盘运动评估函数（计算实际线速度和角速度）
 * @param chassis 底盘结构体指针
 */
void Helm_Evaluation(helm_chassis_t *chassis)
{
  // 计算各轮对旋转角速度的贡献系数（基于轮位置和角度）
  float s1 = (BODY_W * sinf(chassis->rb_wheel.rad_actual) - BODY_L * cosf(chassis->rb_wheel.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->rb_wheel.v_target;
  float s2 = (-BODY_W * sinf(chassis->rf_wheel.rad_actual) - BODY_L * cosf(chassis->rf_wheel.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->rf_wheel.v_target;
  float s3 = (-BODY_W * sinf(chassis->lf_wheel.rad_actual) + BODY_L * cosf(chassis->lf_wheel.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->lf_wheel.v_target;
  float s4 = (BODY_W * sinf(chassis->lb_wheel.rad_actual) + BODY_L * cosf(chassis->lb_wheel.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->lb_wheel.v_target;

  float cal_vx, cal_vy, cal_wz;  // 计算得到的线速度和角速度

  // 线速度计算：各轮速度在X/Y方向的分量平均值
  cal_vx = (chassis->rb_wheel.v_target * cosf(chassis->rb_wheel.rad_actual) +
            chassis->rf_wheel.v_target * cosf(chassis->rf_wheel.rad_actual) +
            chassis->lf_wheel.v_target * cosf(chassis->lf_wheel.rad_actual) +
            chassis->lb_wheel.v_target * cosf(chassis->lb_wheel.rad_actual)) / 4;

  cal_vy = (chassis->rb_wheel.v_target * sinf(chassis->rb_wheel.rad_actual) +
            chassis->rf_wheel.v_target * sinf(chassis->rf_wheel.rad_actual) +
            chassis->lf_wheel.v_target * sinf(chassis->lf_wheel.rad_actual) +
            chassis->lb_wheel.v_target * sinf(chassis->lb_wheel.rad_actual)) / 4;

  // 角速度计算：各轮旋转贡献的平均值（取反）
  cal_wz = -(s1 + s2 + s3 + s4) / 4;

  // 打印调试信息（各轮速度、计算得到的线速度和角速度）
  printf("v1 = %f, v2 = %f, v3 = %f, v4 = %f \n", chassis->rb_wheel.v_target, chassis->rf_wheel.v_target, chassis->lf_wheel.v_target, chassis->lb_wheel.v_target);
  printf("Calculate vx: %f, vy: %f, wz: %f \n", cal_vx, cal_vy, cal_wz);
}

/**
 * @brief 底盘初始化入口函数（调用电机初始化和底盘参数初始化）
 */
void HelmChassis_Init() {
  HelmMotor_Init();          // 初始化电机驱动
  Helm_Chassis_Init(&helmChassis);  // 初始化底盘参数
}