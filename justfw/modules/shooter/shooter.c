//
// Created by Ukua on 2024/1/23.
//

#include "shooter.h"

#include "cmsis_os2.h"
#include "referee.h"
INTF_Shooter_HandleTypeDef g_shooter = {0};

static float fric_speed = 38.0f;
static float one_shoot_angle = 25.0f;
extern referee_data_t *g_referee;
static uint8_t feeder_blocked=0;
static uint32_t feeder_blocked_count=0;
#define Waiting_for_Cooling //超热量检测是否等待冷却
///@brief 拨盘供弹
///@param i 拨弹的个数
void Shoot(int i) {
  if(feeder_blocked){
    Vibrating();
    feeder_blocked=0;
    feeder_blocked_count=0;
    return;
  }
  if(g_referee->game_robot_state.robot_id!=0&&!g_referee->game_robot_state.power_management_shooter_is_output){
    return;
  }
  static uint8_t can_shoot=0;
  static float max_heat=200;
  if(g_referee->game_robot_state.shooter_barrel_heat_limit!=0){
    max_heat=g_referee->game_robot_state.shooter_barrel_heat_limit;
  }
#ifdef Waiting_for_Cooling
  if(can_shoot==0){
    can_shoot = g_referee->power_heat_data.shooter_id1_17mm_cooling_heat<max_heat*0.1f&&g_referee->power_heat_data.shooter_id2_17mm_cooling_heat<max_heat*0.1f;
  }else{
    can_shoot = !(g_referee->power_heat_data.shooter_id1_17mm_cooling_heat>(max_heat-i*10.0f-10)||g_referee->power_heat_data.shooter_id2_17mm_cooling_heat>(max_heat-i*10.0f-10));
  }
#else
  can_shoot = !(g_referee->power_heat_data.shooter_id1_17mm_cooling_heat>(max_heat-i*10.0f-10)||g_referee->power_heat_data.shooter_id2_17mm_cooling_heat>(max_heat-i*10.0f-10));
#endif

  if(can_shoot){
    g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->target_angle + one_shoot_angle * i);
  }
}

///@brief 拨盘抖动函数，解决卡弹
void Vibrating() {
    float old = g_shooter.motor_feeder->real_angle;
    for (int i = 0; i < 5; ++i) {
        g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->real_angle - 2*one_shoot_angle);
        osDelay(100);
        g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->real_angle + 2*one_shoot_angle);
        osDelay(100);
    }
    g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, old);
}
void shoot_one(){
    g_shooter.motor_feeder->target_angle += one_shoot_angle;
}

void feeder_back(){
    g_shooter.motor_feeder->target_angle -= one_shoot_angle;
}

void fric_on(){
    g_shooter.motor_left->set_speed(g_shooter.motor_left,fric_speed);
    g_shooter.motor_right->set_speed(g_shooter.motor_right,fric_speed);
}

void fric_off(){
    g_shooter.motor_left->target_speed = 0;
    g_shooter.motor_right->target_speed = 0;
    g_shooter.motor_feeder->target_angle= g_shooter.motor_feeder->real_angle;
}
void Shooter_Loop(){
  while (1){
    if(fabsf(g_shooter.motor_feeder->target_angle - g_shooter.motor_feeder->real_angle) > one_shoot_angle*5){
      feeder_blocked_count++;
    }
    if(feeder_blocked_count>500){
      feeder_blocked=1;
    }
    osDelay(1);
  }
}
void Shooter_Init() {
    g_shooter.motor_left = pvSharePtr("/motor/shooter_left", sizeof(INTF_Motor_HandleTypeDef));
    g_shooter.motor_right = pvSharePtr("/motor/shooter_right", sizeof(INTF_Motor_HandleTypeDef));
    g_shooter.motor_feeder = pvSharePtr("/motor/shooter_feeder", sizeof(INTF_Motor_HandleTypeDef));
    xTaskCreate(Shooter_Loop,"shooter",512,NULL,0,NULL);
}

