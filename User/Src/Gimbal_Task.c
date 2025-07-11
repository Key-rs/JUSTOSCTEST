//
// Created by Konodoki on 2025/1/11.
//
#include "SentryConfig.h"
#include "SentryLogic.h"
#include "auto_aim.h"
#include "cmsis_os.h"
#include "interface.h"
#include "referee.h"
#include "shooter.h"
#include "tinybus.h"
#include "ui.h"
#include "user.h"
#include "user_lib.h"
#include <stdio.h>
static INTF_Motor_HandleTypeDef *motor;
static INTF_Gimbal_HandleTypeDef *gimbal;
extern RC_ctrl_t *rc_ctrl;
static AutoAimStatus_t *auto_aim_status;
uint32_t last_shoot_tick;
int8_t scan_dir=1;//扫描的方向
uint32_t l_mouse_press_tick=0;
extern referee_data_t *g_referee;
extern uint8_t referee_connected;
void pin_loop(){
  osDelay(3000);
  while (1){
    printf("pin_loop\n");
    //手动控制模式或者没有自瞄目标时用遥控器或键鼠控制
    if(sentryControlMode==Sentry_Manual_Control||xTaskGetTickCount()-auto_aim_status->solve_tick>Sentry_AutoAim_MaxLostTime){
      if(rc_ctrl[0].mouse.x==0&&rc_ctrl[0].mouse.y==0){
        gimbal->set_pitch(gimbal,gimbal->target_pitch + rc_ctrl[0].rc.rocker_r1*0.01f/660.0f);
        gimbal->set_yaw(gimbal,gimbal->target_yaw - rc_ctrl[0].rc.rocker_r_*0.01f/660.0f);
      }else{
        gimbal->set_pitch(gimbal,gimbal->target_pitch - rc_ctrl[0].mouse.y*0.00005f);
        gimbal->set_yaw(gimbal,gimbal->target_yaw - rc_ctrl[0].mouse.x*0.00005f);
      }
    }
    //自瞄
    if((sentryControlMode==Sentry_Manual_Control&&rc_ctrl[0].mouse.press_r)||sentryControlMode==Sentry_Semi_Automatic_Control||sentryControlMode==Sentry_Automatic_Control){
      if(auto_aim_status->tracking==1){
        gimbal->set_pitch(gimbal,auto_aim_status->out_pitch);
        gimbal->set_yaw(gimbal, LowPassFilter(auto_aim_status->out_yaw,gimbal->target_yaw,0.01f));
      }
    }
    ui_g_sentry_track->str_length=sprintf(ui_g_sentry_track->string,auto_aim_status->tracking==1?"Tracking":"Lost");
    //计算是否允许自动火控射击
    static uint8_t can_shoot=0;
    if(can_shoot==0){
      can_shoot = g_referee->power_heat_data.shooter_id1_17mm_cooling_heat<300*0.4f&&g_referee->power_heat_data.shooter_id2_17mm_cooling_heat<300*0.4f;
    }else{
      can_shoot = g_referee->power_heat_data.shooter_id1_17mm_cooling_heat>300*0.9f||g_referee->power_heat_data.shooter_id2_17mm_cooling_heat>300*0.9f;
    }
    //自动火控(长按鼠标左键或者在全自动模式下)
    if(sentryShootMode==Sentry_Shoot_Ready&&((sentryControlMode==Sentry_Automatic_Control&&(!referee_connected||can_shoot))||l_mouse_press_tick>1000)&&xTaskGetTickCount()-auto_aim_status->solve_tick<Sentry_AutoAim_MaxLostTime){
      ui_g_sentry_auto_fire->str_length=sprintf(ui_g_sentry_auto_fire->string,"AutoFire");
      float shoot_cycle=150.0f;
      if(auto_aim_status->yaw_speed>0.5f){
        shoot_cycle=2*PI/auto_aim_status->yaw_speed*1000.0f/6.0f;
      }
      if(fabsf(auto_aim_status->out_yaw-gimbal->real_yaw)<0.025f&& fabsf(auto_aim_status->out_pitch-gimbal->real_pitch)<0.025f&&xTaskGetTickCount()-last_shoot_tick>shoot_cycle){
        Shoot(3);
        last_shoot_tick=xTaskGetTickCount();
      }
    }else{
      ui_g_sentry_auto_fire->str_length=sprintf(ui_g_sentry_auto_fire->string,"ManualFire");
    }
    if(rc_ctrl[0].mouse.press_l){
      l_mouse_press_tick++;
    }else{
      l_mouse_press_tick=0;
    }
    extern uint8_t go_over;

    //全场扫描
    static uint32_t scan_stage_cnt=0;
    static uint8_t scan_stage=0;
    static uint8_t has_locked=0;
    if(scan_stage==0&&go_over){
      if(sentryControlMode==Sentry_Automatic_Control&&xTaskGetTickCount()-auto_aim_status->solve_tick>Sentry_AutoAim_MaxLostTime){
        gimbal->set_pitch(gimbal, sinf(xTaskGetTickCount()/100.0f)*0.20f-0.20f);
        gimbal->set_yaw(gimbal, gimbal->target_yaw+0.001f*scan_dir);
        if(has_locked==1){
          has_locked=0;
          scan_stage=1;
          scan_dir*=-1;
        }
      }else{
        has_locked=1;
      }
    }
    if(scan_stage==1&&go_over){
      gimbal->set_pitch(gimbal, -0.15f);
      gimbal->set_yaw(gimbal, gimbal->target_yaw+0.001f*scan_dir);
      if(scan_stage_cnt>500){
        scan_dir*=-1;
        scan_stage=0;
        scan_stage_cnt=0;
      }
      scan_stage_cnt++;
    }
    osDelay(1);
  }
}

void pin_init(){
  gimbal = pvSharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef ));
  motor = pvSharePtr("/motor/shooter_feeder", sizeof(INTF_Motor_HandleTypeDef));
  auto_aim_status = pvSharePtr("AutoAimStatus", sizeof(AutoAimStatus_t));
  xTaskCreate(pin_loop,"pin",512,NULL,10,NULL);
}
USER_EXPORT(pid_test,pin_init);