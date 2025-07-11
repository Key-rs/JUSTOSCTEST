//
// Created by Konodoki on 2025/3/11.
//
#include "SentryLogic.h"
#include "auto_aim.h"
#include "cmsis_os.h"
#include "interface.h"
#include "referee.h"
#include "ui.h"
#include "unknown_super_cap.h"
#include "user.h"
extern referee_data_t *g_referee;
extern uint32_t last_receive_msg_tick;
uint8_t referee_connected=0;
uint32_t last_running_update_tick=0;
static float super_cap_max_power=12000;
extern SuperCap_State supercap_state;
static uint8_t super_cap_connected=0;
extern SuperCap_State supercap_state;
void ui_task(){
  while (1){
    if(g_referee->last_update_tick>1000&&xTaskGetTickCount()-g_referee->last_update_tick<1000){
      if(referee_connected==0){
        ui_self_id=g_referee->game_robot_state.robot_id;
        _ui_init_g_sentry_0();
        osDelay(100);
        _ui_init_g_sentry_1();
        osDelay(100);
        _ui_init_g_sentry_2();
        osDelay(100);
        _ui_init_g_sentry_3();
        osDelay(100);
        _ui_init_g_sentry_4();
        osDelay(100);
        _ui_init_g_sentry_5();
        osDelay(100);
      }
      super_cap_max_power=g_referee->game_robot_state.chassis_power_limit*100;
      referee_connected=1;
    }
    else{
      super_cap_max_power=12000;
      referee_connected=0;
    }
    if(supercap_state.last_update_tick>1000&&xTaskGetTickCount()-supercap_state.last_update_tick<1000){
      if(super_cap_connected==0){
        Unknown_SuperCap_SetPower(super_cap_max_power);
      }
      super_cap_connected=1;
    }else{
      super_cap_connected=0;
    }
    switch (sentryControlMode) {
    case Sentry_Manual_Control:
      ui_g_sentry_control_mode->str_length=sprintf(ui_g_sentry_control_mode->string,"Manual");
      break;
    case Sentry_Semi_Automatic_Control:
      ui_g_sentry_control_mode->str_length=sprintf(ui_g_sentry_control_mode->string,"Semi_Automatic");
      break;
    case Sentry_Automatic_Control:
      ui_g_sentry_control_mode->str_length=sprintf(ui_g_sentry_control_mode->string,"Automatic");
      break;
    }
    switch (sentryShootMode) {
    case Sentry_Shoot_Standby:
      ui_g_sentry_shoot_state->str_length=sprintf(ui_g_sentry_shoot_state->string,"Standby");
      break;
    case Sentry_Shoot_Ready:
      ui_g_sentry_shoot_state->str_length=sprintf(ui_g_sentry_shoot_state->string,"Ready");
      break;
    case Sentry_Shoot_Shot:
      ui_g_sentry_shoot_state->str_length=sprintf(ui_g_sentry_shoot_state->string,"Shot");
      break;
    }
    switch (sentryGimbalMode) {
    case Sentry_Gimbal_Normal:
      ui_g_sentry_gimbal_mode->str_length=sprintf(ui_g_sentry_gimbal_mode->string,"Normal");
      break;
    case Sentry_Gimbal_Follow:
      ui_g_sentry_gimbal_mode->str_length=sprintf(ui_g_sentry_gimbal_mode->string,"Follow");
      break;
    case Sentry_Outpost:
      ui_g_sentry_gimbal_mode->str_length=sprintf(ui_g_sentry_gimbal_mode->string,"Outpost");
      break;
    }
    ui_g_sentry_minipc_state->color=xTaskGetTickCount()-last_receive_msg_tick<1000?2:5;
    if(xTaskGetTickCount()-last_running_update_tick>1000){
      ui_g_sentry_running->color=ui_g_sentry_running->color==2?5:2;
      last_running_update_tick=xTaskGetTickCount();
    }
    ui_g_sentry_supercap_v->number=supercap_state.cap_voltage*1000.0f;
    _ui_update_g_sentry_0();
    osDelay(10);
    _ui_update_g_sentry_1();
    osDelay(10);
    _ui_update_g_sentry_2();
    osDelay(10);
    _ui_update_g_sentry_3();
    osDelay(10);
    _ui_update_g_sentry_4();
    osDelay(10);
    _ui_update_g_sentry_5();
    osDelay(10);
  }
}
void ui_init(){
  ui_port_init();
  xTaskCreate(ui_task,"ui_task",64,NULL,10,NULL);
}
USER_EXPORT(ui_init,ui_init);