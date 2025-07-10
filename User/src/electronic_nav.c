//
// Created by Konodoki on 2025/3/21.
//
#include "SentryLogic.h"
#include "cmsis_os.h"
#include "interface.h"
#include "navigation_com.h"
#include "referee.h"
#include "tinybus.h"
#include "user.h"
extern RC_ctrl_t *rc_ctrl;
extern referee_data_t *g_referee;
uint8_t last_stage=0;
uint8_t go_over=0;
void electronic_loop(){
  static uint32_t wait_time=0;
  while(1){
    if(wait_time>120000||(sentryControlMode==Sentry_Automatic_Control&&g_referee->game_state.game_progress==4&&last_stage!=4)){
      break;
    }
    last_stage=g_referee->game_state.game_progress;
    wait_time++;
    osDelay(1);
//    if(rc_ctrl[0].rc.switch_left==2&&sentryControlMode==Sentry_Automatic_Control){
//      break;
//    }
//    osDelay(1);
  }
  static uint32_t walk_time=6000;
  while (walk_time--){
    extern void nav_set(void *msg,BusTopicHandle_t *topic);
    Navigation_M2S_PacketTypeDef nav_pp;
    nav_pp.vx=1.2f;
    nav_pp.vy=0;
    nav_pp.w_z=3;
    nav_set(&nav_pp,NULL);
    osDelay(1);
  }
  go_over=1;
  static uint8_t go_return=0; //0:go 1:return
  while (1){
    if(g_referee->game_robot_state.current_HP/g_referee->game_robot_state.maximum_HP>0.8f){
      go_return=0;
    }
    if(g_referee->game_robot_state.current_HP/g_referee->game_robot_state.maximum_HP<0.4f){
      go_return=1;
    }
    if(g_referee->game_robot_state.current_HP==0||g_referee->game_robot_state.maximum_HP==0){
      go_return=0;
    }
    Navigation_S2M_PacketTypeDef pack={0};
    if(go_return){//如果回家1
      pack.x=0;
      pack.y=0;
      pack.yaw=0;
    }else{//如果占点
      pack.x=6.0f;
      pack.y=-4.0f;
      pack.yaw=0;
    }
    NavigationData_Transmit(&pack);
    osDelay(1000);
  }
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
void electronic_init(){
  xTaskCreate(electronic_loop,"electronic_loop",64,NULL,15,NULL);
}
USER_EXPORT(electronic_init,electronic_init);