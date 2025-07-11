//
// Created by Konodoki on 2025/2/26.
//
#include "cmsis_os2.h"
#include "SentryLogic.h"
#include "interface.h"
#include "referee.h"
#include "shooter.h"
#include "user.h"
extern INTF_Shooter_HandleTypeDef g_shooter;
extern RC_ctrl_t *rc_ctrl;
static enum Sentry_Shoot_Mode last_shoot_mode;
static uint8_t last_keyboard_r;
static uint8_t last_keyboard_c;
static uint8_t last_mouse_l;
static uint8_t fric_state=0;
extern referee_data_t *g_referee;
void Sentry_Shooter_Loop(){
  while (1){
    if((sentryShootMode==Sentry_Shoot_Ready&&sentryShootMode!=last_shoot_mode)||(rc_ctrl[0].keyboard.r&&rc_ctrl[0].keyboard.r!=last_keyboard_r)){
      fric_on();
      fric_state=1;
    }
    if((sentryShootMode==Sentry_Shoot_Standby&&sentryShootMode!=last_shoot_mode)||rc_ctrl[0].keyboard.c&&rc_ctrl[0].keyboard.c!=last_keyboard_c){
      fric_off();
      Vibrating();
      fric_state=0;
    }
    static uint32_t last_mannal_shot_tick=0;
    if(sentryShootMode==Sentry_Shoot_Shot||(rc_ctrl[0].mouse.press_l&&last_mouse_l!=rc_ctrl[0].mouse.press_l)){
      if(fric_state){
        if(xTaskGetTickCount() - last_mannal_shot_tick<500){
          while (rc_ctrl->rc.switch_right==3){
            Shoot(1);
            osDelay(10);
          }
        }
        Shoot(1);
        last_mannal_shot_tick=xTaskGetTickCount();
        sentryShootMode=Sentry_Shoot_Ready;
      }
    }
    last_keyboard_r=rc_ctrl[0].keyboard.r;
    last_keyboard_c=rc_ctrl[0].keyboard.c;
    last_mouse_l=rc_ctrl[0].mouse.press_l;
    last_shoot_mode=sentryShootMode;
    osDelay(1);
  }
}
void Sentry_shoot_init(){
  xTaskCreate(Sentry_Shooter_Loop,"Sentry_Shooter",64,NULL,15,NULL);
}
USER_EXPORT(Sentry_shoot,Sentry_shoot_init);
