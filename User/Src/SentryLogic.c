//
// Created by Konodoki on 2025/2/26.
//
#include "SentryLogic.h"
#include "cmsis_os.h"
#include "interface.h"
#include "intf_dr16.h"
#include "tinybus.h"
#include "user.h"
#include "gimbal.h"
static INTF_Gimbal_HandleTypeDef *gimbal;
extern RC_ctrl_t *rc_ctrl;
#define SWITCH_LEFT (rc_ctrl[0].rc.switch_left)
#define SWITCH_RIGHT (rc_ctrl[0].rc.switch_right)
enum Sentry_Control_Mode sentryControlMode=Sentry_Manual_Control;
enum Sentry_Shoot_Mode sentryShootMode=Sentry_Shoot_Standby;
enum Sentry_Gimbal_Mode sentryGimbalMode=Sentry_Gimbal_Normal;
static uint8_t last_switch_left;
static uint8_t last_switch_right;
void sentry_ctrl_loop() {
  //等待遥控器连接
  while (HAL_GetTick()-rc_ctrl[1].update_time>=100||HAL_GetTick()<100){
    osDelay(1);
  }
  last_switch_left=SWITCH_LEFT;
  last_switch_right=SWITCH_RIGHT;
  while (1) {
    switch (SWITCH_LEFT) {
    case 1:
      if(last_switch_right!=SWITCH_RIGHT){
        switch (SWITCH_RIGHT) {
        case 2:
          sentryControlMode=Sentry_Manual_Control;
          break;
        case 3:
          sentryControlMode=Sentry_Semi_Automatic_Control;
          break;
        case 1:
          sentryControlMode=Sentry_Automatic_Control;
          break;
        }
      }
      break;
    case 3:
      if(last_switch_right!=SWITCH_RIGHT){
        switch (SWITCH_RIGHT) {
        case 2:
          sentryShootMode=Sentry_Shoot_Standby;
          break;
        case 3:
          sentryShootMode=Sentry_Shoot_Ready;
          break;
        case 1:
          sentryShootMode=Sentry_Shoot_Shot;
          break;
        }
      }
      break;
    case 2:
      if(last_switch_right!=SWITCH_RIGHT){
        switch (SWITCH_RIGHT) {
        case 2:
          sentryGimbalMode=Sentry_Gimbal_Normal;
          Gimbal_SetMode(gimbal,GIMBAL_MODE_NORMAL);
          break;
        case 3:
          sentryGimbalMode=Sentry_Gimbal_Follow;
          Gimbal_SetMode(gimbal,GIMBAL_MODE_FOLLOW_GYRO);
          break;
        case 1:
          sentryGimbalMode=Sentry_Outpost;
          Gimbal_SetMode(gimbal,GIMBAL_MODE_FOLLOW_GYRO);
          break;
        }
      }
      break;
    }
    last_switch_left=rc_ctrl[0].rc.switch_left;
    last_switch_right=rc_ctrl[0].rc.switch_right;
    osDelay(10);
  }
}
void sentry_logic_init() {
  gimbal = pvSharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef ));
  xTaskCreate(sentry_ctrl_loop, "sentry_ctrl_loop", 64, NULL, 15, NULL);
}
USER_EXPORT(sentry_logic, sentry_logic_init);