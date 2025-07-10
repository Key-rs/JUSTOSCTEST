//
// Created by Konodoki on 2025/2/16.
//
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "cmsis_os.h"
#include "elog.h"
#include "tinybus.h"
#include "user.h"
#include <stdio.h>
int16_t out=0;
uint32_t start_frq=0;
uint32_t stop_frq=500;
uint32_t step_frq=10;
uint32_t now_frq=0;
void matlab_loop(){
  osDelay(5000);
  while (1){
    if(now_frq<stop_frq){
      now_frq+=step_frq;
      out=1;
      osDelay(now_frq);
      out=-1;
      osDelay(now_frq);
    }else{
      out=0;
      osDelay(1);
    }
  }
}
BusTopicHandle_t *tx_topic;
INTF_Motor_HandleTypeDef *motor;
void watch(){
  while (1){
    void GM6020_SetVoltage(uint8_t id, int16_t voltage, BusTopicHandle_t *can_tx_topic);
//    GM6020_SetVoltage(7,out, tx_topic);
//    motor->target_speed=out;
    printf(":%d,%.4f,%lu\r\n",out,motor->real_angle,now_frq);
    osDelay(1);
  }
}
void matlab_init(){
  motor = pvSharePtr("/motor/gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
  tx_topic = xBusTopicRegister("/CAN1/TX");
  // osThreadDef(MainLoopTask, matlab_loop, osPriorityNormal, 0, 128);
  // osThreadCreate(osThread(MainLoopTask), NULL);
  xTaskCreate(matlab_loop,"matlab_loop",128,NULL,15,NULL);
  xTaskCreate(watch,"watch",128,NULL,15,NULL);
}
//USER_EXPORT(matlab,matlab_init);