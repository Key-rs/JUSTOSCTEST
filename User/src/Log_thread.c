//
// Created by Konodoki on 2025/3/9.
//
#include "auto_aim.h"
#include "cmsis_os.h"
#include "elog.h"
#include "referee.h"
#include "tinybus.h"
#include "user.h"
#include <stdio.h>
BusTopicHandle_t *uart_tx_topic;
static AutoAimStatus_t *auto_aim_status;
extern referee_data_t *g_referee;
void log_loop(){
  osDelay(5000);
  while (1){
    char buf[20]={0};
    sprintf(buf,":%u\r\n",g_referee->robot_command.mouse_x);
    INTF_Serial_MessageTypeDef msg;
    msg.len = strlen(buf);
    msg.data = (uint8_t *)buf;
    vBusPublish(uart_tx_topic, &msg);
    osDelay(1);
  }
}
void log_rx(void *message,BusTopicHandle_t *topic){
  INTF_Serial_MessageTypeDef *msg = (INTF_Serial_MessageTypeDef *)message;
  if(msg->data[0]!=0x78||msg->len!=2)
    return;
  g_referee->game_state.game_progress=msg->data[1];
}
void log_init(){
  uart_tx_topic = xBusTopicRegister("/UART/TEST_TX");
  xBusSubscribeFromName("/UART/TEST_RX",log_rx);
  auto_aim_status = pvSharePtr("AutoAimStatus", sizeof(AutoAimStatus_t));
//    xTaskCreate(log_loop,"log_loop",256,NULL,15,NULL);
}
//USER_EXPORT(log_init,log_init);