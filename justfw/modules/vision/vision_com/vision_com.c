//
// Created by Ukua on 2024/3/12.
//

#include "vision_com.h"
#include <stdio.h>

#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "queue.h"

osThreadId Vision_Com_MainLoopTaskHandle;

BusTopicHandle_t *g_autoaim_tx;
BusTopicHandle_t *g_autoaim_rx;

INTF_Chassis_HandleTypeDef *g_vision_chassis;
INTF_Gimbal_HandleTypeDef *g_vision_gimbal;

QueueHandle_t autoAim_msg_queue;
//上位机->下位机
union AutoAim_M2S_Union {
    AutoAim_M2S_PacketTypeDef auto_aim_m2s;
    uint8_t raw[sizeof(AutoAim_M2S_PacketTypeDef)];
};

//下位机->上位机
union AutoAim_S2M_Union {
    AutoAim_S2M_PacketTypeDef auto_aim_s2m;
    uint8_t raw[sizeof(AutoAim_S2M_PacketTypeDef)];
};

void Vision_Com_Solve(void *message, BusTopicHandle_t *topic) {
    INTF_Serial_MessageTypeDef *msg = (INTF_Serial_MessageTypeDef *) message;
    if (msg->len != sizeof(AutoAim_M2S_PacketTypeDef)) {
        return;//长度不对丢包
    }
    union AutoAim_M2S_Union auto_aim_m2s_union;
    memcpy(auto_aim_m2s_union.raw, msg->data, sizeof(AutoAim_M2S_PacketTypeDef));
    if (auto_aim_m2s_union.auto_aim_m2s.header == 0xA5) {
        if (verify_CRC16_check_sum(auto_aim_m2s_union.raw, sizeof(AutoAim_M2S_PacketTypeDef))) { //校验不对丢包{
            xQueueSendFromISR(autoAim_msg_queue,&auto_aim_m2s_union.auto_aim_m2s,0);
        }
    }
}
void AutoAim_Transmit(AutoAim_S2M_PacketTypeDef *aim_msg) {
    union AutoAim_S2M_Union auto_aim_s2m_union;
    auto_aim_s2m_union.auto_aim_s2m=*aim_msg;
    auto_aim_s2m_union.auto_aim_s2m.header=0x5A;

    append_CRC16_check_sum(auto_aim_s2m_union.raw, sizeof(AutoAim_S2M_PacketTypeDef));

    INTF_Serial_MessageTypeDef msg;
    msg.data = auto_aim_s2m_union.raw;
    msg.len = sizeof(AutoAim_S2M_PacketTypeDef);
    vBusPublish(g_autoaim_tx, &msg);
}
void Vision_Com_slove_loop(){
  while (1){
    AutoAim_M2S_PacketTypeDef msg;
    if(xQueueReceive(autoAim_msg_queue,&msg,0)==pdPASS){
      vBusPublish(g_autoaim_rx,&msg);
    }
    osDelay(1);
  }
}
void Vision_Com_Init(void) {
  autoAim_msg_queue= xQueueCreate(5,sizeof(AutoAim_M2S_PacketTypeDef));
    xBusSubscribeFromName("USB_RX", Vision_Com_Solve);
    g_autoaim_tx = xBusTopicRegister("USB_TX");
    g_autoaim_rx = xBusTopicRegister("AutoAim_RX");
    xTaskCreate(Vision_Com_slove_loop, "VisionComLoop", 256, NULL, 10, NULL);
}