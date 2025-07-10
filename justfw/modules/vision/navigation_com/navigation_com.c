//
// Created by Konodoki on 2024-03-21.
//

#include "navigation_com.h"

#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "queue.h"
osThreadId Navigation_Com_MainLoopTaskHandle;

BusTopicHandle_t *g_navigation_tx;
BusTopicHandle_t *g_navigation_rx;

QueueHandle_t navigation_msg_queue;
// 上位机->下位机
union Navigation_M2S_Union {
  Navigation_M2S_PacketTypeDef navigation_com_m2s;
  uint8_t raw[sizeof(Navigation_M2S_PacketTypeDef)];
};

// 下位机->上位机
union Navigation_S2M_Union {
  Navigation_S2M_PacketTypeDef navigation_com_s2m;
  uint8_t raw[sizeof(Navigation_S2M_PacketTypeDef)];
};

void Navigation_Com_Solve(void *message, BusTopicHandle_t *topic) {
  INTF_Serial_MessageTypeDef *msg = (INTF_Serial_MessageTypeDef *)message;
  if (msg->len != sizeof(Navigation_M2S_PacketTypeDef)) {
    return; // 长度不对丢包
  }
  union Navigation_M2S_Union navigation_com_m2s_union;
  memcpy(navigation_com_m2s_union.raw, msg->data,
         sizeof(Navigation_M2S_PacketTypeDef));
  if (navigation_com_m2s_union.navigation_com_m2s.header == 0xA6) {
    if (verify_CRC16_check_sum(
            navigation_com_m2s_union.raw,
            sizeof(Navigation_M2S_PacketTypeDef))) { // 校验不对丢包{
      xQueueSendFromISR(navigation_msg_queue,
                        &navigation_com_m2s_union.navigation_com_m2s, 0);
    }
  }
}

void NavigationData_Transmit(Navigation_S2M_PacketTypeDef *nav_msg) {
  union Navigation_S2M_Union navigation_com_s2m_union = {0};
  navigation_com_s2m_union.navigation_com_s2m = *nav_msg;
  navigation_com_s2m_union.navigation_com_s2m.header = 0x6A;

  append_CRC16_check_sum(navigation_com_s2m_union.raw,
                         sizeof(Navigation_S2M_PacketTypeDef));

  INTF_Serial_MessageTypeDef msg;
  msg.data = navigation_com_s2m_union.raw;
  msg.len = sizeof(Navigation_S2M_PacketTypeDef);
  vBusPublish(g_navigation_tx, &msg);
}
void Navigation_slove_loop() {
  while (1) {
    Navigation_M2S_PacketTypeDef msg;
    if (xQueueReceive(navigation_msg_queue, &msg, 0) == pdPASS) {
      vBusPublish(g_navigation_rx, &msg);
    }
//    static uint32_t slove_cnt=0;
//    slove_cnt++;
//    if(slove_cnt>2000){
//      Navigation_S2M_PacketTypeDef nav;
//      nav.x=1;
//      nav.y=2;
//      nav.yaw=3;
//      NavigationData_Transmit(&nav);
//      slove_cnt=0;
//    }
    osDelay(1);
  }
}
void Navigation_Com_Init(void) {
  navigation_msg_queue = xQueueCreate(5, sizeof(Navigation_M2S_PacketTypeDef));
  xBusSubscribeFromName("USB_RX", Navigation_Com_Solve);
  g_navigation_tx = xBusTopicRegister("USB_TX");
  g_navigation_rx = xBusTopicRegister("Navigation_RX");
  xTaskCreate(Navigation_slove_loop, "NavigaionLoop", 256, NULL, 10, NULL);
}