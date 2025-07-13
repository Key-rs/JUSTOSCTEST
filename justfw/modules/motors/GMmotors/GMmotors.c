#include "GMmotors.h"
GM_BufferTypeDef GM_Buffer[GM_BUFFER_NUM] = {0};

void GM_ControlSend() {
    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        INTF_CAN_MessageTypeDef msg = {
            .id_type = BSP_CAN_ID_STD,
        };

        if (GM_Buffer[i].buffer_0x200_not_null) {
            msg.can_id = 0x200;
            memcpy(msg.data, GM_Buffer[i].buffer_0x200, 8);
            vBusPublish(GM_Buffer[i].can_tx_topic, &msg);
        }
        if (GM_Buffer[i].buffer_0x1FF_not_null) {
            msg.can_id = 0x1FF;
            memcpy(msg.data, GM_Buffer[i].buffer_0x1FF, 8);
            vBusPublish(GM_Buffer[i].can_tx_topic, &msg);
        }
        if (GM_Buffer[i].buffer_0x2FF_not_null) {
            msg.can_id = 0x2FF;
            memcpy(msg.data, GM_Buffer[i].buffer_0x2FF, 8);
            vBusPublish(GM_Buffer[i].can_tx_topic, &msg);
        }
    }
}

void GM_MainLoop() {
    vTaskDelay(pdMS_TO_TICKS(2000));  // 等待电机启动
    while (1) {
        extern void C610_PIDCalc();
        extern void C620_PIDCalc();
        extern void GM6020_PIDCalc();

        C620_PIDCalc();
        C610_PIDCalc();
        GM6020_PIDCalc();
        GM_ControlSend();
        vTaskDelay(2);
    }
}

void GM_Init() {
    GM_Buffer[0].can_tx_topic = xBusTopicRegister("/CAN1/TX");
    GM_Buffer[1].can_tx_topic = xBusTopicRegister("/CAN2/TX");

    // 注册接收话题，仅用于比较话题地址，不发布消息
    GM_Buffer[0].can_rx_topic = xBusTopicRegister("/CAN1/RX");
    GM_Buffer[1].can_rx_topic = xBusTopicRegister("/CAN2/RX");
    // extern void C620_Init();
    // C620_Init();

    xTaskCreate(GM_MainLoop, "GM_Motor", 512, NULL, 200, NULL);
}