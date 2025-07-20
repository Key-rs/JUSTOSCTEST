//
// Created by Ukua on 2024/3/25.
//

#include "pm01.h"

#include "cmsis_os.h"

BusTopicHandle_t g_pm01_can_tx;
BusSubscriberHandle_t g_pm01_can_rx;

PM01_StatusTypeDef g_pm01_status = {0};
PM01_InputTypeDef g_pm01_input = {0};
PM01_OutputTypeDef g_pm01_output = {0};
PM01_TemperatureTypeDef g_pm01_temperature = {0};

osThreadId PM01_MainLoopTaskHandle;


/**
 * @brief 设置PM01工作模式
 * @param mode 模式
 * @param will_save 是否保存到EEPROM
 */
void PM01_SetMode(enum PM01_Mode mode, uint8_t will_save) {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = 0x600;
    msg.data[0] = mode>>8;
    msg.data[1] = mode;
    msg.data[2] = will_save>>8;
    msg.data[3] = will_save;
    vBusPublish(g_pm01_can_tx, &msg);
}


/**
 * @brief 设置PM01输入功率
 * @param power 功率 0~15000 单位0.01w 默认8000
 * @param will_save 是否保存到EEPROM
 */
void PM01_SetPowerInput(uint16_t power, uint8_t will_save) {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = 0x601;
    msg.data[0] = power>>8;
    msg.data[1] = power;
    msg.data[2] = will_save>>8;
    msg.data[3] = will_save;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 设置PM01输出电压
 * @param voltage 电压 550~3000 单位0.01V 默认2400
 * @param will_save 是否保存到EEPROM
 */
void PM01_SetVoltageOutput(uint16_t voltage, uint8_t will_save) {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = 0x602;
    msg.data[0] = voltage>>8;
    msg.data[1] = voltage;
    msg.data[2] = will_save>>8;
    msg.data[3] = will_save;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 设置PM01输出电流
 * @param current 电流 0~1000 单位0.01A 默认800
 * @param will_save 是否保存到EEPROM
 */
void PM01_SetCurrentOutput(uint16_t current, uint8_t will_save) {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = 0x603;
    msg.data[0] = current>>8;
    msg.data[1] = current;
    msg.data[2] = will_save>>8;
    msg.data[3] = will_save;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 请求PM01状态
 */
void PM01_AskStatus() {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.rtr_type = BSP_CAN_RTR_REMOTE;
    msg.can_id = 0x610;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 请求PM01输入状态
 */
void PM01_AskInputStatus() {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.rtr_type = BSP_CAN_RTR_REMOTE;
    msg.can_id = 0x611;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 请求PM01输出状态
 */
void PM01_AskOutputStatus() {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.rtr_type = BSP_CAN_RTR_REMOTE;
    msg.can_id = 0x612;
    vBusPublish(g_pm01_can_tx, &msg);
}

/**
 * @brief 请求PM01温度信息
 */
void PM01_AskTemperature() {
    INTF_CAN_MessageTypeDef msg = {0};
    msg.id_type = BSP_CAN_ID_STD;
    msg.rtr_type = BSP_CAN_RTR_REMOTE;
    msg.can_id = 0x612;
    vBusPublish(g_pm01_can_tx, &msg);
}

void PM01_CAN_RX_CallBack(void *message, BusTopicHandle_t topic){
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *) message;
    if(msg->id_type != BSP_CAN_ID_STD){
        return;
    }
    switch (msg->can_id) {
        //PM01状态
        case 0x610:
            memcpy(&g_pm01_status, msg->data, sizeof(PM01_StatusTypeDef));
            break;
        //PM01输入状态
        case 0x611:
            memcpy(&g_pm01_input, msg->data, sizeof(PM01_InputTypeDef));
            break;
        //PM01输出状态
        case 0x612:
            memcpy(&g_pm01_output, msg->data, sizeof(PM01_OutputTypeDef));
            break;
        //PM01温度
        case 0x613:
            memcpy(&g_pm01_temperature, msg->data, sizeof(PM01_TemperatureTypeDef));
            break;

    }
}

void PM01_MainLoop() {
    while (1) {
        PM01_AskStatus();
        osDelay(10);
        PM01_AskInputStatus();
        osDelay(10);
        PM01_AskOutputStatus();
        osDelay(10);
        PM01_AskTemperature();
        osDelay(10);
    }
}

void PM01_Init() {
    g_pm01_can_tx = xBusTopicRegister("/CAN1/TX");
    g_pm01_can_rx = xBusSubscribeFromName("/CAN1/RX", PM01_CAN_RX_CallBack);

    xTaskCreate(PM01_MainLoop, "PM01_MainLoop", 128, NULL, 5, NULL);
}