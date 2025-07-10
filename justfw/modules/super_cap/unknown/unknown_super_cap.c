//
// Created by Ukua on 2024/3/29.
//

#include "unknown_super_cap.h"

#include "cmsis_os.h"

osThreadId Unknown_SuperCapTaskHandle;

SuperCap_State supercap_state;

BusTopicHandle_t *sc_can;

void Unknown_CAN_CallBack(void *message, BusTopicHandle_t *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *) message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id!=0x211) {
        return;
    }
    uint16_t *pPowerdata=(uint16_t*)msg->data;//CAN收到的8个字节的数组
    supercap_state.input_voltage=(float)pPowerdata[0]/100.f;//输入电压
    supercap_state.cap_voltage=(float)pPowerdata[1]/100.f;//电容电压
    supercap_state.input_current=(float)pPowerdata[2]/100.f;//输入电流
    supercap_state.target_power=(float)pPowerdata[3]/100.f;//设定功率
    supercap_state.last_update_tick=xTaskGetTickCount();
}
void Unknown_SuperCap_SetPower(uint16_t power){
    if(power<3000)power=3000;
    if(power<13000)power=13000;
    INTF_CAN_MessageTypeDef msg={
            .rtr_type=BSP_CAN_RTR_DATA,
            .id_type=BSP_CAN_ID_STD,
            .can_id=0x210,
    };
    msg.data[0]=power>>8;
    msg.data[1]=power;
    vBusPublish(sc_can,&msg);
}

void Unknown_SuperCap_MainLoop(){
    while (1){
        Unknown_SuperCap_SetPower(8000);
        osDelay(100);
    }
}

void Unknown_SuperCap_Init(){
    xBusSubscribeFromName("/CAN1/RX", Unknown_CAN_CallBack);
    sc_can = xBusTopicRegister("/CAN1/TX");
//    osThreadDef(Unknown_SuperCap_MainLoopTask, Unknown_SuperCap_MainLoop, osPriorityLow, 0, 512);
//    Unknown_SuperCapTaskHandle = osThreadCreate(osThread(Unknown_SuperCap_MainLoopTask), NULL);
}
