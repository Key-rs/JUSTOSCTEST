//
// Created by Konodoki on 2024/8/28.
//

#include "SuperCapX9.h"
#include "can.h"
#include "string.h"
#include "tinybus.h"
SuperCap_Data_Receive_typedef g_cap_state;
BusTopicHandle_t supercap_can_tx_topic;
void CAN_T(uint32_t id,uint8_t *data,uint32_t len){
  INTF_CAN_MessageTypeDef msg;
  msg.id_type=CAN_ID_STD;
  msg.rtr_type=CAN_RTR_DATA;
  msg.can_id=id;
  memcpy(msg.data,data,8);
  vBusPublish(supercap_can_tx_topic,&msg);
}
void SuperCap_CanCb(void *message, BusTopicHandle_t topic){
  INTF_CAN_MessageTypeDef *msg=message;
  SuperCap_DataProcess(msg->can_id,msg->data,8,&g_cap_state);
}
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
  int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
  //加0.5使向下取整变成四舍五入
  return b;
}
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
  float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
  return b;
}
void SuperCap_Update(int16_t powerbuff){
  uint8_t data[8]={0};
  data[0]=powerbuff>>8;
  data[1]=powerbuff;
  CAN_Transmit(0x2E,data,8);
}
void SuperCap_Power_Config(SuperCap_Power_Data_Config_typedef configData){
  uint8_t data[8]={0};
  data[0]=configData.Chassis_Power_Max>>8;
  data[1]=configData.Chassis_Power_Max;
  data[2]=configData.Cap_DisCharge_Power_Max>>8;
  data[3]=configData.Cap_DisCharge_Power_Max;
  data[4]=configData.Cap_Charge_Power_Max>>8;
  data[5]=configData.Cap_Charge_Power_Max;
  data[6]=0;
  data[7]=configData.Cap_Output_enable|configData.Cap_Record_enable<<1;
  CAN_Transmit(0x2F,data,8);
}
void SuperCap_DataProcess(uint16_t can_id, uint8_t *data,uint32_t len,SuperCap_Data_Receive_typedef *receive){
  if(len!=8)
    return;
  uint8_t r_data[8]={0};
  memcpy(r_data,data,len);
  if(can_id==0x30){
    int16_t Ucr,I,C,State;
    Ucr=r_data[0]<<8|r_data[1];
    I=r_data[2]<<8|r_data[3];
    C=r_data[4]<<8|r_data[5];
    State=r_data[6]<<8|r_data[7];
    receive->Cap_Voltage= int16_to_float(Ucr,32000,-32000,30,0);
    receive->Cap_Charge_I= int16_to_float(I,32000,-32000,20,-20);
    receive->Cap_Capacity=int16_to_float(C,32000,-32000,2,-2);
    receive->Cap_state.all== State;
  }
  else if(can_id==0x31){
    int16_t B_V,B_I,C_V,C_I;
    B_V=r_data[0]<<8|r_data[1];
    B_I=r_data[2]<<8|r_data[3];
    C_V=r_data[4]<<8|r_data[5];
    C_I=r_data[6]<<8|r_data[7];
    receive->Bat_Voltage=int16_to_float(B_V,32000,-32000,30,0);
    receive->Chassis_Voltage=int16_to_float(C_V,32000,-32000,30,0);
    receive->Bat_I=int16_to_float(B_I,32000,-32000,20,-20);
    receive->Chassis_I=int16_to_float(C_I,32000,-32000,20,-20);
  }
}
void Init_SuperCapX9(){
  xBusSubscribeFromName("/CAN1/RX",SuperCap_CanCb);
  supercap_can_tx_topic=xBusTopicRegister("/CAN1/TX");
}