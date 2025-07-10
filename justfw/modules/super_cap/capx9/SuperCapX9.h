//
// Created by Konodoki on 2024/8/28.
//

#ifndef SUPERCAPTEST_SUPERCAP_H
#define SUPERCAPTEST_SUPERCAP_H
#include "stdint.h"
#define CAN_Transmit(id,data,len) CAN_T(id,data,len)
typedef struct SuperCap_Power_Data_Config{
  int16_t Chassis_Power_Max;
  int16_t Cap_DisCharge_Power_Max;
  int16_t Cap_Charge_Power_Max;
  uint8_t Cap_Output_enable;
  uint8_t Cap_Record_enable;
}SuperCap_Power_Data_Config_typedef;
typedef struct SuperCap_Data_Receive{
  float Cap_Voltage;
  float Cap_Charge_I;
  float Cap_Capacity;
  /**
    bit0        警告
    bit1	电容过压
    bit2	电容过流
    bit3	电容欠压
    bit4	裁判系统欠压
    bit5	未读到CAN通信数据
   */
  union{
    int16_t all;
    struct{
      uint8_t Warning:1;
      uint8_t Cap_Over_V:1;
      uint8_t Cap_Over_I:1;
      uint8_t Cap_Low_V:1;
      uint8_t Bat_Low_V:1;
      uint8_t Loss_Can:1;
    }bits;
  }Cap_state;
  float Bat_Voltage;
  float Bat_I;
  float Chassis_Voltage;
  float Chassis_I;

}SuperCap_Data_Receive_typedef;
void Init_SuperCapX9();
void SuperCap_Update(int16_t powerbuff);
void SuperCap_Power_Config(SuperCap_Power_Data_Config_typedef configData);
void SuperCap_DataProcess(uint16_t can_id, uint8_t *data,uint32_t len,SuperCap_Data_Receive_typedef *receive);
#endif // SUPERCAPTEST_SUPERCAP_H
