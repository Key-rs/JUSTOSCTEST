//
// Created by Konodoki on 2024/10/9.
//

#ifndef HUMANROBOT_HELMMOTOR_H
#define HUMANROBOT_HELMMOTOR_H
#include "stdint.h"
typedef enum {
  DM_Mode_Angle,
  DM_Mode_Speed,
}DM_Mode_typedef;
typedef enum{
  DM_Disable=0,
  DM_Enable=1,
  DM_OverVoltage=8,
  DM_LowVoltage=9,
  DM_OverCurrent=0xA,
  DM_MOS_OverTmp=0xB,
  DM_Coil_OverTmp=0xC,
  DM_Offline=0xD,
  DM_OverLoad=0xE
}DM_Status_typedef;
typedef enum{
  DM_CMD_MotorEnable,
  DM_CMD_MotorDisable,
  DM_CMD_Save_Zero_Position,
  DM_CMD_Clear_Error,
}DM_CMD;
//所有达妙电机的MasterID都为0
typedef struct DM_Motor{
  uint8_t CAN_ID;
  DM_Status_typedef Status;
  float rad;
  float speed;
  float torque;
  float mos_tmp;
  float rotor_tmp;
  uint32_t last_update_time;
}DM_Motor_typedef;
extern DM_Motor_typedef DM_Motors[8];
float DM_uint_to_float(int X_int, float X_min, float X_max,
                       int Bits);
int DM_float_to_uint(float X_float, float X_min, float X_max,
                     int bits);
void HelmMotor_Init();
void HelmMotor_SendCommand(uint16_t CanID,DM_Mode_typedef mode,DM_CMD CMD);
void HelmMotor_SetAngle(uint16_t CanID,float angle,float speed);
void HelmMotor_SetSpeed(uint16_t CanID,float speed);
void HelmMotor_Enable(uint32_t can_id);
void HelmMotor_SetZero(uint32_t can_id);
void HelmMotor_Disable(uint32_t can_id);
#endif // HUMANROBOT_HELMMOTOR_H
