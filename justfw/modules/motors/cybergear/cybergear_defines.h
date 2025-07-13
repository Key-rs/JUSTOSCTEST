//
// Created by Ukua on 2023/9/24.
//

#ifndef JUSTFW_CYBERGEAR_DEFINES_H
#define JUSTFW_CYBERGEAR_DEFINES_H

#define Master_CAN_ID 0x00

#define Communication_Type_GetID 0x00     //获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 	//用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02	//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03	//电机使能运行
#define Communication_Type_MotorStop 0x04	//电机停止运行
#define Communication_Type_SetPosZero 0x06	//设置电机机械零位
#define Communication_Type_CanID 0x07	//更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//读取单个参数
#define Communication_Type_SetSingleParameter 0x12	//设定单个参数
#define Communication_Type_ErrorFeedback 0x15	//故障反馈帧

#define Run_mode 0x7005
#define Iq_Ref 0x7006
#define Spd_Ref 0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain 0.1

#define MAX_P 720
#define MIN_P -720
#define MAX_S 30
#define MIN_S -30
#define MAX_T 12
#define MIN_T -12

#define Motor_Error 0x00
#define Motor_OK 0X01

#define Current_mode 3
#define Position_mode 1
#define Motion_mode 0
#define Speed_mode 2



#endif //JUSTFW_CYBERGEAR_DEFINES_H
