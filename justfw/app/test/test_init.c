//
// Created by ASUS on 25-7-12.
//
#include "test.h"
#ifndef USE_GM_MOTOR_DRIVER
#define USE_GM_MOTOR_DRIVER
#endif


void C620_Init() {
    PID_Init_Config_s angle_pid2 = {
        .Kp=28.01f,
        .Ki=10.00f,
        .Kd=5.1f,
        .MaxOut=5.0f,
        .DeadBand = 0.0f,
        .Improve=PID_Integral_Limit,
};
    PID_Init_Config_s speed_pid2 = {
        .Kp=0.2f,
        .Ki=0.02f,
        .Kd=0.001f,
        .MaxOut=6.0f,
        .DeadBand = 0.0f,
        .Output_LPF_RC=0.1f,
        .Improve=PID_Integral_Limit | PID_OutputFilter,
        .IntegralLimit=1.0f,
};
    PID_Init_Config_s torque_pid2 = {
        .Kp=1000.0f,
        .Ki=5000.0f,
        .Kd=0.0f,
        .MaxOut=C620_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve=PID_Integral_Limit,
        .IntegralLimit=500.0f,
};
    C620_ConfigTypeDef config2 = {
        .motor_id=1,
        .motor_ptr_name="/motor/shooter_left",
        .motor_mode=MOTOR_MODE_SPEED,
        .direction=-1.0f,
        // .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
        .angle_pid_config=&angle_pid2,
        .speed_pid_config=&speed_pid2,
        .torque_pid_config=&torque_pid2,
        .can_rx_topic_name="/CAN1/RX",
        .can_tx_topic_name="/CAN1/TX",
};
    C620_Register(&config2);
}

void Test_init()
{
    C620_Init();

}