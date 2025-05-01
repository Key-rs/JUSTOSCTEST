#include "dm_motor.h"
#include "interface.h"

INTF_Motor_HandleTypeDef *first, *second, *third, *fourth, *big;
void SB_Motor_init()
{
    DM_Motor_ConfigTypeDef config1 = {
        .motor_ptr_name = "/motor/first",
        .angle_offset = 0.0f,
        .can_tx_topic_name = "/CAN1/TX",
        .can_rx_topic_name = "/CAN1/RX",
        .motor_mode = MOTOR_MODE_SPEED,
        .direction = 1.0f,
        .kd = 1.0f,
        .kp = 0.0f,
        .motor_id = 0x01};


    DM_Motor_ConfigTypeDef config2 = {
        .motor_ptr_name = "/motor/second",
        .angle_offset = 0.0f,
        .can_tx_topic_name = "/CAN2/TX",
        .can_rx_topic_name = "/CAN2/RX",
        .motor_mode = MOTOR_MODE_SPEED,
        .direction = -1.0f,
        .kd = 1.0f,
        .kp = 0.0f,
        .motor_id = 0x02};
  
    DM_Motor_ConfigTypeDef config3 = {
        .motor_ptr_name = "/motor/third",
        .angle_offset = 0.0f,
        .can_tx_topic_name = "/CAN2/TX",
        .can_rx_topic_name = "/CAN2/RX",
        .motor_mode = MOTOR_MODE_SPEED,
        .direction = -1.0f,
        .kd = 1.0f,
        .kp = 0.0f,
        .motor_id = 0x03};
  

    DM_Motor_ConfigTypeDef config4 = {
        .motor_ptr_name = "/motor/fourth",
        .angle_offset = 0.0f,
        .can_tx_topic_name = "/CAN1/TX",
        .can_rx_topic_name = "/CAN1/RX",
        .motor_mode = MOTOR_MODE_SPEED,
        .direction = 1.0f,
        .kd = 1.0f,
        .kp = 0.0f,
        .motor_id = 0x04};
    

    DM_Motor_ConfigTypeDef config5 = {
        .motor_ptr_name = "/motor/big",
        .angle_offset = 0.0f,
        .can_tx_topic_name = "/CAN2/TX",
        .can_rx_topic_name = "/CAN2/RX",
        .motor_mode = MOTOR_MODE_SPEED,
        .direction = 1.0f,
        .kd = 1.0f,
        .kp = 0.0f,
        .motor_id = 0x05};

    first = DM_Motor_Register(&config1);
    second = DM_Motor_Register(&config2);
    third = DM_Motor_Register(&config3);
    fourth = DM_Motor_Register(&config4);
    big = DM_Motor_Register(&config5);
}