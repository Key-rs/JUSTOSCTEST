//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_MECANUMWHEEL_CHASSIS_H
#define JUSTFW_MECANUMWHEEL_CHASSIS_H

#include "tinybus.h"
#include "interface.h"


typedef struct MW_Chassis_ResData{
    INTF_Motor_HandleTypeDef *motor_f_r;
    INTF_Motor_HandleTypeDef *motor_f_l;
    INTF_Motor_HandleTypeDef *motor_b_l;
    INTF_Motor_HandleTypeDef *motor_b_r;
}MW_Chassis_ResDataTypeDef;

void MW_Chassis_Init();
void MW_Chassis_SetSpeed(INTF_Chassis_HandleTypeDef *self, float speed_x, float speed_y, float speed_w);
#endif //JUSTFW_MECANUMWHEEL_CHASSIS_H
