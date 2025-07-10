//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_OMNIWHEEL_CHASSIS_H
#define JUSTFW_OMNIWHEEL_CHASSIS_H

#include "tinybus.h"
#include "interface.h"

typedef struct OW_Chassis_ResData{
    INTF_Motor_HandleTypeDef *motor_f_r;
    INTF_Motor_HandleTypeDef *motor_f_l;
    INTF_Motor_HandleTypeDef *motor_b_l;
    INTF_Motor_HandleTypeDef *motor_b_r;
}OW_Chassis_ResDataTypeDef;

void OW_Chassis_Init();

#endif //JUSTFW_OMNIWHEEL_CHASSIS_H
