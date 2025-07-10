//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_GIMBAL_H
#define JUSTFW_GIMBAL_H

#include "interface.h"
#include "tinybus.h"



void Gimbal_Init();
void Gimbal_SetMode(struct INTF_Gimbal_Handle *self,Gimbal_ModeTypeDef mode);
float Gimbal_GetYaw();
float Gimbal_GetPitch();
#endif //JUSTFW_GIMBAL_H
