//
// Created by Ukua on 2024/3/12.
//

#ifndef JUSTFW_VISION_COM_H
#define JUSTFW_VISION_COM_H

#include "tinybus.h"
#include "interface.h"

void Vision_Com_Init();
void AutoAim_Transmit(AutoAim_S2M_PacketTypeDef *aim_msg);
#endif //JUSTFW_VISION_COM_H
