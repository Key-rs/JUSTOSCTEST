//
// Created by Konodoki on 2024-03-21.
//

#ifndef JUSTFW_NAVIGATION_COM_H
#define JUSTFW_NAVIGATION_COM_H
#include "tinybus.h"
#include "interface.h"
void Navigation_Com_Init(void);
void NavigationData_Transmit(Navigation_S2M_PacketTypeDef *nav_msg);
#endif //JUSTFW_NAVIGATION_COM_H
