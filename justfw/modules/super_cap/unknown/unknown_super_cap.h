//
// Created by Ukua on 2024/3/29.
//

#ifndef JUSTFW_UNKNOWN_SUPER_CAP_H
#define JUSTFW_UNKNOWN_SUPER_CAP_H

#include "interface.h"
#include "tinybus.h"
typedef struct{
  float input_voltage;//0.01V
  float cap_voltage;//0.01V
  float input_current;//0.01A
  float target_power;//0.01W
  uint32_t last_update_tick;
}SuperCap_State;
void Unknown_SuperCap_Init();
void Unknown_SuperCap_SetPower(uint16_t power);

#endif //JUSTFW_UNKNOWN_SUPER_CAP_H
