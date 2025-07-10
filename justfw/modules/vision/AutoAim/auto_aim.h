//
// Created by Konodoki on 2025/1/12.
//

#ifndef JUSTFW_AUTO_AIM_H
#define JUSTFW_AUTO_AIM_H
#include "stdint.h"
typedef struct AutoAimStatus{
  float out_yaw;
  float out_pitch;
  float yaw_speed;
  uint8_t tracking;
  uint32_t solve_tick;
}AutoAimStatus_t;
void auto_aim_init();
#endif // JUSTFW_AUTO_AIM_H
