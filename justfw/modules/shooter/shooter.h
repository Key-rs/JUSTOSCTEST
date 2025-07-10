//
// Created by Ukua on 2024/1/23.
//

#ifndef JUSTFW_SHOOTER_H
#define JUSTFW_SHOOTER_H

#include "interface.h"
#include "tinybus.h"

void Shooter_Init();
void Shoot(int i);

///@brief 拨盘抖动函数，解决卡弹
void Vibrating();
void shoot_one();

void feeder_back();

void fric_on();

void fric_off();
#endif //JUSTFW_SHOOTER_H
