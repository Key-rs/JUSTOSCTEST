//
// Created by Ukua on 2023/11/5.
//

/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "user_lib.h"

#include "interface.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

uint8_t GlobalDebugMode = 7;

void *zero_malloc(size_t size) {
    void *ptr = JUST_MALLOC(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x) {
    float y;
    float delta;
    float maxError;

    if (x <= 0) {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit) {
    if (num > Limit) {
        num = Limit;
    } else if (num < -Limit) {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value) {
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue) {
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue) {
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        float len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        float len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang) {
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw) {
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v) {
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v) {
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res) {
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len) {
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

// 低通滤波
float LowPassFilter(float new_data, float old_data, float factor) {
    return old_data * (1 - factor) + new_data * factor;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(uint64_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_int(float x, float x_min, float x_max, int bits) {
    if (x > 0) {
        return float_to_uint(x, 0, x_max, bits);
    } else {
        return -float_to_uint(-x, 0, -x_min, bits);
    }
}

float int_to_float(int x, float x_min, float x_max, int bits) {
    if (x > 0) {
        return uint_to_float(x, 0, x_max, bits);
    } else {
        return -uint_to_float(-x, 0, -x_min, bits);
    }
}

uint32_t getDtick(uint32_t *last_tick) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t ret = *last_tick < current_tick ? current_tick - *last_tick : ~(*last_tick) + current_tick;
    *last_tick = current_tick;

    return ret;
}