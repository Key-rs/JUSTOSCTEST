//
// Created by Ukua on 2023/11/5.
//

#ifndef JUSTFW_USER_LIB_H
#define JUSTFW_USER_LIB_H

/**
 ******************************************************************************
 * @file    user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief   用户实用函数库头文件，提供数学运算、信号处理和数据转换等功能
 ******************************************************************************
 * @attention
 * 本库依赖 CMSIS RTOS 和硬件抽象层（如 HAL），需确保相关接口正确配置。
 ******************************************************************************
 */
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

#define msin(x) (arm_sin_f32(x)) // 使用 ARM 数学库的正弦函数
#define mcos(x) (arm_cos_f32(x)) // 使用 ARM 数学库的余弦函数

extern uint8_t GlobalDebugMode; // 全局调试模式变量

/* 布尔类型定义 */
#ifndef TRUE
#define TRUE 1 /**< 布尔真 */
#endif

#ifndef FALSE
#define FALSE 0 /**< 布尔假 */
#endif

/* 数学相关常量 */
/* 弧度转换系数 */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* 圆周率 */
#ifndef PI
#define PI 3.14159265354f
#endif

/* 宏定义：限制值在指定范围内 */
#define VAL_LIMIT(val, min, max)     \
    do {                             \
        if ((val) <= (min)) {        \
            (val) = (min);           \
        } else if ((val) >= (max)) { \
            (val) = (max);           \
        }                            \
    } while (0)

/* 宏定义：将角度限制在 0~360 度 */
#define ANGLE_LIMIT_360(val, angle)     \
    do {                                \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

/* 宏定义：将角度从 0~360 度转换为 -180~180 度 */
#define ANGLE_LIMIT_360_TO_180(val) \
    do {                            \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

/* 宏定义：取最小值 */
#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
/* 宏定义：取最大值 */
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 分配一块清零的内存，返回指针，需强制转换为目标类型
 *
 * @param size 内存大小（字节）
 * @return void* 指向分配内存的指针
 */
void *zero_malloc(size_t size);

/**
 * @brief 快速计算平方根
 *
 * @param x 输入值（非负）
 * @return float 平方根值
 */
float Sqrt(float x);

/**
 * @brief 绝对值限制
 *
 * @param num 输入值
 * @param Limit 绝对值上限
 * @return float 限幅后的值
 */
float abs_limit(float num, float Limit);

/**
 * @brief 判断符号位
 *
 * @param value 输入值
 * @return float 正返回 1.0f，负返回 -1.0f
 */
float sign(float value);

/**
 * @brief 浮点死区处理
 *
 * @param Value 输入值
 * @param minValue 死区下限
 * @param maxValue 死区上限
 * @return float 处理后的值（在死区内返回 0）
 */
float float_deadband(float Value, float minValue, float maxValue);

/**
 * @brief 浮点数限幅
 *
 * @param Value 输入值
 * @param minValue 下限
 * @param maxValue 上限
 * @return float 限幅后的值
 */
float float_constrain(float Value, float minValue, float maxValue);

/**
 * @brief 16 位整数限幅
 *
 * @param Value 输入值
 * @param minValue 下限
 * @param maxValue 上限
 * @return int16_t 限幅后的值
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

/**
 * @brief 循环限幅
 *
 * @param Input 输入值
 * @param minValue 下限
 * @param maxValue 上限
 * @return float 循环调整后的值
 */
float loop_float_constrain(float Input, float minValue, float maxValue);

/**
 * @brief 角度格式化为 -180~180 度
 *
 * @param Ang 输入角度
 * @return float 格式化后的角度
 */
float theta_format(float Ang);

/**
 * @brief 浮点数四舍五入
 *
 * @param raw 输入浮点数
 * @return int 四舍五入后的整数
 */
int float_rounding(float raw);

/**
 * @brief 三维向量归一化
 *
 * @param v 三维向量
 * @return float* 归一化后的向量
 */
float *Norm3d(float *v);

/**
 * @brief 计算三维向量模长
 *
 * @param v 三维向量
 * @return float 模长
 */
float NormOf3d(float *v);

/**
 * @brief 三维向量叉乘
 *
 * @param v1 向量 1
 * @param v2 向量 2
 * @param res 结果向量
 */
void Cross3d(float *v1, float *v2, float *res);

/**
 * @brief 三维向量点乘
 *
 * @param v1 向量 1
 * @param v2 向量 2
 * @return float 点乘结果
 */
float Dot3d(float *v1, float *v2);

/**
 * @brief 均值滤波
 *
 * @param new_data 新数据
 * @param buf 数据缓冲区
 * @param len 缓冲区长度
 * @return float 滤波后的平均值
 */
float AverageFilter(float new_data, float *buf, uint8_t len);

/**
 * @brief 一阶低通滤波
 *
 * @param new_data 新数据
 * @param old_data 前次输出
 * @param factor 滤波因子（0~1）
 * @return float 滤波后的值
 */
float LowPassFilter(float new_data, float old_data, float factor);

/**
 * @brief 浮点数转换为无符号整数
 *
 * @param x 输入浮点数
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return int 转换后的无符号整数
 */
int float_to_uint(float x, float x_min, float x_max, int bits);

/**
 * @brief 无符号整数转换为浮点数
 *
 * @param x 输入无符号整数
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return float 转换后的浮点数
 */
float uint_to_float(uint64_t x, float x_min, float x_max, int bits);

/**
 * @brief 浮点数转换为有符号整数
 *
 * @param x 输入浮点数
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return int 转换后的有符号整数
 */
int float_to_int(float x, float x_min, float x_max, int bits);

/**
 * @brief 有符号整数转换为浮点数
 *
 * @param x 输入有符号整数
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return float 转换后的浮点数
 */
float int_to_float(int x, float x_min, float x_max, int bits);

/**
 * @brief USB 格式化打印
 *
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
void USB_Printf(const char *fmt, ...);

/**
 * @brief 获取时间差
 *
 * @param last_tick 上次时间戳
 * @return uint32_t 时间差（毫秒）
 */
uint32_t getDtick(uint32_t *last_tick);

/* 宏定义：弧度格式化为 -π ~ π */
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif  // JUSTFW_USER_LIB_H
