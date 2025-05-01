created by AI on 2025/5/1
暂时 for 使用3519电机 取消急停
# user_lib使用说明
## 概述
user_lib 是一个为嵌入式系统设计的实用函数库，特别适用于基于微控制器的电机控制应用。它提供了数学运算、信号处理、数据转换和向量操作等功能，支持电机控制、数据滤波和数据处理等任务。该库设计轻量且高效，适合实时应用场景。
本说明文档概述了库的功能、主要特性以及在项目中的使用方法。
## 功能特性
数学运算：快速平方根、绝对值限幅、符号检测和死区处理。
信号处理：低通滤波和均值滤波，用于平滑传感器数据或控制信号。
数据转换：支持浮点数与整数之间的转换，可指定范围和位深度。
向量操作：三维向量归一化、叉乘、点乘和模长计算。
其他工具：循环限幅、角度格式化、时间差计算等实用功能。
## 依赖
标准 C 库：依赖 math.h、stdlib.h 和 string.h。
硬件抽象层：需要 main.h 和 interface.h 提供硬件相关接口（如 HAL_GetTick）。
内存管理：使用自定义的 JUST_MALLOC 宏进行内存分配。
## 文件结构
user_lib.h：头文件，包含函数声明和宏定义。
user_lib.c：源文件，包含所有函数的实现。
## 配置
包含头文件：
在项目中包含 user_lib.h：
c
#include "user_lib.h"
添加源文件：
将 user_lib.c 添加到项目的编译路径中，确保与依赖的硬件接口文件（如 main.h 和 interface.h）一起编译。
配置内存分配：
确保 JUST_MALLOC 宏已定义，通常在 interface.h 或其他硬件抽象层中实现，用于动态内存分配。
全局调试模式：
库中定义了一个全局变量 GlobalDebugMode（默认值为 7），可用于调试控制。根据项目需求修改其值。
## 函数说明
以下是 user_lib 提供的主要函数及其用法：
1. 内存管理
void *zero_malloc(size_t size)
功能：分配指定大小的内存并清零。
参数：size - 分配的内存大小（字节）。
返回：指向分配内存的指针。
示例：
c
float *buffer = zero_malloc(10 * sizeof(float));
2. 数学运算
float Sqrt(float x)
功能：快速计算平方根（适用于正数）。
参数：x - 输入值（非负）。
返回：平方根值。
示例：
c
float result = Sqrt(16.0f); // 返回 4.0f
float abs_limit(float num, float Limit)
功能：将输入值限制在 [-Limit, Limit] 范围内。
参数：
num - 输入值。
Limit - 绝对值上限。
返回：限幅后的值。
示例：
c
float value = abs_limit(10.0f, 5.0f); // 返回 5.0f
float sign(float value)
功能：返回输入值的符号（正返回 1.0f，负返回 -1.0f）。
示例：
c
float s = sign(-3.2f); // 返回 -1.0f
float float_constrain(float Value, float minValue, float maxValue)
功能：将浮点值限制在指定范围内。
示例：
c
float constrained = float_constrain(15.0f, 0.0f, 10.0f); // 返回 10.0f
float loop_float_constrain(float Input, float minValue, float maxValue)
功能：循环限幅，将值限制在指定范围内，超出范围时循环调整。
示例：
c
float angle = loop_float_constrain(190.0f, -180.0f, 180.0f); // 返回 -170.0f
float theta_format(float Ang)
功能：将角度格式化为 -180° 到 180°。
示例：
c
float formatted = theta_format(190.0f); // 返回 -170.0f
3. 信号处理
float AverageFilter(float new_data, float *buf, uint8_t len)
功能：均值滤波，更新缓冲区并返回平均值。
参数：
new_data - 新输入数据。
buf - 数据缓冲区。
len - 缓冲区长度。
示例：
c
float buf[5] = {0};
float avg = AverageFilter(10.0f, buf, 5);
float LowPassFilter(float new_data, float old_data, float factor)
功能：一阶低通滤波。
参数：
new_data - 新输入数据。
old_data - 前一次输出值。
factor - 滤波因子（0 到 1）。
示例：
c
float filtered = LowPassFilter(10.0f, 5.0f, 0.2f); // 返回 6.0f
4. 数据转换
int float_to_uint(float x, float x_min, float x_max, int bits)
功能：将浮点数映射到无符号整数（指定位数）。
示例：
c
int uint_val = float_to_uint(5.0f, 0.0f, 10.0f, 16); // 映射到 0-65535
float uint_to_float(uint64_t x, float x_min, float x_max, int bits)
功能：将无符号整数转换回浮点数。
示例：
c
float float_val = uint_to_float(32768, 0.0f, 10.0f, 16); // 返回约 5.0f
int float_to_int(float x, float x_min, float x_max, int bits)
功能：将浮点数映射到有符号整数。
示例：
c
int int_val = float_to_int(-5.0f, -10.0f, 10.0f, 16);
float int_to_float(int x, float x_min, float x_max, int bits)
功能：将有符号整数转换回浮点数。
示例：
c
float float_val = int_to_float(-32768, -10.0f, 10.0f, 16); // 返回约 -5.0f
5. 向量操作
float *Norm3d(float *v)
功能：归一化三维向量。
示例：
c
float v[3] = {3.0f, 4.0f, 0.0f};
Norm3d(v); // v 变为 {0.6f, 0.8f, 0.0f}
float NormOf3d(float *v)
功能：计算三维向量的模长。
示例：
c
float v[3] = {3.0f, 4.0f, 0.0f};
float len = NormOf3d(v); // 返回 5.0f
void Cross3d(float *v1, float *v2, float *res)
功能：计算两个三维向量的叉乘。
示例：
c
float v1[3] = {1.0f, 0.0f, 0.0f}, v2[3] = {0.0f, 1.0f, 0.0f}, res[3];
Cross3d(v1, v2, res); // res = {0.0f, 0.0f, 1.0f}
float Dot3d(float *v1, float *v2)
功能：计算两个三维向量的点乘。
示例：
c
float v1[3] = {1.0f, 2.0f, 3.0f}, v2[3] = {4.0f, 5.0f, 6.0f};
float dot = Dot3d(v1, v2); // 返回 32.0f
6. 时间相关
uint32_t getDtick(uint32_t *last_tick)
功能：计算自上次调用以来的时间差（毫秒）。
参数：last_tick - 存储上一次时间戳的指针。
返回：时间差（毫秒）。
示例：
c
uint32_t last = HAL_GetTick();
uint32_t dt = getDtick(&last); // 返回时间差
## 使用示例
以下是一个简单的示例，展示如何使用 user_lib 配合 dm_motor.h 进行电机控制：
c
#include "dm_motor.h"
#include "user_lib.h"

void MotorControlExample(void) {
    // 初始化电机
    DM_Motor_Init();

    // 获取电机句柄（假设已注册）
    INTF_Motor_HandleTypeDef *motor = g_DM_motors[0];

    // 设置目标速度
    motor->set_speed(motor, 10.0f); // 设置速度为 10 rad/s

    // 主循环
    while (1) {
        // 读取当前速度
        float current_speed = motor->real_speed;

        // 应用低通滤波平滑速度数据
        static float filtered_speed = 0.0f;
        filtered_speed = LowPassFilter(current_speed, filtered_speed, 0.1f);

        // 限制输出扭矩
        float torque = float_constrain(filtered_speed * 0.5f, DM_T_MIN, DM_T_MAX);

        // 设置扭矩
        motor->set_torque(motor, torque);

        osDelay(5); // 延时 5ms
    }
}
## 注意事项
浮点运算精度：在资源受限的嵌入式系统中，浮点运算可能影响性能。必要时可优化为定点运算。
内存管理：JUST_MALLOC 必须正确实现，避免内存泄漏。
线程安全：部分函数（如 AverageFilter）使用静态变量或共享缓冲区，多线程使用时需加锁。
范围检查：数据转换函数（如 float_to_uint）会自动限幅，但需确保输入范围合理以避免溢出。
