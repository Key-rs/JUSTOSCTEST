//
// Created by Ukua on 2024/3/25.
//

#ifndef JUSTFW_PM01_H
#define JUSTFW_PM01_H

#include "interface.h"
#include "tinybus.h"

enum PM01_Mode {
    PM01_MODE_DISABLE = 0, //关闭
    PM01_MODE_CHARGE_ONLY = 1, //负载输出关闭
    PM01_MODE_RUNNING = 2, //负载输出打开
};

typedef struct PM01_Status{
    //状态
    uint8_t is_ready:1;
    uint8_t is_running:1;
    uint8_t is_warning:1;//报警
    uint8_t input_on:1;//电源输入开关
    uint8_t output_on:1;//输出负载开关
    uint8_t is_current_control:1;//恒流控制
    uint8_t is_voltage_control:1;//恒压控制
    uint8_t is_power_control:1;//恒功率控制
    uint8_t reserved:7;//保留位
    uint8_t has_error;//报错

    //错误代码
    uint16_t error_code;
}PM01_StatusTypeDef;//0x610

typedef struct PM01_Input{
    uint16_t power;//单位0.01w
    uint16_t voltage;//单位0.01V
    uint16_t current;//单位0.01A
}PM01_InputTypeDef;//0x611

typedef struct PM01_Output{
    uint16_t power;//单位0.01w
    uint16_t voltage;//单位0.01V
    uint16_t current;//单位0.01A
}PM01_OutputTypeDef;//0x612

typedef struct PM01_Temperature{
    uint16_t temperature;//单位0.1℃
    uint16_t total_time;//单位h,累计运行时间
    uint16_t running_time;//单位min,本次运行时间
}PM01_TemperatureTypeDef;//0x613

void PM01_Init();

#endif //JUSTFW_PM01_H
