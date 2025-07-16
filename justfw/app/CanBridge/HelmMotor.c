//
// Created by Konodoki on 2024/10/9.
// 文件名: HelmMotor.c
// 描述: 舵机电机控制模块实现文件，包含电机CAN通信协议处理、状态解析、控制指令发送（使能/禁用/角度/速度设置等）核心功能。
//       适配基于CAN总线的DM系列电机，支持角度模式（0x100基址）和速度模式（0x200基址）。
//

#include "HelmMotor.h"       // 电机控制接口头文件
#include "can.h"             // CAN总线基础驱动
#include "stm32f4xx_hal_can.h" // STM32 HAL库CAN模块
#include <math.h>            // 数学运算函数（如fabsf）
#include <string.h>

#include "intf_can.h"
#include "tinybus_intf.h"

// 电机状态参数最大值定义
#define P_MAX 12.5f          // 位置最大值（单位：rad）
#define V_MAX 45.f           // 速度最大值（单位：rad/s）
#define T_MAX 10.f           // 扭矩最大值（单位：N·m）

// 电机状态数组（最多支持8个电机）
DM_Motor_typedef DM_Motors[8] = {0};

/**
 * @brief 将整数编码值转换为实际物理值（解包函数）
 * @param X_int 原始整数值（ADC采样或协议传输值）
 * @param X_min 物理值最小值
 * @param X_max 物理值最大值
 * @param Bits 编码位数（决定动态范围）
 * @return 转换后的物理浮点值
 */
float DM_uint_to_float(int X_int, float X_min, float X_max, int Bits) {
  float span = X_max - X_min;       // 物理值范围跨度
  float offset = X_min;             // 物理值偏移量
  return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset; // 线性映射公式
}

/**
 * @brief 将物理浮点值转换为整数编码值（打包函数）
 * @param X_float 原始物理浮点值
 * @param X_min 物理值最小值
 * @param X_max 物理值最大值
 * @param bits 编码位数（决定动态范围）
 * @return 转换后的整数值（用于协议传输）
 */
int DM_float_to_uint(float X_float, float X_min, float X_max, int bits) {
  float span = X_max - X_min;       // 物理值范围跨度
  float offset = X_min;             // 物理值偏移量
  return (int)((X_float - offset) * ((float)((1 << bits) - 1)) / span); // 线性映射公式
}

/**
 * @brief 通过CAN总线发送数据（底层发送函数）
 * @param can_id CAN标准帧ID（11位）
 * @param data 待发送的8字节数据（DLC固定为8）
 */
void CAN_Transmit(uint32_t id,uint8_t *data,char* topicname) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type=CAN_ID_STD;
    msg.rtr_type=CAN_RTR_DATA;
    msg.can_id=id;
    memcpy(msg.data,data,8);
    vBusPublishFromName(topicname,&msg);
}

/**
 * @brief CAN接收中断回调函数（处理电机反馈数据）
 * @param header CAN接收帧头（包含ID、DLC等信息）
 * @param data 接收的8字节数据（电机反馈数据）
 */
void DM_Motor_CAN_CallBack(INTF_CAN_MessageTypeDef *header, uint8_t *data) {
  if (header->id_type != BSP_CAN_ID_STD)  // 仅处理标准ID为0的反馈帧（根据协议定义）
    return;

  uint8_t id = (data[0] & 0x0F) - 0x01;  // 提取电机ID（低4位，0~7）
  if (id > 7)                            // 校验ID有效性
    return;

  DM_Motors[id].Status = data[0] >> 4;   // 电机状态（高4位）

  // 解析位置/速度/扭矩的原始整数值（按协议格式解包）
  uint16_t P_int = (uint16_t)(((data[1]) << 8) | (data[2]));       // 16位位置值
  uint16_t V_int = (uint16_t)((data[3]) << 4) | ((data[4]) >> 4);  // 12位速度值
  uint16_t T_int = (uint16_t)((data[4] & 0xF) << 8) | ((uint16_t)(data[5])); // 12位扭矩值

  // 转换为实际物理值（使用解包函数）
  DM_Motors[id].rad = DM_uint_to_float(P_int, -P_MAX, P_MAX, 16);    // 位置（rad）
  DM_Motors[id].speed = DM_uint_to_float(V_int, -V_MAX, V_MAX, 12);  // 速度（rad/s）
  DM_Motors[id].torque = DM_uint_to_float(T_int, -T_MAX, T_MAX, 12); // 扭矩（N·m）

  // 解析温度数据（直接转换为浮点值）
  DM_Motors[id].mos_tmp = (float)(data[6]);       // MOS管温度（℃）
  DM_Motors[id].rotor_tmp = (float)(data[7]);     // 转子温度（℃）

  DM_Motors[id].last_update_time = HAL_GetTick(); // 更新最后通信时间戳
}

/**
 * @brief 使能指定CAN ID的电机
 * @param can_id 电机CAN ID（1~8）
 */
void HelmMotor_Enable(uint32_t can_id) {
  // 根据ID奇偶性选择模式（奇数：角度模式，偶数：速度模式），发送使能命令
  HelmMotor_SendCommand(can_id, can_id % 2 != 0 ? DM_Mode_Angle : DM_Mode_Speed, DM_CMD_MotorEnable);
}

/**
 * @brief 禁用指定CAN ID的电机
 * @param can_id 电机CAN ID（1~8）
 */
void HelmMotor_Disable(uint32_t can_id) {
  // 根据ID奇偶性选择模式（奇数：角度模式，偶数：速度模式），发送禁用命令
  HelmMotor_SendCommand(can_id, can_id % 2 != 0 ? DM_Mode_Angle : DM_Mode_Speed, DM_CMD_MotorDisable);
}

/**
 * @brief 初始化电机数组的CAN ID（系统启动时调用）
 */
void HelmMotor_Init() {
  // 为8个电机分配CAN ID（1~8）
  for (int i = 0; i < 8; ++i) {
    DM_Motors[i].CAN_ID = i + 1;
  }
}

/**
 * @brief 通用电机命令发送函数（构造CAN数据帧）
 * @param CanID 电机CAN ID（1~8）
 * @param mode 控制模式（角度模式/速度模式）
 * @param CMD 具体命令（使能/禁用/保存零位等）
 */
void HelmMotor_SendCommand(uint16_t CanID, DM_Mode_typedef mode, DM_CMD CMD) {
  uint32_t base_can_id = 0;
  // 根据模式确定CAN基址（角度模式：0x100，速度模式：0x200）
  if (mode == DM_Mode_Angle) {
    base_can_id = 0x100;
  } else {
    base_can_id = 0x200;
  }

  uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 默认全1数据
  uint32_t can_id = base_can_id + CanID; // 计算最终CAN ID（基址+电机ID）

  // 根据命令修改数据帧第8字节（协议规定）
  switch (CMD) {
    case DM_CMD_MotorEnable:    data[7] = 0xfc; break; // 使能命令码
    case DM_CMD_MotorDisable:   data[7] = 0xfd; break; // 禁用命令码
    case DM_CMD_Save_Zero_Position: data[7] = 0xfe; break; // 保存零位命令码
    case DM_CMD_Clear_Error:    data[7] = 0xfb; break; // 清除错误命令码
  }

  CAN_Transmit(can_id, data,"/CAN2/TX"); // 调用底层发送函数
}

/**
 * @brief 设置电机目标角度（角度模式专用）
 * @param CanID 电机CAN ID（1~8）
 * @param angle 目标角度（rad），小于0.01视为0
 * @param speed 目标速度（rad/s），小于0.01视为0
 */
void HelmMotor_SetAngle(uint16_t CanID, float angle, float speed) {
  // 小量滤波（避免微小抖动）
  if (fabsf(angle) < 0.01) angle = 0;
  if (fabsf(speed) < 0.01f) speed = 0;

  // 将浮点值按字节拆分（用于组包）
  uint8_t *Postion_Tmp = (uint8_t *)&angle;
  uint8_t *Velocity_Tmp = (uint8_t *)&speed;

  // 构造8字节数据帧（前4字节角度，后4字节速度）
  uint8_t data[8] = {0};
  data[0] = *(Postion_Tmp);
  data[1] = *(Postion_Tmp + 1);
  data[2] = *(Postion_Tmp + 2);
  data[3] = *(Postion_Tmp + 3);
  data[4] = *(Velocity_Tmp);
  data[5] = *(Velocity_Tmp + 1);
  data[6] = *(Velocity_Tmp + 2);
  data[7] = *(Velocity_Tmp + 3);

  // 角度模式CAN ID（基址0x100 + 电机ID）
  CAN_Transmit(CanID + 0x100, data,"/CAN2/TX");
}

/**
 * @brief 设置电机目标速度（速度模式专用）
 * @param CanID 电机CAN ID（1~8）
 * @param speed 目标速度（rad/s），限制在±16rad/s内
 */
void HelmMotor_SetSpeed(uint16_t CanID, float speed) {
  // 小量滤波（避免微小抖动）
  if (fabsf(speed) < 0.01f) speed = 0;
  // 速度限幅（保护电机）
  if (speed > 16.0f) speed = 16.0f;
  else if (speed < -16.0f) speed = -16.0f;

  // 将浮点值按字节拆分（用于组包）
  uint8_t *Velocity_Tmp = (uint8_t *)&speed;

  // 构造4字节速度数据帧（后4字节保留0）
  uint8_t data[8] = {0};
  data[0] = *(Velocity_Tmp);
  data[1] = *(Velocity_Tmp + 1);
  data[2] = *(Velocity_Tmp + 2);
  data[3] = *(Velocity_Tmp + 3);

  // 速度模式CAN ID（基址0x200 + 电机ID）
  CAN_Transmit(CanID + 0x200, data,"/CAN2/TX");
}

/**
 * @brief 保存电机当前位置为零位（校准功能）
 * @param can_id 电机CAN ID（1~8）
 */
void HelmMotor_SetZero(uint32_t can_id) {
  // 根据ID奇偶性选择模式（奇数：角度模式，偶数：速度模式），发送保存零位命令
  HelmMotor_SendCommand(can_id, can_id % 2 != 0 ? DM_Mode_Angle : DM_Mode_Speed, DM_CMD_Save_Zero_Position);
}