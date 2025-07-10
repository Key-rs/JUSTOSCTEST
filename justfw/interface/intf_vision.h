//
// Created by Ukua on 2024/3/12.
//

#ifndef JUSTFW_INTF_VISION_H
#define JUSTFW_INTF_VISION_H

#include <stdint.h>

typedef struct AutoAim_S2M_Packet {
  uint8_t header;            // 包头0x5A
  uint8_t detect_color : 1;  // 0-检测红色装甲板 1-检测蓝色装甲板
  uint8_t task_mode : 2;     // 0-自动选择 1-自瞄 2-自动打符
  uint8_t reset_tracker : 1; // 重置跟踪器，不知道就给0
  uint8_t is_play : 1;       // 对局是否开始，不知道就给0
  uint8_t change_target : 1; // 切换目标，不知道就给0
  uint8_t reserved : 2;      // 保留位直接给0
  float roll;                // 枪管的roll
  float pitch;               // 枪管的pitch
  float yaw;                 // 枪管的yaw
  float aim_x;               // 当前瞄准的位置x，可以不给
  float aim_y;               // 当前瞄准的位置y，可以不给
  float aim_z;               // 当前瞄准的位置z，可以不给
  uint16_t game_time;        // (s) 当前对局的时间 [0, 450] 不知道给0
  uint32_t timestamp;        // (ms) 本地时间戳
  uint16_t checksum;         // 校验位自动算
} __attribute__((packed)) AutoAim_S2M_PacketTypeDef;

typedef struct AutoAim_M2S_Packet {
  uint8_t header;
  uint8_t state : 2;      // 当前自瞄状态 0-未跟踪 1-自瞄跟踪中 2-打符跟踪中
  uint8_t id : 3;         // 当前自瞄的东西 aim: 0-前哨战 6-guard 7-基地
  uint8_t armors_num : 3; // 装甲板数量 2-平衡 3-前哨站 4-普通
  float x;                // 装甲板的x
  float y;                // 装甲板的y
  float z;                // 装甲板的z
  float yaw;              // 装甲板的yaw
  float vx;               // 装甲板的x速度
  float vy;               // 装甲板的y速度
  float vz;               // 装甲板的z速度
  float v_yaw;            // 装甲板旋转的速度
  float r1;               // 目标中心到前后装甲板的距离
  float r2;               // 目标中心到左右装甲板的距离
  float dz; // 另一对装甲板的相对于被跟踪装甲板的高度差
  uint32_t cap_timestamp; // (ms) 接收到的电控时间戳
  uint16_t t_offset;      // (ms) speed t offset
  uint16_t checksum;
} __attribute__((packed)) AutoAim_M2S_PacketTypeDef;
#endif // JUSTFW_INTF_VISION_H
