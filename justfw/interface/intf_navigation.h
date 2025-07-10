//
// Created by Konodoki on 2024-03-21.
//

#ifndef JUSTFW_INTF_NAVIGATION_H
#define JUSTFW_INTF_NAVIGATION_H
#include <stdint.h>

typedef struct Navigation_S2M_Packet
{
    uint8_t header;//0x6A
    float x;
    float y;
    float yaw;
    uint16_t checksum;
} __attribute__((packed)) Navigation_S2M_PacketTypeDef;

typedef struct Navigation_M2S_Packet
{
    uint8_t header;//0xA6
    float vx;
    float vy;
    float vz;
    float w_x;
    float w_y;
    float w_z;
    uint16_t checksum;
} __attribute__((packed)) Navigation_M2S_PacketTypeDef;
#endif //JUSTFW_INTF_NAVIGATION_H
