//
// Created by Konodoki on 2024/12/27.
//
#include "SentryConfig.h"
#include "SentryLogic.h"
#include "cmsis_os.h"
#include "interface.h"
#include "referee.h"
#include "rng.h"
#include "tinybus.h"
#include "user.h"
#include <stdio.h>

#include "user_lib.h"
extern RC_ctrl_t* rc_ctrl;
extern float INS_SUM_angle[3]; //角度的累计值
Navigation_M2S_PacketTypeDef nav_msg;
uint32_t last_nav_msg_tick = 0;
BusTopicHandle_t* helm_can_topic_tx;
static INTF_Gimbal_HandleTypeDef* gimbal;

static int DM_float_to_uint(float X_float, float X_min, float X_max,
                            int bits)
{
    float span = X_max - X_min;
    float offset = X_min;
    return (int)((X_float - offset) * ((float)((1 << bits) - 1)) / span);
}

void Set_speed(float x, float y, float z, float yaw, uint8_t cor)
{
    int x_speed_int = DM_float_to_uint(x, -20, 20, 16);
    int y_speed_int = DM_float_to_uint(y, -20, 20, 16);
    int z_speed_int = DM_float_to_uint(z, -20, 20, 16);
    int yaw_int = DM_float_to_uint(yaw, -20, 20, 15);
    INTF_CAN_MessageTypeDef msg = {
        .id_type = BSP_CAN_ID_STD,
        .can_id = 0x21,
        .data[0] = x_speed_int >> 8,
        .data[1] = x_speed_int,
        .data[2] = y_speed_int >> 8,
        .data[3] = y_speed_int,
        .data[4] = z_speed_int >> 8,
        .data[5] = z_speed_int,
        .data[6] = yaw_int >> 8 | cor << 7,
        .data[7] = yaw_int,
    };
    vBusPublish(helm_can_topic_tx, &msg);
}

void maintask_loop()
{
    osDelay(3000);
    uint32_t rotate_count = 0;
    uint32_t rng;
    float last_vx = 0, last_vy = 0, last_vz = 0;
    float rotate_cycle = 500.0f;
    while (1)
    {
        if (sentryControlMode == Sentry_Automatic_Control)
        {
            if (xTaskGetTickCount() - last_nav_msg_tick < 100)
            {
                Set_speed(nav_msg.vx, nav_msg.vy, nav_msg.w_z,
                          loop_float_constrain(-gimbal->motor_yaw->real_angle, -PI,PI),
                          sentryGimbalMode == Sentry_Gimbal_Follow);
            }
            else
            {
                Set_speed(0, 0, 10.0f, 0, 0);
            }
        }
        else
        {
            float vx = 1.0f * rc_ctrl[1].rc.rocker_l1 / 660.0f;
            float vy = -1.0f * rc_ctrl[1].rc.rocker_l_ / 660.0f;
            if (rc_ctrl[0].keyboard.w != 0 || rc_ctrl[0].keyboard.s != 0 || rc_ctrl[0].keyboard.a != 0 || rc_ctrl[0].
                keyboard.d != 0)
            {
                vx = (rc_ctrl[0].keyboard.w - rc_ctrl[0].keyboard.s) * 1.3f;
                vy = (-rc_ctrl[0].keyboard.a + rc_ctrl[0].keyboard.d) * 1.3f;
            }
            rotate_count++;
            if (rotate_count > 1000)
            {
                rotate_count = 0;
                HAL_RNG_GenerateRandomNumber(&hrng, &rng);
                rng &= 0xFFFF;
                rotate_cycle = 200.0f + (float)rng / 65535.0f / 2.0f * 200.0f;
            }
            float speed = 0;
            if (sentryControlMode == Sentry_Manual_Control && sentryGimbalMode == Sentry_Gimbal_Follow)
            {
                float rc = rc_ctrl[1].rc.dial / 660.0f;
                speed = (fabsf(arm_sin_f32(xTaskGetTickCount() / rotate_cycle)) + 0.5f) * 10.0f * rc;
                if (rc_ctrl[0].keyboard.shift)
                {
                    speed = LowPassFilter(10.0f, last_vz, 0.001f);
                }
            }
            else if (sentryControlMode == Sentry_Manual_Control && sentryGimbalMode == Sentry_Outpost)
            {
                speed = LowPassFilter(10.0f, last_vz, 0.001f);
            }
            else
            {
                speed = LowPassFilter(10.0f * rc_ctrl[1].rc.dial / 660.0f, last_vz, 0.005f);
            }
            vx = LowPassFilter(vx, last_vx, 0.0025f);
            vy = LowPassFilter(vy, last_vy, 0.0025f);
            Set_speed(vx, vy, speed, loop_float_constrain(-gimbal->motor_yaw->real_angle, -PI,PI),
                      sentryGimbalMode == Sentry_Gimbal_Follow);
            last_vx = vx;
            last_vy = vy;
            last_vz = speed;
        }
        osDelay(1);
    }
}

void nav_set(void* msg, BusTopicHandle_t* topic)
{
    nav_msg = (*(Navigation_M2S_PacketTypeDef*)msg);
    last_nav_msg_tick = xTaskGetTickCount();
}

void user_control_init()
{
    gimbal = pvSharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef));
    xBusSubscribeFromName("Navigation_RX", nav_set);
    helm_can_topic_tx = xBusTopicRegister("/CAN1/TX");
    // osThreadDef(MainLoopTask, maintask_loop, osPriorityNormal, 0, 128);
    // osThreadCreate(osThread(MainLoopTask), NULL);
    xTaskCreate(maintask_loop, "maintask_loop", 128, NULL, 5, NULL);

}

USER_EXPORT(user_control, user_control_init);
