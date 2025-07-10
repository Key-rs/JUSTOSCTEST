//
// Created by Ukua on 2024/1/22.
//

#include "omniwheel_chassis.h"

#include "cmsis_os.h"

//轮子到底盘中心的距离 单位m
#define Chassis_Radius 0.222f
//电机的减速比
#define Motor_DECELE_RATIO 19.203f //这是3508的
//轮子半径 单位m
#define WHEEL_Radius 0.0763f

osThreadId OW_Chassis_MainLoopTaskHandle;
INTF_Chassis_HandleTypeDef *g_ow_chassis;

void OW_Chassis_SetSpeed(INTF_Chassis_HandleTypeDef *self, float speed_x, float speed_y, float speed_w) {
    OW_Chassis_ResDataTypeDef *priv = self->private_data;

    g_ow_chassis->target_speed_x=speed_x;
    g_ow_chassis->target_speed_y=speed_y;
    g_ow_chassis->target_speed_w=speed_w;

    //直径152 30

    priv->motor_f_r->set_speed(priv->motor_f_r,Motor_DECELE_RATIO*(speed_x-speed_y+speed_w/WHEEL_Radius*Chassis_Radius));
    priv->motor_f_l->set_speed(priv->motor_f_l,Motor_DECELE_RATIO*(speed_x+speed_y+speed_w/WHEEL_Radius*Chassis_Radius));
    priv->motor_b_l->set_speed(priv->motor_b_l,Motor_DECELE_RATIO*(-speed_x+speed_y+speed_w/WHEEL_Radius*Chassis_Radius));
    priv->motor_b_r->set_speed(priv->motor_b_r,Motor_DECELE_RATIO*(-speed_x-speed_y+speed_w/WHEEL_Radius*Chassis_Radius));
}

void OW_Chassis_MainLoop() {
    while (1) {
        OW_Chassis_SetSpeed(g_ow_chassis,g_ow_chassis->target_speed_x,g_ow_chassis->target_speed_y,g_ow_chassis->target_speed_w);
        osDelay(5);
    }
}


void OW_Chassis_Init() {
    g_ow_chassis = pvSharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));

    OW_Chassis_ResDataTypeDef *priv = g_ow_chassis->private_data = JUST_MALLOC(sizeof(OW_Chassis_ResDataTypeDef));

    priv->motor_f_r = pvSharePtr("/motor/OW_F_R", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_f_l = pvSharePtr("/motor/OW_F_L", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_b_l = pvSharePtr("/motor/OW_B_L", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_b_r = pvSharePtr("/motor/OW_B_R", sizeof(INTF_Motor_HandleTypeDef));


    xTaskCreate(OW_Chassis_MainLoop, "OW_Chassis_MainLoopTask", 256, NULL, 5, NULL);
}
