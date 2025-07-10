//
// Created by Ukua on 2024/1/22.
//

#include "mecanumwheel_chassis.h"

#include "cmsis_os.h"

osThreadId MW_Chassis_MainLoopTaskHandle;
INTF_Chassis_HandleTypeDef *g_mw_chassis;

//轮子中心构成的矩形的大小 单位m
#define Chassis_Width 0.48
#define Chassis_Length 0.42
//电机的减速比
#define Motor_DECELE_RATIO 19.203f //这是3508的
//轮子半径 单位m
#define WHEEL_R 0.0763f

/// @brief 设置底盘速度
/// \param self
/// \param speed_x 单位m/s
/// \param speed_y 单位m/s
/// \param speed_w 单位rad/s
void MW_Chassis_SetSpeed(INTF_Chassis_HandleTypeDef *self, float speed_x, float speed_y, float speed_w) {
    MW_Chassis_ResDataTypeDef *priv = self->private_data;

    g_mw_chassis->target_speed_x=speed_x;
    g_mw_chassis->target_speed_y=speed_y;
    g_mw_chassis->target_speed_w=speed_w;
    priv->motor_f_r->set_speed(priv->motor_f_r,((speed_x - speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO /WHEEL_R));
    priv->motor_f_l->set_speed(priv->motor_f_l,((speed_x + speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO /WHEEL_R));
    priv->motor_b_l->set_speed(priv->motor_b_l,((-speed_x + speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO /WHEEL_R));
    priv->motor_b_r->set_speed(priv->motor_b_r,((-speed_x - speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO /WHEEL_R));
}

#include "DR16.h"
void MW_Chassis_MainLoop() {
    while (1) {
        MW_Chassis_SetSpeed(g_mw_chassis,g_mw_chassis->target_speed_x,g_mw_chassis->target_speed_y,g_mw_chassis->target_speed_w);
        osDelay(5);
    }
}


void MW_Chassis_Init() {
    g_mw_chassis = pvSharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));

    MW_Chassis_ResDataTypeDef *priv = g_mw_chassis->private_data = JUST_MALLOC(sizeof(MW_Chassis_ResDataTypeDef));

    priv->motor_f_r = pvSharePtr("/motor/MW_F_R", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_f_l = pvSharePtr("/motor/MW_F_L", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_b_l = pvSharePtr("/motor/MW_B_L", sizeof(INTF_Motor_HandleTypeDef));
    priv->motor_b_r = pvSharePtr("/motor/MW_B_R", sizeof(INTF_Motor_HandleTypeDef));


    xTaskCreate(MW_Chassis_MainLoop, "MW_Chassis_MainLoopTask", 256, NULL, 5, NULL);

}
