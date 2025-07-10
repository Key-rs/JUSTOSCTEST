//
// Created by Ukua on 2024/1/22.
//

#include "gimbal.h"
#include "BMI088driver.h"
#include "GM6020.h"
#include "interface.h"
#include <stdio.h>

#include "cmsis_os.h"
#include "user_lib.h"
osThreadId Gimbal_MainLoopTaskHandle;
INTF_Gimbal_HandleTypeDef *g_gimbal;
extern float INS_angle[3];
extern float INS_SUM_angle[3];
PIDInstance gimbal_yaw_gyro_pid;
PIDInstance gimbal_pitch_gyro_pid;
extern bmi088_real_data_t bmi088_real_data;

#define GIMBAL_YAW INS_SUM_angle[2]
#define GIMBAL_YAW_SPEED bmi088_real_data.gyro[2]
#define GIMBAL_PITCH INS_angle[0]

float Gimbal_GetYaw(){
  switch (g_gimbal->mode) {
  case GIMBAL_MODE_NORMAL:
    return g_gimbal->motor_yaw->real_angle;
  case GIMBAL_MODE_FOLLOW_GYRO:
    return GIMBAL_YAW;
  }
}
float Gimbal_GetPitch(){
  //TODO 注意我这里c板由于是倒置的所以加了个PI
  return loop_float_constrain(GIMBAL_PITCH+3.1415f,-PI,PI);
}
void Gimbal_SetYaw(struct INTF_Gimbal_Handle *self, float target_yaw) {
    //限制yaw轴角度
    if (self->yaw_limit_max==self->yaw_limit_min) {//未设置限制
        self->target_yaw = target_yaw;
    }else{
        if (target_yaw>self->yaw_limit_max){
            self->target_yaw = self->yaw_limit_max;
        }else if (target_yaw<self->yaw_limit_min){
            self->target_yaw = self->yaw_limit_min;
        }else{
            self->target_yaw = target_yaw;
        }
    }
}

void Gimbal_SetPitch(struct INTF_Gimbal_Handle *self, float target_pitch) {
    //限制pitch轴角度
    if (self->pitch_limit_max==self->pitch_limit_min) {//未设置限制
        self->target_pitch = target_pitch;
    }else{
        if (target_pitch>self->pitch_limit_max){
            self->target_pitch = self->pitch_limit_max;
        }else if (target_pitch<self->pitch_limit_min){
            self->target_pitch = self->pitch_limit_min;
        }else{
            self->target_pitch = target_pitch;
        }
    }
}

void Gimbal_SetMode(struct INTF_Gimbal_Handle *self,Gimbal_ModeTypeDef mode){
    switch (mode) {
        //这边做一点处理来防止模式切换时甩头
        case GIMBAL_MODE_NORMAL:
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_ANGLE);
            self->set_yaw(self,self->motor_yaw->real_angle);
            break;
        case GIMBAL_MODE_FOLLOW_GYRO:
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_SPEED);
            self->set_yaw(self,GIMBAL_YAW);
            break;
    }
    self->mode = mode;
}

void Gimbal_MainLoop() {
    extern volatile float q0, q1, q2, q3;
    //TODO 注意我这里C板初始位置是倒置的所以给了个初始角度
    q0= 0.0f;//初始角度
    q1= 1.0f;//初始角度
    osDelay(1000); //等待电机,陀螺仪初始化
    while (HAL_GetTick() - g_gimbal->motor_yaw->update_time>1000||HAL_GetTick() - g_gimbal->motor_pitch->update_time>1000){
      osDelay(1);
    }
    while (1) {
      float a=0;
        switch (g_gimbal->mode) {
            case GIMBAL_MODE_NORMAL:
              g_gimbal->motor_yaw->set_angle(g_gimbal->motor_yaw,g_gimbal->target_yaw);
              g_gimbal->real_yaw = g_gimbal->motor_yaw->real_angle;
              break;
            case GIMBAL_MODE_FOLLOW_GYRO:
              g_gimbal->real_yaw = GIMBAL_YAW;
              g_gimbal->motor_yaw->set_speed(g_gimbal->motor_yaw,PIDCalculate(&gimbal_yaw_gyro_pid,-loop_float_constrain(g_gimbal->target_yaw-g_gimbal->real_yaw,-PI,PI),0));
              break;
        }
        g_gimbal->real_pitch = Gimbal_GetPitch();
        g_gimbal->motor_pitch->set_speed(g_gimbal->motor_pitch,PIDCalculate(&gimbal_pitch_gyro_pid,-loop_float_constrain(g_gimbal->target_pitch-g_gimbal->real_pitch,-PI,PI),0));
        osDelay(1);
    }
}



void Gimbal_Init() {
    g_gimbal = Bus_SharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef ));

    g_gimbal->mode=GIMBAL_MODE_NORMAL;

    g_gimbal->motor_yaw = Bus_SharePtr("/motor/gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    g_gimbal->motor_pitch = Bus_SharePtr("/motor/gimbal_pitch", sizeof(INTF_Motor_HandleTypeDef));

    g_gimbal->set_pitch = Gimbal_SetPitch;
    g_gimbal->set_yaw = Gimbal_SetYaw;
    g_gimbal->set_mode = Gimbal_SetMode;

    g_gimbal->target_yaw=0;
    g_gimbal->target_pitch=0;

    g_gimbal->pitch_limit_max = 0.15f;
    g_gimbal->pitch_limit_min = -0.4f;
    g_gimbal->yaw_limit_max = 0;
    g_gimbal->yaw_limit_min = 0;

    PID_Init_Config_s gimbal_yaw_angle_config = {
        .Kp=40.0f,
        .Ki=120.0f,
        .Kd=8.0f,
        .CoefB=0.05f,
        .CoefA=0.05f,
        .IntegralLimit=200.0f,
        .MaxOut=30.0f * RPM2RPS,
        .Improve=PID_Integral_Limit|PID_Derivative_On_Measurement|PID_ChangingIntegrationRate,
    };
    PIDInit(&gimbal_yaw_gyro_pid,&gimbal_yaw_angle_config);

    PID_Init_Config_s gimbal_pitch_config = {
            .Kp=15.0f,
            .Ki=1.0f,
            .Kd=0.0f,
            .MaxOut=5.0f * RPM2RPS,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
            .IntegralLimit=5.0f,
    };
    PIDInit(&gimbal_pitch_gyro_pid,&gimbal_pitch_config);


    xTaskCreate(Gimbal_MainLoop,"Gimbal_MainLoopTask",256,NULL,5,NULL);
}
