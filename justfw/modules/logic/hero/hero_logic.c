//
// Created by Ukua on 2024/3/28.
//

#include "hero_logic.h"

#include "cmsis_os.h"
#include "kalman.h"
#include "user_lib.h"

osThreadId Hero_Logic_MainLoopTaskHandle;

#include "referee.h"

referee_data_t *g_hero_logic_referee;

INTF_Chassis_HandleTypeDef *g_hero_logic_chassis;

INTF_Gimbal_HandleTypeDef *g_hero_logic_gimbal;

RC_ctrl_t *hero_logic_rc_ctrl;

#define CHASSIS_SPEED_X_MAX 10.0f
#define CHASSIS_SPEED_Y_MAX 10.0f
#define CHASSIS_SPEED_W_MAX 10.0f


#define GIMBAL_YAW_SENSITIVITY 0.0003f
#define GIMBAL_PITCH_SENSITIVITY 0.0002f

//INTF_Motor_HandleTypeDef *test_m;

//PIDInstance chassis_follow_gimbal;

BusTopicHandle_t hero_shoot_one,hero_feeder_back,hero_fric_on,hero_fric_off;


extKalman_t hero_kalman_mouse_x_speed,hero_kalman_mouse_y_speed;
float hero_mouse_x_speed_buffer[10]={0};
float hero_mouse_y_speed_buffer[10]={0};
void Hero_Logic_MainLoop() {
    osDelay(5000);//等待其他模块初始化

    g_hero_logic_gimbal->set_mode(g_hero_logic_gimbal,GIMBAL_MODE_FOLLOW_GYRO);
    g_hero_logic_gimbal->set_pitch(g_hero_logic_gimbal,0);

    static uint8_t last_sw_l=0;

    while (1) {
        extern uint8_t g_dr16_is_connected;
        if(g_dr16_is_connected) {
            //底盘控制
//
            if(hero_logic_rc_ctrl[0].rc.switch_right==2){
                float vy = (hero_logic_rc_ctrl[0].keyboard.w - hero_logic_rc_ctrl[0].keyboard.s)*10.0f;
                float vx = (hero_logic_rc_ctrl[0].keyboard.a - hero_logic_rc_ctrl[0].keyboard.d)*10.0f;
                g_hero_logic_chassis->target_speed_y = vy * sinf(g_hero_logic_gimbal->real_yaw) - vx * cosf(g_hero_logic_gimbal->real_yaw);
                g_hero_logic_chassis->target_speed_x = vy * cosf(g_hero_logic_gimbal->real_yaw) + vx * sinf(g_hero_logic_gimbal->real_yaw);

                //        g_if_logic_chassis->target_speed_w= if_logic_rc_ctrl[0].keyboard.q*10.0f-if_logic_rc_ctrl[0].keyboard.e*10.0f;
//            g_if_logic_chassis->target_speed_w = PIDCalculate(&chassis_follow_gimbal, g_if_logic_gimbal->real_yaw, 0);


                g_hero_logic_gimbal->target_yaw-=hero_logic_rc_ctrl[0].rc.rocker_r_/660.0f*0.01f;
                g_hero_logic_gimbal->target_yaw-=hero_logic_rc_ctrl[0].mouse.x*GIMBAL_YAW_SENSITIVITY;
                g_hero_logic_gimbal->target_pitch-=hero_logic_rc_ctrl[0].mouse.y*GIMBAL_PITCH_SENSITIVITY;

                float speed_x = AverageFilter(KalmanFilter(&hero_kalman_mouse_x_speed, hero_logic_rc_ctrl[0].mouse.x),
                                              hero_mouse_x_speed_buffer, 10);
                float speed_y = AverageFilter(KalmanFilter(&hero_kalman_mouse_y_speed, hero_logic_rc_ctrl[0].mouse.y),
                                              hero_mouse_y_speed_buffer, 10);

                g_hero_logic_gimbal->target_yaw -= speed_x * GIMBAL_YAW_SENSITIVITY;
                g_hero_logic_gimbal->target_pitch -= speed_y * GIMBAL_PITCH_SENSITIVITY;

//                    g_if_logic_gimbal->target_yaw+=if_logic_rc_ctrl[0].rc.rocker_l_/660.0f*0.01f;
//                    g_if_logic_gimbal->target_pitch+=if_logic_rc_ctrl[0].rc.rocker_l1/660.0f*0.05f;

//            if(if_logic_rc_ctrl[0].keyboard.shift){
//                g_hero_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
//            } else{
//                g_hero_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
//            }
            }else{
                g_hero_logic_chassis->target_speed_x= hero_logic_rc_ctrl[0].rc.rocker_l_ / 660.0f * CHASSIS_SPEED_X_MAX;
                g_hero_logic_chassis->target_speed_y= hero_logic_rc_ctrl[0].rc.rocker_l1 / 660.0f * CHASSIS_SPEED_Y_MAX;
                g_hero_logic_chassis->target_speed_w= -hero_logic_rc_ctrl[0].rc.dial / 660.0f * CHASSIS_SPEED_W_MAX;

                g_hero_logic_gimbal->target_yaw+=hero_logic_rc_ctrl[0].rc.rocker_r_/660.0f*0.01f;
                g_hero_logic_gimbal->target_pitch+=hero_logic_rc_ctrl[0].rc.rocker_r1/660.0f*0.05f;

                if(last_sw_l==3&&hero_logic_rc_ctrl[0].rc.switch_left==1){
                    vBusPublish(hero_shoot_one,NULL);
                }
                if(last_sw_l==3&&hero_logic_rc_ctrl[0].rc.switch_left==2){
                    vBusPublish(hero_feeder_back,NULL);
                }
                if(hero_logic_rc_ctrl[0].rc.switch_right==1){
                    vBusPublish(hero_fric_on,NULL);
                }else{
                    vBusPublish(hero_fric_off,NULL);
                }
                last_sw_l = hero_logic_rc_ctrl[0].rc.switch_left;

            }

        }
        osDelay(10);
    }
}

void Hero_Logic_Init() {
    g_hero_logic_chassis = pvSharePtr("chassis",sizeof(INTF_Chassis_HandleTypeDef));
    g_hero_logic_gimbal = pvSharePtr("gimbal",sizeof(INTF_Gimbal_HandleTypeDef));
    hero_logic_rc_ctrl = pvSharePtr("DR16",sizeof(INTF_Chassis_HandleTypeDef));

    hero_shoot_one = xBusTopicRegister("/signal/shoot_one");
    hero_feeder_back = xBusTopicRegister("/signal/feeder_back");
    hero_fric_on = xBusTopicRegister("/signal/fric_on");
    hero_fric_off = xBusTopicRegister("/signal/fric_off");

//    test_m = Bus_SharePtr("/motor/test",sizeof(INTF_Motor_HandleTypeDef));
//
//    PID_Init_Config_s follow_gimbal_config = {
//            .Kp=8.0f,
//            .Ki=0.0f,
//            .Kd=0.0f,
//            .MaxOut=5.0f * RPM2RPS,
//            .DeadBand = 0.0f,
//            .Improve=PID_OutputFilter|PID_Integral_Limit,//|PID_Derivative_On_Measurement,
//            .IntegralLimit=1.0f,
//    };
//    PIDInit(&chassis_follow_gimbal,&follow_gimbal_config);
//
//    KalmanCreate(&kalman_mouse_x_speed,1,60);
//    KalmanCreate(&kalman_mouse_y_speed,1,60);
    /********按键映射***********/
    xBusSubscribeFromName("/signal/keyboard",NULL);

    xTaskCreate(Hero_Logic_MainLoop, "Hero_Logic_MainLoop", 512, NULL, 1, NULL);
}
