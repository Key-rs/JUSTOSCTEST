//
// Created by 2712155195 on 2025/7/11.
//
#include "shoot.h"
#define max_angle (30.0f/180.0f)*PI //pitch轴转动角度
#define sk_angle -((12.0f/180.0f)*PI) //偏移角度
#define one_angle  (100.0f/180.0f)*PI
RC_ctrl_t* Shoot_rc_ctrl;
//312
INTF_Motor_HandleTypeDef* motor1;//摩擦轮
INTF_Motor_HandleTypeDef* motor2;//摩擦轮
INTF_Motor_HandleTypeDef* motor3;//拨盘
INTF_Motor_HandleTypeDef* motor4;//pitch轴

extern int g_dr16_is_connected;

//卡弹处理
void Block_dispose()
{
    for(int i = 0;i < 3;i++)
    {
        motor3->target_speed = -100.0f;
        vTaskDelay(500);
        motor3->target_speed = 100.0f;
        vTaskDelay(500);
    }
}

static void Shoot_MainLoop()
{
    vTaskDelay(3000);

    static float angle_change = 0;
    float get_realangal = 0;
    int block_cnt = 0; // 卡弹计数器
    const int block_threshold = 300; // 阈值
    int is_block_handling = 0; // 卡弹处理中标志
    static uint32_t last_shoot_tick = 0; // 上次发射时间戳
    const uint32_t shoot_cooldown_ms = 10; // 最小发射间隔

    motor4->target_angle = sk_angle;

    while (1)
    {
        motor1->target_speed = 0.0f;
        motor2->target_speed = 0.0f;
        motor3->target_speed = 0.0f;
        if(Shoot_rc_ctrl[0].rc.switch_left == 2)
        {
            motor1->target_speed = 30.0f;
            motor2->target_speed = 30.0f;
            motor3->target_speed = 0.0f;//0.0f;
            //连发模式
             if (Shoot_rc_ctrl[0].rc.rocker_r1 > 100.0f)
             {
                 motor3->motor_mode = MOTOR_MODE_SPEED;
                 motor3->target_speed = 400.0f;
             }
             if(Shoot_rc_ctrl[0].rc.rocker_r1 < -100.0f)
             {
                 motor3->target_speed = -50.0f;
             }
            //单发模式
//             if (Shoot_rc_ctrl[0].rc.rocker_r1 < -100.0f)
//             {
//                 motor3->motor_mode = MOTOR_MODE_ANGLE;
//                 get_realangal = motor3->real_angle;
//                 motor3->target_angle = get_realangal + one_angle;
//                 while (fabs(motor3->target_angle - motor3->real_angle) > 1.0f)
//                 {
//                     vTaskDelay(pdMS_TO_TICKS(10));
//                 }
//                 while(1)
//                 {
//                     if(Shoot_rc_ctrl[0].rc.rocker_r1 >= -100.0f)
//                     {
//                         break;
//                     }
//                     vTaskDelay(pdMS_TO_TICKS(10));
//                 }
//             }

            // if(Shoot_rc_ctrl[0].rc.rocker_l1 > 50.0f)
            // {
            //     motor3->target_speed = Shoot_rc_ctrl[0].rc.rocker_l1 / 660.0f * 500.0f;
            // }
            //     uint32_t now = xTaskGetTickCount();
            //     if(now - last_shoot_tick < shoot_cooldown_ms)
            //     {
            //         // 发射过快，禁止发射
            //         motor3->target_speed = 0.0f;
            //     }
            //     else
            //     {
            //         motor3->target_speed = Shoot_rc_ctrl[0].rc.rocker_l1 / 660.0f * 500.0f;
            //         last_shoot_tick = now;
            //     }
            //     // 卡弹判据
            //     if(fabs(motor3->target_speed - motor3->real_speed) > 100.0f)
            //     {
            //         block_cnt++;
            //     }
            //     else
            //     {
            //         block_cnt = 0;
            //     }
            //     // 达到卡弹阈值且未在处理
            //     if(block_cnt > block_threshold && !is_block_handling)
            //     {
            //         is_block_handling = 1;
            //         Block_dispose();
            //         block_cnt = 0;
            //         is_block_handling = 0;
            //     }
            // }
            // else
            // {
            //     block_cnt = 0;
            // }
            // if(Shoot_rc_ctrl[0].rc.rocker_l1 < -50.0f)
            // {
            //     motor3->target_speed = -100.0f;
            // }
            // motor4->target_angle = (Shoot_rc_ctrl[0].rc.rocker_r1 / 660.0f * max_angle) + sk_angle;
        }
        //左拨盘控制pitch轴
        if (Shoot_rc_ctrl[0].rc.dial < -1.0f)
        {
         angle_change += (0.5f/180.0f)*PI;
         if (angle_change >= max_angle)
         {
             angle_change = max_angle;
         }
         vTaskDelay(33);
        }
        if (Shoot_rc_ctrl[0].rc.dial > 1.0f)
        {
         angle_change += -((0.5f/180.0f)*PI);
         if(angle_change <= -max_angle)
         {
             angle_change = -max_angle;
         }
         vTaskDelay(33);
        }
        motor4->target_angle = angle_change + sk_angle;

//        motor4->target_angle = Shoot_rc_ctrl[0].rc.rocker_r1 / 660.0f * max_angle + sk_angle;
        vTaskDelay(1);
    }
}

void Shoot_Init()
{
    motor1 = pvSharePtr("GMMotor1", sizeof(INTF_Motor_HandleTypeDef));
    motor2 = pvSharePtr("GMMotor2", sizeof(INTF_Motor_HandleTypeDef));
    motor3 = pvSharePtr("Turntable", sizeof(INTF_Motor_HandleTypeDef));
    motor4 = pvSharePtr("motor_pitch", sizeof(INTF_Motor_HandleTypeDef));


    Shoot_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(Shoot_MainLoop, "Shoot_MainLoop", 512, NULL, 240, NULL);
}