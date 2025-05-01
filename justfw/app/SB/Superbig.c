// creat by tilihuo123 on 2025/3/16
// finished by tilihuo123 on 2025/3/26
// RT第一人称对抗赛充气轮

#include "SuperBig.h"

RC_ctrl_t *Super_logic_rc_ctrl;

// 轮子参数

#define WHEEL_BASE 0.4864f // 左右轮间距 (m)，486.4 mm
// 最大速度限制
float MAX_MOTOR_SPEED = 20.0; // 最大电机速度

#define JOYSTICK_MAX 660.0f

static void Differential_Control_4WD(float left_joystick_x, float left_joystick_y, float right_joystick_x, float mv)
{
    // 右摇杆死区

    if (fabs(right_joystick_x) < 100.0f)
    {
        right_joystick_x = 0.0f;
    }

    float v = (left_joystick_y / JOYSTICK_MAX);   // 线速度 (m/s)
    float w = -(right_joystick_x / JOYSTICK_MAX); // 角速度 (rad/s)，右转为负

    float w_left;
    float w_right;
    if (v == 0 && w == 0)
    {
        w_left = 0;
        w_right = 0;
    }
    else
    {
        w_left = ((v - w * WHEEL_BASE) / 2) * mv;
        w_right = ((v + w * WHEEL_BASE) / 2) * mv;
    }
    if (w_left > mv)
        w_left = mv;
    if (w_left < -mv)
        w_left = -mv;
    if (w_right > mv)
        w_right = mv;
    if (w_right < -mv)
        w_right = -mv;

    first->set_speed(first, w_right);   // 右前轮
    second->set_speed(second, w_left);  // 左前轮
    third->set_speed(third, w_left);    // 左后轮
    fourth->set_speed(fourth, w_right); // 右后轮
}
static void Big_Control(float right_joystick_y)
{
    if (fabs(right_joystick_y) <= 440)
        right_joystick_y = 0;
    float big_speed = right_joystick_y * 10 / JOYSTICK_MAX;
    big->set_speed(big, big_speed);
}

static void SuperBig_MainLoop()
{

    while (1)
    {
        switch (Super_logic_rc_ctrl[0].rc.switch_right)
        {
        case RC_SW_UP:
            MAX_MOTOR_SPEED = 10;
            break;

        case RC_SW_MID:
            MAX_MOTOR_SPEED = 15;
            break;
        case RC_SW_DOWN:
            MAX_MOTOR_SPEED = 20;
            break;
        }

        Differential_Control_4WD(
            Super_logic_rc_ctrl[0].rc.rocker_l_,
            Super_logic_rc_ctrl[0].rc.rocker_l1,
            Super_logic_rc_ctrl[0].rc.rocker_r_,
            MAX_MOTOR_SPEED);
        Big_Control(Super_logic_rc_ctrl[0].rc.rocker_r1);
        vTaskDelay(1);
    }
}

void SuperBig_Init()
{
    SB_Motor_init();
    Super_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(SuperBig_MainLoop, "SuperBig_MainLoop", 1024, NULL, 240, NULL);
}