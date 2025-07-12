#include <stdio.h>
#include "test.h"
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "tinybus_intf.h"

RC_ctrl_t *test_logic_rc_ctrl;

INTF_Motor_HandleTypeDef *test_motor;
extern  int g_dr16_is_connected;

void Test_MainLoop()
{
int angle=0;
    while (1)
    {
        angle = angle+1;
        // if(g_dr16_is_connected)
        // {
        //     // printf("USB_OK");
        //     // float anglex = test_logic_rc_ctrl[0].rc.rocker_l_/660.0f*10.0f;
        //     // test_motor->set_angle(test_motor, anglex);
        //     vTaskDelay(10);
        // }
        test_motor->set_speed(test_motor, 2.0f);;
        printf("%f\n",test_motor->real_speed);
        vTaskDelay(10);
    }
}

void Test_Start()
{
    test_motor = pvSharePtr("/motor/shooter_left", sizeof(INTF_Motor_HandleTypeDef));
    test_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(Test_MainLoop, "Test_MainLoop", 512, NULL, 240, NULL);
}