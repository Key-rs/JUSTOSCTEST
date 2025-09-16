#include "shoot.h"
#define max_angle 1.06 //pitch轴转动角度
#define min_angle 0.05
#define sk_angle -((12.0f/180.0f)*PI) //偏移角度
#define one_angle  (100.0f/180.0f)*PI
RC_ctrl_t* Shoot_rc_ctrl;
#include "GimbalFollow.h"
//312
INTF_Motor_HandleTypeDef* motor1;//摩擦轮
INTF_Motor_HandleTypeDef* motor2;//摩擦轮
INTF_Motor_HandleTypeDef* motor3;//拨盘
INTF_Motor_HandleTypeDef* motor4;//pitch轴

LowPassFilterConfig output_filter;
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
/**
 * 将一个数从源区间映射到目标区间
 * @param x 要映射的数
 * @param in_min 源区间最小值
 * @param in_max 源区间最大值
 * @param out_min 目标区间最小值
 * @param out_max 目标区间最大值
 * @return 映射后的结果
 */
float map_range(float x, float in_min, float in_max, float out_min, float out_max) {
    // 对输入值进行限幅，确保它在[in_min, in_max]范围内
    float clamped_x = fmaxf(in_min, fminf(x, in_max));

    // 计算源区间的长度
    float in_range = in_max - in_min;
    // 计算目标区间的长度
    float out_range = out_max - out_min;

    // 防止除零错误
    if (in_range == 0) {
        // 如果源区间长度为0，返回目标区间的中点
        return (out_min + out_max) / 2.0f;
    }

    // 映射公式：(x - 源最小值)/源区间长度 * 目标区间长度 + 目标最小值
    return (clamped_x - in_min) * out_range / in_range + out_min;
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

    // motor4->target_angle = sk_angle;

    PIDInstance pidtest;
    pidtest.Kp = 7.0f;
    pidtest.Ki = 0.9f;
    pidtest.Kd = 0.025f;

    PID_Init_Config_s pid_config = {
        .Kp = pidtest.Kp,
        .Ki = pidtest.Ki,
        .Kd = pidtest.Kd,
        .MaxOut = 100.0f,        // 输出限幅
        .IntegralLimit = 100.0f, // 积分限幅
        .DeadBand = 0,         // 死区阈值
        .Improve = PID_Trapezoid_Intergral |   // 启用梯形积分
                   PID_Integral_Limit |       // 启用积分限幅
                   PID_DerivativeFilter |     // 启用微分滤波
                   PID_OutputFilter,          // 启用输出滤波
        .CoefA = 0.1f,                 // 变速积分参数A
        .CoefB = 0.2f,                 // 变速积分参数B
        .Derivative_LPF_RC = 0.01f,    // 微分滤波时间常数
        .Output_LPF_RC = 0.01f         // 输出滤波时间常数
    };
    // 初始化PID控制器
    PIDInit(&pidtest, &pid_config);
    float last_angle = 0;
    LowPassFilter_Init(&output_filter, 0.5f);
    while (1)
    {
        motor1->target_speed = 0.0f;
        motor2->target_speed = 0.0f;
        motor3->target_speed = 0.0f;
        if(Shoot_rc_ctrl[0].rc.switch_right == 3)
        {
            motor1->target_speed = 20.0f;
            motor2->target_speed = 20.0f;
        }else
        {
            motor1->target_speed = 0.0f;
            motor2->target_speed = 0.0f;
        }
        if (Shoot_rc_ctrl[0].rc.dial > 100.0f)
        {
            motor3->motor_mode = MOTOR_MODE_SPEED;
            motor3->target_speed = 400.0f;
        }
        if(Shoot_rc_ctrl[0].rc.dial < -100.0f)
        {
            motor3->target_speed = -50.0f;
        }

        float an=map_range(Shoot_rc_ctrl[0].rc.rocker_r1,-660,660, min_angle, max_angle);
        motor4->target_speed=LowPassFilter_Apply(&output_filter,PIDCalculate(&pidtest, motor4->real_angle, an));

        // printf("%f,%f,%f\n", motor4->real_angle,motor4->target_speed,an);
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