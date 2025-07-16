#ifndef CAN_Bridge
#define CAN_Bridge

#include "interface.h"
#include "tinybus.h"



// typedef struct C620_Config {
//     uint32_t motor_id;
//     char *motor_ptr_name;  // 共享指针名
//     float angle_offset;
//     float gear_ratio;                   // 减速比 默认0=19.203 影响速度角度
//     float direction;                    // 电机方向（电机角度、输出乘以该系数，设置-1反向）
//     INTF_Motor_ModeTypeDef motor_mode;  // 运行模式
//     char *can_rx_topic_name;
//     char *can_tx_topic_name;
//     float *other_feedback_of_angle;  // 其他的角度源(用于替换PID闭环)为空时默认使用电机本身传感器
//     float *other_feedback_of_speed;  // 其他的速度源(用于替换PID闭环)为空时默认使用电机本身传感器
//     float torque_feed_forward;       // 力矩环前馈参数
//     PID_Init_Config_s *speed_pid_config;
//     PID_Init_Config_s *angle_pid_config;
//     PID_Init_Config_s *torque_pid_config;
// } C620_ConfigTypeDef;

typedef struct CanBridge_ResData {
    BusSubscriberHandle_t can_rx_topic;
    BusTopicHandle_t can_tx_topic;

    float gear_ratio;  // 减速比
    int32_t total_rounds;
    float offset_angle;
    int16_t last_ecd;
    float torque_feed_forward;  // 力矩环前馈参数

    BusSubscriberHandle_t can_trans_rx_topic;
    BusTopicHandle_t can_trans_tx_topic;
} CanBridge_ResDataTypeDef;


/*
 * ↑ y (车头方向)
 * |
 * |     x
 * ------>
 */
typedef struct INTF_Helm_Chassis_Handle {
    float target_speed_x;
    float target_speed_y;
    float target_speed_w;
    float real_speed_x;
    float real_speed_y;
    float real_speed_w;

    char *chassis_ptr_name;  // 共享指针名
    char *can_rx_topic_name;
    char *can_tx_topic_name;
    char *can_trans_rx_topic_name;
    char *can_trans_tx_topic_name;

    /**
     * @brief 设置底盘速度
     * @param self 底盘句柄
     * @param speed_x 欲设置的底盘x轴速度（m/s）
     * @param speed_y 欲设置的底盘y轴速度（m/s）
     * @param speed_w 欲设置的底盘w轴速度（rad/s）
     */
    void (*set_speed)(struct INTF_Chassis_Handle *self, float speed_x, float speed_y, float speed_w);  // 设置底盘速度

    void *private_data;
} INTF_Helm_Chassis_HandleTypeDef;

// INTF_Motor_HandleTypeDef *C620_Register(C620_ConfigTypeDef *config);
// int16_t C620_Torque2Current(float torque);
#endif