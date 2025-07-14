//
// Created by Ukua on 2023/9/24.
// 小米CyberGear电机驱动实现（基于CAN总线通信）

#include "cybergear.h"
#include "user_lib.h"

// 全局变量：电机实例管理
static uint16_t idx = 0;                  // 当前已注册电机数量索引
INTF_Motor_HandleTypeDef *g_cybergear_motors[CYBERGEAR_MOTOR_NUM] = {0};  // 电机句柄数组（最大支持CYBERGEAR_MOTOR_NUM个）

// 角度校准偏移量（用于修正电机初始位置误差）
#define OFFSET_ANGLE 0.396692792f
//float offsets[4] = {-OFFSET_ANGLE,OFFSET_ANGLE+3.14f,OFFSET_ANGLE+3.14f,-OFFSET_ANGLE};

// 主循环任务句柄（FreeRTOS任务）
osThreadId Cybergear_MainLoopTaskHandle;

// 电机控制参数范围定义（用于浮点数与16位整数转换）
#define P_MIN (-12.5f)    // 位置最小值（单位：rad）
#define P_MAX 12.5f       // 位置最大值
#define V_MIN (-30.0f)    // 速度最小值（单位：rad/s）
#define V_MAX 30.0f       // 速度最大值
#define KP_MIN 0.0f       // 位置环P参数最小值
#define KP_MAX 500.0f     // 位置环P参数最大值
#define KD_MIN 0.0f       // 速度环D参数最小值
#define KD_MAX 5.0f       // 速度环D参数最大值
#define T_MIN (-12.0f)    // 扭矩最小值（单位：N·m）
#define T_MAX 12.0f       // 扭矩最大值
#define A_MIN (-4*3.1415926535)  // 绝对角度最小值（单位：rad）
#define A_MAX (4*3.1415926535)   // 绝对角度最大值

// 扩展CAN ID结构体（用于解析/构造CAN消息ID）
struct exCanIdInfo {
    uint32_t id: 8;    // 电机ID（8位）
    uint32_t data: 16; // 数据字段（16位）
    uint32_t mode: 5;  // 控制模式（5位）
    uint32_t res: 3;   // 保留位（3位）
};

/**
 * @brief 发送电机启用指令（通过CAN总线）
 * @param id 目标电机ID
 * @param master_id 主控制器ID
 * @param can_tx_topic CAN发送话题句柄（用于发布消息）
 */
void Cybergear_Enable(uint8_t id, uint16_t master_id, BusTopicHandle_t can_tx_topic) {
    // 构造扩展CAN ID（模式3：启用指令）
    struct exCanIdInfo txCanIdEx = {
        .mode = 3,       // 控制模式：启用
        .id = id,        // 目标电机ID
        .res = 0,        // 保留位清零
        .data = master_id, // 主控制器ID
    };

    // 构造CAN消息（数据域全0）
    INTF_CAN_MessageTypeDef msg = {
        .id_type = BSP_CAN_ID_EXT,  // 扩展帧格式
        .can_id = *((uint32_t *) &txCanIdEx),  // 转换结构体为32位ID
    };
    memset(msg.data, 0, 8);  // 数据域清零（协议规定启用指令无数据）
    vBusPublish(can_tx_topic, &msg);  // 通过总线发布消息
}

/**
 * @brief 发送电机控制指令（多参数模式）
 * @param id 目标电机ID
 * @param torque 目标扭矩（N·m）
 * @param MechPosition 目标机械位置（rad）
 * @param speed 目标速度（rad/s）
 * @param kp 位置环P参数
 * @param kd 速度环D参数
 * @param can_tx_topic CAN发送话题句柄
 */
void Cybergear_ControlMode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd,
                           BusTopicHandle_t can_tx_topic) {
    // 构造扩展CAN ID（模式1：控制指令）
    struct exCanIdInfo txCanIdEx = {
        .mode = 1,       // 控制模式：多参数控制
        .id = id,        // 目标电机ID
        .res = 0,        // 保留位清零
        .data = float_to_uint(torque, T_MIN, T_MAX, 16),  // 扭矩值转换为16位整数
    };

    // 构造CAN消息（数据域包含位置、速度、KP、KD参数）
    INTF_CAN_MessageTypeDef msg = {
        .id_type = BSP_CAN_ID_EXT,  // 扩展帧格式
        .can_id = *((uint32_t *) &txCanIdEx),  // 转换结构体为32位ID
        // 位置值（高8位+低8位）
        .data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8,
        .data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16),
        // 速度值（高8位+低8位）
        .data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8,
        .data[3] = float_to_uint(speed, V_MIN, V_MAX, 16),
        // KP值（高8位+低8位）
        .data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8,
        .data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16),
        // KD值（高8位+低8位）
        .data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8,
        .data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16),
    };
    vBusPublish(can_tx_topic, &msg);  // 通过总线发布消息
}

/**
 * @brief 发送电机重置指令（通过CAN总线）
 * @param id 目标电机ID
 * @param master_id 主控制器ID
 * @param can_tx_topic CAN发送话题句柄
 */
void Cybergear_Reset(uint8_t id, uint16_t master_id, BusTopicHandle_t can_tx_topic) {
    // 构造扩展CAN ID（模式4：重置指令）
    struct exCanIdInfo txCanIdEx = {
        .mode = 4,       // 控制模式：重置
        .id = id,        // 目标电机ID
        .res = 0,        // 保留位清零
        .data = master_id, // 主控制器ID
    };

    // 构造CAN消息（无数据域）
    INTF_CAN_MessageTypeDef msg = {
        .id_type = BSP_CAN_ID_EXT,  // 扩展帧格式
        .can_id = *((uint32_t *) &txCanIdEx),  // 转换结构体为32位ID
    };
    vBusPublish(can_tx_topic, &msg);  // 通过总线发布消息
}

/**
 * @brief 触发运控模式更新（当前未启用）
 * @note 被注释的代码为潜在实现，需根据实际协议完善
 */
void Cybergear_Trigger(struct INTF_Motor_Handle *self) {
    // 示例代码（未启用）：根据目标参数发送控制指令
    // Cybergear_ControlMode(self->motor_id, self->target_torque * self->direction,
    //                      self->target_angle * self->direction,
    //                      self->target_speed * self->direction,
    //                      ((Cybergear_ResDataTypeDef *) self->private_data)->kp,
    //                      ((Cybergear_ResDataTypeDef *) self->private_data)->kd,
    //                      ((Cybergear_ResDataTypeDef *) self->private_data)->can_tx_topic);
}

/**
 * @brief 设置电机控制模式（暂未实现）
 */
void Cybergear_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    return;  // TODO: 需根据协议实现模式切换逻辑
}

/**
 * @brief 设置电机目标速度并触发更新
 * @param speed 目标速度（rad/s）
 */
void Cybergear_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;  // 更新目标速度
    Cybergear_Trigger(self);     // 触发控制指令更新
}

/**
 * @brief 设置电机目标角度并触发更新
 * @param angle 目标角度（rad）
 */
void Cybergear_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;  // 更新目标角度
    Cybergear_Trigger(self);     // 触发控制指令更新
}

/**
 * @brief 设置电机目标扭矩并触发更新
 * @param torque 目标扭矩（N·m）
 */
void Cybergear_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;  // 更新目标扭矩
    Cybergear_Trigger(self);       // 触发控制指令更新
}

/**
 * @brief 重置指定电机（通过CAN发送重置指令）
 */
void Cybergear_Reset_t(struct INTF_Motor_Handle *self) {
    Cybergear_Reset(self->motor_id, Master_CAN_ID,
                   ((Cybergear_ResDataTypeDef *)self->private_data)->can_tx_topic);
}

/**
 * @brief 启用指定电机（通过CAN发送启用指令）
 */
void Cybergear_Enable_t(struct INTF_Motor_Handle *self) {
    Cybergear_Enable(self->motor_id, Master_CAN_ID,
                    ((Cybergear_ResDataTypeDef *) self->private_data)->can_tx_topic);
}

/**
 * @brief 禁用指定电机（暂未实现）
 * @note TODO: 需补充禁用指令协议逻辑
 */
void Cybergear_Disable_t(struct INTF_Motor_Handle *self) {
    //TODO: 补全disable（发送禁用指令）
    return;
}

/**
 * @brief 电机主控制循环（FreeRTOS任务）
 * @note 负责电机状态监控、重连和控制指令发送
 */
void Cybergear_MainLoop() {
    osDelay(1000);  // 启动延迟（等待电机初始化）

    // 初始化阶段：启用所有注册电机
    for (int i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
        Cybergear_Enable(g_cybergear_motors[i]->motor_id, Master_CAN_ID,
                        ((Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data)->can_tx_topic);
    }
    osDelay(1000);  // 等待启用完成

    while (1) {  // 主循环
        for (int i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
            INTF_Motor_HandleTypeDef *m = g_cybergear_motors[i];
            Cybergear_ResDataTypeDef *priv = (Cybergear_ResDataTypeDef *) m->private_data;

            // 状态监控：判断电机是否断开（超过1秒未更新）
            if (xTaskGetTickCount() - priv->last_update_tick > 1000 && !priv->has_disconnected) {
                priv->has_disconnected = 1;  // 标记断开
            }
            // 重连逻辑：断开后重新收到数据则重新启用
            if (priv->has_disconnected && xTaskGetTickCount() - priv->last_update_tick < 100) {
                Cybergear_Enable(m->motor_id, Master_CAN_ID, priv->can_tx_topic);
                priv->has_disconnected = 0;  // 清除断开标记
            }

            // 根据控制模式获取目标参数
            float t_torque = 0, t_speed = 0, t_angle = 0;
            switch (m->motor_mode) {
                case MOTOR_MODE_ANGLE:  // 角度模式
                    t_angle = m->target_angle;
                    break;
                case MOTOR_MODE_SPEED:  // 速度模式
                    t_speed = m->target_speed;
                    break;
                case MOTOR_MODE_TORQUE: // 扭矩模式
                    t_torque = m->target_torque;
                    break;
                case MOTOR_MODE_MIT:    // 综合模式（位置+速度+扭矩）
                    t_angle = m->target_angle;
                    t_speed = m->target_speed;
                    t_torque = m->target_torque;
                    break;
            }

            // 发送控制指令（包含角度偏移修正）
            Cybergear_ControlMode(m->motor_id,
                                t_torque * m->direction,  // 方向系数修正
                                (t_angle - priv->angle_offset) * m->direction,  // 角度偏移修正
                                t_speed * m->direction,   // 方向系数修正
                                priv->kp,                 // 位置环P参数
                                priv->kd,                 // 速度环D参数
                                priv->can_tx_topic);      // CAN发送话题
        }
        osDelay(1);  // 循环周期1ms
    }
}

/**
 * @brief CAN消息接收回调（处理电机反馈数据）
 * @param message CAN消息指针（类型为INTF_CAN_MessageTypeDef）
 * @param topic 订阅的CAN接收话题句柄
 */
void Cybergear_CAN_CallBack(void *message, BusTopicHandle_t topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *) message;

    // 过滤无效消息（仅处理扩展帧且ID在指定范围内）
    if (msg->id_type == BSP_CAN_ID_STD || msg->can_id < 0x2000000 || msg->can_id > 0x3000000) {
        return;
    }

    // 解析扩展CAN ID（获取电机ID和模式）
    struct exCanIdInfo *info = (struct exCanIdInfo *) &msg->can_id;
    uint8_t id = info->data & 0x7F;  // 提取电机ID（低7位）

    // 查找匹配的电机实例
    INTF_Motor_HandleTypeDef *m = NULL;
    int i;
    for (i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
        // 匹配条件：电机ID一致且CAN接收话题匹配
        if (g_cybergear_motors[i]->motor_id == id &&
            ((Cybergear_ResDataTypeDef*)g_cybergear_motors[i]->private_data)->can_rx_topic->pxTopic == topic) {
            m = g_cybergear_motors[i];
            break;
        }
        // 遍历完未找到则返回
        if (i == CYBERGEAR_MOTOR_NUM - 1) return;
    }

    // 初始化阶段：校准角度偏移（首次接收数据时）
    if (m->motor_state == MOTOR_STATE_INIT) {
        ((Cybergear_ResDataTypeDef *) m->private_data)->angle_offset +=
            -uint_to_float((msg->data[0] << 8) + msg->data[1], A_MIN, A_MAX, 16) * m->direction;
        m->motor_state = MOTOR_STATE_RUNNING;  // 切换为运行状态
    }

    // 更新电机状态和时间戳
    ((Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data)->last_update_tick = xTaskGetTickCount();
    m->update_time = HAL_GetTick();  // 记录更新时间（HAL库时间戳）

    // 解析反馈数据并转换为物理量（考虑方向系数和偏移）
    m->real_angle = uint_to_float((msg->data[0] << 8) + msg->data[1], A_MIN, A_MAX, 16) * m->direction +
                    ((Cybergear_ResDataTypeDef *) m->private_data)->angle_offset;  // 实际角度
    m->real_speed = uint_to_float((msg->data[2] << 8) + msg->data[3], V_MIN, V_MAX, 16) * m->direction;  // 实际速度
    m->real_torque = uint_to_float((msg->data[4] << 8) + msg->data[5], T_MIN, T_MAX, 16) * m->direction;  // 实际扭矩
}


void Cybergear_Register(Cybergear_ConfigTypeDef *config) {
    INTF_Motor_HandleTypeDef *motor = pvSharePtr(config->motor_ptr_name, sizeof(INTF_Motor_HandleTypeDef));
    motor->motor_id = config->motor_id;
    motor->motor_mode = config->motor_mode;
    motor->motor_state = MOTOR_STATE_INIT;
    motor->target_speed = 0.0f;
    motor->real_speed = 0.0f;
    motor->target_angle = 0.0f;
    motor->real_angle = 0.0f;
    motor->target_torque = 0.0f;
    motor->real_torque = 0.0f;
    motor->direction = config->direction;

    motor->set_torque = Cybergear_SetTorque_t;
    motor->set_speed = Cybergear_SetSpeed_t;
    motor->set_angle = Cybergear_SetAngle_t;
    motor->set_mode = Cybergear_Setmode_t;

    motor->private_data = JUST_MALLOC(sizeof(Cybergear_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(Cybergear_ResDataTypeDef));
    ((Cybergear_ResDataTypeDef *) motor->private_data)->angle_offset = config->angle_offset;
    ((Cybergear_ResDataTypeDef *) motor->private_data)->can_rx_topic = xBusSubscribeFromName(config->can_rx_topic_name,
                                                                                             Cybergear_CAN_CallBack);
    ((Cybergear_ResDataTypeDef *) motor->private_data)->can_tx_topic = xBusTopicRegister(config->can_tx_topic_name);
    ((Cybergear_ResDataTypeDef *) motor->private_data)->kp = config->kp;
    ((Cybergear_ResDataTypeDef *) motor->private_data)->kd = config->kd;
    g_cybergear_motors[idx++] = motor;
}


void Cybergear_Init() {

  //注册电机开始
  Cybergear_ConfigTypeDef pitch_config = {
      .motor_id = 127,
      .motor_ptr_name = "motor_pitch",
      .angle_offset = 0,
      .direction = 1.0f,
      .motor_mode = MOTOR_MODE_ANGLE,
      .kd=0.5f,
      .kp=5.0f,
      .can_rx_topic_name = "/CAN1/RX",
      .can_tx_topic_name = "/CAN1/TX",
  };
  Cybergear_Register(&pitch_config);
    //注册电机结束

    // osThreadDef(Cybergear_MainLoopTask, Cybergear_MainLoop, osPriorityLow, 0, 256);
    // Cybergear_MainLoopTaskHandle = osThreadCreate(osThread(Cybergear_MainLoopTask), NULL);
    xTaskCreate(Cybergear_MainLoop, "Cybergear_MainLoopTask", 512, NULL, 5, NULL);
}


