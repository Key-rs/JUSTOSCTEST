//
// Created by Ukua on 2023/9/24.
//

#include "cybergear.h"

#include "user_lib.h"

static uint16_t idx = 0;
INTF_Motor_HandleTypeDef *g_cybergear_motors[CYBERGEAR_MOTOR_NUM] = {0};

#define OFFSET_ANGLE 0.396692792f
//float offsets[4] = {-OFFSET_ANGLE,OFFSET_ANGLE+3.14f,OFFSET_ANGLE+3.14f,-OFFSET_ANGLE};


osThreadId Cybergear_MainLoopTaskHandle;

#define P_MIN (-12.5f)
#define P_MAX 12.5f
#define V_MIN (-30.0f)
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-12.0f)
#define T_MAX 12.0f

#define A_MIN (-4*3.1415926535)
#define A_MAX (4*3.1415926535)

struct exCanIdInfo {
    uint32_t id: 8;
    uint32_t data: 16;
    uint32_t mode: 5;
    uint32_t res: 3;
};

void Cybergear_Enable(uint8_t id, uint16_t master_id, BusTopicHandle_t can_tx_topic) {
    struct exCanIdInfo txCanIdEx = {
            .mode = 3,
            .id = id,
            .res = 0,
            .data = master_id,
    };

    INTF_CAN_MessageTypeDef msg = {
            msg.id_type = BSP_CAN_ID_EXT,
            msg.can_id = *((uint32_t *) &txCanIdEx),
    };
    memset(msg.data, 0, 8);
    vBusPublish(can_tx_topic, &msg);
}

void Cybergear_ControlMode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd,
                           BusTopicHandle_t can_tx_topic) {
    struct exCanIdInfo txCanIdEx = {
            .mode = 1,
            .id = id,
            .res = 0,
            .data = float_to_uint(torque, T_MIN, T_MAX, 16),
    };

    INTF_CAN_MessageTypeDef msg = {
            .id_type=BSP_CAN_ID_EXT,
            .can_id= *((uint32_t *) &txCanIdEx),
            .data[0]=float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8,
            .data[1]=float_to_uint(MechPosition, P_MIN, P_MAX, 16),
            .data[2]=float_to_uint(speed, V_MIN, V_MAX, 16) >> 8,
            .data[3]=float_to_uint(speed, V_MIN, V_MAX, 16),
            .data[4]=float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8,
            .data[5]=float_to_uint(kp, KP_MIN, KP_MAX, 16),
            .data[6]=float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8,
            .data[7]=float_to_uint(kd, KD_MIN, KD_MAX, 16),
    };
    vBusPublish(can_tx_topic, &msg);
}

void Cybergear_Reset(uint8_t id, uint16_t master_id, BusTopicHandle_t can_tx_topic) {
    struct exCanIdInfo txCanIdEx = {
            .mode = 4,
            .id = id,
            .res = 0,
            .data = master_id,
    };

    INTF_CAN_MessageTypeDef msg = {
            .id_type=BSP_CAN_ID_EXT,
            .can_id= *((uint32_t *) &txCanIdEx),
    };
    vBusPublish(can_tx_topic, &msg);
}

//触发运控模式更新
void Cybergear_Trigger(struct INTF_Motor_Handle *self) {
//    Cybergear_ControlMode(self->motor_id, self->target_torque * self->direction, self->target_angle * self->direction,
//                          self->target_speed * self->direction, ((Cybergear_ResDataTypeDef *) self->private_data)->kp,
//                          ((Cybergear_ResDataTypeDef *) self->private_data)->kd,
//                          ((Cybergear_ResDataTypeDef *) self->private_data)->can_tx_topic);
}

void Cybergear_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    return;
}

void Cybergear_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
    Cybergear_Trigger(self);
}

void Cybergear_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
    Cybergear_Trigger(self);
}

void Cybergear_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
    Cybergear_Trigger(self);
}

void Cybergear_Reset_t(struct INTF_Motor_Handle *self) {
    Cybergear_Reset(self->motor_id, Master_CAN_ID, ((Cybergear_ResDataTypeDef *)self->private_data)->can_tx_topic);
}

void Cybergear_Enable_t(struct INTF_Motor_Handle *self) {
    Cybergear_Enable(self->motor_id, Master_CAN_ID, ((Cybergear_ResDataTypeDef *) self->private_data)->can_tx_topic);
}

void Cybergear_Disable_t(struct INTF_Motor_Handle *self) {
    //TODO: 补全disable
    return;
}


void Cybergear_MainLoop() {
    osDelay(1000); //等待小米电机启动
    for (int i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
        Cybergear_Enable(g_cybergear_motors[i]->motor_id, Master_CAN_ID,
                         ((Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data)->can_tx_topic);
    }
    osDelay(1000);
    while (1) {
        for (int i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
            INTF_Motor_HandleTypeDef *m = g_cybergear_motors[i];
            Cybergear_ResDataTypeDef *priv = (Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data;
            if(xTaskGetTickCount() - priv->last_update_tick>1000&&!priv->has_disconnected){
              priv->has_disconnected=1;
            }
            if(priv->has_disconnected&&xTaskGetTickCount() - priv->last_update_tick<100){
              Cybergear_Enable(g_cybergear_motors[i]->motor_id, Master_CAN_ID,
                               ((Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data)->can_tx_topic);
              priv->has_disconnected=0;
            }
            float t_torque=0,t_speed=0,t_angle=0;
            switch (m->motor_mode) {
              case MOTOR_MODE_ANGLE:
                t_angle=m->target_angle;
                break;
              case MOTOR_MODE_SPEED:
                t_speed=m->target_speed;
                break;
              case MOTOR_MODE_TORQUE:
                t_torque=m->target_torque;
                break;
              case MOTOR_MODE_MIT:
                t_angle=m->target_angle;
                t_speed=m->target_speed;
                t_torque=m->target_torque;
                break;
              }
            Cybergear_ControlMode(m->motor_id, t_torque * m->direction,
                                  (t_angle - priv->angle_offset) * m->direction,//修正offset后的角度值
                                  t_speed * m->direction,
                                  priv->kp,
                                  priv->kd,
                                  priv->can_tx_topic);
        }
        osDelay(1);
    }
}

void Cybergear_CAN_CallBack(void *message, BusTopicHandle_t topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *) message;

    if (msg->id_type == BSP_CAN_ID_STD || msg->can_id < 0x2000000 || msg->can_id > 0x3000000) {
        return;
    }

    struct exCanIdInfo *info = (struct exCanIdInfo *) &msg->can_id;

    uint8_t id = info->data & 0x7F;
    INTF_Motor_HandleTypeDef *m = NULL;
    int i;
    for (i = 0; i < CYBERGEAR_MOTOR_NUM; i++) {
        if (g_cybergear_motors[i]->motor_id == id &&
            ((Cybergear_ResDataTypeDef*)g_cybergear_motors[i]->private_data)->can_rx_topic->pxTopic == topic) {

            m = g_cybergear_motors[i];
            break;
        }

        //未匹配，直接返回
        if (i == CYBERGEAR_MOTOR_NUM - 1) {
            return;
        }
    }

    if (m->motor_state == MOTOR_STATE_INIT) {
        //电机初始化角度归零
        ((Cybergear_ResDataTypeDef *) m->private_data)->angle_offset +=
                -uint_to_float((msg->data[0] << 8) + msg->data[1], A_MIN, A_MAX, 16) * m->direction;
        m->motor_state = MOTOR_STATE_RUNNING;
    }
    ((Cybergear_ResDataTypeDef *) g_cybergear_motors[i]->private_data)->last_update_tick=xTaskGetTickCount();
    m->update_time=HAL_GetTick();
    m->real_angle = uint_to_float((msg->data[0] << 8) + msg->data[1], A_MIN, A_MAX, 16) * m->direction +
                    ((Cybergear_ResDataTypeDef *) m->private_data)->angle_offset;
    m->real_speed = uint_to_float((msg->data[2] << 8) + msg->data[3], V_MIN, V_MAX, 16) * m->direction;
    m->real_torque = uint_to_float((msg->data[4] << 8) + msg->data[5], T_MIN, T_MAX, 16) * m->direction;
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


