#include "Steadywin_MIT.h"
#define STEADY_MASTER_ID 0x00
#include "user_lib.h"

static uint8_t count;
static INTF_Motor_HandleTypeDef* motors[STEADYWIN_MOTOR_MAX_NUM] = {0};

static void _motor_can_msg_init(INTF_Motor_HandleTypeDef* self, INTF_CAN_MessageTypeDef* msg) {
    msg->can_id = self->motor_id;
    msg->id_type = BSP_CAN_ID_STD;
    memset(msg->data, 0xFF, 8);
}

static void _motor_get_err(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_GET_ERR;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    msg.data[6] = 0x00;
    msg.data[7] = 0xFB;

    Bus_Publish(priv->can_tx_topic, &msg);
}

static void _motor_clear_err(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_GET_ERR;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    // msg.data[6] = 0xFF;
    msg.data[7] = 0xFB;

    Bus_Publish(priv->can_tx_topic, &msg);
}

static void _motor_enable(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFC;

    Bus_Publish(priv->can_tx_topic, &msg);
}

static void _motor_disable(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFD;

    Bus_Publish(priv->can_tx_topic, &msg);
}

static void _motor_set_zero(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFE;

    Bus_Publish(priv->can_tx_topic, &msg);
}

static void _motor_send_mit(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_CONTRO;
    INTF_CAN_MessageTypeDef msg;
    _motor_can_msg_init(self, &msg);

    float angle_mapped = loop_float_constrain(self->real_angle, -12.5, 12.5);
    uint16_t position = (angle_mapped + 12.5) * 65535 / 25;
    // memcpy(msg.data, &position, 2);
    // msg.data[0] = position;
    // msg.data[1] = position >> 8;
    float speed = float_constrain(self->target_speed, -32.5f, 32.5f);
    uint16_t int_speed = (self->target_speed + 65) * 4095 / 130;
    uint16_t kp = priv->kp * 4095 / 500;
    uint16_t kd = priv->kd * 4095 / 500;
    uint16_t torque = self->target_torque;

    msg.data[0] = position >> 8;  // 高八位
    msg.data[1] = position;       // 低八位
    msg.data[2] = int_speed >> 8;
}

static void _motor_set_angle(INTF_Motor_HandleTypeDef* self, float angle) {
    self->target_angle = angle;
}

static void _motor_set_speed(INTF_Motor_HandleTypeDef* self, float speed) {
    self->target_speed = speed;
}

static void _motor_set_torque(INTF_Motor_HandleTypeDef* self, float torque) {
    self->target_torque = torque;
}

static void _motor_can_cb(void* message, Bus_TopicHandleTypeDef* topic) {
    INTF_CAN_MessageTypeDef* msg = (INTF_CAN_MessageTypeDef*)message;

    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id != STEADY_MASTER_ID) {
        return;
    }

    INTF_Motor_HandleTypeDef* m = NULL;
    SteadyWin_MIT_ResDataTypedef* priv = NULL;
    uint8_t id = msg->data[0];

    // 匹配电机
    for (uint8_t i = 0; i < count; i++) {
        if (motors[i]->motor_id == id) {
            m = motors[i];
        }
    }

    if (m == NULL) {
        return;
    }

    priv = m->private_data;

    switch (priv->last_command) {
    case STEADYWIN_MIT_COMMAND_ENABLE:
        /* code */
        break;

    default:
        break;
    }
}

static void _motor_mainloop() {
    while (1) {
        for (uint8_t i = 0; i < count; i++) {
            INTF_Motor_HandleTypeDef* m = motors[i];
            SteadyWin_MIT_ResDataTypedef* priv = (SteadyWin_MIT_ResDataTypedef*)m->private_data;

            switch (m->motor_state) {
            case MOTOR_STATE_INIT:
                _motor_enable(m);
                priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;

                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            case MOTOR_STATE_RUNNING:
            case MOTOR_STATE_ERROR:
                _motor_get_err(m);
                priv->last_command = STEADYWIN_MIT_COMMAND_GET_ERR;
                vTaskDelay(pdMS_TO_TICKS(20));

                if (priv->recived == false) {
                    m->motor_state = MOTOR_STATE_INIT;
                    continue;
                }
                priv->recived = false;

                switch (priv->err) {
                case STEADYWIN_MIT_ERR_OVERVOLTAGE:
                case STEADYWIN_MIT_ERR_UNDERVOLTAGE:
                case STEADYWIN_MIT_ERR_OVERTEMP:
                case STEADYWIN_MIT_ERR_FOC_FREQ_HIGH:
                    _motor_disable(m);
                    // 严重问题，需要停机
                    m->motor_state = MOTOR_STATE_ERROR;
                    continue;

                case STEADYWIN_MIT_ERR_OVERCURRENT:
                case STEADYWIN_MIT_ERR_START_FAIL:
                case STEADYWIN_MIT_ERR_SOFTWARE:
                    //  一般性问题，清空问题后继续运行
                    priv->last_command = STEADYWIN_MIT_COMMAND_CLEAR_ERR;
                    _motor_clear_err(m);
                    vTaskDelay(pdMS_TO_TICKS(20));
                    if (priv->recived == false) {
                        m->motor_state = MOTOR_STATE_INIT;
                        continue;
                    }
                    priv->recived = false;

                    break;
                default:
                    break;
                }  // STEADYWIN_MIT_ERR

                _motor_send_mit(m);
                priv->last_command = STEADYWIN_MIT_COMMAND_CONTRO;
                vTaskDelay(pdMS_TO_TICKS(20));

                if (priv->recived == false) {
                    m->motor_state = MOTOR_STATE_INIT;
                    continue;
                }
                priv->recived = false;

                break;
            }  // MOTOR_STATUS
        }

        vTaskDelay(1);
    }
}

INTF_Motor_HandleTypeDef* SteadyWin_Register(SteadyWin_MIT_ConfigTyepdef* config) {
    if (count >= STEADYWIN_MOTOR_MAX_NUM) {
        return NULL;
    }

    INTF_Motor_HandleTypeDef* motor = Bus_SharePtr(config->motor_name, sizeof(INTF_Motor_HandleTypeDef));
    motors[count++] = motor;

    motor->motor_id = config->motor_id;
    motor->motor_mode = MOTOR_MODE_MIT;
    motor->motor_state = MOTOR_STATE_INIT;
    motor->target_speed = 0.0f;
    motor->real_speed = 0.0f;
    motor->target_angle = 0.0f;
    motor->real_angle = 0.0f;
    motor->target_torque = 0.0f;
    motor->real_torque = 0.0f;
    motor->angle_offset = config->angle_offset;
    motor->direction = config->direction;

    motor->set_angle = _motor_set_angle;
    motor->set_mode = NULL;
    motor->set_speed = _motor_set_speed;
    motor->set_torque = _motor_set_torque;

    SteadyWin_MIT_ResDataTypedef* prid = JUST_MALLOC(sizeof(SteadyWin_MIT_ResDataTypedef));
    memset(prid, 0, sizeof(SteadyWin_MIT_ResDataTypedef));

    prid->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name, _motor_can_cb);
    prid->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);

    prid->kd = config->kd;
    prid->kp = config->kp;
    prid->last_command = STEADYWIN_MIT_COMMAND_NONE;

    motor->private_data = prid;

    return motor;
}

// void SteadyWin_Init() {
//     osThreadDef(SteadyWin_MIT_MainLoopTask, _motor_mainloop, osPriorityLow, 0, 256);
//     SteadyWin_MIT_MainLoopHandle = osThreadCreate(osThread(SteadyWin_MIT_MainLoopTask), NULL);
// }