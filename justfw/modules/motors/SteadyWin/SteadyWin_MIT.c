#include "SteadyWin_MIT.h"

#define STEADY_MASTER_ID 0x00
#include "motor_manager_config.h"
#include "user_lib.h"

static uint8_t count;
static INTF_Motor_HandleTypeDef* motors[STEADYWIN_MOTOR_MAX_NUM] = {0};

#ifdef STEADYWIN_CAN_USE_MANAGER
#include <stdio.h>

static void apply_control(ITNF_ManagerdMotor_HandleTypedef* manager) {
    INTF_Motor_HandleTypeDef* m = manager->motor;
    SteadyWin_MIT_ResDataTypedef* priv = m->private_data;
    priv->kd = manager->mit_parms.kd;
    priv->kp = manager->mit_parms.kp;
}

static void print_info(ITNF_ManagerdMotor_HandleTypedef* manager, char* buff, uint16_t len) {
    INTF_Motor_HandleTypeDef* m = manager->motor;
    SteadyWin_MIT_ResDataTypedef* priv = m->private_data;

    snprintf(buff, len, "ErrorCode:%d\n\rParms kp:%f,kd:%f\n\r", priv->err, priv->kp, priv->kd);
}
#endif

static void motor_can_msg_init(INTF_Motor_HandleTypeDef* self, INTF_CAN_MessageTypeDef* msg) {
    msg->can_id = self->motor_id;
    msg->id_type = BSP_CAN_ID_STD;
    msg->rtr_type = BSP_CAN_RTR_DATA;
    memset(msg->data, 0xFF, 8);
}

static void motor_get_err(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_GET_ERR;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    msg.data[6] = 0x00;
    msg.data[7] = 0xFB;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_clear_err(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_GET_ERR;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    // msg.data[6] = 0xFF;
    msg.data[7] = 0xFB;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_enable(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFC;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_disable(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFD;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_set_zero(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    msg.data[7] = 0xFE;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_send_mit(INTF_Motor_HandleTypeDef* self) {
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    priv->last_command = STEADYWIN_MIT_COMMAND_CONTRO;
    INTF_CAN_MessageTypeDef msg;
    motor_can_msg_init(self, &msg);

    float angle_mapped = loop_float_constrain(self->target_angle, -12.5, 12.5);
    uint16_t position = (angle_mapped + 12.5) * 65535 / 25;
    float speed = float_constrain(self->target_speed, -65, 65);
    uint16_t int_speed = (speed + 65) * 4095 / 130;
    uint16_t kp = priv->kp * 4095 / 500;
    uint16_t kd = priv->kd * 4095 / 500;
    uint16_t torque = (self->target_torque + 255 * priv->torque_constant * priv->gear_rate) * 4095 / (450 * priv->torque_constant * priv->gear_rate);

    msg.data[0] = position >> 8;    // 高八位
    msg.data[1] = position & 0xFF;  // 低八位
    msg.data[2] = int_speed >> 4;
    msg.data[3] = (kp >> 8 & 0x0F) | (int_speed & 0x0F) << 4;
    msg.data[4] = kp & 0xFF;
    msg.data[5] = kd >> 4;
    msg.data[6] = ((kd << 4) & 0xF0) | ((torque >> 8) & 0x0F);
    msg.data[7] = torque & 0xFF;

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_set_angle(INTF_Motor_HandleTypeDef* self, float angle) {
    self->target_angle = angle;
}

static void motor_set_speed(INTF_Motor_HandleTypeDef* self, float speed) {
    self->target_speed = speed;
}

static void motor_set_torque(INTF_Motor_HandleTypeDef* self, float torque) {
    self->target_torque = torque;
}

static void motor_can_cb(void* message, BusSubscriberHandle_t subscriber) {
    INTF_CAN_MessageTypeDef* msg = (INTF_CAN_MessageTypeDef*)message;

    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id != STEADY_MASTER_ID) {
        return;
    }

    INTF_Motor_HandleTypeDef* m = (INTF_Motor_HandleTypeDef*)subscriber->context;
    SteadyWin_MIT_ResDataTypedef* priv = NULL;
    uint8_t id = msg->data[0];

    if (m->motor_id != id)
        return;

    priv = m->private_data;
    m->motor_state = MOTOR_STATE_RUNNING;

    switch (priv->last_command) {
    case STEADYWIN_MIT_COMMAND_CLEAR_ERR:
        priv->err = STEADYWIN_MIT_ERR_NONE;
        break;

    case STEADYWIN_MIT_COMMAND_GET_ERR:
        priv->err = msg->data[1];
        break;

    default:
        {
            uint16_t position = msg->data[1] << 8 | msg->data[2];
            uint16_t speed = msg->data[3] << 8 | (msg->data[4] & 0xF0) >> 4;
            uint16_t torque = (msg->data[4] & 0xF) << 8 | msg->data[5];
    
            m->real_angle = position * 25.0f / 65535.0f - 12.5;
            m->real_speed = speed * 130.0f / 4095.0f - 65;
            m->real_torque = torque * (450 * priv->torque_constant * priv->gear_rate) / 4095 - (225 * priv->torque_constant * priv->gear_rate);
    
            break;
        }
    }

    priv->recived = true;
}

static void motor_mainloop() {
    while (1) {
        for (uint8_t i = 0; i < count; i++) {
            INTF_Motor_HandleTypeDef* m = motors[i];
            SteadyWin_MIT_ResDataTypedef* priv = (SteadyWin_MIT_ResDataTypedef*)m->private_data;

            switch (m->motor_state) {
            case MOTOR_STATE_INIT:
                motor_enable(m);
                priv->last_command = STEADYWIN_MIT_COMMAND_ENABLE;

                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            case MOTOR_STATE_RUNNING:
            case MOTOR_STATE_ERROR:
                motor_get_err(m);
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
                    motor_disable(m);
                    // 严重问题，需要停机
                    m->motor_state = MOTOR_STATE_ERROR;
                    continue;

                case STEADYWIN_MIT_ERR_OVERCURRENT:
                case STEADYWIN_MIT_ERR_START_FAIL:
                case STEADYWIN_MIT_ERR_SOFTWARE:
                    //  一般性问题，清空问题后继续运行
                    priv->last_command = STEADYWIN_MIT_COMMAND_CLEAR_ERR;
                    motor_clear_err(m);
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

                motor_send_mit(m);
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

static void _motor_handle_init(INTF_Motor_HandleTypeDef* self, SteadyWin_MIT_ConfigTyepdef* config) {
    if (count >= STEADYWIN_MOTOR_MAX_NUM) {
        return;
    }

    motors[count++] = self;

    self->motor_id = config->motor_id;
    self->motor_mode = MOTOR_MODE_MIT;
    self->motor_state = MOTOR_STATE_INIT;
    self->target_speed = 0.0f;
    self->real_speed = 0.0f;
    self->target_angle = 0.0f;
    self->real_angle = 0.0f;
    self->target_torque = 0.0f;
    self->real_torque = 0.0f;
    self->angle_offset = config->angle_offset;
    self->direction = config->direction;

    self->set_angle = motor_set_angle;
    self->set_mode = NULL;

    self->set_speed = motor_set_speed;
    self->set_torque = motor_set_torque;
    self->enable = motor_enable;
    self->disable = motor_disable;
    self->reset = motor_set_zero;  // 将当前位置设置为零点

    SteadyWin_MIT_ResDataTypedef* prid = JUST_MALLOC(sizeof(SteadyWin_MIT_ResDataTypedef));
    memset(prid, 0, sizeof(SteadyWin_MIT_ResDataTypedef));

    prid->can_rx_topic = xBusSubscribeFromName(config->can_rx_topic_name, motor_can_cb);
    prid->can_rx_topic->context = (void*)self;
    prid->can_tx_topic = xBusTopicRegister(config->can_tx_topic_name);

    prid->kd = config->kd;
    prid->kp = config->kp;
    prid->last_command = STEADYWIN_MIT_COMMAND_NONE;

    self->private_data = prid;

#ifdef STEADYWIN_CAN_USE_MANAGER
#include "motor_manager.h"
    ITNF_ManagerdMotor_HandleTypedef* managed_motor = ManagedMotor_Create(self);
    managed_motor->motor_name = config->motor_name;
    managed_motor->mit_parms.kd = config->kd;
    managed_motor->mit_parms.kp = config->kp;
    managed_motor->ApplyControl = apply_control;
    managed_motor->MotorInfo = print_info;
#endif
}

INTF_Motor_HandleTypeDef* GIM3505_8_Register(SteadyWin_MIT_ConfigTyepdef* config) {
    INTF_Motor_HandleTypeDef* self = pvSharePtr(config->motor_name, sizeof(INTF_Motor_HandleTypeDef));
    if (self == NULL)
        return NULL;
    _motor_handle_init(self, config);
    SteadyWin_MIT_ResDataTypedef* priv = self->private_data;
    if (priv == NULL) {
        JUST_FREE(self);
        return NULL;
    }

    // 通过这里可以设置思泰威不同型号的电机
    priv->gear_rate = 8;
    priv->torque_constant = 0.52;

    return self;
}

void SteadyWinMIT_Init() {
    (void)motor_set_zero;  // 暂时没有使用的函数
    xTaskCreate(motor_mainloop, "SteadyWinMotor", 256, NULL, 250, NULL);
}
