#include "Odrive_CAN.h"

// #include "interface.h"
#include "list.h"
#include "motor_manager_config.h"
#include "user_lib.h"
static List_t motors;

// static void context;

static void motor_clear_err(INTF_Motor_HandleTypeDef *self) {
    INTF_CAN_MessageTypeDef msg;
    Odrive_CAN_ResDataTypedef *priv = self->private_data;
    msg.rtr_type = CAN_RTR_DATA;
    msg.id_type = CAN_ID_STD;
    msg.can_id = self->motor_id << 5 | 0x18;
    bzero(msg.data, 8);

    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_enable(INTF_Motor_HandleTypeDef *self) {
    motor_clear_err(self);
    INTF_CAN_MessageTypeDef msg;
    Odrive_CAN_ResDataTypedef *priv = self->private_data;
    msg.rtr_type = CAN_RTR_DATA;
    msg.id_type = CAN_ID_STD;
    msg.can_id = self->motor_id << 5 | 0x07;
    // self->motor_state = MOTOR_STATE_INIT;
    if (self->motor_state == MOTOR_STATE_DISABLE) {
        self->motor_state = MOTOR_STATE_INIT;
    }

    msg.data[0] = 0x08;  // 开启电机
    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_disable(INTF_Motor_HandleTypeDef *self) {
    self->motor_state = MOTOR_STATE_DISABLE;
    INTF_CAN_MessageTypeDef msg;
    Odrive_CAN_ResDataTypedef *priv = self->private_data;
    msg.rtr_type = CAN_RTR_DATA;
    msg.id_type = CAN_ID_STD;
    msg.can_id = self->motor_id << 5 | 0x07;
    self->motor_state = MOTOR_STATE_DISABLE;
    msg.data[0] = 0x01;  // 关闭电机
    vBusPublish(priv->can_tx_topic, &msg);
}

static void motor_set_speed(INTF_Motor_HandleTypeDef *self, float speed) {
    self->target_speed = speed;
}

static void motor_set_angle(INTF_Motor_HandleTypeDef *self, float angle) {
    self->target_angle = angle;
}

static void motor_set_torque(INTF_Motor_HandleTypeDef *self, float torque) {
    self->target_torque = torque;
}

static void motor_send_mit(INTF_Motor_HandleTypeDef *self) {
    Odrive_CAN_ResDataTypedef *priv = self->private_data;
    INTF_CAN_MessageTypeDef msg;
    msg.can_id = self->motor_id << 5 | 0x008;
    msg.id_type = CAN_ID_STD;
    msg.rtr_type = CAN_RTR_DATA;

    float angle_mapped = float_constrain(self->target_angle, -12.5, 12.5) + self->angle_offset;
    uint16_t position = (angle_mapped + 12.5) * 65535 / 25;
    float speed = float_constrain(self->target_speed, -65, 65);
    uint16_t int_speed = (speed + 65) * 4095 / 130;
    uint16_t kp = priv->kp * 4095 / 500;
    uint16_t kd = priv->kd * 4095 / 5;

    float f_torque = float_constrain(self->target_torque, -50, 50);
    uint16_t torque = (f_torque + 50) * 4095 / 100;

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

void motor_send_mit_cmd(INTF_Motor_HandleTypeDef *self) {
    Odrive_CAN_ResDataTypedef *priv = self->private_data;
    INTF_CAN_MessageTypeDef msg;
    msg.can_id = self->motor_id << 5 | 0x00C;
    msg.id_type = CAN_ID_STD;
    msg.rtr_type = CAN_RTR_DATA;

    bzero(msg.data, 8);

    memcpy(&msg.data[0], &self->target_angle, 4);
    float speed = float_constrain(self->target_speed, -65, 65);
    uint16_t int_speed = (speed + 65) * 4095 / 130;
    memcpy(&msg.data[4], &int_speed, 2);

    float f_torque = float_constrain(self->target_torque, -50, 50);
    uint16_t torque = (f_torque + 50) * 4095 / 100;
    memcpy(&msg.data[6], &torque, 2);

    vBusPublish(priv->can_tx_topic, &msg);
}

static void can_callback(void *message, BusSubscriberHandle_t subscriber) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;

    INTF_Motor_HandleTypeDef *m = (INTF_Motor_HandleTypeDef *)subscriber->context;
    Odrive_CAN_ResDataTypedef *priv = (Odrive_CAN_ResDataTypedef *)m->private_data;
    if (m->motor_id != msg->can_id >> 5)
        return;

    switch (msg->can_id & 0x01F) {
    case 0x01:
        // 电机心跳包
        priv->axis_error = *(uint32_t *)(&msg->data[0]);
        priv->axis_status = *(uint8_t *)(&msg->data[4]);
        uint8_t flag = *(uint8_t *)(&msg->data[5]);
        priv->motor_error = (flag & 0x01) == 1;
        priv->encoder_error = ((flag >> 1) & 1) == 1;
        priv->controller_error = ((flag >> 2) & 1) == 1;
        priv->error = ((flag >> 3) & 1) == 1;
        priv->trajectory_done = ((flag >> 7) & 1) == 1;
        priv->life = msg->data[7];
        break;

    case 0x08:
        // MIT接收包
        uint16_t position = msg->data[1] << 8 | msg->data[2];
        uint16_t speed = msg->data[3] << 8 | (msg->data[4] & 0xF0) >> 4;
        uint16_t torque = (msg->data[4] & 0xF) << 8 | msg->data[5];

        m->real_angle = position * 25.0f / 65535.0f - 12.5;
        m->real_speed = speed * 130.0f / 4095.0f - 65;
        m->real_torque = torque * 100.0f / 4095.0f - 50;
        break;

        // case 0x09:
        //     // 电机位置包
        //     // m->real_angle = *(float *)(&msg->data[0]);
        //     // m->real_speed = *(float *)(&msg->data[4]);
        //     break;

    default:
        break;
    }
    if (m->motor_state == MOTOR_STATE_DISABLE) {
        return;
    }

    if (priv->axis_status == 0x08) {
        m->motor_state = MOTOR_STATE_RUNNING;
    } else {
        m->motor_state = MOTOR_STATE_INIT;
    }
}

static void odrive_mainLoop() {
    while (true) {
        ListItem_t *item = listGET_HEAD_ENTRY(&motors);
        while (item != listGET_END_MARKER(&motors)) {
            vTaskDelay(pdMS_TO_TICKS(20));

            INTF_Motor_HandleTypeDef *m = listGET_LIST_ITEM_OWNER(item);
            Odrive_CAN_ResDataTypedef *priv = m->private_data;

            // if (priv->error != false) {
            //     // 出现异常
            //     motor_disable(m);
            //     m->motor_state = MOTOR_STATE_ERROR;
            //     vTaskDelay(pdMS_TO_TICKS(10));
            //     motor_clear_err(m);
            // }

            // if (m->motor_state == MOTOR_STATE_INIT) {
            //     motor_clear_err(m);
            //     vTaskDelay(pdMS_TO_TICKS(10));
            //     motor_enable(m);
            //     // m->motor_state = MOTOR_STATE_RUNNING;
            // }

            // if (m->motor_state == MOTOR_STATE_RUNNING && priv->axis_status != 0x08) {
            //     m->motor_state = MOTOR_STATE_INIT;
            // }

            // if (m->motor_state == MOTOR_STATE_RUNNING) {
            //     // (void)motor_send_mit;   // 简直就是一坨
            //     // motor_send_mit_cmd(m);  // 无法直接设置Kp Ki Kd值，但是可以正常使用
            //     motor_send_mit(m);
            // }
            // if (m->motor_state == MOTOR_STATE_RUNNING && priv->axis_status != 0x08) {
            //     m->motor_state = MOTOR_STATE_INIT;
            // }

            // switch (m->motor_state) {
            // case MOTOR_STATE_INIT:
            //     motor_clear_err(m);
            //     vTaskDelay(10);
            //     motor_enable(m);
            //     break;

            // default:
            //     break;
            // }

            if (m->motor_state != MOTOR_STATE_DISABLE) {
                motor_send_mit(m);
            }

            item = listGET_NEXT(item);
            vTaskDelay(20);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#ifdef ODRIVE_USE_MANAGER

#include <stdio.h>
static void apply_control(ITNF_ManagerdMotor_HandleTypedef *manager) {
    INTF_Motor_HandleTypeDef *m = manager->motor;
    Odrive_CAN_ResDataTypedef *priv = m->private_data;
    priv->kd = manager->mit_parms.kd;
    priv->kp = manager->mit_parms.kp;
}

static void print_info(ITNF_ManagerdMotor_HandleTypedef *manager, char *buff, uint16_t len) {
    INTF_Motor_HandleTypeDef *m = manager->motor;
    Odrive_CAN_ResDataTypedef *priv = m->private_data;
    // printf("ErrorCode:%d\n\r", priv->axis_error);
    // printf("Parms kp:%f,kd:%f\n\r", priv->kp, priv->kd);

    snprintf(buff, len, "ErrorCode:%d\n\rParms kp:%f,kd:%f\n\r", priv->axis_error, priv->kp, priv->kd);
}

#endif

INTF_Motor_HandleTypeDef *Odrive_Register(Odrive_CAN_ConfigTypedef *config) {
    INTF_Motor_HandleTypeDef *m = pvSharePtr(config->motor_name, sizeof(INTF_Motor_HandleTypeDef));
    bzero(m, sizeof(INTF_Motor_HandleTypeDef));
    Odrive_CAN_ResDataTypedef *priv = JUST_MALLOC(sizeof(Odrive_CAN_ResDataTypedef));
    m->angle_offset = config->angle_offset;
    m->direction = config->direction;
    // m->enable = true;
    m->motor_state = MOTOR_STATE_INIT;
    m->motor_id = config->motor_id;
    m->private_data = priv;
    m->motor_mode = MOTOR_MODE_MIT;
    m->target_angle = 0;
    m->target_speed = 0;
    m->target_torque = 0;

    m->set_angle = motor_set_angle;
    m->set_speed = motor_set_speed;
    m->enable = motor_enable;
    m->disable = motor_disable;
    m->set_torque = motor_set_torque;

    priv->can_tx_topic = xBusTopicRegister(config->can_tx_topic_name);
    priv->can_rx_topic = xBusSubscribeFromName(config->can_rx_topic_name, can_callback);
    priv->can_rx_topic->context = (void *)m;
    priv->axis_error = ODRIVE_AXIS_ERROR_UNDEFINED;
    priv->axis_status = ODRIVE_AXIS_STATUS_UNDEFINED;
    priv->kd = config->kd;
    priv->kp = config->kp;
    priv->error = false;
    priv->axis_error = false;
    priv->encoder_error = false;
    priv->controller_error = false;

    ListItem_t *item = JUST_MALLOC(sizeof(ListItem_t));
    listSET_LIST_ITEM_OWNER(item, m);
    vListInsertEnd(&motors, item);

#ifdef ODRIVE_USE_MANAGER
#include "motor_manager.h"
    ITNF_ManagerdMotor_HandleTypedef *managed_motor = ManagedMotor_Create(m);
    managed_motor->motor_name = config->motor_name;
    managed_motor->ApplyControl = apply_control;
    managed_motor->MotorInfo = print_info;
    managed_motor->mit_parms.kd = config->kd;
    managed_motor->mit_parms.kp = config->kp;
#endif

    return m;
}

void Odrive_Init() {
    vListInitialise(&motors);
    xTaskCreate(odrive_mainLoop, "OdriveMotor", 256, NULL, 240, NULL);
}

void Odrive_DeInit() {
    Odrive_CAN_ConfigTypedef config = {
        .motor_id =1,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
        .kp = 0.01f,
        .kd = 0.001f,
        .motor_name = "test_motor"};
     Odrive_Register(&config);
    Odrive_Init();
}

