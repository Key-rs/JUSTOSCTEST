#include "CanBridge.h"
#include <stdio.h>
#include "HelmChassis.h"   // 底盘控制头文件（定义底盘结构体和接口）
#include "HelmMotor.h"     // 舵机电机控制头文件（定义电机控制接口）
#include "motor_manager_config.h"  // 电机管理配置头文件
#include "user_lib.h"      // 用户自定义库头文件

// 全局变量定义
helm_chassis_solve_param_t param;  // 底盘解算参数结构体（传递控制输入与反馈）
uint32_t last_enable_tick = 0;     // 最后一次启用电机的时间戳（用于定期启用）
uint32_t last_msg_tick = 0;        // 最后一次接收CAN消息的时间戳（用于判断通信状态）

/**
 * @brief CAN消息接收回调函数（处理CAN1/CAN2的消息）
 * @param message CAN消息指针（类型为INTF_CAN_MessageTypeDef）
 * @param subscriber 订阅者句柄（用于区分不同CAN总线的订阅主题）
 * 功能：
 *       - CAN1（主题"CAN1/RX"）：解析控制指令（速度/偏航角），更新底盘解算参数并调用底盘控制函数
 *       - CAN2（主题"CAN2/RX"）：转发消息至DM电机专用回调函数（处理电机状态反馈）
 */
void CanBridge_CAN_CallBack(void *message, BusSubscriberHandle_t subscriber) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;  // 转换消息指针类型

    // 仅处理标准ID且ID为0x21的CAN消息（过滤无效消息）
    if (msg->id_type != BSP_CAN_ID_STD || msg->can_id != 0x21) {
        return;
    }
    last_msg_tick = HAL_GetTick();  // 更新最后消息时间戳

    uint8_t id = msg->can_id;       // 提取CAN ID（此处固定为0x21）
    uint8_t data[8] = {0};          // 消息数据缓冲区
    for (int i = 0; i < 8; i++) {   // 复制消息数据到本地缓冲区
        data[i] = msg->data[i];
    }

    // 根据订阅主题区分CAN1和CAN2的处理逻辑
    if (strcmp(subscriber->pxTopic->pcName, "/CAN1/RX") == 0) {  // CAN1：控制指令处理
        // 解析原始整数值（速度/偏航角的16位/15位编码值）
        uint16_t x_speed_int = (uint16_t)(((data[0]) << 8) | (data[1]));  // X方向速度原始值
        uint16_t y_speed_int = (uint16_t)(((data[2]) << 8) | (data[3]));  // Y方向速度原始值
        uint16_t z_speed_int = (uint16_t)(((data[4]) << 8) | (data[5]));  // Z方向速度原始值
        uint16_t yaw_int = (uint16_t)(((data[6] & 0b01111111) << 8) | (data[7]));  // 偏航角原始值（15位）

        // 转换为实际物理值（使用DM_uint_to_float解包函数）
        float x_speed = DM_uint_to_float(x_speed_int, -20, 20, 16);  // X速度（m/s）
        float y_speed = DM_uint_to_float(y_speed_int, -20, 20, 16);  // Y速度（m/s）
        float z_speed = DM_uint_to_float(z_speed_int, -20, 20, 16);  // Z速度（rad/s）
        float yaw = DM_uint_to_float(yaw_int, -20, 20, 15);          // 偏航角（rad）

        // 小量滤波（避免微小抖动）
        if (fabsf(x_speed) < 0.01f) x_speed = 0;
        if (fabsf(y_speed) < 0.01f) y_speed = 0;
        if (fabsf(z_speed) < 0.01f) z_speed = 0;
        if (fabsf(yaw) < 0.01f) yaw = 0;

        // 更新底盘解算参数
        param.cor = (data[6] >> 7 == 1) ? GLOBAL_COORDINATE : CHASSIS_COORDINATE;  // 坐标系选择（最高位）
        param.yaw = yaw;                  // 目标偏航角
        param.vx_input = x_speed;         // X方向输入速度
        param.vy_input = y_speed;         // Y方向输入速度
        param.vw_input = z_speed;         // 旋转输入速度

        Helm_Chassis_Ctrl(&param, &helmChassis);  // 调用底盘控制函数（解算车轮目标值）

        // 同步底盘解算后的车轮目标角度到反馈字段（用于其他模块访问）
        param.lf_wheel_rad_actual = helmChassis.lf_wheel.rad_target;  // 左前轮目标角度
        param.rf_wheel_rad_actual = helmChassis.rf_wheel.rad_target;  // 右前轮目标角度
        param.lb_wheel_rad_actual = helmChassis.lb_wheel.rad_target;  // 左后轮目标角度
        param.rb_wheel_rad_actual = helmChassis.rb_wheel.rad_target;  // 右后轮目标角度

    } else if (strcmp(subscriber->pxTopic->pcName, "/CAN2/RX") == 0) {  // CAN2：DM电机反馈处理
        extern void DM_Motor_CAN_CallBack(INTF_CAN_MessageTypeDef *header, uint8_t *data);  // 声明外部电机回调
        DM_Motor_CAN_CallBack(msg, data);  // 转发消息至电机状态处理函数
    }
}

/**
 * @brief 应用电机控制参数（预留函数，当前未实现）
 * @param manager 电机管理句柄（包含电机控制参数）
 * 说明：用于将管理参数（如PID参数）同步到电机私有数据中，当前为空需补充实现。
 */
static void apply_control(ITNF_ManagerdMotor_HandleTypedef *manager) {
    INTF_Motor_HandleTypeDef *m = manager->motor;  // 获取电机句柄
    CanBridge_ResDataTypeDef *priv = m->private_data;  // 获取电机私有数据

    // 示例逻辑（需根据实际需求补充）：
    // priv->angle_pid.Kp = manager->pid_parms.angle_kp;
    // priv->speed_pid.Kp = manager->pid_parms.velocity_kp;
}

/**
 * @brief 打印电机参数（预留函数，当前被注释）
 * @param manager 电机管理句柄
 * @param buff 输出缓冲区
 * @param len 缓冲区长度
 * 说明：用于将电机PID参数等信息格式化输出到缓冲区，当前注释未启用。
 */
static void print_info(ITNF_ManagerdMotor_HandleTypedef *manager, char *buff, uint16_t len) {
    INTF_Motor_HandleTypeDef *m = manager->motor;  // 获取电机句柄
    CanBridge_ResDataTypeDef *priv = m->private_data;  // 获取电机私有数据

    // 示例逻辑（当前被注释）：
    // snprintf(buff, len, "Angle: kp=%f ki=%f kd=%f\n\rSpeed: kp=%f ki=%f kd=%f\n\r",
    //          priv->angle_pid.Kp, priv->angle_pid.Ki, priv->angle_pid.Kd,
    //          priv->speed_pid.Kp, priv->speed_pid.Ki, priv->speed_pid.Kd);
}

/**
 * @brief 注册底盘控制句柄（初始化底盘模块）
 * @param config 底盘配置参数（包含电机名称、CAN主题等）
 * @return 底盘控制句柄指针（用于后续控制）
 * 功能：
 *       - 分配底盘句柄内存并初始化默认参数
 *       - 分配私有数据内存并初始化
 *       - 订阅CAN接收主题并注册CAN发送主题
 *       - 绑定速度设置函数到电机控制接口
 */
INTF_Helm_Chassis_HandleTypeDef *CanBridge_Register(INTF_Helm_Chassis_HandleTypeDef *config) {
    // 分配底盘句柄内存（通过共享指针获取，确保唯一性）
    INTF_Helm_Chassis_HandleTypeDef *chass = pvSharePtr(config->chassis_ptr_name, sizeof(INTF_Helm_Chassis_HandleTypeDef));

    // 初始化底盘目标速度（X/Y/旋转方向）
    chass->target_speed_w = 0.0f;
    chass->target_speed_x = 0.0f;
    chass->target_speed_y = 0.0f;

    // 分配并初始化私有数据内存（存储CAN主题等信息）
    chass->private_data = JUST_MALLOC(sizeof(CanBridge_ResDataTypeDef));
    memset(chass->private_data, 0, sizeof(CanBridge_ResDataTypeDef));
    CanBridge_ResDataTypeDef *priv = chass->private_data;

    // 订阅CAN接收主题（绑定当前回调函数）
    priv->can_rx_topic = xBusSubscribeFromName(config->can_rx_topic_name, CanBridge_CAN_CallBack);
    priv->can_trans_rx_topic=xBusSubscribeFromName(config->can_trans_rx_topic_name, CanBridge_CAN_CallBack);
    // 注册CAN发送主题（用于后续发送控制消息）
    priv->can_tx_topic = xBusTopicRegister(config->can_tx_topic_name);
    priv->can_trans_tx_topic=xBusTopicRegister(config->can_trans_tx_topic_name);


    // 绑定速度设置函数到电机控制接口（使用HelmMotor模块的SetSpeed函数）

    return chass;
}

/**
 * @brief 电机在线检测函数（定期检查并重新启用离线电机）
 * 功能：遍历偶数ID电机（0,2,4,6），检查其最后更新时间和状态：
 *       - 若离线超1秒或未启用，尝试重新启用该电机及其下一个ID电机
 */
void Online_detect() {
    uint32_t now_tick = HAL_GetTick();  // 当前时间戳
    for (int i = 0; i < 8; i += 2) {    // 遍历偶数ID（0,2,4,6）
        volatile uint32_t offline_tick = now_tick - DM_Motors[i].last_update_time;  // 计算离线时间
        if (offline_tick > 1000 || DM_Motors[i].Status != DM_Enable) {  // 离线或未启用
            vTaskDelay(10);  // 延时确保命令发送间隔
            HelmMotor_Enable(i);  // 启用当前ID电机
            vTaskDelay(10);
            HelmMotor_Enable(i + 1);  // 启用下一个ID电机（i+1）
        }
    }
}

/**
 * @brief 底盘主控制循环（周期性执行电机控制）
 * 功能：
 *       - 通信正常时：根据底盘解算结果设置转向电机角度和驱动电机速度
 *       - 通信超时（离线）时：停止所有驱动电机
 *       - 定期（每3秒）重新启用所有电机（防止意外禁用）
 */
void CanBridge_MainLoop() {
    if (HAL_GetTick() - last_msg_tick > 1000) {  // 通信超时（离线状态）
        // 停止所有驱动电机（ID2/4/6/8）
        HelmMotor_SetSpeed(2, 0); vTaskDelay(5);
        HelmMotor_SetSpeed(4, 0); vTaskDelay(5);
        HelmMotor_SetSpeed(6, 0); vTaskDelay(5);
        HelmMotor_SetSpeed(8, 0); vTaskDelay(5);
    } else {  // 通信正常（在线状态）
        // 设置转向电机目标角度（ID1/3/5/7，角度取反适配机械安装方向）
        HelmMotor_SetAngle(1, -helmChassis.lf_wheel.rad_target, 10); vTaskDelay(5);  // 左前轮
        HelmMotor_SetAngle(7, -helmChassis.rf_wheel.rad_target, 10); vTaskDelay(5);  // 右前轮
        HelmMotor_SetAngle(5, -helmChassis.rb_wheel.rad_target, 10); vTaskDelay(5);  // 右后轮
        HelmMotor_SetAngle(3, -helmChassis.lb_wheel.rad_target, 10); vTaskDelay(5);  // 左后轮

        // 设置驱动电机目标速度（速度=目标线速度/车轮半径，转换为电机转速）
        HelmMotor_SetSpeed(2, helmChassis.lf_wheel.v_target / WHEEL_RADIUS); vTaskDelay(5);  // 左前轮
        HelmMotor_SetSpeed(8, helmChassis.rf_wheel.v_target / WHEEL_RADIUS); vTaskDelay(5);  // 右前轮
        HelmMotor_SetSpeed(6, helmChassis.rb_wheel.v_target / WHEEL_RADIUS); vTaskDelay(5);  // 右后轮
        HelmMotor_SetSpeed(4, helmChassis.lb_wheel.v_target / WHEEL_RADIUS); vTaskDelay(5);  // 左后轮
    }

    // 定期（每3秒）重新启用所有电机（防止意外禁用）
    if (HAL_GetTick() - last_enable_tick > 3000) {
        for (int i = 1; i <= 8; ++i) {
            HelmMotor_Enable(i);       // 启用ID1~8电机
            vTaskDelay(10);             // 确保命令间隔（避免总线拥堵）
        }
        last_enable_tick = HAL_GetTick();  // 更新最后启用时间戳
    }

    vTaskDelay(10);  // 主循环延时（控制执行频率）
}

/**
 * @brief 底盘模块初始化函数（启动主控制循环任务）
 * @param chass 底盘控制句柄
 * 功能：
 *       - 初始化私有数据（可选）
 *       - 创建FreeRTOS任务，启动CanBridge_MainLoop主循环
 */
void CanBridge_Init() {
    // CanBridge_ResDataTypeDef *priv = chass->private_data;  // 获取私有数据（当前未使用）

    // 创建FreeRTOS任务（任务名、栈大小、参数、优先级、任务句柄）
    xTaskCreate(CanBridge_MainLoop, "CanBridge_MainLoop", 1024, NULL, 1, NULL);
}

void System_Init() {
    // 1. 定义底盘配置参数（需根据实际CAN主题名称、指针名称等填写）
    INTF_Helm_Chassis_HandleTypeDef chassis_config = {
        .chassis_ptr_name = "MainChassis",  // 底盘共享指针名称（需全局唯一）
        .can_rx_topic_name = "/CAN1/RX",     // CAN接收主题名称（与CanBridge_CAN_CallBack中处理的主题一致）
        .can_trans_rx_topic_name = "/CAN2/RX", // 转发CAN接收主题名称
        .can_tx_topic_name = "/CAN1/TX",     // CAN发送主题名称（用于发送控制消息）
        .can_trans_tx_topic_name = "/CAN2/TX" // 转发CAN发送主题名称
    };
    CanBridge_Register(&chassis_config);
    // 2. 调用注册函数获取底盘句柄
    // INTF_Helm_Chassis_HandleTypeDef *main_chassis = CanBridge_Register(&chassis_config);

    // 3. 启动底盘主循环（通过CanBridge_Init函数）
    CanBridge_Init();  // 该函数会创建FreeRTOS任务运行CanBridge_MainLoop
}