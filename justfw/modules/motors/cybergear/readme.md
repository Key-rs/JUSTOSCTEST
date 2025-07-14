# 小米电机CyberGear驱动模块

要注册一个电机，在Cybergear_Init()中添加相关代码即可
例如
```objectivec
//电机注册开始
//定义电机配置
Cybergear_ConfigTypeDef leglf_config = {
    .motor_id = 1,//电机ID，对应在上位机中设置的id
    .motor_ptr_name = "/motor/leg-l-f",//电机共享指针名
    .angle_offset = -OFFSET_ANGLE,//初始角度偏移(小米电机无绝对值编码器)
    .direction = 1.0f,//电机方向，参考`intf_motor.h`
    .motor_mode = MOTOR_MODE_TORQUE,//电机初始状态控制模式
    .can_rx_topic_name = "/CAN1/RX",//CAN接收话题名
    .can_tx_topic_name = "/CAN1/TX",//CAN发送话题名
    .kp = 1.0f,//运控模式kp,和角度控制、速度控制有关
    .kd = 0.5f,//运控模式kd,和角度控制、速度控制有关
};
//注册这个电机
Cybergear_Register(&leglf_config);
    
//电机注册结束
```

