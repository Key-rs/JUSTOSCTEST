#ifndef __BSP_REFEREE_H_
#define __BSP_REFEREE_H_

#include "main.h"
//------UI-----

//#define Robot_UI_ID UI_Data_RobotID_RStandard3   通过裁判系统直接读取当前ID，此句抛弃仅为理解
#define Robot_UI_ID g_referee->game_robot_state.robot_id
//#define Cilent_ID UI_Data_CilentID_RStandard3        //机器人角色设置

//----------协议部分------------------
#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)
#define __FALSE 100
/****************************开始标志*********************/
#define UI_SOF 0xa5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016a
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8

typedef enum {
    GAME_STATE_CMD_ID = 0x0001,  //比赛状态数据（包括比赛类型（0-3bit）当前比赛阶段（4-7bit）。。。）
    GAME_RESULT_CMD_ID = 0x0002,  //比赛结果
    GAME_ROBOT_HP_CMD_ID = 0x0003,  //机器人血量
//    DART_LAUNCH_STATUS_CMD_ID = 0x0004,  //飞镖发射状态，飞镖发射后发送
//    BONUS_PENALTY_STATES_CMD_ID = 0x0005,  //人工智能挑战赛加成与惩罚状态
    FIELD_EVENTS_CMD_ID = 0x0101,  //场地事件
//    SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,  //补给站动作标识数据
//    SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103,  //请求补弹
    REFEREE_WARNING_CMD_ID = 0x0104,  //裁判警告数据
    DART_LAUNCH_INFO_ID = 0x0105,  //飞镖发射倒计时
    ROBOT_STATE_CMD_ID = 0x0201,  //机器人状态
    POWER_HEAT_DATA_CMD_ID = 0x0202,  //实时功率热量数据
    ROBOT_POS_CMD_ID = 0x0203,  //机器人位置数据
    BUFF_MUSK_CMD_ID = 0x0204,  //机器人增益数据，改变后发送
//    DRONE_STATE_CMD_ID = 0x0205,  //空中机器人能量状态数据
    ROBOT_HURT_CMD_ID = 0x0206,  //伤害状态数据，伤害发生后发送
    SHOOT_DATA_CMD_ID = 0x0207,  //实时射击数据，子弹发送后发送
    BULLET_REMAINING_CMD_ID = 0x0208,  //子弹剩余发射数（空中机器人，哨兵机器人发送）
    RFID_STATE_CMD_ID = 0x0209,  //机器人RFID状态
    DART_CLIENT_CMD_ID = 0x020A,  //飞镖客户端指令
    ALL_ROBOT_POS_CMD_ID = 0x020B,  //(哨兵)己方地面机器人位置数据
    RADAR_PROGRESS_CMD_ID = 0x020C,  //(雷达)标记进度
    SENTRY_DECISION_CMD_ID = 0x020D,  //(哨兵)哨兵自主决策信息同步
    RADAR_DECISION_CMD_ID = 0x020E,  //(雷达)雷达自主决策信息同步
    STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,  //机器人交互数据
    DIY_CONTROLLER_CMD_ID = 0x0302,  //自定义控制器交互数据
    RECEIVE_CLIENT_MAP_CMD_ID = 0x0303,  //客户端小地图交互数据
    KEYBOARD_MOUSE_CMD_ID = 0x0304,  //键盘鼠标通过图传发送
    RADAR_SEND_CLIENT_MAP_CMD_ID = 0x0305,  //客户端小地图接收雷达数据
    DIY_CONTROLLER_SIMULATE_CMD_ID =  0x0306,  //自定义控制器模拟数据
    SENTRY_SEND_CLIENT_MAP_CMD_ID = 0x0307,  //(半自动兵种)客户端小地图接收数据
    SEND_CLIENT_MAP_CMD_ID = 0x0308,  //客户端小地图接收数据
    DIY_CONTROLLER2_CMD_ID = 0x0309,  //自定义控制器交互数据
} referee_cmd_id_e;

typedef struct {
    uint8_t SOF;  //帧头，默认为0XA5
    uint16_t data_length; //数据帧中data的长度
    uint8_t seq;//包序号
    uint8_t CRC8;//帧头crc8校验
} frame_header_struct_t;//帧头

typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
    uint16_t CMD_ID;
} frame_header_ui_t;//UI帧头


typedef enum {
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct {
    frame_header_struct_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

#pragma pack(pop)
//************************************
/* 机器人id */
#define REF_ROBOT_RED_HERO            1
#define REF_ROBOT_RED_ENGINEER        2
#define REF_ROBOT_RED_STANDARD_3      3
#define REF_ROBOT_RED_STANDARD_4      4
#define REF_ROBOT_RED_STANDARD_5      5
#define REF_ROBOT_RED_DRONE           6
#define REF_ROBOT_RED_SENTRY          7
#define REF_ROBOT_RED_DART            8
#define REF_ROBOT_RED_RADAR           9
#define REF_ROBOT_RED_OUTPOST        10
#define REF_ROBOT_RED_BASE           11
#define REF_ROBOT_BLUE_HERO         101
#define REF_ROBOT_BLUE_ENGINEER     102
#define REF_ROBOT_BLUE_STANDARD_3   103
#define REF_ROBOT_BLUE_STANDARD_4   104
#define REF_ROBOT_BLUE_STANDARD_5   105
#define REF_ROBOT_BLUE_DRONE        106
#define REF_ROBOT_BLUE_SENTRY       107
#define REF_ROBOT_BLUE_DART         108
#define REF_ROBOT_BLUE_RADAR        109
#define REF_ROBOT_BLUE_OUTPOST      110
#define REF_ROBOT_BLUE_BASE         111

/* 选手端id */
#define REF_CLIENT_RED_HERO         0x0101
#define REF_CLIENT_RED_ENGINEER     0x0102
#define REF_CLIENT_RED_STANDARD_3   0x0103
#define REF_CLIENT_RED_STANDARD_4   0x0104
#define REF_CLIENT_RED_STANDARD_5   0x0105
#define REF_CLIENT_RED_DRONE        0x0106
#define REF_CLIENT_BLUE_DRONE       0x016A
#define REF_CLIENT_BLUE_HERO        0x0165
#define REF_CLIENT_BLUE_ENGINEER    0x0166
#define REF_CLIENT_BLUE_STANDARD_3  0x0167
#define REF_CLIENT_BLUE_STANDARD_4  0x0168
#define REF_CLIENT_BLUE_STANDARD_5  0x0169
#define REF_CLIENT_SERVER           0x8080 //裁判系统,用于哨兵雷达自主决策


/* 比赛类型 */
#define REF_GAME_TYPE_UC 1 //超对
#define REF_GAME_TYPE_S 2 //单项赛
#define REF_GAME_TYPE_UA 3 //人工智能挑战赛
#define REF_GAME_TYPE_3v3 4 //3v3
#define REF_GAME_TYPE_1v1 5 //1v1
/* 比赛进程 */
#define REF_GAME_PROGRESS_NOT_START 0 //未开始
#define REF_GAME_PROGRESS_PREPARING 1 //准备阶段
#define REF_GAME_PROGRESS_15S_COUNTDOWN 2 //15s裁判系统自检
#define REF_GAME_PROGRESS_5S_COUNTDOWN 3 //5s倒计时
#define REF_GAME_PROGRESS_RUNNING 4 //比赛进行中
#define REF_GAME_PROGRESS_END 5 //比赛结束
typedef struct {
    uint8_t game_type: 4;//比赛类型
    uint8_t game_progress: 4;//比赛进程
    uint16_t stage_remain_time;//当前阶段剩余时间
    uint64_t SyncTimeStamp;//unix时间
} ext_game_state_t;//0x0001

/* 获胜方 */
#define REF_WINNER_DRAW 0 //平局
#define REF_WINNER_RED 1 //红方胜
#define REF_WINNER_BLUE 2 //蓝方胜

typedef struct {
    uint8_t winner;//获胜方
} ext_game_result_t;//0x0002

typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved1;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_hp_t;//0x0003

typedef struct {
    uint8_t supply_area1_is_occupied: 1;//己方与兑换区不重叠的补给区占领状态
    uint8_t supply_area2_is_occupied: 1;//己方与兑换区重叠的补给区占领状态
    uint8_t supply_area3_is_occupied: 1;//己方补给区占领状态(仅 RMUL)
    uint8_t energy_small_is_activated: 1;//己方能量机关激活点占领状态
    uint8_t energy_big_is_activated: 1;//己方能量机关激活状态
    uint8_t high_ground1_is_occupied: 2;//己方中央高地的占领状态，1为被己方占领，2为被对方占领
    uint8_t high_ground2_is_occupied: 2;//己方梯形高地的占领状态，1为被已占领
    uint16_t last_dart_hit_time: 10;//对方飞镖最后命中时间(0-420，开局为0)
    uint8_t dart_last_hit_position: 3;//对方飞镖最后命中目标,开局为0 1:击中前哨站 2:击中基地固定目标 3:击中基地随机固定目标 4:击中基地随机移动目标
    uint8_t center_area_occupation_status: 2;//中心增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领（仅RMUL)
    uint8_t reserved: 8;//保留
} ext_event_data_t;//0x0101

/* 出弹口打开状态 */
#define REF_SUPPLY_STATUS_CLOSE 0 //出弹口关闭
#define REF_SUPPLY_STATUS_PREPARING 1 //出弹口准备中
#define REF_SUPPLY_STATUS_OPEN 2 //弹丸释放
typedef struct {
    uint8_t reserved;//保留
    uint8_t supply_robot_id;//补弹机器人
    uint8_t supply_projectile_step;//出弹口打开状态
    uint8_t supply_projectile_num;//出弹数量
} ext_supply_projectile_action_t;//0x0102

//typedef struct
//{
//    uint8_t supply_projectile_id;
//    uint8_t supply_robot_id;
//    uint8_t supply_num;
//} ext_supply_projectile_booking_t;//0x0103

typedef struct {
    uint8_t level;//己方最后一次判罚等级
    uint8_t offending_robot_id;//违规机器人id
    uint8_t count;//最后一次违规等级对应的违规次数
} ext_referee_warning_t;//0x0104

typedef struct {
    uint8_t dart_remaining_time;//飞镖发射剩余时间 单位s
    uint8_t dart_last_hit_position: 3;//飞镖最后命中目标,开局为0 1:击中前哨站 2:击中基地固定目标 3:击中基地随机固定目标 4:为击中基地随机移动目标
    uint8_t dart_last_hit_count: 3;//飞镖最后击中目标累计次数 最大为4
    uint8_t dart_target: 2;//飞镖选定的击打目标 开局/选中前哨站为0 选择基地固定目标为1，选中基地随机固定目标为2 中基地随机移动目标为3
    uint16_t reserved : 8;//保留
} ext_dart_remaining_time_t;//0x0105

typedef struct {
    uint8_t robot_id;//本机器人id
    uint8_t robot_level;//机器人等级
    uint16_t current_HP;//机器人当前血量
    uint16_t maximum_HP;//机器人血量上限
    uint16_t shooter_barrel_cooling_value;//机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;//机器人枪口热量上限
    uint16_t chassis_power_limit;//底盘功率上限
    uint8_t power_management_gimbal_is_output: 1;
    uint8_t power_management_chassis_is_output: 1;
    uint8_t power_management_shooter_is_output: 1;
} ext_game_robot_state_t;//0x0201

typedef struct {
    uint16_t  reserved_chassis_volt;//(已弃用)chassis口输出电压 单位mV
    uint16_t  reserved_chassis_current;//(已弃用)chassis口输出电流 单位mA
    float  reserved_chassis_power;//(已弃用)底盘功率 单位W
    uint16_t buffer_energy;//缓存能量 单位J
    uint16_t shooter_id1_17mm_cooling_heat;//第1个17mm发射机构的枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;//第2个17mm发射机构的枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;//42mm发射机构的枪口热量
} ext_power_heat_data_t;//0x0202

typedef struct {
    float x;
    float y;
    float yaw;
} ext_game_robot_pos_t;//0x0203

typedef struct //0x0204
{
    uint8_t recovery_buff;//恢复 血量上限的百分比
    uint8_t cooling_buff;//冷却 百分比
    uint8_t defence_buff;//加防 百分比
    uint8_t vulnerability_buff;//减防 百分比
    uint16_t attack_buff;//加攻 百分比

    //为真条件：bit0:>50% bit1:>=30% bit2:>=15% bit3:>=5%
//    uint8_t remaining_energy;//bit 0-4：机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。
    uint8_t is_energy_more_50: 1;
    uint8_t is_energy_more_30: 1;
    uint8_t is_energy_more_15: 1;
    uint8_t is_energy_more_5: 1;
} ext_buff_t;


/* 空中机器人状态 */
#define REF_DRONE_COOLING 0 //空中机器人冷却中
#define REF_DRONE_READY 1 //空中机器人冷却完毕
#define REF_DRONE_RUNNING 2 //空中机器人支援中
typedef struct {
    uint8_t drone_status;//空中机器人状态
    uint8_t attack_time;//空中机器人当前状态的剩余时间 单位s 向下取整
} ext_drone_state_t;//0x0205

/* 伤害类型 */
#define REF_HURT_TYPE_ATTACKED 0 //被弹丸击打
#define REF_HURT_TYPE_REFEREE_OFFLINE 1 //裁判系统模块离线
#define REF_HURT_TYPE_OVER_SHOOT_SPEED 2 //超射速
#define REF_HURT_TYPE_OVERHEAT 3 //枪口超热量
#define REF_HURT_TYPE_OVER_POWER 4 //超功率
#define REF_HURT_TYPE_COLLISION 5 //碰撞
typedef struct {
    uint8_t armor_id: 4;//当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t hurt_type: 4;
} ext_hurt_data_t;//0x0206

typedef struct {
    uint8_t bullet_type;//弹丸类型 1:17mm 2:42mm
    uint8_t shooter_id;//发射机构id 1:第一个17mm 2:第二个17mm 3:42mm
    uint8_t bullet_freq;//发射频率 单位Hz
    float bullet_speed;//弹丸初速度 单位m/s
} ext_shoot_data_t;//0x0207

typedef struct {
    uint16_t bullet_remaining_num_17mm;//17mm允许发弹量
    uint16_t bullet_remaining_num_42mm;//42mm允许发弹量
    uint16_t coin_remaining_num;//剩余金币量
} ext_bullet_remaining_t;//0x0208

/* RFID状态 */
#define REF_RFID_OUR_BASE 0x00000001//我方基地 RFID
#define REF_RFID_OUR_HIGH_GROUND1 0x00000002//己方环形高地增益点
#define REF_RFID_ENEMY_HIGH_GROUND1 0x00000004//对方环形高地增益点
#define REF_RFID_OUR_HIGH_GROUND2 0x00000008//己方R3/B3梯形高地增益点
#define REF_RFID_ENEMY_HIGH_GROUND2 0x00000010//对方R3/B3梯形高地增益点
#define REF_RFID_OUR_HIGH_GROUND3 0x00000020//己方R4/B4梯形高地增益点
#define REF_RFID_ENEMY_HIGH_GROUND3 0x00000040//对方R4/B4梯形高地增益点
#define REF_RFID_OUR_ENERGY_ACTIVATE_POS 0x00000080//己方能量机关激活点
#define REF_RFID_OUR_FLY_POS1 0x00000100//己方飞坡增益点（靠近己方一侧飞坡前）
#define REF_RFID_OUR_FLY_POS2 0x00000200//己方飞坡增益点（靠近己方一侧飞坡后）
#define REF_RFID_ENEMY_FLY_POS1 0x00000400//对方飞坡增益点（靠近对方一侧飞坡前）
#define REF_RFID_ENEMY_FLY_POS2 0x00000800//对方飞坡增益点（靠近对方一侧飞坡后）
#define REF_RFID_OUR_OUTPOST 0x00001000//己方前哨站增益点
#define REF_RFID_OUR_SUPPLY 0x00002000//己方补血点
#define REF_RFID_OUR_SENTRY 0x00004000//己方哨兵巡逻区
#define REF_RFID_ENEMY_SENTRY 0x00008000//对方方哨兵巡逻区
#define REF_RFID_OUR_RESOURCE_LAND 0x00010000//己方资源区
#define REF_RFID_ENEMY_RESOURCE_LAND 0x00020000//对方资源区
#define REF_RFID_OUR_EXCHANGE_STATION 0x00040000//己方兑换区
#define REF_RFID_CENTER_BUFF 0x00080000//(RMUL)中心增益点
//TODO:更新对应位置
//eg. if(rfid_status & REF_RFID_OUR_BASE)//如果识别到我方基地

typedef struct {
    /* RFID 状态 - 各位表示是否检测到相应的 RFID 卡 */
    uint32_t our_base : 1;                 /* bit 0：己方基地增益点 */
    uint32_t our_central_highland : 1;     /* bit 1：己方中央高地增益点 */
    uint32_t enemy_central_highland : 1;   /* bit 2：对方中央高地增益点 */
    uint32_t our_trapezoid_highland : 1;   /* bit 3：己方梯形高地增益点 */
    uint32_t enemy_trapezoid_highland : 1;  /* bit 4：对方梯形高地增益点 */
    uint32_t our_fly_slope_front : 1;      /* bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前） */
    uint32_t our_fly_slope_back : 1;       /* bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后） */
    uint32_t enemy_fly_slope_front : 1;    /* bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前） */
    uint32_t enemy_fly_slope_back : 1;     /* bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后） */
    uint32_t our_terrain_cross_lower_center : 1; /* bit 9：己方地形跨越增益点（中央高地下方） */
    uint32_t our_terrain_cross_upper_center : 1; /* bit 10：己方地形跨越增益点（中央高地上方） */
    uint32_t enemy_terrain_cross_lower_center : 1; /* bit 11：对方地形跨越增益点（中央高地下方） */
    uint32_t enemy_terrain_cross_upper_center : 1; /* bit 12：对方地形跨越增益点（中央高地上方） */
    uint32_t our_terrain_cross_lower_road : 1; /* bit 13：己方地形跨越增益点（公路下方） */
    uint32_t our_terrain_cross_upper_road : 1; /* bit 14：己方地形跨越增益点（公路上方） */
    uint32_t enemy_terrain_cross_lower_road : 1; /* bit 15：对方地形跨越增益点（公路下方） */
    uint32_t enemy_terrain_cross_upper_road : 1; /* bit 16：对方地形跨越增益点（公路上方） */
    uint32_t our_fort : 1;                 /* bit 17：己方堡垒增益点 */
    uint32_t our_outpost : 1;              /* bit 18：己方前哨站增益点 */
    uint32_t our_supply_non_overlap : 1;   /* bit 19：己方与兑换区不重叠的补给区/RMUL 补给区 */
    uint32_t our_supply_overlap : 1;       /* bit 20：己方与兑换区重叠的补给区 */
    uint32_t our_resource_island : 1;      /* bit 21：己方大资源岛增益点 */
    uint32_t enemy_resource_island : 1;    /* bit 22：对方大资源岛增益点 */
    uint32_t center_gain : 1;              /* bit 23：中心增益点（仅 RMUL 适用） */
    uint32_t reserved : 8;                 /* bit 24-31：保留 */
} ext_rfid_state_t;//0x0209


/* 飞镖发射口状态 */
#define REF_DART_LAUNCH_CLOSE 1 //飞镖发射口关闭
#define REF_DART_LAUNCH_OPENING 2 //飞镖发射口正在开启或关闭
#define REF_DART_LAUNCH_OPEN 0 //飞镖发射口开启
typedef struct {
    uint8_t dart_launch_opening_status; //飞镖发射口开闭状态
    uint8_t reserved;
    uint16_t target_change_time; //切换击打目标时的比赛剩余时间 单位s
    uint16_t latest_launch_cmd_time; //最后一次操作手确定发射指令时的比赛剩余时间 单位s
} ext_dart_client_cmd_t; //0x020A

//场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边
//向红方停机坪为 Y 轴正方向。
typedef struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_standard_5_x;
    float reserved_standard_5_y;
} ext_ground_robot_position_t; //0x020B

typedef struct {
    uint8_t mark_hero_progress:1;
    uint8_t mark_engineer_progress:1;
    uint8_t mark_standard_3_progress:1;
    uint8_t mark_standard_4_progress:1;
    uint8_t mark_sentry_progress:1;
} ext_radar_mark_data_t;//0x020C

typedef struct {
    uint16_t bullet_bought: 11;//除远程兑换外，成功兑换的发弹量
    uint8_t bullet_bought_remote: 4;//远程兑换成功的发弹量
    uint8_t hp_bought: 4;//成功远程兑换血量的次数
    uint8_t can_free_respawn: 1;//是否可以免费复活
    uint8_t can_buy_respawn: 1;//是否可以兑换复活
    uint16_t money_to_buy_respawn: 10;//兑换复活所需金币
    uint8_t reserved1: 1;//保留位
    uint8_t is_out_of_fight: 1;//是否脱战
    uint16_t bullet_can_be_bought: 11;//队伍剩余可兑换发弹量
    uint8_t reserved2: 4;//保留位
} ext_sentry_info_t;//0x020D

typedef struct {
    uint8_t chance_left: 2;//剩余双倍易伤次数
    uint8_t is_doubled: 1;//对方是否正在被触发双倍易伤
} ext_radar_info_t;//0x020E

/************************操作帧*************************/
//操作帧
typedef struct {
    uint16_t data_cmd_id; //子内容id
    uint16_t sender_id; //发送者id，需要与自身id匹配
    uint16_t receiver_id; //接收者id
    uint8_t user_data[128];//实际最大112

} ext_student_interactive_data_t;//0x0301

//删除图层层
typedef struct {
    uint8_t operate_type;//0:空操作 1:删除图层 2:删除所有图层
    uint8_t layer;
} ext_client_custom_graphic_delete_t;//sub id 0x0100

//ui
typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t radius: 10;
    uint32_t end_x: 11;
    uint32_t end_y: 11;              //浮点数据
} ext_float_data;

//ui
typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t radius: 10;
    uint32_t end_x: 11;
    uint32_t end_y: 11;
} graphic_data_struct_t;

typedef struct {
    graphic_data_struct_t graphic_data_struct;
} ext_client_custom_graphic_single_t;//sub id 0x0101

typedef struct {
    graphic_data_struct_t graphic_data_struct[2];
} ext_client_custom_graphic_double_t;//sub id 0x0102

typedef struct {
    graphic_data_struct_t graphic_data_struct[5];
} ext_client_custom_graphic_5_t;//sub id 0x0103

typedef struct {
    graphic_data_struct_t graphic_data_struct[7];
} ext_client_custom_graphic_7_t;//sub id 0x0104

//校验?
typedef struct {
    graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;//sub id 0x0110

//哨兵自主决策指令
typedef struct {
    uint8_t reborn_confirm: 1;//哨兵重生确认，不确认时及时读条已完成也不会复活
    uint8_t reborn_immediately: 1;//哨兵确认兑换立即复活，确认后当满足规则要求会立即消耗金币兑换

    //此值开局仅能为 0，此后哨兵可将其从 0 修改至 X，则消
    //耗 X 金币成功兑换 X 允许发弹量。此后哨兵可将其从 X 修改至
    //X+Y，以此类推。
    uint16_t bullet_to_buy: 11;//兑换发弹量，单调递增，设置后前往补血点即可兑换
    uint8_t buy_remote_bullet: 4;//请求远程兑换发弹量，单调递增且每次仅能增加1
    uint8_t buy_remote_hp: 4;//请求远程兑换血量，单调递增且每次仅能增加1
} sentry_cmd_t;//sub id 0x120

//雷达自主决策指令
typedef struct {
    uint8_t radar_cmd;//请求触发双倍易伤，单调递增且每次仅能自增1
} radar_cmd_t;//sub id 0x0121

/****************小地图交互数据*******************/
//选手端下发数据
typedef struct {
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;//云台手按下的键盘按键通用键值
    uint8_t target_robot_id;
    uint8_t cmd_source;//信息来源的 ID，见手册附录
} ext_map_command_t;//0x0303

//选手端接收数据(雷达)单位cm
typedef struct {
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} ext_client_map_command_t;//0x0305

//半自动指令
typedef struct {
    uint8_t intention;//意图(1:到目标点攻击 2：到目标点防守 3：移动到目标点)

    //小地图左下角为坐标原点，水平向右为 X 轴正
    //方向，竖直向上为 Y 轴正方向。显示位置将按
    //照场地尺寸与小地图尺寸等比缩放，超出边界
    //的位置将在边界处显示
    uint16_t start_position_x;//单位dm
    uint16_t start_position_y;//单位dm

    //增量相较于上一个点位进行计算，共 49 个新
    //点位，X 与 Y 轴增量对应组成点位
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;//需与自身 ID 匹配
} map_data_t;//0x0307

//己方机器人常规链路发送自定义消息
typedef struct {
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];//utf16
} custom_info_t;//0x0308


/***************图传链路数据*****************/
//自定义控制器
typedef struct {
    uint8_t data[30];
} ext_custom_robot_data_t;//0x0302
//自定义控制器2
typedef struct{
    uint8_t data[30];
}ext_robot_custom_data_t;//0x0309

//键鼠数据
typedef struct {
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;//0x0304

/***************非图传数据*****************/
//自定义控制器模拟键鼠
typedef struct {
    uint16_t key_value;
    uint16_t x_position: 12;
    uint16_t mouse_left: 4;
    uint16_t y_position: 12;
    uint16_t mouse_right: 4;
    uint16_t reserved;
} custom_client_data_t;//0x0306

//typedef struct {
//    float data1;
//    float data2;
//    float data3;
//    uint8_t data4;
//} custom_data_t;
//
//typedef struct {
//    uint8_t data[64];
//} ext_up_stream_data_t;
//
//typedef struct {
//    uint8_t data[32];
//} ext_download_stream_data_t;


typedef struct {
    uint32_t radius: 10;
    uint32_t end_x: 11;
    uint32_t end_y: 11;
} float_buffer_1_t;

typedef union {
    float_buffer_1_t t_buffer;
    int32_t f;
} float_buffer_u;


typedef struct {
    graphic_data_struct_t Graph_Control;
    uint8_t show_Data[30];
} ext_string_data;                  //打印字符串数据


typedef struct referee_data {
    uint32_t last_update_tick;
    ext_game_state_t game_state;                    // 0x0001 比赛状态
    ext_game_result_t game_result;                  // 0x0002 比赛结果
    ext_game_robot_hp_t game_robot_hp;              // 0x0003 机器人血量
    ext_event_data_t event_data;                    // 0x0101 场地事件
    // ext_supply_projectile_action_t supply_projectile_action; // 0x0102 补给站动作（已弃用）
    ext_referee_warning_t referee_warning;          // 0x0104 裁判警告
    ext_dart_remaining_time_t dart_remaining_time;  // 0x0105 飞镖剩余时间
    ext_game_robot_state_t game_robot_state;        // 0x0201 机器人状态
    ext_power_heat_data_t power_heat_data;          // 0x0202 功率热量数据
    ext_game_robot_pos_t game_robot_pos;            // 0x0203 机器人位置
    ext_buff_t buff;                                // 0x0204 增益
    // ext_drone_state_t drone_state;                 // 0x0205 空中机器人状态（已弃用）
    ext_hurt_data_t hurt_data;                      // 0x0206 受伤数据
    ext_shoot_data_t shoot_data;                    // 0x0207 射击数据
    ext_bullet_remaining_t bullet_remaining;        // 0x0208 弹丸剩余
    ext_rfid_state_t rfid_state;                    // 0x0209 场地交互状态
    ext_dart_client_cmd_t dart_client_cmd;          // 0x020A 飞镖的客户端指令
    ext_ground_robot_position_t all_robot_position; // 0x020B 所有机器人的位置
    ext_radar_mark_data_t radar_mark_data;          // 0x020C 雷达标记数据
    ext_sentry_info_t sentry_info;                  // 0x020D 哨兵信息
    ext_radar_info_t radar_info;                    // 0x020E 雷达信息
    ext_student_interactive_data_t student_interactive_data; // 0x0301 机器人交互数据
    ext_custom_robot_data_t custom_robot_data;      // 0x0302 自定义控制器client->robot
    ext_map_command_t map_command;                  // 0x0303 选手端小地图交互数据
    ext_robot_command_t robot_command;              // 0x0304 图传键鼠
    ext_client_map_command_t client_map_command;    // 0x0305 雷达小地图
    custom_client_data_t custom_client_data;        // 0x0306 自定义控制器模拟键鼠
    map_data_t map_data;                            // 0x0307 半自动兵种接收地图
    custom_info_t custom_info;                      // 0x0308 己方机器人常规链路发送自定义消息
    ext_robot_custom_data_t robot_custom_data;      // 0x0309 自定义控制器robot->client
}referee_data_t;


extern char IF_SpeedUpdate;

extern void init_referee_struct_data(void);

extern void Referee_Solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(float *power, float *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_id1_17mm_limit_and_id1(uint16_t *id1_limit, uint16_t *id1);

extern void get_shoot_id2_17mm_limit_and_id2(uint16_t *id2_limit, uint16_t *id2);

extern void get_shoot_id1_17mm_speed_and_limit(uint16_t *id1_limit, float *id1);

extern void get_shoot_id2_17mm_speed_and_limit(uint16_t *id2_limit, float *id2);

extern void get_chassis_power_limit(uint16_t *power_limit);

extern void get_game_type(uint8_t *game_type);

extern ext_game_robot_state_t *get_robot_state_Point(void);

void line_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate,
               uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
               uint32_t End_x, uint32_t End_y);

void rectangle_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer,
                    uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x,
                    uint32_t End_y);

void
circle_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
            uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

void arc_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
              uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width,
              uint32_t Start_x,
              uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);

void float_draw(ext_float_data *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
                uint32_t Start_y, float Graph_Float);

void char_draw(ext_string_data *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
               uint32_t Start_y, uint8_t *Char_Data);


void ui_delete(uint8_t del_operate, uint8_t del_layer);

int ui_refresh(int cnt, ...);

int char_refresh(ext_string_data string_data);

void get_shoot_id1_17mm_limit_and_id1(uint16_t *id1_limit, uint16_t *id1);

void get_shoot_id1_17mm_speed_and_limit(uint16_t *id1_limit, float *id1);

void Referee_Init();


#endif
