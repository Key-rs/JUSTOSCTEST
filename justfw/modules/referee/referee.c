/*
****************************(C) COPYRIGHT 2021 乘风战队****************************
  * @file       bsp_referee.c/h
  * @brief      裁判系统通信协议接口，通过和裁判系统的学生串口进行通信，来获所需要的信息。
  * @note
=====================================================================================
*协议版本：V1.7.0
*通讯协议格式：frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16，整包校验)
*frame_header格式:SOF(1-byte)+data_length(2-byte)+seq(1-byte)+CRC8(1-byte)
*             SOF(1-byte = offest 0)          --> 数据帧起始字节，固定值为 0xA5
* 					  data_length(2-byte = offest 1)  --> 数据帧中 data 的长度
*						  seq (1-byte  = offest 3 )       --> 包序号
*						  CRC8 (1-byte = offest 4)        --> 帧头 CRC8 校验
*cmd_id内容：
*          - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*					 |   命令码   |	 数据段长度	|		功能说明
*					 |	 0x0001 	|			11      |	 比赛状态数据，1Hz 周期发送
*					 |	 0x0002 	| 		1 		  |	 比赛结果数据，比赛结束后发送
*					 |	 0x0003 	|			28 			|	 比赛机器人血量数据，1Hz 周期发送
*					 |	 0x0005 	|			11			|	 人工智能挑战赛加成与惩罚区状态，1Hz 周期发送
*					 |	 0x0101 	|			4 			|	 场地事件数据，1Hz 周期发送
*					 |	 0x0102 	|			3 			|	 场地补给站动作标识数据，动作发生后发送
*          |   0x0103   |     2       |  请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
*					 |	 0x0104 	|			2 			|	 裁判警告数据，警告发生后发送
*					 |	 0x0105	  |			1				|	 飞镖发射口倒计时，1Hz 周期发送
*					 |	 0x0201 	|			15 			|	 机器人状态数据，10Hz 周期发送
*					 |	 0x0202 	|			14 			|	 实时功率热量数据，50Hz 周期发送
*					 |	 0x0203 	|			16 			|	 机器人位置数据，10Hz 发送
*					 |	 0x0204 	|			1 			|	 机器人增益数据，1Hz 周期发送
*					 |	 0x0205 	|			3 			|	 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
*					 |	 0x0206 	|			1	 			|	 伤害状态数据，伤害发生后发送
*					 |	 0x0207 	|			6 			|	 实时射击数据，子弹发射后发送
*					 |	 0x0208 	|			2 			|	 弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送，1Hz 周期发送
*					 |	 0x0209 	|			4 			|	 机器人 RFID 状态，1Hz 周期发送
*					 |	 0x020A 	|			12 			|	 飞镖机器人客户端指令数据，10Hz 周期发送
*					 |	 0x0301 	|			n 			|	 机器人间交互数据，发送方触发发送,上限10Hz
*          |   0x0302   |     n       |  自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
*          |   0x0303   |     15      |  客户端小地图交互数据，触发发送
*          |   0x0304   |     12      |  键盘、鼠标信息，通过图传串口发送
*          |   0x0305   |     10      |  客户端小地图接收信息
*
*						- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*					ID说明：
*					1. 机器人 ID：1，英雄(红)；2，工程(红)；3/4/5，步兵(红)；6，空中(红)；7，哨兵(红)；9，雷达站
*					（红）；101，英雄(蓝)；102，工程(蓝)；103/104/105，步兵(蓝)；106，空中(蓝)；107，哨兵(蓝)；
*					109，雷达站（蓝）。
*					2. 客户端 ID ： 0x0101 为 英 雄 操 作 手 客 户 端 ( 红 ) ； 0x0102 ， 工 程 操 作 手 客 户 端 (( 红 ) ；
*					0x0103/0x0104/0x0105，步兵操作手客户端(红)；0x0106，空中操作手客户端((红)； 0x0165，英
*					雄操作手客户端(蓝)；0x0166，工程操作手客户端(蓝)；0x0167/0x0168/0x0169，步兵操作手客户
*					端步兵(蓝)；0x016A，空中操作手客户端(蓝)。
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Oct-16-2020     ghyll           1. 未完成
*  V2.0.1     Aug-8-2021      king            2. 完成
*  V3.0.0     2024/3/17       Ukua            3. 主打一个传承
*  V3.1.0     2025/3/16       Ukua            4. 被迫更新
@verbatim
==============================================================================

==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2020 dzxh****************************
*/
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "crc8_crc16.h"
#include "referee.h"
#include "interface.h"
#include "tinybus.h"

BusTopicDef_t *g_referee_tx;
BusSubscriberDef_t *g_referee_rx;


frame_header_struct_t referee_receive_header;//接收头帧
frame_header_struct_t referee_send_header;//发送头帧

//ext_game_state_t game_state;
//ext_game_result_t game_result;
//ext_game_robot_HP_t game_robot_HP_t;
//
//ext_event_data_t field_event;
//ext_supply_projectile_action_t supply_projectile_action_t;
////ext_supply_projectile_booking_t supply_projectile_booking_t;
//ext_referee_warning_t referee_warning_t;
//
//ext_game_robot_state_t robot_state;
//ext_power_heat_data_t power_heat_data_t;
//ext_game_robot_pos_t game_robot_pos_t;
//ext_buff_t buff_t;
//aerial_robot_energy_t robot_energy_t;
//ext_hurt_data_t robot_hurt_t;
//ext_shoot_data_t shoot_data_t;
//ext_bullet_remaining_t bullet_remaining_t;
//ext_student_interactive_data_t student_interactive_data_t;
//ext_client_map_command_t ext_client_map_command;

referee_data_t *g_referee;

uint16_t shoot_num = 0;

char IF_SpeedUpdate = false;

uint8_t UI_SEQ;
uint16_t Cilent_ID;

/****************************************串口驱动映射************************************/
void Referee_SendData(uint8_t *data, uint16_t len) {
    INTF_Serial_MessageTypeDef msg;
    msg.len = len;
    msg.data = data;
    vBusPublish(g_referee_tx, &msg);
}

void Referee_SendByte(uint8_t ch) {
    Referee_SendData(&ch, 1);
}

///**
//	* @name           init_referee_struct_data
//  * @brief          初始化内存
//  * @author         RM
//  * @param[in]      void
//  * @retval         void
//	* @note
//	*
//*/
//void init_referee_struct_data(void)
//{
//    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
//    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
//
//    memset(&game_state, 0, sizeof(ext_game_state_t));
//    memset(&game_result, 0, sizeof(ext_game_result_t));
//    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));
//
//
//    memset(&field_event, 0, sizeof(ext_event_data_t));
//    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
//    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
//    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
//
//
//    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
//    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
//    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
//    memset(&buff_t, 0, sizeof(ext_buff_t));
//    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
//    memset(&robot_hurt_t, 0, sizeof(ext_hurt_data_t));
//    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
//    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
//
//    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
//    memset(&ext_client_map_command, 0, sizeof(ext_client_map_command_t));
//
//}

/**
	* @name           Referee_Solve
  * @brief          解算数据
  * @author         RM
  * @param[in]      数据
  * @retval         void
	* @note
	*
*/
uint32_t err = 0;
void Referee_Solve(uint8_t *frame) {
    frame_header_struct_t head;

    uint16_t cmd_id = 0;
    uint8_t index = 0;

    memcpy(&head, frame, sizeof(frame_header_struct_t));

    if (head.SOF != 0xA5) {
        return;//帧头错误，丢包
    }
    if (!verify_CRC8_check_sum(frame, sizeof(frame_header_struct_t))) {
        err++;
        return;//CRC8校验失败，丢包
    }
    if (!verify_CRC16_check_sum(frame, 5 + 2 + head.data_length + 2)) {//帧头+cmd_id+data+CRC16
        err++;
        return;//CRC16校验失败，丢包
    }
    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    g_referee->last_update_tick= xTaskGetTickCount();
    switch (cmd_id) {
        case GAME_STATE_CMD_ID: {
            memcpy(&g_referee->game_state, frame + index, sizeof(ext_game_state_t));
        }
            break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&g_referee->game_result, frame + index, sizeof(ext_game_result_t));
        }
            break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&g_referee->game_robot_hp, frame + index, sizeof(ext_game_robot_hp_t));
        }
            break;
        case FIELD_EVENTS_CMD_ID: {
            memcpy(&g_referee->event_data, frame + index, sizeof(ext_event_data_t));
        }
            break;
//        case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
//            memcpy(&g_referee->supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
//        }
            break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(&g_referee->referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
            break;
        case DART_LAUNCH_INFO_ID: {
            memcpy(&g_referee->dart_remaining_time, frame + index, sizeof(ext_dart_remaining_time_t));
        }
            break;
        case ROBOT_STATE_CMD_ID: {
            memcpy(&g_referee->game_robot_state, frame + index, sizeof(ext_game_robot_state_t));
        }
            break;
        case POWER_HEAT_DATA_CMD_ID: {
            memcpy(&g_referee->power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
            break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&g_referee->game_robot_pos, frame + index, sizeof(ext_game_robot_pos_t));
        }
            break;
        case BUFF_MUSK_CMD_ID: {
            memcpy(&g_referee->buff, frame + index, sizeof(ext_buff_t));
        }
            break;
//        case DRONE_STATE_CMD_ID: {
//            memcpy(&g_referee->drone_state, frame + index, sizeof(ext_drone_state_t));
//        }
            break;
        case ROBOT_HURT_CMD_ID: {
            memcpy(&g_referee->hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
            break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&g_referee->shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
            break;
        case BULLET_REMAINING_CMD_ID: {
            memcpy(&g_referee->bullet_remaining, frame + index, sizeof(ext_bullet_remaining_t));
        }
            break;
        case RFID_STATE_CMD_ID: {
            memcpy(&g_referee->rfid_state, frame + index, sizeof(ext_rfid_state_t));
        }
            break;
        case DART_CLIENT_CMD_ID: {
            memcpy(&g_referee->dart_client_cmd, frame + index, sizeof(ext_dart_client_cmd_t));
        }
            break;
        case ALL_ROBOT_POS_CMD_ID: {
            memcpy(&g_referee->all_robot_position, frame + index, sizeof(ext_ground_robot_position_t));
        }
            break;
        case RADAR_PROGRESS_CMD_ID: {
            memcpy(&g_referee->radar_mark_data, frame + index, sizeof(ext_radar_mark_data_t));

        }
            break;
        case SENTRY_DECISION_CMD_ID: {
            memcpy(&g_referee->sentry_info, frame + index, sizeof(ext_sentry_info_t));
        }
            break;
        case RADAR_DECISION_CMD_ID: {
            memcpy(&g_referee->radar_info, frame + index, sizeof(ext_radar_info_t));
        }
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID: {
            memcpy(&g_referee->student_interactive_data, frame + index, sizeof(ext_student_interactive_data_t));
        }
            break;
        case DIY_CONTROLLER_CMD_ID: {
            memcpy(&g_referee->custom_robot_data, frame + index, sizeof(ext_custom_robot_data_t));
        }
            break;
        case RECEIVE_CLIENT_MAP_CMD_ID: {
            memcpy(&g_referee->map_command, frame + index, sizeof(ext_map_command_t));
        }
            break;
        case KEYBOARD_MOUSE_CMD_ID: {
            memcpy(&g_referee->robot_command, frame + index, sizeof(ext_robot_command_t));
        }
            break;
        case RADAR_SEND_CLIENT_MAP_CMD_ID: {
            memcpy(&g_referee->client_map_command, frame + index, sizeof(ext_client_map_command_t));
        }
            break;
        case DIY_CONTROLLER_SIMULATE_CMD_ID: {
            memcpy(&g_referee->custom_client_data, frame + index, sizeof(custom_client_data_t));
        }
            break;
        case SENTRY_SEND_CLIENT_MAP_CMD_ID: {
            memcpy(&g_referee->map_data, frame + index, sizeof(map_data_t));
        }
            break;
        case SEND_CLIENT_MAP_CMD_ID: {
            memcpy(&g_referee->custom_info, frame + index, sizeof(custom_info_t));
        }
            break;
        case DIY_CONTROLLER2_CMD_ID: {
            memcpy(&g_referee->custom_robot_data, frame + index, sizeof(ext_custom_robot_data_t));
        }
            break;
        default: {
            break;
        }
    }
    // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
    if (*(frame+index+head.data_length+2) == 0xA5)
    { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
        Referee_Solve(frame+index+head.data_length+2);
    }
}

void ui_cilent_id_slove(void) {
    switch (g_referee->game_robot_state.robot_id) {
        case (UI_Data_RobotID_RHero):
            Cilent_ID = UI_Data_CilentID_RHero;
            break;
        case (UI_Data_RobotID_REngineer):
            Cilent_ID = UI_Data_CilentID_REngineer;
            break;
        case (UI_Data_RobotID_RStandard1):
            Cilent_ID = UI_Data_CilentID_RStandard1;
            break;
        case (UI_Data_RobotID_RStandard2):
            Cilent_ID = UI_Data_CilentID_RStandard2;
            break;
        case (UI_Data_RobotID_RStandard3):
            Cilent_ID = UI_Data_CilentID_RStandard3;
            break;
        case (UI_Data_RobotID_RAerial):
            Cilent_ID = UI_Data_CilentID_RAerial;
            break;
        case (UI_Data_RobotID_BHero):
            Cilent_ID = UI_Data_CilentID_BHero;
            break;
        case (UI_Data_RobotID_BEngineer):
            Cilent_ID = UI_Data_CilentID_BEngineer;
            break;
        case (UI_Data_RobotID_BStandard1):
            Cilent_ID = UI_Data_CilentID_BStandard1;
            break;
        case (UI_Data_RobotID_BStandard2):
            Cilent_ID = UI_Data_CilentID_BStandard2;
            break;
        case (UI_Data_RobotID_BStandard3):
            Cilent_ID = UI_Data_CilentID_BStandard3;
            break;
        case (UI_Data_RobotID_BAerial):
            Cilent_ID = UI_Data_CilentID_BAerial;
            break;
        default:
            Cilent_ID = UI_Data_CilentID_RStandard1;
            break;
    }
}
//------------UI----------------------
/**
	* @name           ui_delete
  * @brief          删除UI操作
  * @author         king
  * @param[in]      del_operate 对应头文件删除操作 del_layer 删除的层 取值0-9
  * @retval         void
	* @note
	*
*/
void ui_delete(uint8_t del_operate, uint8_t del_layer) {
    uint8_t *framepoint;
    uint16_t frametail = 0xFFFF;
    int16_t loop_control;
    uint8_t buffer[512] = {0};
    uint8_t num = 0;

    frame_header_ui_t framehead;
    ext_student_interactive_data_t student_interactive_data;
    ext_client_custom_graphic_delete_t client_custom_graphic_delete;

    framepoint = (uint8_t *) &framehead;
    framehead.SOF = UI_SOF;
    framehead.data_length = 8;
    framehead.seq = UI_SEQ;
    framehead.CRC8 = get_CRC8_check_sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;

    ui_cilent_id_slove();
    student_interactive_data.data_cmd_id = UI_Data_ID_Del;
    student_interactive_data.sender_id = Robot_UI_ID;
    student_interactive_data.receiver_id = Cilent_ID;

    client_custom_graphic_delete.operate_type = del_operate;
    client_custom_graphic_delete.layer = del_layer;

    frametail = get_CRC16_check_sum(framepoint, sizeof(framehead), frametail);
    framepoint = (uint8_t *) &student_interactive_data;
    frametail = get_CRC16_check_sum(framepoint, sizeof(student_interactive_data), frametail);
    framepoint = (uint8_t *) &client_custom_graphic_delete;
    frametail = get_CRC16_check_sum(framepoint, sizeof(client_custom_graphic_delete), frametail);

    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (uint8_t *) &student_interactive_data;
    for (loop_control = 0; loop_control < sizeof(student_interactive_data); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (uint8_t *) &client_custom_graphic_delete;
    for (loop_control = 0; loop_control < sizeof(client_custom_graphic_delete); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (unsigned char *) &frametail;
    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;                                               //发送CRC16校验值
    }
    Referee_SendData(buffer, num);
    UI_SEQ++;
}

/************************************************绘制直线*************************************************
**参数：*graphic_data graphic_data_struct_t变量指针，用于存放图形数据
        graphic_name[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
void
line_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
          uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x,
          uint32_t End_y) {
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->operate_tpye = Graph_Operate;
    graphic_data->layer = Graph_Layer;
    graphic_data->color = Graph_Color;
    graphic_data->width = Graph_Width;
    graphic_data->start_x = Start_x;
    graphic_data->start_y = Start_y;
    graphic_data->end_x = End_x;
    graphic_data->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void rectangle_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer,
                    uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x,
                    uint32_t End_y) {
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->graphic_tpye = UI_Graph_Rectangle;
    graphic_data->operate_tpye = Graph_Operate;
    graphic_data->layer = Graph_Layer;
    graphic_data->color = Graph_Color;
    graphic_data->width = Graph_Width;
    graphic_data->start_x = Start_x;
    graphic_data->start_y = Start_y;
    graphic_data->end_x = End_x;
    graphic_data->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
       imagename[3]   图片名称，用于标识更改
       Graph_Operate   图片操作，见头文件
       Graph_Layer    图层0-9
       Graph_Color    图形颜色
       Graph_Width    图形线宽
       Start_x、Start_x    圆心坐标
       Graph_Radius  图形半径
**********************************************************************************************************/
void
circle_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
            uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius) {
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->graphic_tpye = UI_Graph_Circle;
    graphic_data->operate_tpye = Graph_Operate;
    graphic_data->layer = Graph_Layer;
    graphic_data->color = Graph_Color;
    graphic_data->width = Graph_Width;
    graphic_data->start_x = Start_x;
    graphic_data->start_y = Start_y;
    graphic_data->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
       imagename[3]   图片名称，用于标识更改
       Graph_Operate   图片操作，见头文件
       Graph_Layer    图层0-9
       Graph_Color    图形颜色
       Graph_Width    图形线宽
       Graph_StartAngle,Graph_EndAngle    开始，终止角度
       Start_y,Start_y    圆心坐标
       x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void arc_draw(graphic_data_struct_t *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
              uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width,
              uint32_t Start_x,
              uint32_t Start_y, uint32_t x_Length, uint32_t y_Length) {
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->graphic_tpye = UI_Graph_Arc;
    graphic_data->operate_tpye = Graph_Operate;
    graphic_data->layer = Graph_Layer;
    graphic_data->color = Graph_Color;
    graphic_data->width = Graph_Width;
    graphic_data->start_x = Start_x;
    graphic_data->start_y = Start_y;
    graphic_data->start_angle = Graph_StartAngle;
    graphic_data->end_angle = Graph_EndAngle;
    graphic_data->end_x = x_Length;
    graphic_data->end_y = y_Length;

}

/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
       imagename[3]   图片名称，用于标识更改
       Graph_Operate   图片操作，见头文件
       Graph_Layer    图层0-9
       Graph_Color    图形颜色
       Graph_Width    图形线宽
       Graph_Size     字号
       Graph_Digit    小数位数
       Start_x、Start_x    开始坐标
       Graph_Float   要显示的变量
**********************************************************************************************************/
void float_draw(ext_float_data *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
                uint32_t Start_y, float Graph_Float) {
    float_buffer_u float_buffer;
    float_buffer.f = (int32_t) (Graph_Float * 1000);
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->graphic_tpye = UI_Graph_Float;
    graphic_data->operate_tpye = Graph_Operate;
    graphic_data->layer = Graph_Layer;
    graphic_data->color = Graph_Color;
    graphic_data->width = Graph_Width;
    graphic_data->start_x = Start_x;
    graphic_data->start_y = Start_y;
    graphic_data->start_angle = Graph_Size;
    graphic_data->end_angle = Graph_Digit;
    graphic_data->radius = float_buffer.t_buffer.radius;
    graphic_data->end_x = float_buffer.t_buffer.end_x;
    graphic_data->end_y = float_buffer.t_buffer.end_y;
}

/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
       imagename[3]   图片名称，用于标识更改
       Graph_Operate   图片操作，见头文件
       Graph_Layer    图层0-9
       Graph_Color    图形颜色
       Graph_Width    图形线宽
       Graph_Size     字号
       Graph_Digit    字符个数
       Start_x、Start_x    开始坐标
       *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void char_draw(ext_string_data *graphic_data, int8_t graphic_name[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
               uint32_t Start_y, uint8_t *Char_Data) {
    int16_t i;
    for (i = 0; i < 3 && graphic_name[i] != '\0'; i++) {
        graphic_data->Graph_Control.graphic_name[2 - i] = graphic_name[i];
    }
    graphic_data->Graph_Control.graphic_tpye = UI_Graph_Char;
    graphic_data->Graph_Control.operate_tpye = Graph_Operate;
    graphic_data->Graph_Control.layer = Graph_Layer;
    graphic_data->Graph_Control.color = Graph_Color;
    graphic_data->Graph_Control.width = Graph_Width;
    graphic_data->Graph_Control.start_x = Start_x;
    graphic_data->Graph_Control.start_y = Start_y;
    graphic_data->Graph_Control.start_angle = Graph_Size;
    graphic_data->Graph_Control.end_angle = Graph_Digit;

    for (i = 0; i < Graph_Digit; i++) {
        graphic_data->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
        ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int ui_refresh(int cnt, ...) {
    uint16_t i;
    graphic_data_struct_t graphic_data;
    uint8_t *framepoint;
    uint16_t frametail = 0xFFFF;
    int16_t loop_control;
    uint8_t buffer[512] = {0};
    uint8_t num = 0;

    frame_header_ui_t framehead;
    ext_student_interactive_data_t student_interactive_data;

    va_list ap;
    va_start(ap, cnt);

    framepoint = (uint8_t *) &framehead;
    framehead.SOF = UI_SOF;
    framehead.data_length = 6 + cnt * 15;
    framehead.seq = UI_SEQ;
    framehead.CRC8 = get_CRC8_check_sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;

    switch (cnt) {
        case 1:
            student_interactive_data.data_cmd_id = UI_Data_ID_Draw1;
            break;
        case 2:
            student_interactive_data.data_cmd_id = UI_Data_ID_Draw2;
            break;
        case 5:
            student_interactive_data.data_cmd_id = UI_Data_ID_Draw5;
            break;
        case 7:
            student_interactive_data.data_cmd_id = UI_Data_ID_Draw7;
            break;
        default:
            return (-1);
    }
    ui_cilent_id_slove();
    student_interactive_data.sender_id = Robot_UI_ID;
    student_interactive_data.receiver_id = Cilent_ID;

    framepoint = (uint8_t *) &framehead;
    frametail = get_CRC16_check_sum(framepoint, sizeof(framehead), frametail);
    framepoint = (uint8_t *) &student_interactive_data;
    frametail = get_CRC16_check_sum(framepoint, sizeof(student_interactive_data), frametail);

    framepoint = (unsigned char *) &framehead;
    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (uint8_t *) &student_interactive_data;
    for (loop_control = 0; loop_control < sizeof(student_interactive_data); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    for (i = 0; i < cnt; i++) {
        graphic_data = va_arg(ap, graphic_data_struct_t);
        framepoint = (uint8_t *) &graphic_data;
        frametail = get_CRC16_check_sum(framepoint, sizeof(graphic_data), frametail);             //CRC16校验
        for (loop_control = 0; loop_control < sizeof(graphic_data); loop_control++) {
            //USART6_SendByte(*framepoint);
            buffer[num++] = *framepoint;
            framepoint++;
        }
    }
    framepoint = (unsigned char *) &frametail;
    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;                                               //发送CRC16校验值
    }
    Referee_SendData(buffer, num);
    va_end(ap);
    UI_SEQ++;
    return 0;
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： 字符

**********************************************************************************************************/
int char_refresh(ext_string_data string_data) {
    ext_string_data graphic_data;
    uint8_t *framepoint;
    uint16_t frametail = 0xFFFF;
    int16_t loop_control;
    uint8_t buffer[512] = {0};
    uint8_t num = 0;

    frame_header_ui_t framehead;
    ext_student_interactive_data_t student_interactive_data;
    graphic_data = string_data;

    framepoint = (uint8_t *) &framehead;
    framehead.SOF = UI_SOF;
    framehead.data_length = 6 + 45;
    framehead.seq = UI_SEQ;
    framehead.CRC8 = get_CRC8_check_sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;


    ui_cilent_id_slove();
    student_interactive_data.data_cmd_id = UI_Data_ID_DrawChar;
    student_interactive_data.sender_id = Robot_UI_ID;
    student_interactive_data.receiver_id = Cilent_ID;

    framepoint = (uint8_t *) &framehead;
    frametail = get_CRC16_check_sum(framepoint, sizeof(framehead), frametail);
    framepoint = (uint8_t *) &student_interactive_data;
    frametail = get_CRC16_check_sum(framepoint, sizeof(student_interactive_data), frametail);
    framepoint = (uint8_t *) &graphic_data;
    frametail = get_CRC16_check_sum(framepoint, sizeof(graphic_data), frametail);

    framepoint = (unsigned char *) &framehead;
    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (uint8_t *) &student_interactive_data;
    for (loop_control = 0; loop_control < sizeof(student_interactive_data); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (uint8_t *) &graphic_data;
    for (loop_control = 0; loop_control < sizeof(graphic_data); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;
    }
    framepoint = (unsigned char *) &frametail;
    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++) {
        //USART6_SendByte(*framepoint);
        buffer[num++] = *framepoint;
        framepoint++;                                               //发送CRC16校验值
    }
    Referee_SendData(buffer, num);
    UI_SEQ++;
    return 0;
}

//------------以下为外部接口--------------
//
//
//ext_game_robot_state_t *get_robot_state_Point(void)
//{
//    return &robot_state;
//}
//
//
///**
// * @brief Get the chassis power and buffer object
// * @author king
// * @mail kings669669@foxmail.com
// * @param power 底盘实时功率 单位：W（瓦）
// * @param buffer 底盘功率缓冲
// */
//void get_chassis_power_and_buffer(float *power, float *buffer)
//{
//    *power = power_heat_data_t.chassis_power;
//    *buffer = power_heat_data_t.chassis_power_buffer;
//}
//
///**
// * @brief Get the robot id object
// * @author king
// * @return uint8_t
// * @note 返回机器人当前ID号
// */
//uint8_t get_robot_id(void)
//{
//    return robot_state.robot_id;
//}
//
///**
// * @brief Get the shoot id1 17mm limit and id1 object
// * @author king
// * @param id1_limit 热量上限
// * @param id1 实时热量
// * @note 获取枪管id1的热量上限和实时热量
// */
//void get_shoot_id1_17mm_limit_and_id1(uint16_t *id1_limit,uint16_t *id1)
//{
//    *id1_limit=robot_state.shooter_id1_17mm_cooling_limit;
//    *id1=power_heat_data_t.shooter_id1_17mm_cooling_heat;
//}
//
///**
// * @brief Get the shoot id2 17mm limit and id2 object
// * @author king
// * @param id2_limit 热量上限
// * @param id2 实时热量
// * @note 获取枪管id2的热量上限和实时热量
// */
//void get_shoot_id2_17mm_limit_and_id2(uint16_t *id2_limit,uint16_t *id2)
//{
//    *id2_limit=robot_state.shooter_id2_17mm_cooling_limit;
//    *id2=power_heat_data_t.shooter_id2_17mm_cooling_heat;
//}
//
///**
// * @brief Get the shoot id1 17mm speed and limit object
// * @author king
// * @param id1_limit 速度限制 m/s
// * @param id1 当前速度 m/s
// * @note 获取枪管id1的速度上限和实时速度
// */
//void get_shoot_id1_17mm_speed_and_limit(uint16_t *id1_limit,float *id1)
//{
//    *id1_limit = robot_state.shooter_id1_17mm_speed_limit;
//    if(shoot_data_t.shooter_id==1)
//    {
//        *id1 = shoot_data_t.bullet_speed;
//    }
//}
//
//
///**
// * @brief Get the shoot id2 17mm speed and limit object
// * @author king
// * @param id2_limit 速度限制 m/s
// * @param id2 当前速度 m/s
// * @note 获取枪管id2的速度上限和实时速度
// */
//void get_shoot_id2_17mm_speed_and_limit(uint16_t *id2_limit,float *id2)
//{
//    *id2_limit = robot_state.shooter_id1_17mm_speed_limit;
//    if(shoot_data_t.shooter_id==2)
//    {
//        *id2 = shoot_data_t.bullet_speed;
//    }
//}
//
///**
// * @brief Get the chassis power limit object
// * @author king
// * @param power_limit 底盘功率上限
// * @note 获取底盘上限，用于超级电容的算法
// */
//void get_chassis_power_limit(uint16_t *power_limit)
//{
//    *power_limit = robot_state.chassis_power_limit;
//}
//
///**
// * @brief Get the game type object
// * @author king
// * @param game_type
// * @note 获取比赛类型
// * 1：RoboMaster 机甲大师赛；
// * 2：RoboMaster 机甲大师单项赛；
// * 3：ICRA RoboMaster 人工智能挑战赛
// * 4：RoboMaster 联盟赛 3V3
// * 5：RoboMaster 联盟赛 1V1
// */
//void get_game_type(uint8_t *game_type)
//{
//    *game_type = game_state.game_type;
//}
void Referee_RX_Callback(void *message, BusTopicHandle_t topic) {
    INTF_Serial_MessageTypeDef *msg = (INTF_Serial_MessageTypeDef *) message;
    Referee_Solve(msg->data);
}

void Referee_Init() {
    g_referee = pvSharePtr("referee_data", sizeof(referee_data_t));
    g_referee_rx = xBusSubscribeFromName("/REFEREE/RX", Referee_RX_Callback);
    g_referee_tx = xBusTopicRegister("/REFEREE/TX");
}

//**************************************************
