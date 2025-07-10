//
// Created by Konodoki on 2025/1/12.
//

#include "auto_aim.h"
#include "SolveTrajectory.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "referee.h"
#include "vision_com.h"
#include <stdio.h>
#include "SentryConfig.h"
#include "user_lib.h"
osThreadId g_auto_aim_task;
extern float INS_angle[3];
static float aim_x,aim_y,aim_z;
static INTF_Gimbal_HandleTypeDef *gimbal;
AutoAimStatus_t *g_auto_aim_status;
//自动插帧相关
AutoAim_M2S_PacketTypeDef last_vision_msg;
uint32_t last_receive_msg_tick=0;
uint32_t last_insert_frame_tick=0;
//自瞄的自动补帧
#define Sentry_AutoAim_AutoInsertEnable 0
#define Sentry_AutoAim_AutoInsertCycle 1 //补帧间隔ms
#define Sentry_AutoAim_AutoInsertMinTime 2 //最短补帧ms
#define Sentry_AutoAim_AutoInsertMaxTime 500 //最长补帧ms
extern referee_data_t *g_referee;
void auto_aim_send(){
  //每隔1ms向视觉发送信息
  while (1){
    uint8_t def_detect_color=1;
    if(g_referee->last_update_tick>1000&&xTaskGetTickCount()-g_referee->last_update_tick<1000){
      def_detect_color=g_referee->game_robot_state.robot_id<100;//小于100目前就是红方
    }
    AutoAim_S2M_PacketTypeDef msg;
    msg.detect_color=def_detect_color;
    msg.task_mode=0;
    msg.reset_tracker=0;
    msg.is_play=0;
    msg.change_target=0;
    msg.reserved=0;
    //-------------------------------------
    //注意这边传给视觉的坐标信息的坐标轴在哪，待会视觉传给你的信息的坐标轴就在哪
    //假设我这里用c板陀螺仪的坐标轴
    msg.roll=0;
    msg.pitch = -Gimbal_GetPitch();//注意c板的安装方向
    msg.yaw = Gimbal_GetYaw();//注意c板的安装方向
    //-------------------------------------
    msg.aim_x = aim_x;//现在瞄准的位置，方便视觉查看
    msg.aim_y = aim_y;
    msg.aim_z = aim_z;
    msg.game_time = 0;
    msg.timestamp = xTaskGetTickCount();
    AutoAim_Transmit(&msg);
    osDelay(1);
  }
}
extern referee_data_t *g_referee;
float fly_time=90.0f;
void SolveT(AutoAim_M2S_PacketTypeDef *target){
  g_auto_aim_status->tracking=target->state;
  if(target->state!=1)
    return;
  struct SolveTrajectoryParams st_param;
  //定义参数
  st_param.k = 0.096f;
  st_param.bullet_type =  BULLET_17;
  st_param.current_v = 36.0f;
  st_param.xw = target->x;
  st_param.yw = target->y;
  st_param.zw = target->z;
  st_param.vxw = target->vx;
  st_param.vyw = target->vy;
  st_param.vzw = target->vz;
  st_param.v_yaw = target->v_yaw;
  st_param.tar_yaw = target->yaw;
  st_param.r1 = target->r1;
  st_param.r2 = target->r2;
  st_param.dz = target->dz;
  st_param.bias_time = (float)xTaskGetTickCount()-(float)target->cap_timestamp+fly_time;
  st_param.s_bias = 0.0f;//最好别动
  st_param.z_bias = -0.00f;
  st_param.armor_id = target->id;
  st_param.armor_num = target->armors_num;

  //弹道解算出来的yaw轴和pitch轴
  //注意这边的坐标轴是你当初发给视觉信息时的坐标轴
  float yaw,pitch;
  autoSolveTrajectory(&st_param,&pitch,&yaw,&aim_x,&aim_y,&aim_z);
  float now_fly_time= Sqrt(aim_x*aim_x+aim_y*aim_y+aim_z*aim_z)/23.0f*1000.0f;
  fly_time= LowPassFilter(now_fly_time,fly_time,0.01f);
  g_auto_aim_status->out_yaw=Gimbal_GetYaw()+loop_float_constrain(loop_float_constrain(yaw-Gimbal_GetYaw(),-PI,PI),-PI,PI);
  g_auto_aim_status->out_pitch=-pitch;
  g_auto_aim_status->solve_tick=xTaskGetTickCount();
}
void auto_aim_solve(void *msg,BusTopicHandle_t *topic){
  //收到视觉的目标信息
  AutoAim_M2S_PacketTypeDef *target=(AutoAim_M2S_PacketTypeDef *)msg;
  last_receive_msg_tick=xTaskGetTickCount();
  last_vision_msg=*target;
  SolveT(target);
}
void Auto_Insert_Frame(){
  while (Sentry_AutoAim_AutoInsertEnable){
    if(xTaskGetTickCount()-last_receive_msg_tick>Sentry_AutoAim_AutoInsertMinTime&&
        xTaskGetTickCount()-last_receive_msg_tick<Sentry_AutoAim_AutoInsertMaxTime&&
        xTaskGetTickCount()-last_insert_frame_tick>Sentry_AutoAim_AutoInsertCycle){
      //开始补帧
      last_vision_msg.x+=last_vision_msg.vx*(xTaskGetTickCount()-last_insert_frame_tick)/1000.0f;
      last_vision_msg.y+=last_vision_msg.vy*(xTaskGetTickCount()-last_insert_frame_tick)/1000.0f;
      last_vision_msg.z+=last_vision_msg.vz*(xTaskGetTickCount()-last_insert_frame_tick)/1000.0f;
      last_vision_msg.yaw+=last_vision_msg.v_yaw*(xTaskGetTickCount()-last_insert_frame_tick)/1000.0f;
      last_insert_frame_tick=last_vision_msg.cap_timestamp=xTaskGetTickCount();
      SolveT(&last_vision_msg);
    }
    osDelay(1);
  }
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
void auto_aim_init(){
  gimbal = pvSharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef ));
  g_auto_aim_status = pvSharePtr("AutoAimStatus", sizeof(AutoAimStatus_t));
  // osThreadDef("auto_aim",auto_aim_send,osPriorityNormal,0,314);
  // g_auto_aim_task = osThreadCreate(osThread(auto_aim),NULL);
  xTaskCreate(auto_aim_send,"auto_aim_send",256,NULL,15,NULL);
  xTaskCreate(Auto_Insert_Frame,"Auto_Insert_Frame",256,NULL,15,NULL);
  xBusSubscribeFromName("AutoAim_RX",auto_aim_solve);
}