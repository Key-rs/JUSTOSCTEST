//
// Created by Konodoki on 2025/2/26.
//
enum Sentry_Control_Mode{
  Sentry_Manual_Control,//手动模式
  Sentry_Semi_Automatic_Control,//半自动模式
  Sentry_Automatic_Control//全自动模式
};
enum Sentry_Shoot_Mode{
  Sentry_Shoot_Standby,//待机
  Sentry_Shoot_Ready,//准备
  Sentry_Shoot_Shot//射击一发
};
enum Sentry_Gimbal_Mode{
  Sentry_Gimbal_Normal,//普通模式
  Sentry_Gimbal_Follow,//云台跟随
  Sentry_Outpost
};
extern enum Sentry_Control_Mode sentryControlMode;
extern enum Sentry_Shoot_Mode sentryShootMode;
extern enum Sentry_Gimbal_Mode sentryGimbalMode;