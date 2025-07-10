////
//// Created by Ukua on 2023/11/4.
////
//
//#include "leg.h"
//#include "cybergear.h"
//
//#include "DR16.h"
//
//
//static uint16_t idx = 0;
//INTF_Leg_HandleTypeDef *g_legs[LEG_NUM] = {0};
//
//osThreadId Leg_MainLoopTaskHandle;
//
//#include "BSP_bmi088.h"
//
//extern bmi088_real_data_t bmi088_real_data;
//extern float INS_angle[3];
//extern RC_ctrl_t rc_ctrl[2];
//
//PID_HandleTypeDef anti_crash_pid, yaw_w_pid;
//
//uint32_t last_time = 0;
//
//uint8_t is_flying = 0;
//uint32_t jump_tick = 0;
//uint8_t last_right_switch = 0;
//
//void Leg_InfoUpdate(INTF_Leg_HandleTypeDef *leg, float pitch) {
//    float xD, yD, xB, yB, BD, A0, B0, xC, yC;
//    xD = JOINT_DISTANCE + THIGH_LEN * mcos(leg->motor_f->real_angle);
//    yD = THIGH_LEN * msin(leg->motor_f->real_angle);
//    xB = THIGH_LEN * mcos(leg->motor_b->real_angle);
//    yB = THIGH_LEN * msin(leg->motor_b->real_angle);
//    BD = powf(xD - xB, 2) + powf(yD - yB, 2);
//    A0 = 2 * CALF_LEN * (xD - xB);
//    B0 = 2 * CALF_LEN * (yD - yB);
//    leg->phi2 = 2 * atan2f(B0 + Sqrt(powf(A0, 2) + powf(B0, 2) - powf(BD, 2)), A0 + BD);
//    xC = xB + CALF_LEN * mcos(leg->phi2);
//    yC = yB + CALF_LEN * msin(leg->phi2);
//
//    leg->phi5 = atan2f(yC, xC - JOINT_DISTANCE / 2);
//    leg->real_len = Sqrt(powf(xC - JOINT_DISTANCE / 2, 2) + powf(yC, 2));
//    leg->phi3 = atan2f(yC - yD, xC - xD);             // 稍后用于计算VMC
//    leg->theta = leg->phi5 - 0.5f * PI - pitch; // 确定方向
//    leg->height = leg->real_len * mcos(leg->theta);
//
//    static float predict_dt = 0.0001f;
//    leg->phi1_pred = leg->motor_b->real_angle + leg->motor_b->real_speed * predict_dt; // 预测下一时刻的关节角度(利用关节角速度)
//    leg->phi4_pred = leg->motor_f->real_angle + leg->motor_f->real_speed * predict_dt;
//    // 重新计算腿长和腿角度
//    xD = JOINT_DISTANCE + THIGH_LEN * mcos(leg->phi4_pred);
//    yD = THIGH_LEN * msin(leg->phi4_pred);
//    xB = 0 + THIGH_LEN * mcos(leg->phi1_pred);
//    yB = THIGH_LEN * msin(leg->phi1_pred);
//    BD = powf(xD - xB, 2) + powf(yD - yB, 2);
//    A0 = 2 * CALF_LEN * (xD - xB);
//    B0 = 2 * CALF_LEN * (yD - yB);
//    float phi2_pred = 2 * atan2f(B0 + Sqrt(powf(A0, 2) + powf(B0, 2) - powf(BD, 2)), A0 + BD);
//    xC = xB + CALF_LEN * mcos(phi2_pred);
//    yC = yB + CALF_LEN * msin(phi2_pred);
//    float phi5_pred = atan2f(yC, xC - JOINT_DISTANCE / 2);
//    // 差分计算腿长变化率和腿角速度
//    leg->phi2_w = (phi2_pred - leg->phi2) / predict_dt; // 稍后用于修正轮速
//    leg->phi5_w = (phi5_pred - leg->phi5) / predict_dt;
//    leg->legd = (Sqrt(powf(xC - JOINT_DISTANCE / 2, 2) + powf(yC, 2)) - leg->real_len) / predict_dt;
//    leg->theta_w = ((phi5_pred - 0.5f * PI - pitch - leg->theta) / predict_dt); // 可以不考虑机体? -predict_dt*chassis.pitch_w
//
//    leg->wheel_w = leg->wheel->real_speed + leg->phi2_w - bmi088_real_data.gyro[0];
//
//    leg->real_force = -((leg->motor_b->real_torque * sinf(leg->motor_b->real_angle - leg->phi5)
//                         + leg->motor_f->real_torque * sinf(leg->motor_f->real_angle - leg->phi5)) / THIGH_LEN);
//    leg->real_torque = -((leg->motor_b->real_torque * cosf(leg->motor_b->real_angle - leg->phi5)
//                          + leg->motor_f->real_torque * cosf(leg->motor_f->real_angle - leg->phi5)) / THIGH_LEN) *
//                       leg->real_len;
//    leg->fn = leg->real_force * cosf(leg->theta) + leg->real_torque * sinf(leg->theta) / leg->real_len;
//
//}
//
//void Leg_VMC_solve(INTF_Leg_HandleTypeDef *leg) {
////    float s23 = msin(leg->phi2 - leg->phi3);
////    float phi25 = leg->phi2 - leg->phi5;
////    float phi35 = leg->phi3 - leg->phi5;
////    float F_m_L = leg->_set_force * leg->real_len;
////    leg->motor_b->set_torque = ((THIGH_LEN * msin(leg->motor_b->real_angle - leg->phi2) *
////                                (leg->set_torque * mcos(phi35) - F_m_L * msin(phi35))) / (leg->real_len * s23));
////    leg->motor_f->set_torque = ((THIGH_LEN * msin(leg->phi3 - leg->motor_f->real_angle) *
////                                (leg->set_torque * mcos(phi25) - F_m_L * msin(phi25))) / (leg->real_len * s23));
//
//    float s23 = msin(leg->phi3 - leg->phi2);
//    float phi25 = leg->phi5 - leg->phi2;
//    float phi35 = leg->phi5 - leg->phi3;
//    float F_m_L = leg->_set_force * leg->real_len;
//    leg->motor_b->target_torque = ((THIGH_LEN * msin(leg->motor_b->real_angle - leg->phi2) *
//                                    (leg->set_torque * mcos(phi35) + F_m_L * msin(phi35))) / (leg->real_len * s23));
//    leg->motor_f->target_torque = ((THIGH_LEN * msin(leg->phi3 - leg->motor_f->real_angle) *
//                                    (leg->set_torque * mcos(phi25) + F_m_L * msin(phi25))) / (leg->real_len * s23));
//
//}
//
//float target_dist = 0.0f;
//float target_v = 0.0f;
//float vel = 0.0f;
//float dist = 0.0f;
//
//void CalcLQR(INTF_Leg_HandleTypeDef *leg) {
//    //for leg len 0.18
////    static float k[12] = {-33.1318, -5.24206, -22.1555, -18.4878, 11.1441, 0.912738,
////                          3.93236, 0.611016, 3.02221, 2.38355, 70.1205, 1.61421};
////    static float k[12] = {-37.3721, -6.47732, -20.1921, -17.5666, 32.2999, 2.01258, 30.6332,
////                          5.77695, 19.2123, 16.1772, 126.621, 3.19389};
////    static float k[12] = {-35.3477, -5.98917, -12.7998, -14.7382, 31.8883, 1.89641, 28.9291,
////                            5.32446, 12.0276, 13.5498, 127.004, 3.30164};
////    static float k[12] = {-42.1365, -8.7541, -8.70994, -13.6936, 36.9092, 1.90471, 43.7719,
////                                9.97006, 9.82586, 15.3318, 121.423, 3.18658};
////    static float k[12] = {-39.041, -7.24702, -10.8721, -15.6924, 34.5649, 2.0147, 35.6303,
////                            7.26081, 11.2778, 16.0257, 124.198, 3.14896};  //stable
////    static float k[12] = {-34.6996, -6.65154, -8.90149, -13.0882, 34.006, 1.85144, 31.3304,
////                            6.65491, 9.11341, 13.2364, 124.765, 3.2981};
////    static float k[12] = {-30.7532, -6.13442, -8.91756, -10.9727, 33.58, 1.70458, 27.5373,
////                                6.14323, 9.05035, 11.0117, 125.188, 3.4252};
////    static float k[12] = {-30.5091, -5.47032, -10.1527, -12.3621, 31.1816, 1.75848, 24.2237,
////                                4.82395, 9.36415, 11.2028, 127.668, 3.4266};
////    static float k[12] = {-28.4937, -5.18533, -6.42937, -10.7382, 30.8992, 1.69037, 22.384,
////                                4.56338, 5.88667, 9.71749, 127.927, 3.48884};
////    static float k[12] = {-30.5391, -5.47264, -6.41644, -11.8261, 31.2852, 1.77293, 24.279,
////                                    4.84065, 5.94282, 10.8247, 127.571, 3.41711}; //stable
////    static float k[12] = {-33.05, -6.43966, -6.29929, -11.8764, 33.8235, 1.80119, 29.6502,
////                            6.43986, 6.42464, 12.0064, 124.951, 3.34909};
////    static float k[12] = {-27.4475, -5.03999, -4.54919, -9.91062, 30.7544, 1.65563, 21.4322,
////                                4.431, 4.14963, 8.96358, 128.059, 3.52048};
////    static float k[12] = {-36.659, -6.93428, -15.403, -15.1262, 34.2247, 1.90674, 33.383,
////6.94521, 15.8427, 15.2508, 124.539, 3.23207};
//    static float k[12] = {-31.4278, -5.64914, -9.12272, -13.0702, 30.5878, 1.76264, 24.3345,
//4.81504, 8.19169, 11.5436, 128.198, 3.26649};
//;
//
//    //for leg len 0.16
////    static float k[12] = {-32.777, -4.86406, -22.1531, -18.2478, 11.479, 0.840247,4.03752,
////                                    0.587768, 3.04007, 2.39772, 70.2197, 1.62004};
////    static float k[12] = {-36.7018, -4.90888, -21.6228, -17.7606, 20.2241, 1.79814,7.98464,
////                                     1.11295, 5.69678, 4.45363, 68.8863, 2.92971};
////    static float k[12] = {-34.7211, -4.82942, -21.6326, -17.3591, 19.9865, 1.78136, 7.34347,
////                                     1.08654, 5.6595, 4.31824, 68.9534, 2.93507};
////    static float k[12] = {-31.9324, -4.73588, -21.8163, -17.4679, 17.3433, 1.47578, 5.88453,
////                                     0.908294, 4.90392, 3.7173, 69.2415, 2.3594};
////    static float k[12] = {-31.2109, -4.6175, -21.1877, -16.988, 24.3954, 1.61671, 16.9631,
////                                     2.72936, 14.2944, 10.9716, 133.617, 3.26633};
////    static float k[12] = {-29.4212, -4.35507, -19.1624, -15.321, 38.168, 2.72644, 28.3496,
////                                     4.60954, 23.048, 17.5394, 120.12, 3.2763};
////    static float k[12] = {-36.8465, -4.61177, -19.0532, -16.7307, 39.1778, 2.80838, 38.7559,
////                                     4.9859, 23.4072, 19.6192, 118.84, 3.15784};
////    static float k[12] = {-34.9439, -5.96428, -19.834, -17.231, 34.6763, 2.1215, 31.3259,
////                                     5.84543, 20.6506, 17.3758, 124.117, 3.05482};
////    static float k[12] = {-32.421, -5.46497, -19.412, -16.8826, 37.2464, 2.24369, 31.9144,
////                                    5.9104, 22.1968, 18.7081, 121.147, 2.88972};
//
//    float T[2] = {0}; // 0 T_wheel  1 T_hip
//    float l = leg->real_len;
//    float lsqr = l * l;
//    // float dist_limit = abs(chassis.target_dist - chassis.dist) > MAX_DIST_TRACK ? sign(chassis.target_dist - chassis.dist) * MAX_DIST_TRACK : (chassis.target_dist - chassis.dist); // todo设置值
//    // float vel_limit = abs(chassis.target_v - chassis.vel) > MAX_VEL_TRACK ? sign(chassis.target_v - chassis.vel) * MAX_VEL_TRACK : (chassis.target_v - chassis.vel);
//    for (uint8_t i = 0; i < 2; ++i) {
//        uint8_t j = i * 6;
//        T[i] = (k[j + 0]) * -leg->theta +
//               (k[j + 1]) * -leg->theta_w +
//               (k[j + 2]) * (target_dist - dist) +
//               (k[j + 3]) * (target_v - vel) +
//               (k[j + 4]) * -INS_angle[1] +
//               (k[j + 5]) * -bmi088_real_data.gyro[0];
//    }
////    leg->T_wheel = abs_limit(T[0], 5.0f);
//    leg->T_wheel = abs_limit(T[0], 5.6f);
////    leg->T_hip = abs_limit(T[1], 60.0f);
//    leg->T_hip = T[1];
//}
//
//extern uint32_t g_last_solve_tick;
//float len_diff = 0.0f;
//void Leg_MainLoop() {
//    osDelay(3000); //等待电机启动
//    last_time = HAL_GetTick();
//    while (1) {
//
////        g_legs[0]->motor_f->set_torque=rc_ctrl[0].rc.rocker_l1 / 660.0f * 1.0f;
////        g_legs[1]->_set_force = rc_ctrl[0].rc.rocker_l1 / 660.0f * 50.0f;
//        for (int i = 0; i < LEG_NUM; i++) {
////            g_legs[i]->_set_force = -rc_ctrl[1].rc.rocker_l1 / 660.0f * 50.0f;
////            g_legs[i]->set_torque = rc_ctrl[1].rc.rocker_l_ / 660.0f * 1.0f;
//
//            Leg_InfoUpdate(g_legs[i], INS_angle[1]);
////            g_legs[i]->_set_force = 5.0f;
//            CalcLQR(g_legs[i]);
//        }
//
//        float yaw_w = bmi088_real_data.gyro[2];
//        float target_turn_t = PID_Update(&yaw_w_pid, (rc_ctrl[0].rc.dial / 660.0f * 2.0f + yaw_w), 1);
//        g_legs[0]->T_wheel -= target_turn_t;
//        g_legs[1]->T_wheel += target_turn_t;
//        volatile static float swerving_speed_ff, ff_coef = 3;
//        //        swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
//        float anti_out = PID_Update(&anti_crash_pid, g_legs[0]->phi5 - g_legs[1]->phi5, 1);
//        g_legs[0]->T_hip += anti_out - target_turn_t * 4.0f;
//
//
//        g_legs[1]->T_hip -= anti_out - target_turn_t * 4.0f;
//        uint32_t now = HAL_GetTick();
//
//        vel = (g_legs[0]->wheel_w + g_legs[1]->wheel_w) * WHEEL_RADIUS / 2;
//        dist = dist + vel * (now - last_time) * 0.001f;
//
//        if (rc_ctrl[0].rc.rocker_r1 / 660.0f * 1.5f - target_v > 0.001f) {
//            target_v += 0.001f;
//        } else if (rc_ctrl[0].rc.rocker_r1 / 660.0f * 1.5f - target_v < -0.001f) {
//            target_v -= 0.001f;
//        } else {
//            target_v = (rc_ctrl[0].rc.rocker_r1 / 660.0f * 1.5f);
//        }
//
//        target_dist += target_v * (now - last_time) * 0.001f;
//
//
//        last_time = now;
//
//
////        char usb_data[200];
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->real_len, g_legs[0]->theta,g_legs[1]->real_len, g_legs[1]->theta);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->real_len, PID_Update(g_legs[0]->len_pid, g_legs[0]->real_len, g_legs[0]->set_len) + 20.0f,g_legs[1]->real_len, PID_Update(g_legs[1]->len_pid, g_legs[1]->real_len, g_legs[1]->set_len) + 20.0f);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->real_len, g_legs[0]->motor_f->real_angle,g_legs[0]->motor_b->real_angle,g_legs[1]->real_len);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", INS_angle[0], g_legs[0]->wheel->real_speed,bmi088_real_data.gyro[0], INS_angle[1]);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->motor1->real_angle, g_legs[0]->motor2->real_angle,g_legs[1]->motor1->real_angle, g_legs[1]->motor2->real_angle);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->motor_b->set_torque, g_legs[0]->motor_f->set_torque,
////                g_legs[1]->motor_b->set_torque, g_legs[1]->motor_f->set_torque);
////        sprintf(usb_data, "leg:%f,%f,%f,%f,%f,%f,%f,%f\n", g_legs[0]->motor_b->real_angle, g_legs[0]->motor_f->real_angle, g_legs[1]->motor_b->real_angle, g_legs[1]->motor_f->real_angle, g_legs[0]->real_len, g_legs[0]->theta, g_legs[1]->real_len, g_legs[1]->theta);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->wheel_w, g_legs[0]->wheel->real_speed,g_legs[1]->wheel_w, g_legs[1]->wheel->real_speed);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->real_force, g_legs[0]->real_torque,g_legs[0]->fn, g_legs[0]->_set_force);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", g_legs[0]->set_torque, g_legs[0]->wheel->set_torque,g_legs[1]->set_torque, g_legs[1]->wheel->set_torque);
////        sprintf(usb_data, "leg:%f,%f,%f,%f\n", bmi088_real_data.gyro[0], bmi088_real_data.gyro[1],bmi088_real_data.gyro[2], g_legs[0]->_set_force);
////        LOG(usb_data)
//
//
//        //遥控器关闭急停
//        if (HAL_GetTick() - g_last_solve_tick > 50) {
//            for (int i = 0; i < LEG_NUM; i++) {
//                g_legs[i]->motor_f->target_torque = 0.0f;
//                g_legs[i]->motor_b->target_torque = 0.0f;
//                g_legs[i]->wheel->target_torque = 0.0f;
//                dist = 0.0f;
//                target_dist = 0.0f;
//                len_diff = 0.0f;
//            }
//            osDelay(1);
//            continue;
//        }
//        switch (rc_ctrl[0].rc.switch_left) {
//            case 1://up
//                for (int i = 0; i < LEG_NUM; ++i) {
//                    g_legs[i]->len_pid->target = 0.14f;
//                }
//                break;
//            case 3://mid
//                for (int i = 0; i < LEG_NUM; ++i) {
//                    g_legs[i]->len_pid->target = 0.18f;
//                }
//                break;
//            case 2://down
//                for (int i = 0; i < LEG_NUM; ++i) {
//                    g_legs[i]->len_pid->target = 0.22f;
//                }
//                break;
//        }
//
//
//        if(rc_ctrl[0].rc.switch_right==2 && last_right_switch==3){
//            jump_tick = HAL_GetTick();
//        }
//        last_right_switch = rc_ctrl[0].rc.switch_right;
//
//        static float fns[20] = {0};
//
//        for (int i = 0; i < LEG_NUM; ++i) {
//            //离地检测
//            for (int j = 0; j < 19; ++j) {
//                fns[j] = fns[j + 1];
//            }
//            fns[19] = g_legs[i]->fn;
//            float fn_sum = 0;
//            for (int j = 0; j < 20; ++j) {
//                fn_sum += fns[j];
//            }
//            if (fn_sum / 20 < 2.0f) {
//                if (!is_flying) {
//                    is_flying = 1;
//                }
//            } else {
//                if (is_flying) {
//                    is_flying = 0;
//                }
//            }
//
//            if (is_flying) {
//                g_legs[i]->wheel->target_torque = 0.0f;
//                g_legs[i]->set_torque = -8.0f * g_legs[i]->theta;
//                g_legs[i]->len_pid->target = 0.2f;
//                g_legs[i]->_set_force = ((g_legs[i]->len_pid->target-g_legs[i]->real_len)*400.0f);
//
//                dist = 0.0f;
//                target_dist = 0.0f;
//                len_diff = 0.0f;
//
//            } else {
//                g_legs[i]->wheel->target_torque = abs_limit(g_legs[i]->T_wheel, 5.6f);
////                g_legs[i]->set_torque = abs_limit(g_legs[i]->T_hip, 60.0f);
//                g_legs[i]->set_torque = g_legs[i]->T_hip;
//
//                g_legs[i]->len_pid->target += (i==0?-len_diff:len_diff);
//
//                g_legs[i]->_set_force = (PID_Update(g_legs[i]->len_pid, g_legs[i]->real_len, 1) + 22.2f);
//            }
//
//            if (jump_tick!=0){
//                if(HAL_GetTick()-jump_tick>70){
//                    jump_tick = 0;
//                }else{
//                    g_legs[i]->_set_force = 200.0f;
//                }
//            }
////            g_legs[i]->_set_force += rc_ctrl[0].rc.rocker_l1/660.f*200.0f;
//            Leg_VMC_solve(g_legs[i]);
//        }
//
//        len_diff += 0.0002f*INS_angle[0];
//        osDelay(1);
//    }
//}
//
//void Leg_Register(char *ptr_name, char *motor1_ptrName, char *motor2_ptrName, char *wheel_ptrName) {
//    INTF_Leg_HandleTypeDef *leg = Bus_SharePtr(ptr_name, sizeof(INTF_Leg_HandleTypeDef));
//    memset(leg, 0, sizeof(INTF_Leg_HandleTypeDef));
//
////    leg->_set_force=20.0f;
//
////    leg->set_len = 0.18f;
//
//    leg->motor_b = Bus_SharePtr(motor1_ptrName, sizeof(INTF_Motor_HandleTypeDef));
//    leg->motor_f = Bus_SharePtr(motor2_ptrName, sizeof(INTF_Motor_HandleTypeDef));
//    leg->wheel = Bus_SharePtr(wheel_ptrName, sizeof(INTF_Motor_HandleTypeDef));
//
//    leg->len_pid = JUST_MALLOC(sizeof(PID_HandleTypeDef));
//
//    //todo:考虑在实例化时支持不同腿不同pid参数
//    PID_Init_Config_s len_pid_config = {
//            .Kp = 400.0f,
//            .Ki = 0.0001f,
//            .Kd = 150.0f,
//            .Improve = PID_Integral_Limit,
//            .IntegralLimit = 1.0f,
//    };
//    PID_Init()
//
//    g_legs[idx++] = leg;
//}
//
//void Leg_Init() {
//    Leg_Register("leg-l", "leg-l-b", "leg-l-f", "leg-ML");
//    Leg_Register("leg-r", "leg-r-b", "leg-r-f", "leg-MR");
////    g_legs[0]->_set_force = 0.5f;
//    //8.0f 12.0f
//    PID_Init(&anti_crash_pid, 12.0f, 2.0f, 0.5f, 1.0f, 0);
//    PID_Init(&yaw_w_pid, 1.5f, 0.1f, 0.5f, 0.5f, 0);
//
//    osThreadDef(Leg_MainLoopTask, Leg_MainLoop, osPriorityLow, 0, 2048);
//    Leg_MainLoopTaskHandle = osThreadCreate(osThread(Leg_MainLoopTask), NULL);
//}