//
// Created by Ukua on 2023/10/25.
//

#ifndef JUSTFW_BSP_BMI088_CONFIG_H
#define JUSTFW_BSP_BMI088_CONFIG_H

#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "PID.h"
#include "cmsis_os.h"
#include "interface.h"
#include "math.h"
#include "tinybus.h"

// #include "bmi08x.h"
#include "arm_math.h"

#ifdef USE_BOARD_C
#define BMI088_SPI_HANDLE hspi1
#define BMI088_SPI_DMA_RX_HANDLE hdma_spi1_rx
#define BMI088_SPI_DMA_TX_HANDLE hdma_spi1_tx

#endif

#ifdef USE_BOARD_D
#define BMI088_SPI_HANDLE hspi2
#define BMI088_SPI_DMA_RX_HANDLE hdma_spi2_rx
#define BMI088_SPI_DMA_TX_HANDLE hdma_spi2_tx
#endif

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

#define SPI_DMA_GYRO_LENGHT 8
#define SPI_DMA_ACCEL_LENGHT 9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS 0
#define IMU_SPI_SHFITS 1
#define IMU_UPDATE_SHFITS 2
#define IMU_NOTIFY_SHFITS 3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define TEMPERATURE_PID_KP 1600.0f  // 温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f     // 温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f     // 温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT 4500.0f   // 温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  // 温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000  // mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define INS_TASK_INIT_TIME 7  // 任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

void BSP_bmi088_Init();

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
extern void INS_task(void const *pvParameters);

/**
 * @brief          校准陀螺仪
 * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
 * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
 * @retval         none
 */
extern void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count);

/**
 * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂
 * @retval         none
 */
extern void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3]);

/**
 * @brief          获取四元数
 * @param[in]      none
 * @retval         INS_quat的指针
 */
extern const float *get_INS_quat_point(void);

/**
 * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
 * @param[in]      none
 * @retval         INS_angle的指针
 */
extern const float *get_INS_angle_point(void);

/**
 * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
 * @param[in]      none
 * @retval         INS_gyro的指针
 */
extern const float *get_gyro_data_point(void);

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
 * @param[in]      none
 * @retval         INS_gyro的指针
 */
extern const float *get_accel_data_point(void);

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
 * @param[in]      none
 * @retval         INS_mag的指针
 */
extern const float *get_mag_data_point(void);

#endif  // JUSTFW_BSP_BMI088_CONFIG_H
