//
// Created by Ukua on 2023/10/25.
//

#include "BSP_bmi088.h"

#include "ist8310driver.h"
#include "justfw_cfg.h"
#include "stdio.h"

#ifdef USE_BOARD_C
extern TIM_HandleTypeDef htim10;

#define HEAT_TIM_HANDLE htim10
#define HEAT_TIM_CHANNEL TIM_CHANNEL_1

#define IMU_temp_PWM(pwm) __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm)  // pwm给定
#endif

#ifdef USE_BOARD_D
extern TIM_HandleTypeDef htim3;

#define HEAT_TIM_HANDLE htim3
#define HEAT_TIM_CHANNEL TIM_CHANNEL_2

#define IMU_temp_PWM(pwm) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm)  // pwm给定

#endif

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(float temp);

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

void AHRS_init(float quat[4], float accel[3], float mag[3]);

void AHRS_update(float time, float gyro[3], float accel[3], float mag[3]);

void get_angle(float quat[4], float *yaw, float *pitch, float *roll);

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

const TaskHandle_t imuTaskHandle;
// tskTaskControlBlock * const imuTaskHandle;

bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_real_data_N;  // bmi088_real_data的相反值，目前只有其gyro

static uint8_t first_temperate;
static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PIDInstance imu_temp_pid;

float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};  // euler angle, unit rad.欧拉角 单位 rad

float INS_quat_N[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // INS_quat的相反值
float INS_angle_N[3] = {0.0f, 0.0f, 0.0f};       // INS_angle的相反值

//----------------多圈计数-----------------
// Warn:注意用其多圈计数前先确保陀螺仪的读数正常(INS_angle读数位于0~PI(-PI))
float _Last_INS_angle[3] = {0};  // 上一次角度，用于多圈计数
int _INS_sum_lap[3] = {0};       // 圈数，用于多圈计数
float INS_SUM_angle[3] = {0};    // 角度的累计值
float INS_SUM_angle_N[3] = {0};  // 角度累计值的相反值
//---------------------------------------

extern SPI_HandleTypeDef BMI088_SPI_HANDLE;
extern DMA_HandleTypeDef BMI088_SPI_DMA_RX_HANDLE;
extern DMA_HandleTypeDef BMI088_SPI_DMA_TX_HANDLE;

static void toEulerAngle(const float q[4], float *roll, float *pitch, float *yaw) {
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q[0] * q[2] - q[3] * q[1]);
    if (fabs(sinp) >= 1)
        *pitch = copysignf(PI / 2, sinp);  // use 90 degrees if out of range
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void BMI088_SPI_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num) {
    SET_BIT(BMI088_SPI_HANDLE.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(BMI088_SPI_HANDLE.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&BMI088_SPI_HANDLE);

    // 失效DMA
    __HAL_DMA_DISABLE(&BMI088_SPI_DMA_RX_HANDLE);

    while (BMI088_SPI_DMA_RX_HANDLE.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&BMI088_SPI_DMA_RX_HANDLE);
    }

    __HAL_DMA_CLEAR_FLAG(&BMI088_SPI_DMA_RX_HANDLE, DMA_LISR_TCIF2);

    BMI088_SPI_DMA_RX_HANDLE.Instance->PAR = (uint32_t)&(BMI088_SPI_HANDLE.Instance->DR);
    // 内存缓冲区1
    BMI088_SPI_DMA_RX_HANDLE.Instance->M0AR = (uint32_t)(rx_buf);
    // 数据长度
    __HAL_DMA_SET_COUNTER(&BMI088_SPI_DMA_RX_HANDLE, num);

    __HAL_DMA_ENABLE_IT(&BMI088_SPI_DMA_RX_HANDLE, DMA_IT_TC);

    // 失效DMA
    __HAL_DMA_DISABLE(&BMI088_SPI_DMA_TX_HANDLE);

    while (BMI088_SPI_DMA_TX_HANDLE.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&BMI088_SPI_DMA_TX_HANDLE);
    }

    __HAL_DMA_CLEAR_FLAG(&BMI088_SPI_DMA_TX_HANDLE, DMA_LISR_TCIF3);

    BMI088_SPI_DMA_TX_HANDLE.Instance->PAR = (uint32_t)&(BMI088_SPI_HANDLE.Instance->DR);
    // 内存缓冲区1
    BMI088_SPI_DMA_TX_HANDLE.Instance->M0AR = (uint32_t)(tx_buf);
    // 数据长度
    __HAL_DMA_SET_COUNTER(&BMI088_SPI_DMA_TX_HANDLE, num);
}

void BMI088_SPI_DMA_ENABLE(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr) {
    // 失效DMA
    __HAL_DMA_DISABLE(&BMI088_SPI_DMA_RX_HANDLE);
    __HAL_DMA_DISABLE(&BMI088_SPI_DMA_TX_HANDLE);
    while (BMI088_SPI_DMA_RX_HANDLE.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&BMI088_SPI_DMA_RX_HANDLE);
    }
    while (BMI088_SPI_DMA_TX_HANDLE.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&BMI088_SPI_DMA_TX_HANDLE);
    }
    // 清除标志位
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));

    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(BMI088_SPI_HANDLE.hdmatx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(BMI088_SPI_HANDLE.hdmatx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(BMI088_SPI_HANDLE.hdmatx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(BMI088_SPI_HANDLE.hdmatx));
    __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(BMI088_SPI_HANDLE.hdmatx));
    // 设置数据地址
    BMI088_SPI_DMA_RX_HANDLE.Instance->M0AR = rx_buf;
    BMI088_SPI_DMA_TX_HANDLE.Instance->M0AR = tx_buf;
    // 设置数据长度
    __HAL_DMA_SET_COUNTER(&BMI088_SPI_DMA_RX_HANDLE, ndtr);
    __HAL_DMA_SET_COUNTER(&BMI088_SPI_DMA_TX_HANDLE, ndtr);
    // 使能DMA
    __HAL_DMA_ENABLE(&BMI088_SPI_DMA_RX_HANDLE);
    __HAL_DMA_ENABLE(&BMI088_SPI_DMA_TX_HANDLE);
}

/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void) {
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(BMI088_SPI_HANDLE.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(BMI088_SPI_HANDLE.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        BMI088_SPI_DMA_ENABLE((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // 开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(BMI088_SPI_HANDLE.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(BMI088_SPI_HANDLE.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        BMI088_SPI_DMA_ENABLE((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(BMI088_SPI_HANDLE.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(BMI088_SPI_HANDLE.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        BMI088_SPI_DMA_ENABLE((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void BSP_SPI_RX_DMA_CB() {
    if (__HAL_DMA_GET_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(BMI088_SPI_HANDLE.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(BMI088_SPI_HANDLE.hdmarx));

        // 陀螺仪读取完毕
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // 加速度计读取完毕
        if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        // 温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);

#ifdef USE_BOARD_C
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
#endif

#ifdef USE_BOARD_D
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
#endif
        }
    }
}

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void INS_task(void const *pvParameters) {
    // wait a time
    osDelay(INS_TASK_INIT_TIME);
    while (BMI088_init()) {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    PID_Init_Config_s imu_temp_pid_config = {
        .Kp = TEMPERATURE_PID_KP,
        .Ki = TEMPERATURE_PID_KI,
        .Kd = TEMPERATURE_PID_KD,
        .Improve = PID_IMPROVE_NONE,
        .IntegralLimit = TEMPERATURE_PID_MAX_IOUT,
        .MaxOut = TEMPERATURE_PID_MAX_OUT,
    };

    PIDInit(&imu_temp_pid, &imu_temp_pid_config);
    //    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);

    // 获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    BMI088_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&BMI088_SPI_HANDLE) != HAL_OK) {
        Error_Handler();
    }

    BMI088_SPI_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        static BaseType_t xHigherPriorityTaskWoken;
        vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    imu_start_dma_flag = 1;

    while (1) {
        // 等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }

        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
                                   &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                         &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

#ifdef USE_IST8310
        extern ist8310_real_data_t ist8310_real_data;
        AHRS_update(0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
#endif
        AHRS_update(0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, NULL);

        INS_quat[0] = q0;
        INS_quat[1] = q1;
        INS_quat[2] = q2;
        INS_quat[3] = q3;
        INS_quat_N[0] = -q0;
        INS_quat_N[1] = -q1;
        INS_quat_N[2] = -q2;
        INS_quat_N[3] = -q3;
        bmi088_real_data_N.gyro[0] = -bmi088_real_data.gyro[0];
        bmi088_real_data_N.gyro[1] = -bmi088_real_data.gyro[1];
        bmi088_real_data_N.gyro[2] = -bmi088_real_data.gyro[2];

        //        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET,
        //                  INS_angle + INS_ROLL_ADDRESS_OFFSET);
        toEulerAngle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET,
                     INS_angle + INS_ROLL_ADDRESS_OFFSET);

        INS_angle_N[0] = -INS_angle[0];
        INS_angle_N[1] = -INS_angle[1];
        INS_angle_N[2] = -INS_angle[2];
        printf("%f",INS_angle_N[2]);
        //----------------多圈计数-----------------
        for (int i = 0; i < 3; ++i) {
            float tmp_angle = PI + INS_angle[i];

            if (INS_angle[i] - _Last_INS_angle[i] > 3)
                _INS_sum_lap[i]--;
            if (INS_angle[i] - _Last_INS_angle[i] < -3)
                _INS_sum_lap[i]++;

            INS_SUM_angle[i] = (float)_INS_sum_lap[i] * 2 * PI + tmp_angle;
            INS_SUM_angle[i] -= PI;
            INS_SUM_angle_N[i] = -INS_SUM_angle[i];
            _Last_INS_angle[i] = INS_angle[i];
        }
    }
}

void AHRS_init(float quat[4], float accel[3], float mag[3]) {
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(float time, float gyro[3], float accel[3], float mag[3]) {
    MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
#ifdef USE_IST8310
    MahonyAHRSupdate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
#endif
}

void get_angle(float q[4], float *yaw, float *pitch, float *roll) {
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(float temp) {
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate) {
        double out = PIDCalculate(&imu_temp_pid, temp, 45.0f);
        if (out < 0.0f) {
            out = 0.0f;
        }
        tempPWM = (uint16_t)out;
        IMU_temp_PWM(tempPWM);
    } else {
        // 在没有达到设置的温度，一直最大功率加热
        // in beginning, max power
        if (temp > 45.0f) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                // 达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.IntegralLimit = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    }
#ifdef USE_BOARD_C
    else if (GPIO_Pin == GPIO_PIN_0) {
        // wake up the task
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
#endif
}

void BSP_bmi088_Init() {
    HAL_TIM_Base_Start(&HEAT_TIM_HANDLE);
    HAL_TIM_PWM_Start(&HEAT_TIM_HANDLE, HEAT_TIM_CHANNEL);

    // extern void imu_cli_register();
    // imu_cli_register();

    // osThreadDef("imuTask", INS_task, osPriorityRealtime, 0, 1024);
    // imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
    xTaskCreate(INS_task, "IMU", 1024, NULL, 240,NULL);
}
