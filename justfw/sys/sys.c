#include "sys.h"

#include <stdio.h>

#include "BSP_IMU_cfg.h"
#include "intf_sys.h"
#include "justfw_cfg.h"
#include "main.h"

static StreamBufferHandle_t stream;

void Sys_Init() {
    __disable_irq();
    DWT_Init(168);
    delay_init();

    xBusInit();
    vSharedPtrInit();
    Stream_Init();
    /*BSP*/
    BSP_UART_Init();
    BSP_USB_Init();
    BSP_CAN_Init();
    Bsp_Buzzer_Init();

#ifdef USE_BMI088
    BSP_bmi088_Init();
#endif

    // CLI_Init();
    Storage_Init();

    // 避免printf() 无输出
    setvbuf(stdout, NULL, _IONBF, 0);

    __enable_irq();
}
