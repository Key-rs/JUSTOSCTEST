#ifndef __SYS_H
#define __SYS_H

#include "justfw_cfg.h"

#include "BSP_CAN.h"
#include "BSP_UART.h"
#include "BSP_USB.h"
#include "BSP_W25Qx.h"
#include "BSP_bmi088.h"
#include "CLI.h"
#include "bsp_buzzer.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"
#include "shared_ptr.h"
#include "storage.h"
#include "stream.h"
#include "tinybus.h"

void Sys_Init();

#endif