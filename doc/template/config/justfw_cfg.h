#ifndef __JUST_CFG_H
#define __JUST_CFG_H

#define SYS_VERSION "v0.5.1"
#define PRINT_OUTPUT_STREAM_NAME USB_TX_BUFFER_NAME  // printf函数重定向

#ifdef STM32F407xx
#define USE_BOARD_C  // 使用C板
#define PLATFROM "C Board"
#endif

#ifdef STM32F446xx
#define PLATFROM "D Board"
#define USE_BOARD_D  // 使用D板

#endif

#endif