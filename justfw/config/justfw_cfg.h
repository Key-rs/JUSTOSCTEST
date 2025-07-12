#ifndef __JUST_CFG_H
#define __JUST_CFG_H

#define SYS_VERSION "v0.5.1"
#define PRINT_OUTPUT_STREAM_NAME USB_TX_BUFFER_NAME  // printf函数重定向

#ifdef STM32F407xx
#define USE_BOARD_C  // 使用C板
#define PLATFROM "C Board"

// #define ARM_MATH_CM4
// #define ARM_MATH_MATRIX_CHECK
// #define ARM_MATH_ROUNDING
// #define ARM_MATH_DSP    // define in arm_math.h

#endif

#endif