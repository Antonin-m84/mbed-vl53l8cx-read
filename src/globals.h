
#ifndef _GLOBAL_APP_H_
#define _GLOBAL_APP_H_
#pragma once

#include "mbed.h"

#define NCS_Pin_C D10
#define NCS_C_GPIO_Port GPIOB

#define PWR_EN_C_Pin D11
#define PWR_EN_C_GPIO_Port GPIOB

/* Macros defined for SPI communication */
#define VL53L8CX_COMMS_CHUNK_SIZE 4096
#define SPI_WRITE_MASK(x) (uint16_t)(x | 0x8000)  // 1
#define SPI_READ_MASK(x)  (uint16_t)(x & ~0x8000) // 0


#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;


#ifdef __cplusplus
}
#endif

#endif /* VL53L8CX_H */