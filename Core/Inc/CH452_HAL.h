/* ===== CH452_HAL.h ===== */
#ifndef __CH452_HAL_H
#define __CH452_HAL_H

#include "stm32f1xx_hal.h"
#include "main.h"      // For hi2c1 handle

// I2C 7-bit address of CH452
#define CH452_I2C_ADDR      (0x30)  // 0x60 >> 1 = 0x30

// Public API
void CH452_Init(void);
void CH452_Clear(void);
void CH452_SetDigit(uint8_t pos, uint8_t val);
void CH452_SetRam(uint8_t pos, uint8_t data);
uint8_t CH452_GetKey(void);

#endif /* __CH452_HAL_H */
