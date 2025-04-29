#pragma once
#include "gpio.h"
#include "stm32f1xx_hal_gpio.h"

#define Read_IO1 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)
#define Read_IO2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)
#define Read_IO3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define Read_IO4 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)
#define Read_IO5 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define Read_IO6 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define Read_IO7 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define Read_IO8 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)
#define Read_IO9 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)
#define Read_IO10 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)
#define Read_IO11 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)
#define Read_IO12 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)
#define TT_MotorCD_Enable() GPIO_SetBits(GPIOA, GPIO_PIN_4)
#define TT_MotorCD_Disable() GPIO_ResetBits(GPIOA, GPIO_PIN_4)
