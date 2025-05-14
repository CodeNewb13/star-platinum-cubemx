#include "motor.h"
#include "gpio.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"

void Motor_Init(void) {
  // HAL_GPIO_WritePin(Motor1_Enable_GPIO_Port, Motor1_Enable_Pin,
  // GPIO_PIN_SET); // Is already set by gpio.c, from cubemx
  // For motor 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // For motor 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  // For motor 3
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Set_Motor3_RPM(10); // It moves itself somehow
  // For motor 4
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void TT_Motor_Init(void) {
  // For motor 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // For motor 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  // For motor 3
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Set_Motor3_RPM(10); // It moves itself somehow
  // For motor 4
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Set_Motor1_RPM(int RPM) {
  RPM = -RPM;
  if (RPM > 400)
    RPM = 400;
  if (RPM < -400)
    RPM = -400;
  if (RPM >= 0) {
    // RPM = RPM * 0.8 + 200;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
  } else {
    // RPM = RPM * 0.8 - 200;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000 + RPM);
  }
}

void Set_Motor2_RPM(int RPM) {
  // solder kebalik
  if (RPM > 400)
    RPM = 400;
  if (RPM < -400)
    RPM = -400;
  if (RPM == 0) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
  } else if (RPM > 0) {
    // RPM = RPM * 0.8 + 200;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
  } else {
    // RPM = RPM * 0.8 - 200;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000 + RPM);
  }
}

void Set_Motor3_RPM(int RPM) {
  if (RPM > 400)
    RPM = 400;
  else if (RPM < -400)
    RPM = -400;

  if (RPM > 0)
    RPM = 1000 - RPM; // HOTFIX: somehow it works, without this this motor
  // moves too fast

  if (RPM >= 0) {
    // RPM = RPM * 0.8 + 200;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
  } else {
    // RPM = RPM * 0.8 - 200;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000 + RPM);
  }
}

void Set_Motor4_RPM(int RPM) {
  if (RPM > 400)
    RPM = 400;
  if (RPM < -400)
    RPM = -400;
  if (RPM == 0) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
  } else if (RPM > 0) {
    // RPM = RPM * 0.8 + 200;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000 - RPM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
  } else {
    // RPM = RPM * 0.8 - 200;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000 + RPM);
  }
}
