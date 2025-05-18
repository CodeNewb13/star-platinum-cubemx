#pragma once

/*
Input 1 2 3
      0 0 0 -> Nothing moving
      0 0 1 -> Conveyor forward
      0 1 0 -> Conveyor reverse
      0 1 1 ->
      1 0 0 -> Rest vertical
      1 0 1 -> Down
      1 1 0 -> Up
      1 1 1 ->
*/
#define CONVEYOR_PUT()                                                         \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);                                   \
  } while (0)

#define CONVEYOR_STOP()                                                        \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);                                   \
  } while (0)

#define CONVEYOR_TAKE()                                                        \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);                                   \
  } while (0)

#define WORM_UP()                                                              \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);                                   \
  } while (0)

#define WORM_DOWN()                                                            \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);                                   \
  } while (0)

#define WORM_DOWN_SENSOR()                                                     \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);                                   \
  } while (0)

#define WORM_UP_SENSOR()                                                       \
  do {                                                                         \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);                                   \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);                                   \
  } while (0)
