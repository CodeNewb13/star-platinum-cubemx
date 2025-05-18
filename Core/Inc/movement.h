#pragma once
#include "bsp.h"
#include "pid.h"
#define basespeed 50

void moveForward(int rpm);
bool checkForwardEnd(void);
bool checkLeftEnd(void);
void moveBackward(int rpm);
void moveLeft(int rpm);
void moveRight(int rpm);
void calibrateOrientation(void);
void rotateCW(int rpm);
void rotateCCW(int rpm);
void stopMotor(void);
void motorForward(int rpm, int offset);
void motorBackward(int rpm, int offset);
void motorLeft(int rpm, int offset);
void motorRight(int rpm, int offset);
void motorCW(int rpm);
void motorCCW(int rpm);

extern uint8_t sensor_back[3];
extern uint8_t sensor_left[4];
extern uint8_t sensor_right[4];
