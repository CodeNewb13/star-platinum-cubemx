#include "Bsp.h"
#include "pid.h"
#define basespeed 50

void moveForward(int rpm);
void moveBackward(int rpm);
void moveLeft(int rpm);
void moveRight(int rpm);
void rotateCW(int rpm);
void rotateCCW(int rpm);
void stopMotor();
void motorForward(int rpm);
void motorBackward(int rpm);
void motorLeft(int rpm);
void motorRight(int rpm);
void motorCW(int rpm);
void motorCCW(int rpm);