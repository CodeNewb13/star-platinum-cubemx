#ifndef __PID_H
#define __PID_H

#include "Bsp.h"
#include <stdint.h>
#include <stdbool.h>


int balance_UP(float Angle, float Mechanical_balance, float Gyro);
void readGreyscale();
void invertSensor ();
int PID();
bool FourLineCross();
int LinePositionStatus();
extern uint8_t sensor[7];
#endif
