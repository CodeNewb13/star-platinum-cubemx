#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
#include <stdint.h>

int balance_UP(float Angle, float Mechanical_balance, float Gyro);
void readGreyscale(void);
void invertSensor(void);
int PID(void);
bool FourLineCross(void);
int LinePositionStatus(void);
extern uint8_t sensor[7];
#endif
