#include "movement.h"
#include "motor.h"
#include "pid.h"
#include "stm32f1xx_hal.h"

/*
 * Motor map
 *              HEAD
 *               ===
 * front left <= 1 4 => front right
 *  rear left <= 2 3 => rear right
 */
extern PID_Controller pid;
extern float yaw;
int calculate_Motor_RPM(const int baseSpeed, int pid) {
  if (pid < 0)
    return 0;
  else
    return pid;
}

int pidcorrection;
void moveForward(int rpm) {
  // while (1) {
  // if(FourLineCross()){
  //   stopMotor();
  //   return;
  // }
  // else{
  pidcorrection = PID_Compute(&pid, 0.0, yaw, 10);
  // pidcorrection = calculate_Motor_RPM(rpm, pidcorrection);
  motorForward(rpm, pidcorrection);
  // HAL_Delay(10);
  //}
  // }
}

bool checkForwardEnd(void) {
  return (sensor_left[1] || sensor_left[2] || sensor_left[3]) &&
         (sensor_right[1] || sensor_right[2] || sensor_right[3]);
}

void moveBackward(int rpm) {
  // while (1) {
  //   if (FourLineCross()) {
  //     stopMotor();
  //     return;
  //   } else {
  //     motorBackward(rpm, 0);
  //   }
  // }
  pidcorrection = PID();
  motorBackward(100, pidcorrection);
}

void moveLeft(int rpm) {

  // motorLeft(rpm);
  // bool leftInitialLine = false;
  //
  // while (!leftInitialLine) {
  //   readGreyscale();
  //   bool lineDetected = false;
  //   for (int i = 0; i < 7; i++) {
  //     if (sensor[i] == 0) {
  //       lineDetected = true;
  //       break;
  //     }
  //   }
  //   if (!lineDetected) {
  //     leftInitialLine = true; // We've left the starting line
  //   }
  // }
  // HAL_Delay(50);
  //
  // while (1) {
  //   int status = LinePositionStatus();
  //   if (status == 1)
  //     motorLeft(rpm / 4); // lower speed
  //   else if (status == 2) {
  //     stopMotor();
  //     return;
  //   }
  // }
  pidcorrection = PID_Compute(&pid, 0.0, yaw, 10);
  Set_Motor1_RPM(-(rpm + pidcorrection));
  Set_Motor2_RPM(rpm - pidcorrection);
  Set_Motor3_RPM(-(rpm - pidcorrection));
  Set_Motor4_RPM(rpm + pidcorrection);
}

void moveRight(int rpm) {
  // while (1) {
  //   int status = LinePositionStatus();
  //   if (!status)
  //     motorRight(rpm);
  //   else if (status == 1)
  //     motorRight(rpm / 4); // lower speed
  //   else if (status == 2) {
  //     stopMotor();
  //     return;
  //   }
  // }
  pidcorrection = PID_Compute(&pid, 0.0, yaw, 10);
  Set_Motor1_RPM(rpm - pidcorrection);
  Set_Motor2_RPM(-(rpm + pidcorrection));
  Set_Motor3_RPM(rpm + pidcorrection);
  Set_Motor4_RPM(-(rpm - pidcorrection));
}

void calibrateOrientation(void) {
  int rpm;
  int mult;
  if (isLeftEmpty() && isRightEmpty()) {
    // Overshoot, move backward
    while (!(isLeftCentered() && isRightCentered())) {
      readGreyscale();
      HAL_Delay(10);
      moveBackward(100);
    }
  } else if (isLeftCentered() && isRightCentered()) {
    // Adjust left/right
    while (!isBackCentered()) {
      readGreyscale();
      if (getLeftRightError() > 0)
        mult = 1;
      else if (getLeftRightError() < 0)
        mult = -1;
      else
        mult = 0;
      rpm = mult * 100;
      motorLeft(rpm, 0);
      HAL_Delay(10);
    }
  } else if (isBackCentered() || isLeftRightSame()) {
    // Adjust forward/backward
    while (!(isLeftCentered() && isRightCentered())) {
      readGreyscale();
      if (getForwardBackwardError() > 0)
        mult = 1;
      else if (getForwardBackwardError() < 0)
        mult = -1;
      rpm = mult * 100;
      motorForward(rpm, 0); // If its negative, it will go backwards
      HAL_Delay(10);
    }
  } else { // rotate
    if (getOrientationError() > 0)
      mult = 1;
    else if (getOrientationError() < 0)
      mult = -1;
    else
      mult = 0;
    rpm = mult * 100;
    motorCW(rpm);
  }
}

void rotateCW(int rpm) {
  while (1) {
    int status = LinePositionStatus();
    if (!status)
      motorCW(rpm);
    else if (status == 1)
      motorCW(rpm / 4); // lower speed
    else if (status == 2) {
      stopMotor();
      return;
    }
  }
}

// Rotate counter clockwise
void rotateCCW(int rpm) {
  while (1) {
    int status = LinePositionStatus();
    if (!status)
      motorCCW(rpm);
    else if (status == 1)
      motorCCW(rpm / 4); // lower speed
    else if (status == 2) {
      stopMotor();
      return;
    }
  }
}

// Motor directions
void stopMotor(void) {
  Set_Motor1_RPM(0);
  Set_Motor2_RPM(0);
  Set_Motor3_RPM(10); // NOTE: must do this, or else it runs too fast at RPM 0
  Set_Motor4_RPM(0);
}

void motorForward(int rpm, int offset) {
  Set_Motor1_RPM(rpm - offset);
  Set_Motor2_RPM(rpm - offset);
  Set_Motor3_RPM(rpm + offset);
  Set_Motor4_RPM(rpm + offset);
}

void motorBackward(int rpm, int offset) {
  Set_Motor1_RPM(-(rpm - offset));
  Set_Motor2_RPM(-(rpm - offset));
  Set_Motor3_RPM(-(rpm + offset));
  Set_Motor4_RPM(-(rpm + offset));
}

void motorLeft(int rpm, int offset) {
  Set_Motor1_RPM(-rpm);
  Set_Motor2_RPM(rpm);
  Set_Motor3_RPM(-rpm);
  Set_Motor4_RPM(rpm);
}

void motorRight(int rpm, int offset) {
  Set_Motor1_RPM(rpm);
  Set_Motor2_RPM(-rpm);
  Set_Motor3_RPM(rpm);
  Set_Motor4_RPM(-rpm);
}

void motorCCW(int rpm) {
  Set_Motor1_RPM(-rpm);
  Set_Motor2_RPM(-rpm);
  Set_Motor3_RPM(rpm);
  Set_Motor4_RPM(rpm);
}

void motorCW(int rpm) {
  Set_Motor1_RPM(rpm);
  Set_Motor2_RPM(rpm);
  Set_Motor3_RPM(-rpm);
  Set_Motor4_RPM(-rpm);
}

// TODO: implement movement based on PID output
