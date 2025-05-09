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
  Set_Motor1_RPM(rpm - pidcorrection);
  Set_Motor2_RPM(rpm - pidcorrection);
  Set_Motor3_RPM(rpm + pidcorrection);
  Set_Motor4_RPM(rpm + pidcorrection);
  // HAL_Delay(10);
  //}
  // }
}

void moveBackward(int rpm) {
  while (1) {
    if (FourLineCross()) {
      stopMotor();
      return;
    } else {
      motorBackward(rpm);
    }
  }
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

void motorForward(int rpm) {
  Set_Motor1_RPM(rpm);
  Set_Motor2_RPM(rpm);
  Set_Motor3_RPM(rpm);
  Set_Motor4_RPM(rpm);
}

void motorBackward(int rpm) {
  Set_Motor1_RPM(-rpm);
  Set_Motor2_RPM(-rpm);
  Set_Motor3_RPM(-rpm);
  Set_Motor4_RPM(-rpm);
}

void motorLeft(int rpm) {
  Set_Motor1_RPM(-rpm);
  Set_Motor2_RPM(rpm);
  Set_Motor3_RPM(-rpm);
  Set_Motor4_RPM(rpm);
}

void motorRight(int rpm) {
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
