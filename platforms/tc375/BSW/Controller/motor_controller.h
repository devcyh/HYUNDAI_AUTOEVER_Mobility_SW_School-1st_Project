#ifndef BSW_CONTROLLER_MOTOR_CONTROLLER_H_
#define BSW_CONTROLLER_MOTOR_CONTROLLER_H_

#include <stdbool.h>

bool MotorController_ProcessJoystickInput (int x, int y);
bool MotorController_ProcessWASDInput (char key);

#endif /* BSW_CONTROLLER_MOTOR_CONTROLLER_H_ */
