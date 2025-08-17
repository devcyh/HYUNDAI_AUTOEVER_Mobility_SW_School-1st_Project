#ifndef ASW_AEB_AEB_H_
#define ASW_AEB_AEB_H_

#include <stdbool.h>
#include <stdint.h>

#include "tof_types.h"

#include "motor_controller_types.h"

bool AEB_UpdateState (const ToFData_t *tof_latest_data, const MotorControllerData_t *motor_controller_latest_data);
bool AEB_GetState (void);
void AEB_GetResult (int *motor_x, int *motor_y, int64_t *emerAlert_cycle_ms);

#endif /* ASW_AEB_AEB_H_ */
