#ifndef ASW_AEB_AEB_H_
#define ASW_AEB_AEB_H_

#include <stdbool.h>
#include <stdint.h>

#include "tof.h"

bool AEB_Update_State (const ToFData_t *tof_latest_data);
bool AEB_Get_State (void);
void AEB_Get_Result (int *motor_x, int *motor_y, int64_t *emerAlert_cycle_ms);

#endif /* ASW_AEB_AEB_H_ */
