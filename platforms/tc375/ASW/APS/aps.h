#ifndef ASW_APS_APS_H_
#define ASW_APS_APS_H_

#include <stdbool.h>
#include <stdint.h>

#include "tof_types.h"
#include "ultrasonic_types.h"

void APS_Init (void);
bool APS_Get_State (void);
void APS_Set_State (bool state);
void APS_Restart (void);
bool APS_Update_Result_Periodic (const ToFData_t *tof_latest_data, const UltrasonicData_t ult_latest_data[],
        uint64_t interval_us);
void APS_Get_Result (int *motor_x, int *motor_y, int64_t *emerAlert_cycle_ms);

#endif /* ASW_APS_APS_H_ */
