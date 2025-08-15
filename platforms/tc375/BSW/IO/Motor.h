#ifndef BSW_IO_MOTOR_H_
#define BSW_IO_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

void Motor_Init (void);

void Motor_movChA_PWM (uint32_t duty, bool dir);
void Motor_stopChA (void);

void Motor_movChB_PWM (uint32_t duty, bool dir);
void Motor_stopChB (void);

#endif /* BSW_IO_MOTOR_H_ */
