#include "motor.h"

#include "gpio.h"
#include "gtm_atom_pwm.h"

void Motor_Init (void)
{
    /* Initialize */
    GtmAtomPwm_Init(); // Init GTM for PWM generation
    GPIO_InitMotor(); // Set dir, break pin as output

    /* Set initial state */
    GtmAtomPwmA_SetDutyCycle(0); // Set duty 0
    GtmAtomPwmB_SetDutyCycle(0); // Set duty 0

    GPIO_SetMotorChADir(true); // Set to forward
    GPIO_SetMotorChBDir(true); // Set to forward

    GPIO_SetMotorChABrake(true); // Activate the brakes
    GPIO_SetMotorChBBrake(true); // Activate the brakes
}

void Motor_movChA_PWM (uint32_t duty, bool dir)
{
    GtmAtomPwmA_SetDutyCycle(duty * 10); // Max input == 100 * 10 (100% PWM duty)
    GPIO_SetMotorChADir(dir); // true(1): 정방향, false(0): 역방향
    GPIO_SetMotorChABrake(false); // 모터 Brake 해제 (true: 정지, false: PWM-A에 따라 동작)
}

void Motor_stopChA (void)
{
    GPIO_SetMotorChABrake(true); // 모터 Brake 활성화
}

void Motor_movChB_PWM (uint32_t duty, bool dir)
{
    GtmAtomPwmB_SetDutyCycle(duty * 10);
    GPIO_SetMotorChBDir(dir);
    GPIO_SetMotorChBBrake(false);
}

void Motor_stopChB (void)
{
    GPIO_SetMotorChBBrake(true);
}
