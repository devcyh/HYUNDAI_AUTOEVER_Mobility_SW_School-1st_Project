#include "Cpu0_Init.h"

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include "bluetooth.h"
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "my_stdio.h"
#include "tof.h"
#include "ultrasonic.h"

#define BLUETOOTH_BUFFER_SIZE 64
#define TOF_BUFFER_SIZE 64
#define ULTRASONIC_BUFFER_SIZE 64

#define BLUETOOTH_MAX_BYTES_PER_CALL (BLUETOOTH_BUFFER_SIZE / 2)
#define TOF_MAX_BYTES_PER_CALL (TOF_BUFFER_SIZE / 2)
#define ULTRASONIC_MAX_EVENTS_PER_CALL (ULTRASONIC_BUFFER_SIZE / 2)

static bool module_init (void)
{
    bool modules_init_success = true;

    modules_init_success = modules_init_success && Bluetooth_Init(BLUETOOTH_BUFFER_SIZE, BLUETOOTH_MAX_BYTES_PER_CALL);
    modules_init_success = modules_init_success && ToF_Init(TOF_BUFFER_SIZE, TOF_MAX_BYTES_PER_CALL);
    modules_init_success = modules_init_success
            && Ultrasonic_Init(ULTRASONIC_BUFFER_SIZE, ULTRASONIC_MAX_EVENTS_PER_CALL);

    Buzzer_Init();
    LED_Init();
    Motor_Init();
    MyStdio_Init();

    return modules_init_success;
}

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

bool core0_init (void)
{
    IfxCpu_enableInterrupts();

    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    return module_init();
}
