#include "ccu.h"

#include "bluetooth.h"
#include "my_stdio.h"
#include "tof.h"
#include "ultrasonic.h"

#include "emer_alert.h"
#include "motor_controller.h"

#include "aeb.h"
#include "aps.h"

#include "output_status.h"

static const uint64_t CYCLE_INTERVAL_US = 50000; // 50000us = 50ms

void run_ccu (void)
{
    static BluetoothData_t bluetooth_latest_data;
    static ToFData_t tof_latest_data;
    static UltrasonicData_t ult_latest_data[ULTRASONIC_COUNT];

    static int pre_motor_x = MOTOR_STOP;
    static int pre_motor_y = MOTOR_STOP;
    static int64_t pre_emerAlert_cycle_ms = EMER_ALERT_OFF;

    while (true)
    {
        /* Process data queues */
        Bluetooth_ProcessQueue();
        ToF_ProcessQueue();
        Ultrasonic_ProcessQueue();

        /* Keep previous output value */
        int motor_x = pre_motor_x;
        int motor_y = pre_motor_y;
        int64_t emerAlert_cycle_ms = pre_emerAlert_cycle_ms;

        /* Get bluetooth data */
        if (Bluetooth_GetLatestData(&bluetooth_latest_data))
        {
//            my_printf("user cmd: %d %d %d %llu\n", bluetooth_latest_data.type, bluetooth_latest_data.param1,
//                    bluetooth_latest_data.param2, bluetooth_latest_data.received_time_us);

            /* Check user commands */
            /* Command priority: 2 (High value - higher priority) */
            switch (bluetooth_latest_data.type)
            {
                case BLUETOOTH_CMD_MOVE :
                    // Set motor inputs
                    motor_x = bluetooth_latest_data.param1;
                    motor_y = bluetooth_latest_data.param2;

                    // APS turns off when user inputs a 'Move' command
                    APS_Set_State(false);

                    break;

                case BLUETOOTH_CMD_PARK :
                    // APS on
                    APS_Set_State(true);

                    break;

                default :
                    break;
            }
        }

        /* Get ToF data */
        if (ToF_GetLatestData(&tof_latest_data))
        {
//            my_printf("ToF/%lf ", tof_latest_data.distance_m);

            /* Update AEB state */
            AEB_Update_State(&tof_latest_data);

            /* Get AEB result */
            /* Command priority: 3 */
            if (AEB_Get_State())
            {
//                APS_Set_State(false); // APS off
            }
            AEB_Get_Result(&motor_x, &motor_y, &emerAlert_cycle_ms);
        }

        /* Get ultrasonic data */
        for (int i = 0; i < ULTRASONIC_COUNT; i++)
        {
            Ultrasonic_GetLatestData(i, &ult_latest_data[i]);
//            my_printf("Ult%d/%d ", ult_latest_data[i].distance_mm);
        }

        /* Check APS & Update APS state */
        if (APS_Get_State() && APS_Update_Result_Periodic(&tof_latest_data, ult_latest_data, CYCLE_INTERVAL_US))
        {
            /* Get APS result */
            /* Command priority: 1 */
            APS_Get_Result(&motor_x, &motor_y, &emerAlert_cycle_ms);
        }

        /* Check motor control input */
        if (!(motor_x == pre_motor_x && motor_y == pre_motor_y))
        {
//            my_printf("%d %d\n", motor_x, motor_y);
            if (MotorController_ProcessJoystickInput(motor_x, motor_y)) // Controll motor
            {
                pre_motor_x = motor_x;
                pre_motor_y = motor_y;
            }
        }

        /* Update emergency alert state */
        EmerAlert_Update_Periodic(emerAlert_cycle_ms);
        pre_emerAlert_cycle_ms = emerAlert_cycle_ms;
    }
}
