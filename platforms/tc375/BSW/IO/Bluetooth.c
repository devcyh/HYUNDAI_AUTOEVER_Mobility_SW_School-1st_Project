#include "bluetooth.h"

#include "asclin.h"

#include "byte_queue.h"
#include "my_ctype.h"

#define BLUETOOTH_FRAME_MAX_LEN 8
#define BLUETOOTH_FRAME_TERMINATOR '\0'

static ByteQueue rx_queue;
static int max_bytes_per_call = 32; // 기본값

static BluetoothData_t latest_data;
static bool data_ready = false;

bool Bluetooth_Init (int buffer_size, int max_bytes)
{
    /* Initialize */
    if (!ByteQueue_Init(&rx_queue, buffer_size))
        return false;

    max_bytes_per_call = max_bytes;

    Asclin1_InitUart();

    return true;
}

void Bluetooth_RxHandler (uint8_t byte)
{
    if (byte == '\n') // 필터링
        return;

    ByteQueue_Push(&rx_queue, byte);
}

static bool parseTwoDigits (const uint8_t *str, int32_t *out_value)
{
    if (!str || !out_value)
        return false;

    if (!my_isdigit((unsigned char) str[0]) || !my_isdigit((unsigned char) str[1]))
        return false;

    *out_value = (int32_t) ((str[0] - '0') * 10 + (str[1] - '0'));
    return true;
}

static bool parseBluetoothPacket (const uint8_t *packet, BluetoothData_t *out)
{
    if (packet[0] == 'M') // Move
    {
        int32_t mx, my;
        bool valid_x = parseTwoDigits(&packet[1], &mx);
        bool valid_y = parseTwoDigits(&packet[3], &my);

        if (!valid_x || !valid_y)
            return false; // 형식 오류

        out->type = BLUETOOTH_CMD_MOVE;
        out->param1 = mx;
        out->param2 = my;
        return true;
    }
    else if (packet[0] == 'P') // Parking
    {
        out->type = BLUETOOTH_CMD_PARK;
        out->param1 = 0;
        out->param2 = 0;
        return true;
    }

    return false; // 알 수 없는 명령
}

void Bluetooth_ProcessQueue (void)
{
    static uint8_t packet[BLUETOOTH_FRAME_MAX_LEN];
    static int index = 0;

    uint8_t byte;
    int pop_cnt = 0;

    while (pop_cnt < max_bytes_per_call && ByteQueue_Pop(&rx_queue, &byte))
    {
        pop_cnt++;
        packet[index++] = byte;

        if (byte == BLUETOOTH_FRAME_TERMINATOR)
        {
            index = 0;
            if (parseBluetoothPacket(packet, &latest_data))
            {
                data_ready = true;
            }
        }
        else if (index >= BLUETOOTH_FRAME_MAX_LEN) // 오버플로 방지
        {
            index = 0;
            packet[0] = BLUETOOTH_FRAME_TERMINATOR;
        }
    }
}

bool Bluetooth_GetLatestData (BluetoothData_t *out)
{
    if (!data_ready)
        return false;

    *out = latest_data;
    data_ready = false;
    return true;
}
