// Enable functions (comment out to disable)
#define LORA_ENABLE // LoRa communication
#define TTN_MAPPER_ENABLE
#define SLCAN_ENABLE // SLCAN output on serial port

// TTN keys
#include "ttn_keys.h"

// LoRa send intervals
#define KWH_PER_CHARGE_MESSAGE 1
#define LONG_MESSAGE_INTERVAL_S 30 //(4 * 3600)
#define SHORT_MESSAGE_INTERVAL_S (3 * 60)
#define LONG_MESSAGE_DR DR_SF11
#define SHORT_MESSAGE_DR DR_SF7

// OLED display
#define BLINK_INTERVAL_MS 300
#define MESSAGE_DURATION_MS 3000

// Car parameters
#define KWH_PER_GID 0.08
#define COMPUTE_ECONOMY_ABOVE_KMH 1 // must be >0

// GPS parameters
#define MAX_GPS_UPDATE_PERIOD_MS 1500 // Consider gps data not valid if no location update during this period

// USB serial output
//#define SERIAL_BAUDRATE 115200
//#define SERIAL_BAUDRATE 230400
#define SERIAL_BAUDRATE 460800
//#define SERIAL_BAUDRATE 921600

// TTN mapper mode: sends SF7 messages containing only GPS data every x seconds, on port 2
// TTN console: DR_SF7 message takes 41ms. Interval 30s --> duty cycle 0.13% --> airtime 4.92s / h --> 30s airtime after 6h.
//#define TTN_MAPPER_INTERVAL_S 30
//#define TTN_MAPPER_DR DR_SF7
