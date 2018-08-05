#include <Arduino.h>
#include "config.h"
#include "functions.h"
#include "types.h"
#include "globalVariables.h"

#include "displayOled.h"

// OLED display
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(/* rotation=*/ U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// Bitmaps
#include "../bitmap/lora_sending.xbm"
#include "../bitmap/lora_not_sending.xbm"
#include "../bitmap/bluetooth.xbm"
#include "../bitmap/car.xbm"
#include "../bitmap/fan.xbm"
#include "../bitmap/gps.xbm"

QueueHandle_t nextMessageQueue;
QueueHandle_t nextIconUpdateQueue;
TimerHandle_t blinkTimer;

void displayMessage( String message ) {
    xQueueOverwrite(nextMessageQueue, &message);
}

void updateIcon(int iconNo, int iconState) {
    int storedIcon[2] = {iconNo, iconState};
    xQueueSend(nextIconUpdateQueue, &storedIcon, 200 / portTICK_PERIOD_MS);
}

void displayInit() {
    // Initialize the OLED display
    u8g2.begin();

    nextMessageQueue = xQueueCreate(1, sizeof( String ) );
    nextIconUpdateQueue = xQueueCreate(10, sizeof( int[2] )); // e.g. {ICON_CAR, ICON_STATE_OFF}

    // Setup and start the timer which makes the icons blink
    blinkTimer = xTimerCreate( "blinkTimer", BLINK_INTERVAL_MS / portTICK_PERIOD_MS, pdFALSE, NULL, timerCallbackThatDoesNothing );
    xTimerStart(blinkTimer, 10);
}

// Show info on the OLED display
void displayTask( void * parameter ) {
    String nextMessage;

    bool blinkState = false;
    int iconState[5];

    iconState[ ICON_LORA      ] = ICON_STATE_OFF;
    iconState[ ICON_BLUETOOTH ] = ICON_STATE_OFF;
    iconState[ ICON_CAR       ] = ICON_STATE_OFF;
    iconState[ ICON_FAN       ] = ICON_STATE_OFF;
    iconState[ ICON_GPS       ] = ICON_STATE_OFF;

    for (;;) {
        // Start with an empty screen, dark background, light foreground
        u8g2.clearBuffer();
        u8g2.setDrawColor(1);

        // If there is a message, display it instead of the standard display
        if ( xQueueReceive(nextMessageQueue, &nextMessage, 0) ) {
            // Background rectangle
            u8g2.drawFrame(0, 0, 128, 64);
            u8g2.drawBox(2, 2, 124, 60);

            // Draw the text black, centered
            u8g2.setDrawColor(0);
            u8g2.setFont(u8g2_font_profont17_tr); // Height 17, Width 9
            u8g2.drawStr( ( ( 128 - 9 * nextMessage.length() ) / 2 ), 36, nextMessage.c_str()); // Start position x = ( displayWidth - charWidth * textLength ) / 2

            // Display the message for some time
            u8g2.sendBuffer();
            vTaskDelay(MESSAGE_DURATION_MS / portTICK_PERIOD_MS);
        }

        // If there is no message to display, draw the standard display
        else {
            // Variables that will contain formatted strings for display
            char batteryStoredKWHString[16];
            char batteryPowerString[16];
            char speedString[16];
            char economyString[16];
            char temperatureString[16];

            // Read and format numerical data
            if ( xSemaphoreTake(carState.semaphore, 100 / portTICK_PERIOD_MS) == pdTRUE ) {
                sprintf( batteryStoredKWHString, "%4.1f", carState.batteryStoredKWH );
                sprintf( batteryPowerString,     "%5.1f", carState.batteryPower );
                sprintf( temperatureString,     "%4.1fC", carState.insideTemperature );

                if ( xSemaphoreTake(gpsState.semaphore, 100 / portTICK_PERIOD_MS) == pdTRUE  ) {
                    // Display speed only if valid
                    if ( gpsState.isValid ) {
                        sprintf( speedString, "%3.0f", gpsState.speed );
                    }
                    else {
                        sprintf( speedString, "---" );
                    }

                    // Display economy only if speed is valid and car is moving
                    if ( gpsState.isValid
                    && ( gpsState.speed > COMPUTE_ECONOMY_ABOVE_KMH ) ) {
                        sprintf( economyString,          "%5.1f", carState.batteryPower / gpsState.speed ); // kW / km/h = kWh/km
                    }
                    else {
                        sprintf( economyString,          "  ---");
                    }
                    xSemaphoreGive(gpsState.semaphore);
                }
                xSemaphoreGive(carState.semaphore);
            }

            // Display numerical data
            u8g2.setFont(u8g2_font_profont17_tr); // Height 17, Width 9
            u8g2.drawStr(   9, 16, speedString );
            u8g2.drawStr(  64, 16, economyString );
            u8g2.drawStr(   0, 38, batteryStoredKWHString );
            u8g2.drawStr(  64, 38, batteryPowerString );
            u8g2.drawStr(  53, 63, temperatureString );

            // Small kWh and kW display
            u8g2.setFont(u8g2_font_artossans8_8r); // Height 7, Width 8
            u8g2.drawStr(  38,  38, "kWh" );
            u8g2.drawStr( 112,  38, "kW" );

            // km/h "logo"
            u8g2.drawStr(  38,  7, "km" );
            u8g2.drawHLine(38,  9, 16);
            u8g2.drawStr(  42, 19, "h" );

            // kWh/km "logo"
            u8g2.drawStr(112,   7, "Wh" );
            u8g2.drawHLine(112, 9, 16);
            u8g2.drawStr(112,  19, "km" );

            // Update the state (local variable) of all the icons for which it is needed
            for(;;) {
                int currentIconBuffer[2];
                if ( xQueueReceive(nextIconUpdateQueue, currentIconBuffer, 10 / portTICK_PERIOD_MS ) ) {
                    iconState[ currentIconBuffer[0] ] = currentIconBuffer[1];
                }
                else {
                    // Exit the loop when the queue is empty
                    break;
                }
            }

            // Define if a blinking icon must be ON or OFF
            if ( xTimerIsTimerActive( blinkTimer ) == pdFALSE ) {
                blinkState = !blinkState;
                xTimerReset( blinkTimer, 10);
            }

            // Update the display for each icon
            bool iconIsOn = false;

            for ( int iconNo = 0; iconNo < ( sizeof( iconState ) / sizeof( iconState[0] ) ); iconNo++ ) {
                // Define if the icon must be ON or OFF
                switch ( iconState[ iconNo ] ) {
                    case ICON_STATE_OFF :
                        iconIsOn = false;
                        break;

                    case ICON_STATE_ON :
                    case ICON_STATE_ACTIVE :
                        iconIsOn = true;
                        break;

                    case ICON_STATE_BLINK :
                    case ICON_STATE_ACTIVE_BLINK :
                        iconIsOn = blinkState;
                        break;

                    default :
                        iconIsOn = false;
                        break;
                }

                // Draw the icons that must be displayed
                if ( iconIsOn ) {
                    switch ( iconNo ) {
                        // LoRa icon can show if data is currently being sent (active) or not
                        case ICON_LORA :
                            if (   ( iconState[ iconNo ] == ICON_STATE_ACTIVE )
                                || ( iconState[ iconNo ] == ICON_STATE_ACTIVE_BLINK )
                                ) {
                                u8g2.drawXBM(  0, 50, lora_sending_width, lora_sending_height, lora_sending_bits);
                            }
                            else {
                                u8g2.drawXBM(  0, 50, lora_not_sending_width, lora_not_sending_height, lora_not_sending_bits);
                            }
                            break;

                        case ICON_BLUETOOTH :
                            u8g2.drawXBM( 31, 48, bluetooth_width,   bluetooth_height,   bluetooth_bits);
                        break;
/* Do not display these 2 icons, space used by temperature
                        case ICON_CAR :
                            u8g2.drawXBM( 53, 52, car_width,         car_height,         car_bits);
                        break;

                        case ICON_FAN :
                            u8g2.drawXBM( 85, 48, fan_width,         fan_height,         fan_bits);
                        break;
*/
                        case ICON_GPS :
                            u8g2.drawXBM(112, 48, gps_width,         gps_height,         gps_bits);
                        break;

                    }
                }
            }
            if ( xSemaphoreTake( i2c_semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {
                u8g2.sendBuffer();
                xSemaphoreGive( i2c_semaphore );
            }
        }
    }
}
