#include <Arduino.h>
#include "config.h"
#include "functions.h"
#include "types.h"

//#include "esp_freertos_hooks.h"

// LoRa
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// CAN interface
#include "ESP32CAN.h" // https://github.com/nhatuan84/arduino-esp32-can-demo
#include "CAN_config.h"
CAN_device_t CAN_cfg; // Must be named "CAN_cfg" and global as the name is used in the library

// GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial Serial1(1);
TimerHandle_t gpsTimer;

// I2C (temperature sensor and OLED display)
#include <Wire.h>
SemaphoreHandle_t i2c_semaphore;

// OLED display
#include "displayOled.h"

// Global variables (protected by semaphore)
CarState carState;
GpsState gpsState;

// Functions needed by LMIC
static const u1_t PROGMEM DEVEUI[8]  = _DEVEUI;
void os_getDevEui(u1_t* buf) {
    memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPEUI[8]  = _APPEUI;
void os_getArtEui(u1_t* buf) {
    memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = _APPKEY;
void os_getDevKey(u1_t* buf) {
    memcpy_P(buf, APPKEY, 16);
}

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

// LoRa events
void onEvent(ev_t ev) {
    switch (ev) {
        case EV_TXCOMPLETE:
            // EV_TXCOMPLETE includes waiting for RX windows. Process the received message here.
            if (LMIC.dataLen) {
                displayMessage( "Received message" );
            }
            updateIcon( ICON_LORA, ICON_STATE_ON );
            break;

        case EV_JOINING:
            displayMessage( "Joining TTN" );
            updateIcon( ICON_LORA, ICON_STATE_ACTIVE_BLINK );
            break;

        case EV_JOIN_FAILED:
            displayMessage( "Join TTN failed" );
            updateIcon( ICON_LORA, ICON_STATE_BLINK );
            break;

        case EV_JOINED:
            displayMessage( "TTN joined" );
            // Disable link check validation (automatically enabled during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            updateIcon( ICON_LORA, ICON_STATE_ON );
            break;
    }
}

// Manage LoRa communication
void ttnTask( void * parameter ) {
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Connect to TTN
    LMIC_setDrTxpow(DR_SF7, 1);
    LMIC_startJoining();

    for (;;) {
        // Execute the LMIC scheduler
        os_runloop_once();

        // Let other tasks run
        vTaskDelay(1);
    }
}

void ttnSendTask( void * parameter ) {
    // Variables used to define if a message must be sent
    float previousKWH = 0;
    uint32_t secondsCounterShort = 0;
    uint32_t secondsCounterLong = 0;

    for(;;) {
        // Loop is running once per second
        secondsCounterLong++;
        secondsCounterShort++;

        if ( xSemaphoreTake( carState.semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {

            // Send a long SF11 message every x seconds (e.g. every 4 hours)
            if ( secondsCounterLong >= LONG_MESSAGE_INTERVAL_S ) {
                if ( xSemaphoreTake( gpsState.semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {

                    // Build the message
                    uint8_t message[11];
                    message[0] = carState.isCharging | carState.doorsAreLocked << 3; // Status byte
                    message[1] = 255 * ( carState.batteryStoredKWH / 30 ); // Charge status byte

                    // Scale latitude, longitude and altitude
                    uint32_t latitude  = ( gpsState.latitude + 90 ) * ( 0xFFFFFF / 180.0 ); // range -90 to +90, mapped to 0 to 16'777'215 (3 bytes)
                    uint32_t longitude = ( gpsState.longitude + 180 ) * ( 0xFFFFFF / 360.0 ); // range -180 to +180, mapped to 0 to 16'777'215 (3 bytes)
                    uint32_t altitude  = ( gpsState.altitude + 200 ) * ( 0xFFFF / 5200 ); // range -200 to +5000, mapped to 0 to 65'535 (2 bytes)

                    message[2] =   latitude         & 0xFF;
                    message[3] = ( latitude >>  8 ) & 0xFF;
                    message[4] = ( latitude >> 16 ) & 0xFF;

                    message[5] =   longitude         & 0xFF;
                    message[6] = ( longitude >>  8 ) & 0xFF;
                    message[7] = ( longitude >> 16 ) & 0xFF;

                    message[8] =   altitude        & 0xFF;
                    message[9] = ( altitude >> 8 ) & 0xFF;

                    message[10] = gpsState.hdop * 10;

                    // Send the message
                    updateIcon(ICON_LORA, ICON_STATE_ACTIVE);
                    LMIC_setDrTxpow(LONG_MESSAGE_DR, 1);
                    LMIC_setTxData2(1, message, sizeof(message), 0);

                    // Reset counters
                    secondsCounterLong = 0;
                    secondsCounterShort = 0;
                    previousKWH = carState.batteryStoredKWH;

                    xSemaphoreGive( gpsState.semaphore );
                }
            }

            // TODO: Send a long SF7 message when shutting down the car

            // Send a short SF7 message every x seconds and at each x kWh increase, whatever happens first (e.g. every 3 minutes or every 1kWh increase)
            if ( ( carState.batteryStoredKWH - previousKWH ) >= KWH_PER_CHARGE_MESSAGE
            ||                           secondsCounterShort >= SHORT_MESSAGE_INTERVAL_S ) {

                // Build the message
                uint8_t message[2];
                message[0] = carState.isCharging | carState.doorsAreLocked << 3; // Status byte
                message[1] = 255 * ( carState.batteryStoredKWH / 30 ); // Charge status byte

                // Send the message
                updateIcon(ICON_LORA, ICON_STATE_ACTIVE);
                LMIC_setDrTxpow(SHORT_MESSAGE_DR, 1);
                LMIC_setTxData2(1, message, sizeof(message), 0);

                // Reset counters
                secondsCounterShort = 0;
                previousKWH = carState.batteryStoredKWH;
            }

            xSemaphoreGive( carState.semaphore );
        }

        // Run about once a second
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void ttnMapperTask( void * parameter ) {
    // Variables used to define if a message must be sent
    uint32_t secondsCounter = 0;

    // Variables used to handle the spreading factor
    dr_t ttnMapperDR = DR_SF7;
    int intervalSeconds = 30;


    for(;;) {
        // Loop is running once per second
        secondsCounter++;

        // Change the spreading factor when pressing the "PRG" button
        if ( digitalRead(0) == 0 )
        {
            switch ( ttnMapperDR )
            {
                case DR_SF7:
                    ttnMapperDR = DR_SF8;
                    intervalSeconds = 30;
                    displayMessage("SF8 (30s)");
                    break;

                case DR_SF8:
                    ttnMapperDR = DR_SF9;
                    intervalSeconds = 30;
                    displayMessage("SF9 (30s)");
                    break;

                case DR_SF9:
                    ttnMapperDR = DR_SF10;
                    intervalSeconds = 60;
                    displayMessage("SF10 (60s)");
                    break;

                case DR_SF10:
                    ttnMapperDR = DR_SF11;
                    intervalSeconds = 90;
                    displayMessage("SF11 (90s)");
                    break;

                case DR_SF11:
                    ttnMapperDR = DR_SF12;
                    intervalSeconds = 180;
                    displayMessage("SF12 (180s)");
                    break;

                default:
                    ttnMapperDR = DR_SF7;
                    intervalSeconds = 30;
                    displayMessage("SF7 (30s)");
                    break;
            }
        }


        if ( secondsCounter >= intervalSeconds )
        {
            if ( xSemaphoreTake( gpsState.semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {

                // Build the message
                uint8_t message[9];

                // Scale latitude, longitude and altitude
                uint32_t latitude  = ( gpsState.latitude + 90 ) * ( 0xFFFFFF / 180.0 ); // range -90 to +90, mapped to 0 to 16'777'215 (3 bytes)
                uint32_t longitude = ( gpsState.longitude + 180 ) * ( 0xFFFFFF / 360.0 ); // range -180 to +180, mapped to 0 to 16'777'215 (3 bytes)
                uint32_t altitude  = ( gpsState.altitude + 200 ) * ( 0xFFFF / 5200 ); // range -200 to +5000, mapped to 0 to 65'535 (2 bytes)

                message[0] =   latitude         & 0xFF;
                message[1] = ( latitude >>  8 ) & 0xFF;
                message[2] = ( latitude >> 16 ) & 0xFF;

                message[3] =   longitude         & 0xFF;
                message[4] = ( longitude >>  8 ) & 0xFF;
                message[5] = ( longitude >> 16 ) & 0xFF;

                message[6] =   altitude        & 0xFF;
                message[7] = ( altitude >> 8 ) & 0xFF;

                message[8] = gpsState.hdop * 10;

                // Send the message
                updateIcon(ICON_LORA, ICON_STATE_ACTIVE);
                LMIC_setDrTxpow(ttnMapperDR, 1);
                LMIC_setTxData2(2, message, sizeof(message), 0);

                // Reset counters
                secondsCounter = 0;

                xSemaphoreGive( gpsState.semaphore );
            }
        }

        // Run about once a second
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void canTask( void * parameter ) {
    CAN_frame_t canRxFrame;

    for (;;) {
        //receive next CAN frame from queue
        if ( xQueueReceive( CAN_cfg.rx_queue, &canRxFrame, 50 / portTICK_PERIOD_MS ) ) {
            // Output the received CAN frame to the serial port (SLCAN format)
            #ifdef SLCAN_ENABLE
                printf( "t%03x%d", canRxFrame.MsgID, canRxFrame.FIR.B.DLC );
                for ( int i = 0; i < canRxFrame.FIR.B.DLC; i++ ){
                    printf( "%02x", canRxFrame.data.u8[i] );
                }
                printf("\r");
            #endif

            // Update battery charge level (message 0x5BC)
            if ( canRxFrame.MsgID == 0x5BC ) {
                // Extract the 10 first bits of the message (byte[0] + 2 first bits of byte[1])
                int gids = ( canRxFrame.data.u8[0] << 2 ) | ( canRxFrame.data.u8[1] >> 6 );
                if ( xSemaphoreTake(carState.semaphore, 100 / portTICK_PERIOD_MS) == pdTRUE ) {
                    carState.batteryStoredKWH = ( (float) gids ) * KWH_PER_GID;
                    xSemaphoreGive(carState.semaphore);
                }
            }

            // Update battery power
            if ( canRxFrame.MsgID == 0x1DB ) {
                float current = 0.5 * twosComplementToInt( ( ( canRxFrame.data.u8[0] << 3 ) | ( canRxFrame.data.u8[1] >> 5 ) ), 11 ); // 11 MSBs, 0.5A per LSB, 2's complement
                float voltage = 0.5 * ( ( canRxFrame.data.u8[2] << 2 ) | ( canRxFrame.data.u8[3] >> 6 ) ); // 10 bits, 0.5V per LSB

                if ( xSemaphoreTake(carState.semaphore, 100 / portTICK_PERIOD_MS) == pdTRUE ) {
                    // convert to kW
                    carState.batteryPower = 0.001 * current * voltage;
                    xSemaphoreGive(carState.semaphore);
                }
            }

            // TODO: Update doors locked state

            // TODO: Update charge cable state

            // TODO: Update climate control state

            //
        }
    }
}

void gpsTask( void * parameter ) {

    gpsTimer = xTimerCreate( "gpsTimer", MAX_GPS_UPDATE_PERIOD_MS / portTICK_PERIOD_MS, pdFALSE, NULL, timerCallbackThatDoesNothing );
    xTimerStart(gpsTimer, 10);

    for (;;) {
        // Interpret the data coming from the GPS module
        while ( Serial1.available() ) {
            gps.encode( Serial1.read() );
        }

        // Update the speed
        if ( gps.speed.isUpdated() ) {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.speed = gps.speed.kmph();
                xSemaphoreGive(gpsState.semaphore);
            }
        }

        // Update the altitude
        if ( gps.altitude.isUpdated() ) {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.altitude = gps.altitude.meters();
                xSemaphoreGive(gpsState.semaphore);
            }
        }

        // Update HDOP and GPS icon
        if ( gps.hdop.isUpdated() ) {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.hdop = gps.hdop.hdop();
                xSemaphoreGive(gpsState.semaphore);
            }
        }

        // Update the location and reset the GPS timeout counter
        if ( gps.location.isUpdated() ) {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.longitude = gps.location.lng();
                gpsState.latitude  = gps.location.lat();
                xSemaphoreGive(gpsState.semaphore);
            }
            xTimerReset( gpsTimer, 10 );
        }

        // Set / reset the gpsState.isActive bit and update the GPS icon
        if ( xTimerIsTimerActive( gpsTimer ) == pdTRUE ) {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.isValid = true;
                updateIcon(ICON_GPS, ICON_STATE_ON);
                xSemaphoreGive(gpsState.semaphore);
            }
        }
        else {
            if ( xSemaphoreTake(gpsState.semaphore, 50 / portTICK_PERIOD_MS) == pdTRUE ) {
                gpsState.isValid = false;
                updateIcon(ICON_GPS, ICON_STATE_BLINK);
                xSemaphoreGive(gpsState.semaphore);
            }
        }

        // 128 Bytes (?) buffer, 9600bps -> 133ms to fill the buffer
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void temperatureTask( void * parameter ) {
    uint32_t rawTemp;
    float temp;

    for(;;) {
        if ( xSemaphoreTake( i2c_semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {
            // Get the temperature reading (2 bytes)
            Wire.requestFrom(0x48, 2);

            // data is returned as 2 bytes big endian. 0.5°C/LSB, 9 bits.
            rawTemp = 0;
            rawTemp =   Wire.read() << 1;
            rawTemp |= (Wire.read() >> 7) & 0x01;

            temp = twosComplementToInt(rawTemp, 9) * 0.5;

            xSemaphoreGive( i2c_semaphore );

            // Store the temperature value
            if ( xSemaphoreTake( carState.semaphore, 100 / portTICK_PERIOD_MS ) == pdTRUE ) {
                carState.insideTemperature = temp;
                xSemaphoreGive( carState.semaphore );
            }

            vTaskDelay(500 / portTICK_PERIOD_MS );
        }
    }
}

void setup() {
    // i2c init (for temperature sensor)
    Wire.begin(21, 17);

    // Serial port init (for debugging)
    Serial.begin( SERIAL_BAUDRATE );

    // GPS serial port init
    Serial1.begin(9600, SERIAL_8N1, 22, 23);

    delay(1000);

    // OLED display init
    displayInit();

    // CANBUS initialization
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_12;
    CAN_cfg.rx_pin_id = GPIO_NUM_13;
    CAN_cfg.rx_queue = xQueueCreate( 10, sizeof( CAN_frame_t ) );
    ESP32Can.CANInit();

    // Create the semaphores protecting the global variables
    carState.semaphore = xSemaphoreCreateMutex();
    gpsState.semaphore = xSemaphoreCreateMutex();
    i2c_semaphore      = xSemaphoreCreateMutex();

    // Start tasks
    xTaskCreatePinnedToCore( displayTask /* Task */, "displayTask" /* Task description */, 2048 /* Stack size */, NULL /* Parameter */, 1 /* Priority */, NULL /* Task handle */, 1 /* Core */);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Let the display start (otherwise messages might not be displayed)

    #ifdef LORA_ENABLE
        xTaskCreatePinnedToCore(ttnTask, "ttnTask", 2048, NULL, 5, NULL, 1); // Must run on the same core than arduino
        #ifdef TTN_MAPPER_ENABLE
            xTaskCreatePinnedToCore(ttnMapperTask, "ttnSendTask", 2048, NULL, 2, NULL, 1);
        #else
            xTaskCreatePinnedToCore(ttnSendTask, "ttnSendTask", 2048, NULL, 2, NULL, 1);
        #endif
    #endif

    xTaskCreatePinnedToCore( canTask, "canTask", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore( gpsTask, "gpsTask", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore( temperatureTask, "temperatureTask", 2048, NULL, 3, NULL, 1);
}

void loop() {
/*
    if ( xSemaphoreTake(carState.semaphore, 100 / portTICK_PERIOD_MS) == pdTRUE ) {
        carState.batteryStoredKWH += random(0, 100) / 1000.0;
        carState.batteryPower = random(0, 80000) / 1000.0;
        xSemaphoreGive(carState.semaphore);
    }
    */
    // Serial.println( touchRead( 2 ) );

    // Nothing to do for now...
    vTaskDelay(60000 / portTICK_PERIOD_MS);
}
