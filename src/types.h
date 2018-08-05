struct CarState {
    float batteryStoredKWH;
    float batteryPower;
    float insideTemperature;
    bool  isCharging;
    bool  doorsAreLocked;
    bool  chargeCableIsPlugged;
    bool  climateControlIsOn;
    SemaphoreHandle_t semaphore;
};

struct GpsState {
    float speed;
    float latitude;
    float longitude;
    float altitude;
    float hdop;
    bool  isValid;
    SemaphoreHandle_t semaphore;
};
