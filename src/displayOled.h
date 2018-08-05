void displayTask( void * parameter );
void displayMessage( String message );
void displayInit();
void updateIcon(int iconNo, int iconState);

// Icons "dictionary"
#define ICON_LORA 0
#define ICON_BLUETOOTH 1
#define ICON_CAR 2
#define ICON_FAN 3
#define ICON_GPS 4

#define ICON_STATE_OFF 0
#define ICON_STATE_ON 1
#define ICON_STATE_BLINK 2
#define ICON_STATE_ACTIVE 3
#define ICON_STATE_ACTIVE_BLINK 4
