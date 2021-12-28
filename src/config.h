// Cubecell GPS - LoRa APRS Config
// 
// you can create three Profiles which can be changed via Menu
//
// Symbol Lookup Table:
// https://blog.thelifeofkenneth.com/2017/01/aprs-symbol-look-up-table.html

#define DEFAULT_PROFILE 0 // Profile to use after Reboot

// Profile 0 - Car Symbol
#define BEACON_SYMBOL ">"
#define BEACON_OVERLAY "/"
#define BEACON_MESSAGE  "Cubecell LoRa Tracker"
#define SB_ACTIVE   true // smart Beacon
#define BEACON_TIMEOUT  60  // fixed Beacon interval in s if Smartbeacon inactive
#define CALLSIGN    "N0CALL-7"   // your callsign

// Profile 1 - Runner Symbol
#define P1_BEACON_SYMBOL "["
#define P1_BEACON_OVERLAY "/"
#define P1_BEACON_MESSAGE  "Running Cubecell LoRa"
#define P1_SB_ACTIVE   true // smart Beacon
#define P1_BEACON_TIMEOUT  60  // fixed Beacon interval in s if Smartbeacon inactive
#define P1_CALLSIGN    "N0CALL-8"   // your callsign

// Profile 2 - Balloon
#define P2_BEACON_SYMBOL "O"
#define P2_BEACON_OVERLAY "/"
#define P2_BEACON_MESSAGE  "Cubecell LoRa powered Balloon"
#define P2_SB_ACTIVE   false // smart Beacon
#define P2_BEACON_TIMEOUT  30  // fixed Beacon interval in s if Smartbeacon inactive
#define P2_CALLSIGN    "N0CALL-12"   // your callsign


// Global Settings for All Profiles
// 

//#ifndef Lora_RGB
#define LORA_RGB true  //activate Neopixel RGB LED
//#endif
#define MENU_IDLE_TIMEOUT     7000     // Auto exit the menu if no button pressed in this amount of ms
#define DISPLAY_IDLE_TIMEOUT     15000     // Auto Shut OFF Display after re-activating in Shut-Off Mode
#define VBAT_CORRECTION       1.004     // Edit this for calibrating your battery voltage

#define RF_FREQUENCY                                433775000 // Hz
#define TX_OUTPUT_POWER                             20        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 200 // Define the payload size here (max 255?)

#define DEBUG false // Enable/Disable debug output over the serial console
#define EXT_GPS_DATA    false   // Feed external GPS NMEA Data via Serial for 

#define PTT_ACTIVE  false // ptt output
#define PTT_IO_PIN  0   // ptt pin
#define PTT_REVERSE false // invert ptt pin
#define PTT_START_DELAY 150 // delay RF for x ms
#define PTT_END_DELAY 0 // delay after sending

// Smart Beacon Parameters
#define SB_TURN_MIN 25 // minimum Turn Angle in degrees
#define SB_SLOW_RATE 300 // every n seconds at SLOW_SPEED (between SLOW and FAST is interpolated)
#define SB_SLOW_SPEED   10 // kmh slow speed
#define SB_FAST_RATE    60 // every n seconds at FAST_SPEED (between SLOW and FAST is interpolated)
#define SB_FAST_SPEED 100 // kmh fast speed
#define SB_MIN_TX_DIST  100 // minimum Distance between Beacons
#define SB_MIN_BCN  5 // seconds minimum Smart Beacon interval

#define BEACON_BUTTON_TX    true // Push Button >2s to Tx Position
#define ENHANCE_PRECISION   true // APRS DAO Extension. Adds two Digits of Precision to Lat and Lon
