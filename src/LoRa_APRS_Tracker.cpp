
#include "GPS_Air530Z.h"
#include "LoRaWan_APP.h"
#include "config.h"
#include "display.h"
#include <APRS-Decoder.h>
#include <Arduino.h>
#include <BMP180.h>
#include <TimeLib.h>
#include <Wire.h>

// initialize the library with the numbers of the interface pins
char   PRESSURESHOW[4];    // initializing a character of size 4 for showing the  result
char   TEMPARATURESHOW[4]; // initializing a character of size 4 for showing the temparature result
BMP085 bmp;
//#include <EEPROM.h>
Air530ZClass   gps;
extern uint8_t isDispayOn; // Defined in LoRaWan_APP.cpp

void setup_lora();
void setup_gps();
void sleep_gps();
void userKey();

String create_lat_aprs(RawDegrees lat);
String create_long_aprs(RawDegrees lng);
String create_lat_aprs_dao(RawDegrees lat);
String create_long_aprs_dao(RawDegrees lng);
String create_dao_aprs(RawDegrees lat, RawDegrees lng);
String createDateString(time_t t);
String createTimeString(time_t t);
String getSmartBeaconState();
String padding(unsigned int number, unsigned int width);

static bool   BatteryIsConnected   = false;
static String batteryVoltage       = "";
static bool DSB_ACTIVE = SB_ACTIVE; // initial Smartbeacon State can be changed via menu
static int DBEACON_TIMEOUT = BEACON_TIMEOUT; // initial Beacon Rate
static bool DSCREEN_OFF = false; // initial Screen timeout deactivated
static int DPROFILE_NR = DEFAULT_PROFILE;
static String DBEACON_SYMBOL = BEACON_SYMBOL;
static String DBEACON_OVERLAY = BEACON_OVERLAY;
static String DBEACON_MESSAGE = BEACON_MESSAGE;
static String DCALLSIGN = CALLSIGN;
static int DGPSMODE = GPSMODE;
static int DSOSTIMEOUT = SOSTIMEOUT;

static bool send_update = true;
static bool is_txing = false;
static RadioEvents_t RadioEvents;

int16_t txNumber;

int16_t rssi, snr, rxSize;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

#define MENU_CNT 8

char *menu[MENU_CNT] = {"Screen OFF", "Sleep", "Send now", "Faster Upd", "Slower Upd", "Tracker mode", "Profile", "Exit"}; //"Reset GPS", "Bat V/%"

enum eMenuEntries
{
  SCREEN_OFF,
  SLEEP,
  SEND_NOW,
  FASTER_UPD,
  SLOWER_UPD,
  TRACKER_MODE,
  PROFILE,
  EXITM
  // RESET_GPS,
  // BAT_V_PCT
};
int  currentMenu   = 0;
bool menuMode      = false;
bool sleepMode     = false;
bool displayBatPct = false; // Change here if you want to see the battery as percent vs voltage (not recommended because it is inacurate unless you go edit some min and max voltage values in the base libraries with values specific to your battery)
bool screenOffMode = false; // Enable normal operation with the screen off - for more battery saving

void                displayMenu();
void                executeMenu(void);
static TimerEvent_t menuIdleTimeout;
static void         OnMenuIdleTimeout();
static TimerEvent_t displayIdleTimeout;
static void         OnDisplayIdleTimeout();
void                switchModeToSleep();
void                VextON(void);
void                VextOFF(void);
void                OnTxDone(void);
void                OnTxTimeout(void);
float               getBattVoltage();
uint8_t             getBattStatus();
void                switchScrenOffMode();
void                switchScrenOnMode();
void                activateProfile(int profileNr);
// int address = 0;
// byte value;
//  cppcheck-suppress unusedFunction
void setup() {
  boardInitMcu();
  Serial.begin(115200);
  delay(500);
  Serial.println("CubeCell LoRa APRS Tracker, DJ1AN");
  setup_display();
  VextON(); // activate RGB Pixel
  show_display("DJ1AN", "CubeCell", "LoRa APRS Tracker", 500);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
  } else {
    BMPFOUND = true;
  }

  setup_gps();
  setup_lora();

  if (PTT_ACTIVE) {
    pinMode(PTT_IO_PIN, OUTPUT);
    digitalWrite(PTT_IO_PIN, PTT_REVERSE ? HIGH : LOW);
  }

  if (BEACON_BUTTON_TX) {
    pinMode(USER_KEY, INPUT);
    attachInterrupt(USER_KEY, userKey, FALLING);
  }

  TimerInit(&menuIdleTimeout, OnMenuIdleTimeout);
  TimerInit(&displayIdleTimeout, OnDisplayIdleTimeout);

  Serial.println("Smart Beacon is " + getSmartBeaconState());
  show_display("INFO", "Smart Beacon is " + getSmartBeaconState(), 500);
  show_display("INFO", "Activate Profile #" + String(DPROFILE_NR), 500);
  activateProfile(DPROFILE_NR);
  Serial.println("Setup done.");
  show_display("INFO", "Init done", "Waiting for GPS", 500);

  if (getBattStatus() > 0) {
    BatteryIsConnected = true;
  }
  batteryVoltage = String(getBattVoltage());
  // EEPROM.begin(512);
}

// cppcheck-suppress unusedFunction
void loop() {
  // userButton.tick();

  // value = EEPROM.read(address);

  // Serial.print(address);
  // Serial.print("\t");
  // Serial.print(value, DEC);
  // Serial.println();
  //  advance to the next address of the EEPROM
  // address = address + 1;

  // there are only 512 bytes of EEPROM, from 0 to 511, so if we're
  // on address 512, wrap around to address 0
  // if (address == 512) {
  //  address = 0;
  //}

  if (EXT_GPS_DATA) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      // Serial.print(c);
      gps.encode(c);
    }
  } else {
    while (gps.available() > 0) {
      char c = gps.read();
      // Serial.print(c);
      gps.encode(c);
    }
  }

  bool          gps_time_update     = gps.time.isUpdated();
  bool          gps_loc_update      = gps.location.isUpdated();
  bool          gps_loc_valid       = gps.location.isValid();
  static time_t nextBeaconTimeStamp = -1;

  static double       currentHeading          = 0;
  static double       previousHeading         = 0;
  static unsigned int rate_limit_message_text = 0;

  if (gps.time.isValid()) {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  }

  if (gps_loc_valid) {
    if (gps_loc_update && nextBeaconTimeStamp <= now()) {
      send_update = true;
      if (DSB_ACTIVE) {
        currentHeading = gps.course.deg();
        // enforce message text on slowest Config.smart_beacon.slow_rate
        rate_limit_message_text = 0;
      } else {
        // enforce message text every n's Config.beacon.timeout frame
        if (DBEACON_TIMEOUT * rate_limit_message_text > 30) {
          rate_limit_message_text = 0;
        }
      }
    }
  }

  static double   lastTxLat       = 0.0;
  static double   lastTxLng       = 0.0;
  static double   lastTxdistance  = 0.0;
  static uint32_t txInterval      = 60000L; // Initial 60 secs internal
  static uint32_t lastTxTime      = millis();
  static int      speed_zero_sent = 0;

  if (!send_update && gps_loc_update && DSB_ACTIVE && gps_loc_valid) {
    uint32_t lastTx = millis() - lastTxTime;
    currentHeading  = gps.course.deg();
    lastTxdistance  = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lastTxLat, lastTxLng);
    if (lastTx >= txInterval) {
      // Trigger Tx Tracker when Tx interval is reach
      // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
      if (lastTxdistance > 20) {
        send_update = true;
      }
    }

    if (!send_update) {
      // Get headings and heading delta
      double headingDelta = abs(previousHeading - currentHeading);

      if (lastTx > SB_MIN_BCN * 1000) {
        // Check for heading more than 25 degrees
        if (headingDelta > SB_TURN_MIN && lastTxdistance > SB_MIN_TX_DIST) {
          send_update = true;
        }
      }
    }
  }

  if (send_update && gps_loc_update && gps_loc_valid) {
    send_update         = false;
    nextBeaconTimeStamp = now() + (DSB_ACTIVE ? SB_SLOW_RATE : DBEACON_TIMEOUT);

    APRSMessage msg;
    String      lat;
    String      lng;
    String      dao;
    String      aprsmsg;

    msg.setSource(DCALLSIGN);
    msg.setDestination("APZASR-1");

    if (!ENHANCE_PRECISION) {
      lat = create_lat_aprs(gps.location.rawLat());
      lng = create_long_aprs(gps.location.rawLng());
    } else {
      lat = create_lat_aprs_dao(gps.location.rawLat());
      lng = create_long_aprs_dao(gps.location.rawLng());
      dao = create_dao_aprs(gps.location.rawLat(), gps.location.rawLng());
    }

    String alt     = "";
    int    alt_int = max(-99999, min(999999, (int)gps.altitude.feet()));
    if (alt_int < 0) {
      alt = "/A=-" + padding(alt_int * -1, 5);
    } else {
      alt = "/A=" + padding(alt_int, 6);
    }

    String course_and_speed = "";
    int    speed_int        = max(0, min(999, (int)gps.speed.knots()));
    if (speed_zero_sent < 3) {
      String speed      = padding(speed_int, 3);
      int    course_int = max(0, min(360, (int)gps.course.deg()));
      /* course in between 1..360 due to aprs spec */
      if (course_int == 0) {
        course_int = 360;
      }
      String course    = padding(course_int, 3);
      course_and_speed = course + "/" + speed;
    }
    if (speed_int == 0) {
      /* speed is 0.
         we send 3 packets with speed zero (so our friends know we stand still).
         After that, we save airtime by not sending speed/course 000/000.
         Btw, even if speed we really do not move, measured course is changeing
         (-> no useful / even wrong info)
      */
      if (speed_zero_sent < 3) {
        speed_zero_sent += 1;
      }
    } else {
      speed_zero_sent = 0;
    }

    if (BMPFOUND && WEATHER_DATA) {
      telemetry();
      String TEMPERATURE = String(TEMP);
      String PRESSURE    = String(PRESS);
      // REMOVE LAST CHARACTER FROM PRESSURE
      PRESSURE.remove(PRESSURE.length() - 1); // PRESSURE ON APRS IS ONLY 5 DIGITS
      // Uncomment to enable APRS telemetry for sensors with more data.
      /// String HUMIDITY = String(HUMID);
      /// String WIND_SPEED    = String(WIND_S);
      /// String WIND_DIRECTION    = String(WIND_DIR);
      /// String RAINX = String(RAIN);
      // Using ... notifies APRS that the fields are NULL thus not showing up
      // in the telemetry.
      String HUMIDITY       = "...";
      String WIND_SPEED     = "...";
      String WIND_DIRECTION = "...";
      String RAINX          = "...";

      Serial.print("Weather data enabled");
      aprsmsg = "!" + lat + DBEACON_OVERLAY + lng + DBEACON_SYMBOL + WIND_DIRECTION + "/" + WIND_SPEED + "g" + RAINX + "t" + TEMPERATURE + "h" + HUMIDITY + "b" + PRESSURE;

      aprsmsg += DBEACON_MESSAGE;
      // t = temprature
      // h = humidity
      // b = pressure
    }

    if (STATIC_BEACON && !WEATHER_DATA) {
      aprsmsg = "!" + DBEACON_LATITUDE + DBEACON_OVERLAY + DBEACON_LONGITUDE + DBEACON_SYMBOL + DBEACON_MESSAGE;
    }

    if (!STATIC_BEACON && !WEATHER_DATA) {
      aprsmsg = "!" + lat + DBEACON_OVERLAY + lng + DBEACON_SYMBOL + course_and_speed + alt;
      // message_text every 10's packet (i.e. if we have beacon rate 1min at high
      // speed -> every 10min). May be enforced above (at expirey of smart beacon
      // rate (i.e. every 30min), or every third packet on static rate (i.e.
      // static rate 10 -> every third packet)
      if (!(rate_limit_message_text++ % 10)) {
        aprsmsg += DBEACON_MESSAGE;
        if (BatteryIsConnected) {
          aprsmsg += " - U: " + batteryVoltage + "V";
        }
      }

      if (ENHANCE_PRECISION) {
        aprsmsg += " " + dao;
        // aprsmsg += " ";
        // aprsmsg += TEMPERATURE;
        // aprsmsg += "C ";
        // aprsmsg += PRESSURE;
        // aprsmsg += "Pa";
      }
    }

    msg.getAPRSBody()->setData(aprsmsg);
    String data = msg.encode();

    // show_display("<< TX >>", data);
    if (LORA_RGB)
      turnOnRGB(COLOR_SEND, 0); // change rgb color
    is_txing = true;

    if (PTT_ACTIVE) {
      digitalWrite(PTT_IO_PIN, PTT_REVERSE ? LOW : HIGH);
      delay(PTT_START_DELAY);
    }

    sprintf(txpacket, "<%c%c%s", (char)(255), (char)(1), data.c_str());
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    Serial.println(txpacket);

    if (DSB_ACTIVE) {
      lastTxLat       = gps.location.lat();
      lastTxLng       = gps.location.lng();
      previousHeading = currentHeading;
      lastTxdistance  = 0.0;
      lastTxTime      = millis();
    }

    if (PTT_ACTIVE) {
      delay(PTT_END_DELAY);
      digitalWrite(PTT_IO_PIN, PTT_REVERSE ? HIGH : LOW);
    }
  }

  if (gps_time_update) {

    if (getBattStatus() > 0) {
      BatteryIsConnected = true;
    }
    batteryVoltage = String(getBattVoltage());

    if (!menuMode && !screenOffMode) {

      // show_display(CALLSIGN + String("") , (is_txing ? "TX" : ""),
      show_display(DCALLSIGN + String(""), is_txing, createDateString(now()) + " " + createTimeString(now()), String("Sats: ") + gps.satellites.value() + " HDOP: " + gps.hdop.hdop(), String("Nxt Bcn: ") + (DSB_ACTIVE ? "~" : "") + createTimeString(nextBeaconTimeStamp), BatteryIsConnected ? (String("Bat: ") + batteryVoltage + "V") : "Powered via USB", String("Smart Beacon: " + getSmartBeaconState()));
    }

    //Serial.print("Sats:");Serial.print(gps.satellites.value());
    //Serial.print(" Lat:");Serial.print(gps.location.lat());
    //Serial.print(" Lon:");Serial.print(gps.location.lng());
    //Serial.print(" Alt:");Serial.print(gps.altitude.meters());
    //Serial.print(" Speed:");Serial.println(gps.speed.kmph());

    if (DSB_ACTIVE) {
      // Change the Tx internal based on the current speed
      int curr_speed = (int)gps.speed.kmph();
      if (curr_speed < SB_SLOW_SPEED) {
        txInterval = SB_SLOW_RATE * 1000;
      } else if (curr_speed > SB_FAST_SPEED) {
        txInterval = SB_FAST_RATE * 1000;
      } else {
        /* Interval inbetween low and high speed
           min(slow_rate, ..) because: if slow rate is 300s at slow speed <=
           10km/h and fast rate is 60s at fast speed >= 100km/h everything below
           current speed 20km/h (100*60/20 = 300) is below slow_rate.
           -> In the first check, if curr speed is 5km/h (which is < 10km/h), tx
           interval is 300s, but if speed is 6km/h, we are landing in this
           section, what leads to interval 100*60/6 = 1000s (16.6min) -> this
           would lead to decrease of beacon rate in between 5 to 20 km/h. what
           is even below the slow speed rate.
        */
        txInterval = min(SB_SLOW_RATE, SB_FAST_SPEED * SB_FAST_RATE / curr_speed) * 1000;
      }
    }
  }


  // SOS Timeout Beacon without Position
  uint32_t lastTx = millis() - lastTxTime;
  if(lastTx > DSOSTIMEOUT && DSOSTIMEOUT > 0){
    if (getBattStatus() > 0){BatteryIsConnected = true;}
    batteryVoltage = String(getBattVoltage());
    APRSMessage msg;
    String aprsmsg;

    aprsmsg = ">" + DBEACON_MESSAGE;

    if (BatteryIsConnected) {
        aprsmsg += " - U: " + batteryVoltage + "V";
    }

    msg.setSource(DCALLSIGN);
    msg.setDestination("APZASR-1");
    msg.getAPRSBody()->setData(aprsmsg);
    String data = msg.encode();
 
    //show_display("<< TX >>", data);
    if(LORA_RGB) turnOnRGB(COLOR_SEND,0); //change rgb color
    is_txing = true;

    if (PTT_ACTIVE) {
      digitalWrite(PTT_IO_PIN, PTT_REVERSE ? LOW : HIGH);
      delay(PTT_START_DELAY);
    }

    sprintf(txpacket, "<%c%c%s", (char)(255), (char)(1), data.c_str());
    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
    Serial.println(txpacket);

    lastTxTime      = millis();

    if (PTT_ACTIVE) {
      delay(PTT_END_DELAY);
      digitalWrite(PTT_IO_PIN, PTT_REVERSE ? HIGH : LOW);
    }

  }


  if ((EXT_GPS_DATA == false) && (millis() > 5000 && gps.charsProcessed() < 10)) {
    Serial.println("Check your GPS - No GPS Data");
  }
  Radio.IrqProcess();

  Radio.IrqProcess( );

  if (sleepMode) {
    lowPowerHandler();
  }
}

void setup_lora() {

  txNumber = 0;
  rssi     = 0;
  snr      = 0;

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  // RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 5000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  show_display("INFO", "LoRa init done!", 500);
  Serial.println("Lora init Done.");
}

void setup_gps() {
  gps.begin();
    gps.begin();
    //gps.sendcmd("$PMTK886,3*2B\r\n");
}

void sleep_gps() {
  gps.end();
}

char *s_min_nn(uint32_t min_nnnnn, int high_precision) {
  /* min_nnnnn: RawDegrees billionths is uint32_t by definition and is n'telth
   * degree (-> *= 6 -> nn.mmmmmm minutes) high_precision: 0: round at decimal
   * position 2. 1: round at decimal position 4. 2: return decimal position 3-4
   * as base91 encoded char
   */

  static char buf[6];
  min_nnnnn = min_nnnnn * 0.006;

  if (high_precision) {
    if ((min_nnnnn % 10) >= 5 && min_nnnnn < 6000000 - 5) {
      // round up. Avoid overflow (59.999999 should never become 60.0 or more)
      min_nnnnn = min_nnnnn + 5;
    }
  } else {
    if ((min_nnnnn % 1000) >= 500 && min_nnnnn < (6000000 - 500)) {
      // round up. Avoid overflow (59.9999 should never become 60.0 or more)
      min_nnnnn = min_nnnnn + 500;
    }
  }

  if (high_precision < 2)
    sprintf(buf, "%02u.%02u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100));
  else
    sprintf(buf, "%c", (char)((min_nnnnn % 1000) / 11) + 33);
  // Like to verify? type in python for i.e. RawDegrees billions 566688333: i =
  // 566688333; "%c" % (int(((i*.0006+0.5) % 100)/1.1) +33)
  return buf;
}

String create_lat_aprs(RawDegrees lat) {
  char str[20];
  char n_s = 'N';
  if (lat.negative) {
    n_s = 'S';
  }
  // we like sprintf's float up-rounding.
  // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
  // ;)
  sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 0), n_s);
  String lat_str(str);
  return lat_str;
}

String create_lat_aprs_dao(RawDegrees lat) {
  // round to 4 digits and cut the last 2
  char str[20];
  char n_s = 'N';
  if (lat.negative) {
    n_s = 'S';
  }
  // we need sprintf's float up-rounding. Must be the same principle as in
  // aprs_dao(). We cut off the string to two decimals afterwards. but sprintf %
  // may round to 60.0000 -> 5360.0000 (53° 60min is a wrong notation ;)
  sprintf(str, "%02d%s%c", lat.deg, s_min_nn(lat.billionths, 1 /* high precision */), n_s);
  String lat_str(str);
  return lat_str;
}

String create_long_aprs(RawDegrees lng) {
  char str[20];
  char e_w = 'E';
  if (lng.negative) {
    e_w = 'W';
  }
  sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 0), e_w);
  String lng_str(str);
  return lng_str;
}

String create_long_aprs_dao(RawDegrees lng) {
  // round to 4 digits and cut the last 2
  char str[20];
  char e_w = 'E';
  if (lng.negative) {
    e_w = 'W';
  }
  sprintf(str, "%03d%s%c", lng.deg, s_min_nn(lng.billionths, 1 /* high precision */), e_w);
  String lng_str(str);
  return lng_str;
}

String create_dao_aprs(RawDegrees lat, RawDegrees lng) {
  // !DAO! extension, use Base91 format for best precision
  // /1.1 : scale from 0-99 to 0-90 for base91, int(... + 0.5): round to nearest
  // integer https://metacpan.org/dist/Ham-APRS-FAP/source/FAP.pm
  // http://www.aprs.org/aprs12/datum.txt
  //

  char str[10];
  sprintf(str, "!w%s", s_min_nn(lat.billionths, 2));
  sprintf(str + 3, "%s!", s_min_nn(lng.billionths, 2));
  String dao_str(str);
  return dao_str;
}

String createDateString(time_t t) {
  return String(padding(day(t), 2) + "." + padding(month(t), 2) + "." + padding(year(t), 4));
}

String createTimeString(time_t t) {
  return String(padding(hour(t), 2) + "." + padding(minute(t), 2) + "." + padding(second(t), 2));
}

String getSmartBeaconState() {
  if (DSB_ACTIVE) {
    return "On";
  }
  return "Off";
}

String padding(unsigned int number, unsigned int width) {
  String result;
  String num(number);
  if (num.length() > width) {
    width = num.length();
  }
  for (unsigned int i = 0; i < width - num.length(); i++) {
    result.concat('0');
  }
  result.concat(num);
  return result;
}

void OnTxDone(void) {
  // Serial.print("TX done!");
  // Radio.Sleep( );
  if (LORA_RGB)
    turnOnRGB(0, 0);
  is_txing = false;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  // Serial.println("TX Timeout......");
  if (LORA_RGB)
    turnOnRGB(0, 0);
  is_txing = false;
}

void userKey(void) {
  delay(10);
  uint16_t keyDownTime = 0;
  while (digitalRead(USER_KEY) == LOW) {
    // Serial.println(digitalRead(USER_KEY));
    keyDownTime++;
    delay(1);
    if (keyDownTime >= 1000) {
      if (menuMode) {
        // Serial.println("Button >1000 in menu");
        executeMenu();

        break;
      } else {
        // Serial.println("Button >1000 outside menu");
        send_update = true;
        break;
      }
    }
  }

  if (keyDownTime < 700) {
    // Serial.println("Button <700");
    // send_update = true;
    // displayMenu();
    // menuMode = true;
    // if (sleepMode)
    //{
    if (screenOffMode) {
      screenOffMode = false;
      VextON();
      setup_display();
      //  display.init();
      isDispayOn = 1;
      TimerSetValue(&displayIdleTimeout, DISPLAY_IDLE_TIMEOUT);
      TimerStart(&displayIdleTimeout);

    }
    // switchModeOutOfSleep();
    //}
    // else if (screenOffMode)
    //{
    //  switchScreenOnMode();
    //}
    // else
    //{
    else if (menuMode) {
      currentMenu++;
      if (currentMenu >= MENU_CNT) {
        currentMenu = 0;
      }
      displayMenu();
    } else {
      menuMode    = true;
      currentMenu = 0;
      displayMenu();
      //  deviceState = DEVICE_STATE_SLEEP;
    }
    TimerSetValue(&menuIdleTimeout, MENU_IDLE_TIMEOUT);
    TimerStart(&menuIdleTimeout);
    // Serial.println("Menu timeout start");
    //}
  } else {
    // keydown > 700
    if (menuMode) {
      TimerSetValue(&menuIdleTimeout, MENU_IDLE_TIMEOUT);
      TimerStart(&menuIdleTimeout);
      // Serial.println("Menu timeout start");
    }
  }
}

void displayMenu() {

  String currentOption = menu[currentMenu];
  String currentValue;
  String currentValue2;
  currentOption.toUpperCase();
  switch (currentMenu) {
  case SCREEN_OFF:
    currentValue  = DSCREEN_OFF ? "Active" : "Inactive";
    currentValue2 = DSCREEN_OFF ? "Display Timeout: " + String(DISPLAY_IDLE_TIMEOUT / 1000) + "s" : "Display Always On";
    break;
  case FASTER_UPD:
    currentValue  = "Fixed Rate: " + String(DBEACON_TIMEOUT) + "s";
    currentValue2 = DSB_ACTIVE ? "When SmartBeacon is OFF" : "";
    break;
  case SLOWER_UPD:
    currentValue  = "Fixed Rate: " + String(DBEACON_TIMEOUT) + "s";
    currentValue2 = DSB_ACTIVE ? "When SmartBeacon is OFF" : "";
    break;
  case TRACKER_MODE:
    currentValue  = DSB_ACTIVE ? "SmartBeacon: Active" : "SmartBeacon: Inactive";
    currentValue2 = DSB_ACTIVE ? "Dynamic Beaconing" : "Fixed Beacon: " + String(DBEACON_TIMEOUT) + "s";
    break;
  case SLEEP:
    currentValue2 = "Go to Sleep Now";
    break;
  case SEND_NOW:
    currentValue2 = "Send Beacon Now";
    break;
  case PROFILE:
    currentValue  = "Profile " + String(DPROFILE_NR) + ": " + DCALLSIGN;
    currentValue2 = DBEACON_MESSAGE;
    break;
  }

  show_display_menu("Menu            " + String(currentMenu + 1) + "/" + String(MENU_CNT), "", currentOption, currentValue, currentValue2);
}

void executeMenu(void) {
  TimerStop(&menuIdleTimeout);

  switch (currentMenu) {
  case SCREEN_OFF:
    if (DSCREEN_OFF) {
      DSCREEN_OFF = false;
      TimerStop(&displayIdleTimeout);
      // menuMode = false;
      displayMenu();
    } else {
      // switchScrenOffMode();
      TimerSetValue(&displayIdleTimeout, DISPLAY_IDLE_TIMEOUT);
      TimerStart(&displayIdleTimeout);
      // menuMode = false;
      DSCREEN_OFF = true;
      displayMenu();
    }
    break;

  case SLEEP:
    // sleepActivatedFromMenu = true;
    switchModeToSleep();
    menuMode = false;
    break;

  case SEND_NOW:
    // deviceState = DEVICE_STATE_SEND;
    send_update = true;
    menuMode    = false;
    break;

  case FASTER_UPD:
    if (DBEACON_TIMEOUT > 10) {
      DBEACON_TIMEOUT -= 10;
    }
    displayMenu();
    // menuMode = false;
    break;

  case SLOWER_UPD:
    DBEACON_TIMEOUT += 10;
    displayMenu();
    // menuMode = false;
    break;

  case TRACKER_MODE:
    // trackerMode = !trackerMode;
    DSB_ACTIVE = !DSB_ACTIVE;
    // DSB_ACTIVE = true;
    // menuMode = false;
    send_update = true;
    displayMenu();
    break;

  case PROFILE:
    DPROFILE_NR += 1;
    if (DPROFILE_NR > 4) {
      DPROFILE_NR = 0;
    }
    activateProfile(DPROFILE_NR);
    displayMenu();
    break;

  case EXITM:
    // displayDebugInfo();
    menuMode = false;
    break;

    // case RESET_GPS:
    //   stopGPS();
    //   delay(1000);
    //   startGPS();
    //   deviceState = DEVICE_STATE_CYCLE;
    //   menuMode = false;
    //   break;

    // case BAT_V_PCT:
    //   displayBatPct = !displayBatPct;
    //   menuMode = false;
    //   break;

  default:
    menuMode = false;
    break;
  }
  // if (!screenOffMode)
  //{
  //   display.clear();
  //   display.display();
  // }
}

static void OnMenuIdleTimeout() {
  TimerStop(&menuIdleTimeout);

  if (menuMode) {
    menuMode = false;
    if (DSCREEN_OFF) {
      TimerReset(&displayIdleTimeout);
    }
  }
}

static void OnDisplayIdleTimeout() {
  TimerStop(&displayIdleTimeout);

  if (menuMode) {
    // restart Timer
    TimerReset(&displayIdleTimeout);
    // TimerSetValue(&displayIdleTimeout, DISPLAY_IDLE_TIMEOUT);
    // TimerStart(&displayIdleTimeout);
  } else {
    // menuMode = false;
    // if (!screenOffMode)
    //{
    //   display.clear();
    //   display.display();
    // }
    // }
    switchScrenOffMode();
  }
}

void switchModeToSleep() {
  sleepMode = true;
  // if (!screenOffMode)
  //{
  // if (!isDispayOn)
  //{
  // display.wakeup();
  // isDispayOn = 1;
  //}
  displayLogoAndMsg("Sleeping...", 2000);
  sleep_display();
  VextOFF();
  // display.sleep();
  isDispayOn = 0;
  //}
  //#ifdef DEBUG
  // else
  //{
  Serial.println("Going to sleep...");
  //}
  //#endif
  sleep_gps();
  Radio.Sleep(); // Not sure this is needed. It is called by LoRaAPP.cpp in various places after TX done or timeout. Most probably in 99% of the cases it will be already called when we get here.
  LoRaWAN.sleep();

  // sendLastLoc = trackerMode; // After wake up, if tracker mode enabled - send the last known location before waiting for GPS
  //#ifdef VIBR_SENSOR
  // setVibrAutoWakeUp();
  //#endif
  // deviceState = DEVICE_STATE_SLEEP;
}

// RGB LED power on
void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

// RGB LED power off
void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

float getBattVoltage() {
  uint16_t battVoltage;
  detachInterrupt(USER_KEY); // reading battery voltage is messing up with the pin and driving it down, which simulates a long press for our interrupt handler

  battVoltage  = getBatteryVoltage();
  float_t batV = ((float_t)battVoltage * VBAT_CORRECTION) / 1000; // Multiply by the appropriate value for your own device to adjust the measured value after calibration
  attachInterrupt(USER_KEY, userKey, FALLING);                    // Attach again after voltage reading is done
  return batV;
}

uint8_t getBattStatus() {
  uint8_t batteryLevel;
  // float_t batteryLevelPct;
  detachInterrupt(USER_KEY); // reading battery voltage is messing up with the pin and driving it down, which simulates a long press for our interrupt handler

  // get Battery Level 1-254 Returned by BoardGetBatteryLevel
  //!!In Practice 0 is never returned? No Battery = 254 !!
  /*                                0: USB,
   *                                 1: Min level,
   *                                 x: level
   *                               254: fully charged,
   *                               255: Error
   */
  batteryLevel = BoardGetBatteryLevel();
  // batteryLevelPct = ((float_t)batteryLevel - BAT_LEVEL_EMPTY) * 100 / (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);

  attachInterrupt(USER_KEY, userKey, FALLING); // Attach again after voltage reading is done
  return batteryLevel;
}

void switchScrenOffMode() {
  screenOffMode = true;
  // displayLogoAndMsg("Scren off....", 2000);
  VextOFF();
  stop_display();
  isDispayOn = 0;
}

void switchScreenOnMode() {
  screenOffMode = false;
  VextON();
  setup_display();
  isDispayOn = 1;
  displayLogoAndMsg("Screen on...", 1000);
  // display.clear();
  // display.display();
}

void activateProfile(int profileNr) {
  Serial.print("Activate Profile #: ");
  Serial.println(profileNr);
  switch (profileNr) {
  case 0:
    DSB_ACTIVE      = SB_ACTIVE;
    DBEACON_TIMEOUT = BEACON_TIMEOUT;
    DBEACON_MESSAGE = BEACON_MESSAGE;
    DBEACON_OVERLAY = BEACON_OVERLAY;
    DBEACON_SYMBOL  = BEACON_SYMBOL;
    DCALLSIGN       = CALLSIGN;
    break;
  case 1:
    DSB_ACTIVE      = P1_SB_ACTIVE;
    DBEACON_TIMEOUT = P1_BEACON_TIMEOUT;
    DBEACON_MESSAGE = P1_BEACON_MESSAGE;
    DBEACON_OVERLAY = P1_BEACON_OVERLAY;
    DBEACON_SYMBOL  = P1_BEACON_SYMBOL;
    DCALLSIGN       = P1_CALLSIGN;
    break;
  case 2:
    DSB_ACTIVE      = P2_SB_ACTIVE;
    DBEACON_TIMEOUT = P2_BEACON_TIMEOUT;
    DBEACON_MESSAGE = P2_BEACON_MESSAGE;
    DBEACON_OVERLAY = P2_BEACON_OVERLAY;
    DBEACON_SYMBOL  = P2_BEACON_SYMBOL;
    DCALLSIGN       = P2_CALLSIGN;
void activateProfile(int profileNr){
  Serial.print("Activate Profile #: ");Serial.println(profileNr);
  String GPSMODESTR;
  switch (profileNr) {
    case 0:
      DSB_ACTIVE = SB_ACTIVE;
      DBEACON_TIMEOUT = BEACON_TIMEOUT;
      DBEACON_MESSAGE = BEACON_MESSAGE;
      DBEACON_OVERLAY = BEACON_OVERLAY;
      DBEACON_SYMBOL = BEACON_SYMBOL;
      DCALLSIGN = CALLSIGN;
      DGPSMODE = GPSMODE;
      DSOSTIMEOUT = SOSTIMEOUT * 1000;

    break;
    case 1:
      DSB_ACTIVE = P1_SB_ACTIVE;
      DBEACON_TIMEOUT = P1_BEACON_TIMEOUT;
      DBEACON_MESSAGE = P1_BEACON_MESSAGE;
      DBEACON_OVERLAY = P1_BEACON_OVERLAY;
      DBEACON_SYMBOL = P1_BEACON_SYMBOL;
      DCALLSIGN = P1_CALLSIGN;
      DGPSMODE = P1_GPSMODE;
      DSOSTIMEOUT = P1_SOSTIMEOUT * 1000;

    break;
    case 2:
      DSB_ACTIVE = P2_SB_ACTIVE;
      DBEACON_TIMEOUT = P2_BEACON_TIMEOUT;
      DBEACON_MESSAGE = P2_BEACON_MESSAGE;
      DBEACON_OVERLAY = P2_BEACON_OVERLAY;
      DBEACON_SYMBOL = P2_BEACON_SYMBOL;
      DCALLSIGN = P2_CALLSIGN;
      DGPSMODE = P2_GPSMODE;
      DSOSTIMEOUT = P2_SOSTIMEOUT * 1000;

    break;
  case 3:
    DSB_ACTIVE      = P3_SB_ACTIVE;
    DBEACON_TIMEOUT = P3_BEACON_TIMEOUT;
    DBEACON_MESSAGE = P3_BEACON_MESSAGE;
    DBEACON_OVERLAY = P3_BEACON_OVERLAY;
    DBEACON_SYMBOL  = P3_BEACON_SYMBOL;
    DCALLSIGN       = P3_CALLSIGN;
    WEATHER_DATA    = true;
    break;
  case 4:
    DSB_ACTIVE        = P4_SB_ACTIVE;
    DBEACON_TIMEOUT   = P4_BEACON_TIMEOUT;
    DBEACON_MESSAGE   = P4_BEACON_MESSAGE;
    DBEACON_OVERLAY   = P4_BEACON_OVERLAY;
    DBEACON_SYMBOL    = P4_BEACON_SYMBOL;
    DBEACON_LATITUDE  = BEACON_LATITUDE;
    DBEACON_LONGITUDE = BEACON_LONGITUDE;
    DCALLSIGN         = P4_CALLSIGN;
    STATIC_BEACON     = true;
  }
}

void telemetry() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  TEMP = bmp.readTemperature();
  // convert temp to farhenheit
  // APRS expects temp in Farenheit
  TEMP = (TEMP * 9 / 5) + 32;
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  PRESS = bmp.readPressure();
  // HUMID = bmp.readHumidity();
  delay(500);
}
  GPSMODESTR = "$PMTK886,"+String(DGPSMODE)+"*2B\r\n";
  gps.sendcmd(GPSMODESTR);
}
