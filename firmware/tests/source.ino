/**************************************************************
  1. RAKwireless WIsBlock Meshtastic Starter Kit US915 SKU 116016 
      --> has WisBlock Base (RAK 19007)
      --> WIsBlock Core (RAK4611) which has --> (RAK4631 and this already has RAK 4630 soldered )

  2. RAK15002 SD (SD Card Module with FAT32 form and Module PID 100031) 
  3. RAK12500 (GPS location module) (u-blox ZOE-M8Q) GNSS logger
  - Logs battery voltage + estimated % + uptime + GNSS fix & position
  - CSV on SD: track.csv
  - 10-minute duty cycle
  - Each cycle: wake GNSS -> attempt fix for up to 90s -> log -> GNSS power save
**************************************************************/
#include "GnssTypes.h"
#include <Adafruit_TinyUSB.h>  // keeps Serial stable on many nRF52 cores
#include <SPI.h> //Serial Peripheral Interface, fast data transfer between microcontroller and peripheral so need this for SD Card!
#include <SD.h> //using Arduino built in
#include <Wire.h> //need this to turn on MCU's I2C controller or GNSS won't initialize
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


SFE_UBLOX_GNSS myGNSS;

// ====== SETTINGS ======
//moving 10 feet is notable shift
#define USE_GNSS               1          // set 0 for battery-only logging

//cycle settings
#define LOG_INTERVAL_MS        60000UL   // 1 minute in milliseconds 
#define GNSS_ATTEMPT_MS         10000UL   // 10 seconds in milliseconds

#define MIN_SIV_FOR_VALID           4     // require at least 4 satellites in view
#define CSV_PATH            "track.csv"   // IMPORTANT: no leading "/"
#define SD_RETRY_MS            30000UL    // retry SD every 30s if it fails

// ====== STATE ======
bool sdOK = false;
unsigned long lastLog = 0;
unsigned long lastSDRetry = 0;

// ===================== BATTERY =====================
float readBatteryV() {
  int raw = analogRead(WB_A0);
  return raw * (3.6f / 1023.0f) * 2.0f; // the tested scale
}

int batteryPercentFromVoltage(float v) {
  if (v >= 4.20f) return 100;
  if (v <= 3.20f) return 0;

  float pct;
  if (v > 4.00f) {
    pct = 80.0f + (v - 4.00f) * (20.0f / 0.20f);
  } else if (v > 3.70f) {
    pct = 20.0f + (v - 3.70f) * (60.0f / 0.30f);
  } else {
    pct = 0.0f + (v - 3.20f) * (20.0f / 0.50f);
  }

  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)(pct + 0.5f);
}

// ===================== SD =====================
void tryInitSD() {
  Serial.println("Init SD (RAK15002 style)...");
  delay(300);

  if (!SD.begin()) {
    Serial.println("SD mount failed (FAT32? module seated? card inserted?)");
    sdOK = false;
    return;
  }

  Serial.println("✅ SD mounted");
  sdOK = true;

  if (!SD.exists(CSV_PATH)) {
    File f = SD.open(CSV_PATH, FILE_WRITE);
    if (f) {
      f.println("uptime_min,batt_v,batt_pct,fixType,siv,lat_1e7,lon_1e7,surfaceFix");
      f.close();
      Serial.println("✅ Created track.csv with header");
    } else {
      Serial.println("❌ Could not create track.csv");
      sdOK = false;
    }
  }
}

// ===================== GNSS =====================
bool initGNSS() {
#if USE_GNSS
  Serial.println("Init GNSS...");
  Wire.begin(); //turns on MCU's i2C controller (SDA/Data, SCL/Clock) --> send to RAK12500 GNSS Module
  //InterIntegrated Circuit communication for chips on same board 

  if (!myGNSS.begin(Wire)) { //myGNSS.begin(Wire) tells GNSS library to use I2C bus to talk to GNSS chip
    Serial.println("❌ GNSS not detected on I2C");
    return false;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);

  // Put GNSS into u-blox power save between attempts (should be supported by library version)
  myGNSS.powerSaveMode(true);

  Serial.println("✅ GNSS ready (power save enabled)");
  return true;
#else
  return true;
#endif
}

// -------- GNSS power wrappers --------
void wakeGNSS() {
#if USE_GNSS
  myGNSS.powerSaveMode(false);  // disable GNSS power save
  delay(100);
#endif
}

void sleepGNSS() {
#if USE_GNSS
  myGNSS.powerSaveMode(true);   // enable GNSS power save
#endif
}

// -------- Attempt GNSS fix for up to 90s --------
//GnssSnapshot defined in GnssTypes.h
GnssSnapshot getGnssSnapshot() {
  GnssSnapshot snapshot;
  snapshot.fixType = -1; 
  snapshot.siv = -1;
  snapshot.lat = snapshot.lon = 0;
  snapshot.surfaceFix = false;

#if USE_GNSS
  wakeGNSS();

  unsigned long start = millis();

  //while statement means try to get a fix for up to 10 seconds
  while (millis() - start < GNSS_ATTEMPT_MS) {
    int fix = (int)myGNSS.getFixType(); 
    int siv = (int)myGNSS.getSIV();

    snapshot.fixType = fix;
    snapshot.siv = siv;

    // accept if we have fix + enough satellites
    //if gnss gets a valid fix early, we break out of the loop so like from 2 seconds 
    if (fix >= 2 && siv >= MIN_SIV_FOR_VALID) {
      snapshot.lat = myGNSS.getLatitude();
      snapshot.lon = myGNSS.getLongitude();
      snapshot.surfaceFix = true;
      break;
    }

    delay(250); //originally did delay 1000 but changed to 250 to poll faster inside the 10s window
  } //end of while loop

  unsigned long duration = millis() - start;
  Serial.print("GNSS attempt duration ms = ");
  Serial.println(duration);

  sleepGNSS();
#endif

  return snapshot;
} //end of function

//Basically two main parts of of code you have to do void setup () and then void loop()
//===================== SETUP (runs once) =====================
void setup() {
  unsigned long timeout = millis();
  Serial.begin(115200);
  while (!Serial) {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }

  Serial.println("\n ✅ Firmware starting: Battery + GNSS + SD logger");

  tryInitSD();

#if USE_GNSS
  if (!initGNSS()) {
    Serial.println("⚠️ GNSS init failed (will log battery + no-fix)");
  }
#endif

  lastLog = millis();
  lastSDRetry = millis();
} //end of void setup


// ===================== LOOP (runs forever) =====================
void loop() {
  // retry SD sometimes if it failed (when sdOK ==false) so we try to remount
  // --> can reseat Sd card and recover without rebooting
  if (!sdOK && (millis() - lastSDRetry >= SD_RETRY_MS)) {
    lastSDRetry = millis();
    tryInitSD();
  }

  //MAIN SCHEDULER
  // main duty-cycle trigger, logs every interval
  if (millis() - lastLog >= LOG_INTERVAL_MS) { //says only do a logging cycle every LOG_INTERVAL_MS
    lastLog += LOG_INTERVAL_MS; //keeps timing stable even if cycle takes a little time

    //takes the battery snapshot
    float battV = readBatteryV(); //reads voltage
    int battPct = batteryPercentFromVoltage(battV); //converts to approximate percentage
    float uptimeMin = millis() / 60000.0f;  //logs uptime minutes

    GnssSnapshot gSnapshot = getGnssSnapshot();

    // serial print (once per cycle)
    Serial.print("LOG t=");
    Serial.print(uptimeMin, 2);
    Serial.print(" min batt=");
    Serial.print(battV, 3);
    Serial.print("V (");
    Serial.print(battPct);
    Serial.print("%)");

#if USE_GNSS
    Serial.print(" fix=");
    Serial.print(gSnapshot.fixType);
    Serial.print(" siv=");
    Serial.print(gSnapshot.siv);
    if (gSnapshot.surfaceFix) {
      Serial.print(" lat=");
      Serial.print(gSnapshot.lat);
      Serial.print(" lon=");
      Serial.print(gSnapshot.lon);
    } else {
      Serial.print(" (no surface fix)");
    }
#endif
    Serial.println();

    // write to SD
    if (sdOK) {
      File file = SD.open(CSV_PATH, FILE_WRITE); //writes one CSV row per cycle
      if (file) {
        file.print(uptimeMin, 2); file.print(",");
        file.print(battV, 3);     file.print(",");
        file.print(battPct);      file.print(",");
        
        file.print(gSnapshot.fixType);    file.print(",");
        file.print(gSnapshot.siv);        file.print(",");

        if (gSnapshot.surfaceFix){
          file.print(gSnapshot.lat);        file.print(",");
          file.print(gSnapshot.lon);        file.print(",");
        } else {
          file.print(",");        // blank latitude
          file.print(",");        // blank longitude
        }

        file.println(gSnapshot.surfaceFix ? 1: 0);
        file.close();

        Serial.println("✅ Logged 1 row to track.csv");
      } else {
        Serial.println("❌ Could not open track.csv (SD became unavailable?)");
        sdOK = false;
      }
    } else {
      Serial.println("⚠️ SD not available, skipping log");
    }
  }

  // idle (we'll replace with MCU sleep next)
  delay(50); //later can do 1000, keeps loop from spinning too much, does NOT sleep MCU "deeply"
  //it is still awake (deep sleep is a separate process)
 } //end of void loop

