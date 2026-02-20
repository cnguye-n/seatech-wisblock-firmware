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


  Goal (test-mode):
  - Every 1 minute: wake GNSS, try for up to 15 seconds, log result, gnss go back to sleep
  - If no valid fix -> leave lat/lon blank (Serial + CSV)
  - Put GNSS into power-save between attempts

**************************************************************/
#include <Adafruit_TinyUSB.h>  // keeps Serial stable on many nRF52 cores
#include <SPI.h> //Serial Peripheral Interface, fast data transfer between microcontroller and peripheral so need this for SD Card!
#include <SD.h> //using Arduino built in library
#include <Wire.h> //need this to turn on MCU's I2C controller or GNSS won't initialize
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

struct GnssSnapshot;
GnssSnapshot getGnssSnapshot();

SFE_UBLOX_GNSS myGNSS;

// ====== SETTINGS ======
//moving 10 feet is notable shift
#define USE_GNSS               1          // set 0 for battery-only logging

//cycle settings
#define LOG_INTERVAL_MS        60000UL   // 1 minute in milliseconds 
#define GNSS_ATTEMPT_MS         15000UL  // 30 seconds in milliseconds

#define MIN_SIV_FOR_VALID           3     // require at least 3 or 4 satellites in view for real fix
#define CSV_PATH            "track.csv"   // IMPORTANT: no leading "/"
#define SD_RETRY_MS            30000UL    // retry SD every 30s if it fails

//#define DEBUG true //for debugging purposes

// ====== STATE ======
bool sdOK = false;
unsigned long lastLog = 0;
unsigned long lastSDRetry = 0;

//for GNSS - will be used in setup() to make sure to not call GNSS functions unless GNSS is initialized
bool gnssOK = false;
unsigned long lastGNSSRetry = 0;
#define GNSS_RETRY_MS 30000UL

void enableWisBlockSensorRails() {
  // Turn on possible WisBlock sensor power rails (safe for testing)
  pinMode(WB_IO1, OUTPUT); digitalWrite(WB_IO1, HIGH);
  pinMode(WB_IO2, OUTPUT); digitalWrite(WB_IO2, HIGH);
  pinMode(WB_IO3, OUTPUT); digitalWrite(WB_IO3, HIGH);
  pinMode(WB_IO4, OUTPUT); digitalWrite(WB_IO4, HIGH);
  pinMode(WB_IO5, OUTPUT); digitalWrite(WB_IO5, HIGH);
  pinMode(WB_IO6, OUTPUT); digitalWrite(WB_IO6, HIGH);
  delay(300);
}

// ===================== GNSS SNAPSHOT TYPE =====================
struct GnssSnapshot {
  int fixType;      //u-block fixType (0..5 is typical value)
  int siv;          //satellites in view
  long lat   ;      //degrees * 1e-7 (SparkFun library format)
  long lon;         // degrees * 1e-7
  bool surfaceFix;  //true only if we have valid fix during attempt window
};

// ===================== BATTERY =====================
float readBatteryV() {
  const float VBAT_MV_PER_LSB = 3000.0f / 4096.0f; // 12-bit, 3.0V ref
  const float VBAT_DIVIDER_COMP = 1.73f;           // RAK divider factor, can play with this number
  float raw = analogRead(WB_A0);                   // 0..4095
  float mv = raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  return mv / 1000.0f;                             // volts
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
    Serial.println("SD mount failed (FAT32? module seated? card inserted?????)");
    sdOK = false;
    return;
  }

  Serial.println("✅ SD mounted");
  sdOK = true;

  if (!SD.exists(CSV_PATH)) {
    File file = SD.open(CSV_PATH, FILE_WRITE);
    if (file) {
      file.println("uptime_min, batt_v, batt_pct, fixType, siv, lat_1e7, lon_1e7, surfaceFix");
      file.close();
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
  delay(3000); //give GNSS time to boot (wire already running)

  bool ok = false;
  for (int i = 0; i < 10; i++) {
    if (myGNSS.begin(Wire, 0x42)) {
      ok = true;
      break;
    }
    delay(250);
  }

   if (!ok) {
    Serial.println("❌ GNSS not detected at I2C addr 0x42");
    Wire.beginTransmission(0x42);
    uint8_t err = Wire.endTransmission();
    Serial.print("I2C probe 0x42 err=");
    Serial.println(err); // 0 = ACK
    return false;
    }

  Serial.println("✅ GNSS detected at 0x42");

  //CONFIGURE MODULE
  myGNSS.setI2COutput(COM_TYPE_UBX);
  //FIX: enable automatic PVT so library fetches fresh solutions --> helps avoid stale values
  myGNSS.setAutoPVT(true);
  myGNSS.powerSaveMode(false); //stay awake after init so we only call sleepGNSS after we logged

  Serial.println("✅ GNSS ready (AutoPVT on; staying awake after init)");
  return true;

#else
  return true;
#endif
} //end of function

// -------- GNSS power wrappers --------
void wakeGNSS() {
#if USE_GNSS
  if (!gnssOK) return;

  Serial.println("🔆 GNSS WAKE");
  myGNSS.powerSaveMode(false);  // disable GNSS power save
  delay(200); //give time to wake

  //added debug code, if there is 1, power save enabled, 0 means full power, 255 means command failed/no response
  uint8_t psm = myGNSS.getPowerSaveMode();
  Serial.print("PSM state = ");
  Serial.println(psm);
#endif
} //end of function

void sleepGNSS() {
#if USE_GNSS
  if (!gnssOK) return;
  Serial.println("🌙 GNSS SLEEP");
  myGNSS.powerSaveMode(true);   // enable GNSS power save
  delay(50);
#endif
}

// ===================== GNSS ATTEMPT WINDOW =====================
// This function ONLY sets lat/lon if we get a valid fix DURING the window.
// If we do not, lat/lon remain 0 and we print/write blanks.
// -------- Attempt GNSS fix for up to 90s or whatever value we chose --------
GnssSnapshot getGnssSnapshot() {
  GnssSnapshot snapshot;
  snapshot.fixType = -1; 
  snapshot.siv = -1;
  snapshot.lat = 0;
  snapshot.lon = 0;
  snapshot.surfaceFix = false;


#if USE_GNSS
  if (!gnssOK) return snapshot; 
  wakeGNSS();

  unsigned long start = millis();

  //while statement means try to get a fix for up to 10 seconds
  while (millis() - start < GNSS_ATTEMPT_MS) {

    //Force library to pull/update the latest PVT data
    //if return false, we didn't get fresh update yet
    bool fresh = myGNSS.getPVT();

    if (fresh){
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
  Serial.begin(115200); //115200 is the baud rate --> 115200 is a fast serial speed and common default. MAKE SURE 115200 IS IN THE SERIAL MONITOR 
  while (!Serial) {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }

  Serial.println("\n ✅ Firmware starting: Battery + GNSS + SD logger");
  enableWisBlockSensorRails(); //turn on the bus

  //for reading battery voltage-using rakwireless code approach
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(2);
  analogRead(WB_A0); //throw away first sample 

  //A. I2C init 
  Wire.begin();
  Wire.setClock(100000);   // safe I2C speed
  delay(50);

  //B. Initiatize DF
  tryInitSD();

  //C. Initialize GNSS 
#if USE_GNSS
  gnssOK = initGNSS();
  if (!gnssOK) {
    Serial.println("⚠️ GNSS init failed (will log battery + no-fix)");
  }
#endif

  lastLog = millis();
  lastSDRetry = millis();
  lastGNSSRetry = millis();
} //end of void setup


// ===================== LOOP (runs forever) =====================
void loop() {
  // A. retry SD sometimes if it failed (when sdOK ==false) so we try to remount
  if (!sdOK && (millis() - lastSDRetry >= SD_RETRY_MS)) {
    lastSDRetry = millis();
    tryInitSD();
  }

  //B. Retry GNSS if init failed
  #if USE_GNSS
  if (!gnssOK && (millis() - lastGNSSRetry >= GNSS_RETRY_MS)) {
    lastGNSSRetry = millis();
    Serial.println("🔁 Retrying GNSS init...");
    gnssOK = initGNSS();
  }
#endif

  //MAIN SCHEDULER
  // main duty-cycle trigger, logs every interval
  if (millis() - lastLog >= LOG_INTERVAL_MS) { //says only do a logging cycle every LOG_INTERVAL_MS
    lastLog += LOG_INTERVAL_MS; //keeps timing stable even if cycle takes a little time

    //takes the battery snapshot
    float battV = readBatteryV(); //reads voltage
    int battPct = batteryPercentFromVoltage(battV); //converts to approximate percentage
    float uptimeMin = millis() / 60000.0f;  //logs uptime minutes

    GnssSnapshot gSnapshot = getGnssSnapshot();

    // serial print (once per cycle) --> NO LAT/LON UNLESS SURFACEFIX IS TRUE
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

    // -------- SD logging (blank lat/lon if no surfaceFix) --------
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
