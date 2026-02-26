//firmware includes altitude, horizontal accuracy, and speed 
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
//keeps Serial stable on many nRF52 cores (RAK4631 is nRF520 based)
#include <Adafruit_TinyUSB.h>

//built in Arduino SD library
#include <SD.h>

//Serial Peripheral Interface
//fast data transfer between microcontroller and peripheral so need this for SD Card!
#include <SPI.h>

//Wire is the I2C(Inte-Integrated Circuit which is 2 wire communication) library
//RAK12500 GNSS access via I2C (address 0x42), GNSS doesn't respond unless I2C is initialized
#include <Wire.h>

//SparkFun u-blox GNSS library --> provdes SFE_UBLOX class and functions like getlatitude(), powersavemode(), etc/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

struct GnssSnapshot;
GnssSnapshot getGnssSnapshot();

//create global GNSS object
SFE_UBLOX_GNSS myGNSS;

// ====== SETTINGS ======
#define USE_GNSS 1  //set 0 for battery-only logging and basically will remove GNSS code at compile time

//cycle settings
#define LOG_INTERVAL_MS 60000UL//300000UL  // how long the cycle will be 5 - minutes in milliseconds
#define GNSS_ATTEMPT_MS 60000UL   //NEW: 60s for testing; set back to 15000UL later

#define MIN_SIV_FOR_VALID 3   // Minimum Sattelites in View threshold: require at least 3 or 4 satellites in view for real fix
#define CSV_PATH "track.csv"  // IMPORTANT: no leading "/"
#define SD_RETRY_MS 30000UL   // retry SD every 30s if it fails

//NEW: if GNSS attempt window takes almost the whole interval, avoid "back-to-back" cycles
//0 = keep your original stable cadence behavior (lastLog += interval)
//1 = schedule next run from NOW (no catch-up)
#define SCHED_FROM_NOW 1 //NEW

//NEW: UART GNSS settings (matches RAK official approach)
#define USE_GNSS_UART 1 //NEW: 1 = UART (Serial1). 0 = (your old I2C approach - not used here)
#define GNSS_UART_BAUD_DEFAULT 9600 //NEW: fallback if you don't want scan
#define GNSS_UART_SCAN_BAUDS 1 //NEW: do the RAK-style baud scan at boot

//NEW: power reset GNSS rail at boot like RAK official code
#define GNSS_POWER_RESET_AT_BOOT 1 //NEW

//NEW: test-mode option: keep GNSS awake between cycles (warm start)
//Set to 0 later for power saving
#define GNSS_KEEP_AWAKE_BETWEEN_CYCLES 1 //NEW

// ====== STATE ======
bool sdOK = false;              //if SD currently mounted and stable
unsigned long lastLog = 0;      //stores the last time we logged, for interval scheduling
unsigned long lastSDRetry = 0;  //stores last time we retried SD init, to avoid retrying every loop

//for GNSS - will be used in setup() to make sure to not call GNSS functions unless GNSS is initialized
bool gnssOK = false;              //tracks if GNSS initialized successfully
unsigned long lastGNSSRetry = 0;  //last time we retried GNSS init
#define GNSS_RETRY_MS 30000UL     //retry GNSS init every x seconds if it failed

void enableWisBlockSensorRails() {
  // Turn on possible WisBlock sensor power rails (safe for testing)
  //on Wisblock bases, the IO rails (WB_I01, etc. ) can control/enable sensor power lines
  pinMode(WB_IO1, OUTPUT);
  digitalWrite(WB_IO1, HIGH);
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  pinMode(WB_IO3, OUTPUT);
  digitalWrite(WB_IO3, HIGH);
  pinMode(WB_IO4, OUTPUT);
  digitalWrite(WB_IO4, HIGH);
  pinMode(WB_IO5, OUTPUT);
  digitalWrite(WB_IO5, HIGH);
  pinMode(WB_IO6, OUTPUT);
  digitalWrite(WB_IO6, HIGH);

  delay(300);  //give hardware time to stabalize after rails come up
}

// ===================== GNSS SNAPSHOT TYPE =====================
struct GnssSnapshot {
  int fixType;      //u-block fixType (0=no fix, 2=2D, 3=3D) --> tells us quality of our fix
  int siv;          //satellites in view
  long lat;         //degrees * 1e-7 (SparkFun library format)
  long lon;         //degrees * 1e-7
  long altMm;       //NEW: altitude in millimeters (SparkFun format uses mm)
  uint32_t hAccMm;  //NEW: horizontal accuracy estimate in millimeters
  long speedMmps;   //NEW: ground speed in millimeters per second
  bool surfaceFix;  //true only if we got a valid fix during attempt window

  //GNSS UTC timespace (valid when timeValid==true) bc can only get timestamp from GNSS
  bool timeValid;
  uint16_t year;
  uint8_t month, day, hour, minute, second;

  //constructor for initilizing
  GnssSnapshot()
    : fixType(-1), siv(-1), lat(0), lon(0), altMm(0), hAccMm(0), speedMmps(0), surfaceFix(false),  //NEW
      timeValid(false), year(0), month(0), day(0), hour(0), minute(0), second(0) {}
};

// ===================== BATTERY =====================
/* Function: readBatteryV()
   Reads battery voltage from WB_A0 using the onboard divider
   This will convert ADC counts into Voltage
*/
float readBatteryV() {                              //accurate for most part
  const float VBAT_MV_PER_LSB = 3000.0f / 4096.0f;  // 12-bit, 3.0V ref
  const float VBAT_DIVIDER_COMP = 1.73f;            // RAK divider factor, can tweak based on calibration

  // 1. take raw ADC reading from battery measurement pin --> returns (0..4095)
  float raw = analogRead(WB_A0);

  // 2. Convert raw counts to millivolts at ADC, then scale to battery millivolts
  float mv = raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  // 3. Return volts(V) instead of millivolts (mV)
  return mv / 1000.0f;
}

int batteryPercentFromVoltage(float v) {  //general estimate
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
//This function tries to mount SD card and ensure CSV exists
void tryInitSD() {
  Serial.println("Init SD (RAK15002 style)...");  //print status to Serial
  delay(300);                                     //delay to help some SD modules stabalize after power

  //1. attempt to mount SD and see if successful
  if (!SD.begin()) {
    Serial.println("SD mount failed (FAT32? module seated? card inserted?????)");
    sdOK = false;
    return;  //exit function early
  }

  Serial.println("✅ SD mounted");
  sdOK = true;

  //2. creates file and writes header of csv
  if (!SD.exists(CSV_PATH)) {
    File file = SD.open(CSV_PATH, FILE_WRITE);

    if (file) {
      file.println("utc_iso,latitude,longitude,altitude_m,hAcc_m,speed_mps,surfaceFix,fixType,siv,uptime_min,batt_v,batt_pct"); //NEW
      file.close();
      Serial.println("✅ Created track.csv with header");
    } else {
      Serial.println("❌ Could not create track.csv");
      sdOK = false;  //if we can't create file, SD basically not usable
    }

  }  //end of outer if
}  //end of function

// ===================== GNSS =====================
//This function initializes GNSS over UART (Serial1) like RAK official example
bool initGNSS() {
#if USE_GNSS
  Serial.println("Starting Init of GNSS..."); //print status to Serial
  delay(500);

#if GNSS_POWER_RESET_AT_BOOT
  //NEW: power reset GNSS rail like RAK official code (helps when GNSS gets “stuck”)
  pinMode(WB_IO2, OUTPUT);      //NEW
  digitalWrite(WB_IO2, LOW);    //NEW
  delay(1000);                  //NEW
  digitalWrite(WB_IO2, HIGH);   //NEW
  delay(1000);                  //NEW
#endif

#if USE_GNSS_UART
  bool ok = false; //NEW
#if GNSS_UART_SCAN_BAUDS
  //NEW: RAK-style baud scan (some modules are configured to different rates)
  int baud[7] = {9600,14400,19200,38400,56000,57600,115200}; //NEW
  for (int i = 0; i < (int)(sizeof(baud)/sizeof(int)); i++) { //NEW
    Serial1.begin(baud[i]); //NEW
    delay(50);              //NEW
    if (myGNSS.begin(Serial1) == true) { //NEW
      Serial.print("✅ GNSS baud rate: "); //NEW
      Serial.println(baud[i]);            //NEW
      ok = true;                          //NEW
      break;                              //NEW
    }
    Serial1.end(); //NEW
    delay(200);    //NEW
  }
#else
  //NEW: fixed baud if you don't want scan
  Serial1.begin(GNSS_UART_BAUD_DEFAULT); //NEW
  delay(50); //NEW
  ok = (myGNSS.begin(Serial1) == true); //NEW
#endif

  if (!ok) { //NEW
    Serial.println("❌ GNSS not detected on UART (Serial1)"); //NEW
    return false; //NEW
  }

  Serial.println("✅ GNSS serial connected"); //NEW

  //match RAK example config
  myGNSS.setUART1Output(COM_TYPE_UBX);  //NEW
  myGNSS.setI2COutput(COM_TYPE_UBX);    //NEW (turn off NMEA noise if I2C is alive)
  myGNSS.saveConfiguration();           //NEW

  //NEW: make sure GNSS is awake after init
  myGNSS.powerSaveMode(false); //NEW
  delay(200);                  //NEW

  Serial.println("✅ GNSS ready (UART)"); //NEW
  return true; //NEW
#else
  return false; //NEW (we aren't using I2C path here)
#endif

#else
  return true;
#endif
}  //end of initGNSS() function

// -------- GNSS power wrappers --------
//This function wakes GNSS (exit power save mode) before attempting to read
void wakeGNSS() {
#if USE_GNSS
  if (!gnssOK) return;  //if GNSS never initialized, do nothing
  Serial.println("GNSS WAKE"); //print status to Serial

  myGNSS.powerSaveMode(false); //wake GNSS
  delay(200);                  //give time respond after waking
#endif
}  //end of function

//This function will put GNSS into power save mode
void sleepGNSS() {
#if USE_GNSS
  if (!gnssOK) return;
  Serial.println("GNSS SLEEP"); //print status to Serial
  myGNSS.powerSaveMode(true);
  delay(50);
#endif
}

// ===================== GNSS ATTEMPT WINDOW =====================
// This function ONLY sets lat/lon if we get a valid fix DURING the window.
// If we do not, lat/lon remain 0 and we print/write blanks.
GnssSnapshot getGnssSnapshot() {

  //1. Create snapshot struct that we'll return
  GnssSnapshot snapshot;

#if USE_GNSS
  //2. If GNSS isn't initialized, return blank snapshot
  if (!gnssOK) return snapshot;

  //3. Wake GNSS before trying
  wakeGNSS();

  //4. record start time for the attempt window
  unsigned long start = millis();

  //5. while statement means try to get a fix until the attempt window expires
  while (millis() - start < GNSS_ATTEMPT_MS) {

    //heartbeat so you KNOW the code is alive inside the window
    static unsigned long lastDot = 0; //NEW
    if (millis() - lastDot >= 1000) { //NEW
      Serial.print(".");              //NEW
      lastDot = millis();             //NEW
    }

    //NEW: read fix + siv (UART polling)
    int fix = (int)myGNSS.getFixType(); //NEW
    int siv = (int)myGNSS.getSIV();     //NEW
    snapshot.fixType = fix;             //NEW
    snapshot.siv = siv;                 //NEW

    //NEW: time fields (only valid when module has good time)
    uint16_t y = myGNSS.getYear(); //NEW
    uint8_t mo = myGNSS.getMonth(); //NEW
    uint8_t d = myGNSS.getDay(); //NEW
    uint8_t h = myGNSS.getHour(); //NEW
    uint8_t mi = myGNSS.getMinute(); //NEW
    uint8_t s = myGNSS.getSecond(); //NEW
    if (y >= 2020 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) { //NEW
      snapshot.timeValid = true; //NEW
      snapshot.year = y; //NEW
      snapshot.month = mo; //NEW
      snapshot.day = d; //NEW
      snapshot.hour = h; //NEW
      snapshot.minute = mi; //NEW
      snapshot.second = s; //NEW
    }

    // accept if we have fix + enough satellites
    if (fix >= 2 && siv >= MIN_SIV_FOR_VALID) {
      snapshot.lat = myGNSS.getLatitude();          //NEW
      snapshot.lon = myGNSS.getLongitude();         //NEW
      snapshot.altMm = myGNSS.getAltitude();        //NEW
      snapshot.hAccMm = myGNSS.getHorizontalAccuracy(); //NEW
      snapshot.speedMmps = myGNSS.getGroundSpeed(); //NEW
      snapshot.surfaceFix = true;
      break;
    }

    delay(250);  //check about 4 times per second
  }

  Serial.println(); //newline after dots

  //Logs How long the attempt take to get a fix
  unsigned long duration = millis() - start;
  Serial.print("GNSS attempt duration ms = ");
  Serial.println(duration);

#if GNSS_KEEP_AWAKE_BETWEEN_CYCLES
  //NEW: do NOT sleep GNSS (test mode)
#else
  //Put GNSS back to sleep to conserve power between cycle
  sleepGNSS();
#endif

#endif
  return snapshot;
}  //end of getGnssSnapshot() function

//===================== SETUP (runs once at boot) =====================
/* Our setup() initializes Serial, turns rails on, configures ADC, starts I2C, 
   mounts SD and initializes GNSS
*/
void setup() {

  unsigned long timeout = millis();  //used to limit how long we wait for serial connection

  //1. Start Serial
  Serial.begin(115200);

  //2. wait for serial to come up
  while (!Serial) {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }
  Serial.println("\n ✅ Firmware starting: Battery + GNSS + SD logger");

  //3. Turn on rails so GNSS/SD Modules can power correctly
  enableWisBlockSensorRails();

  //for reading battery voltage-using rakwireless code approach
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(2);
  analogRead(WB_A0);  //throw away first sample

  //A. I2C init (still ok to leave; SD may not use it but harmless)
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  //B. Initiatize SD
  tryInitSD();

  //C. Initialize GNSS
#if USE_GNSS
  gnssOK = initGNSS();
  if (!gnssOK) {
    Serial.println("⚠️ GNSS init failed (will log battery + no-fix)");
  }
#endif

  // IMMEDIATE LOG AT BOOT
  lastLog = millis() - LOG_INTERVAL_MS;
  lastSDRetry = millis();
  lastGNSSRetry = millis();
}

// ===================== LOOP (runs forever) =====================
void loop() {
  // A. retry SD sometimes if it failed
  if (!sdOK && (millis() - lastSDRetry >= SD_RETRY_MS)) {
    lastSDRetry = millis();
    tryInitSD();
  }

  //B. Retry GNSS if init failed
#if USE_GNSS
  if (!gnssOK && (millis() - lastGNSSRetry >= GNSS_RETRY_MS)) {
    lastGNSSRetry = millis();
    Serial.println("Retrying GNSS init...");
    gnssOK = initGNSS();
  }
#endif

  //MAIN SCHEDULER
  if (millis() - lastLog >= LOG_INTERVAL_MS) {

#if SCHED_FROM_NOW
    lastLog = millis();            //NEW: prevents “back-to-back” if GNSS attempt is long
#else
    lastLog += LOG_INTERVAL_MS;    //original: keeps cadence stable
#endif

    float battV = readBatteryV();
    int battPct = batteryPercentFromVoltage(battV);
    float uptimeMin = millis() / 60000.0f;

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

    Serial.print(" lat=");
    if (gSnapshot.surfaceFix) {
      float latDeg = gSnapshot.lat / 10000000.0f;
      Serial.print(latDeg, 7);
    }
    Serial.print(" lon=");
    if (gSnapshot.surfaceFix) {
      float lonDeg = gSnapshot.lon / 10000000.0f;
      Serial.print(lonDeg, 7);
    }

    Serial.print(" alt_m="); //NEW
    if (gSnapshot.surfaceFix) {
      Serial.print(gSnapshot.altMm / 1000.0f, 2);
    }

    Serial.print(" hAcc_m="); //NEW
    if (gSnapshot.surfaceFix) {
      Serial.print(gSnapshot.hAccMm / 1000.0f, 2); //mm -> m
    }

    Serial.print(" speed_mps="); //NEW
    if (gSnapshot.surfaceFix) {
      Serial.print(gSnapshot.speedMmps / 1000.0f, 3); //mm/s -> m/s
    }
#endif
    Serial.println();

    // -------- SD logging (blank fields if no surfaceFix) --------
    if (sdOK) {
      File file = SD.open(CSV_PATH, FILE_WRITE);
      if (file) {
        // 1) utc_iso
        if (gSnapshot.timeValid) {
          char buf[25];
          snprintf(buf, sizeof(buf), "%04u-%02u-%02uT%02u:%02u:%02uZ",
                   gSnapshot.year, gSnapshot.month, gSnapshot.day,
                   gSnapshot.hour, gSnapshot.minute, gSnapshot.second);
          file.print(buf);
        }
        file.print(",");

        // 2) latitude, longitude, altitude_m, hAcc_m, speed_mps  //NEW
        if (gSnapshot.surfaceFix) {
          float latDeg = gSnapshot.lat / 10000000.0f;
          float lonDeg = gSnapshot.lon / 10000000.0f;
          float altM = gSnapshot.altMm / 1000.0f;
          float hAccM = gSnapshot.hAccMm / 1000.0f;       //NEW
          float speedMps = gSnapshot.speedMmps / 1000.0f; //NEW

          file.print(latDeg, 7);
          file.print(",");
          file.print(lonDeg, 7);
          file.print(",");
          file.print(altM, 2);
          file.print(",");
          file.print(hAccM, 2);    //NEW
          file.print(",");
          file.print(speedMps, 3); //NEW
          file.print(",");
        } else {
          file.print(",");
          file.print(",");
          file.print(",");
          file.print(","); //NEW: blank hAcc_m
          file.print(","); //NEW: blank speed_mps
        }

        // 3) surfaceFix, fixType, siv
        file.print(gSnapshot.surfaceFix ? 1 : 0);
        file.print(",");
        file.print(gSnapshot.fixType);
        file.print(",");
        file.print(gSnapshot.siv);
        file.print(",");

        // 4) uptime, battery
        file.print(uptimeMin, 2);
        file.print(",");
        file.print(battV, 3);
        file.print(",");
        file.println(battPct);

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

  delay(50);
}