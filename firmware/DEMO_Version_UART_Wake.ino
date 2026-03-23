//successfully creates folders, full wake cycle
//CURRENT DEMO CODE FOR NO SLEEP TESTING BC SHORT CYCLE
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
#define GNSS_ATTEMPT_MS 60000UL   //60s for testing; set back to 15000UL later

#define MIN_SIV_FOR_VALID 3   // Minimum Sattelites in View threshold: require at least 3 or 4 satellites in view for real fix
#define SD_RETRY_MS 30000UL   // retry SD every 30s if it fails

//if GNSS attempt window takes almost the whole interval, avoid "back-to-back" cycles
//0 = keep your original stable cadence behavior (lastLog += interval)
//1 = schedule next run from NOW (no catch-up)
#define SCHED_FROM_NOW 1 
//UART GNSS settings (matches RAK official approach)
#define USE_GNSS_UART 1 // 1 = UART (Serial1)
#define GNSS_UART_BAUD_DEFAULT 9600 // fallback if we don't want scan
#define GNSS_UART_SCAN_BAUDS 1 

//power reset GNSS rail at boot like RAK official code
#define GNSS_POWER_RESET_AT_BOOT 1 //NEW

// test-mode option: keep GNSS awake between cycles (warm start)
//Set to 0 later for power saving
#define GNSS_KEEP_AWAKE_BETWEEN_CYCLES 1 

// ====== RUN FOLDER PATHS ======
char g_runDir[16] = {0};     //ex: "RUN001"
char g_csvPath[32] = {0};    //ex: "RUN001/track.csv"
char g_infoPath[32] = {0};   //ex: "RUN001/info.txt"

// ====== STATE ======
bool sdOK = false;              //if SD currently mounted and stable
unsigned long lastLog = 0;      //stores the last time we logged, for interval scheduling
unsigned long lastSDRetry = 0;  //stores last time we retried SD init, to avoid retrying every loop

//for GNSS - will be used in setup() to make sure to not call GNSS functions unless GNSS is initialized
bool gnssOK = false;              //tracks if GNSS initialized successfully
unsigned long lastGNSSRetry = 0;  //last time we retried GNSS init
#define GNSS_RETRY_MS 30000UL     //retry GNSS init every x seconds if it failed

bool runFolderCreated = false;    //only create RUN### once per boot (prevents RUN001 header-only + RUN002 data issue)

//NEW: measure "GNSS awake" time (from wake command to sleep command)
unsigned long g_lastGnssAwakeMs = 0; 

void enableWisBlockSensorRails() {
  // Turn on possible WisBlock sensor power rails (safe for testing) --> later optimize by only enabling rails we need
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

//create next available run folder RUN001, RUN002, ...
bool createNextRunFolder() { 
  for (int i = 1; i <= 999; i++) { 
    snprintf(g_runDir, sizeof(g_runDir), "RUN%03d", i); 
    if (!SD.exists(g_runDir)) { 
      if (!SD.mkdir(g_runDir)) { 
        Serial.println("❌ Could not create run folder"); 
        return false;
      }
      snprintf(g_csvPath, sizeof(g_csvPath), "%s/track.csv", g_runDir); 
      snprintf(g_infoPath, sizeof(g_infoPath), "%s/info.txt", g_runDir);
      Serial.print("✅ Created run folder: "); 
      Serial.println(g_runDir); 
      return true; 
    }
  }
  Serial.println("❌ No available RUN### folder slots"); 
  return false;
}

//write info.txt once per run (at boot)
void writeRunInfoFile() { 
  File f = SD.open(g_infoPath, FILE_WRITE); 
  if (!f) { 
    Serial.println("❌ Could not write info.txt");
    return; 
  }

  f.println("SEAtech GNSS Logger Run Info"); 
  f.print("Build: "); f.print(__DATE__); f.print(" "); f.println(__TIME__); 
  f.print("LOG_INTERVAL_MS="); f.println(LOG_INTERVAL_MS); 
  f.print("GNSS_ATTEMPT_MS="); f.println(GNSS_ATTEMPT_MS); 
  f.print("MIN_SIV_FOR_VALID="); f.println(MIN_SIV_FOR_VALID); 

  f.print("SCHED_FROM_NOW="); f.println(SCHED_FROM_NOW); 
  f.print("USE_GNSS_UART="); f.println(USE_GNSS_UART); 
  f.print("GNSS_UART_SCAN_BAUDS="); f.println(GNSS_UART_SCAN_BAUDS);
  f.print("GNSS_POWER_RESET_AT_BOOT="); f.println(GNSS_POWER_RESET_AT_BOOT); 
  f.print("GNSS_KEEP_AWAKE_BETWEEN_CYCLES="); f.println(GNSS_KEEP_AWAKE_BETWEEN_CYCLES); 

  f.close(); 
  Serial.println("✅ Wrote info.txt"); 
}

// ===================== GNSS SNAPSHOT TYPE =====================
struct GnssSnapshot {
  int fixType;      //u-block fixType (0=no fix, 2=2D, 3=3D) --> tells us quality of our fix
  int siv;          //satellites in view
  long lat;         //degrees * 1e-7 (SparkFun library format)
  long lon;         //degrees * 1e-7
  long altMm;       //altitude in millimeters
  uint32_t hAccMm;  //horizontal accuracy estimate in millimeters
  long speedMmps;   //ground speed in millimeters per second
  bool surfaceFix;  //true only if we got a valid fix during attempt window

  //GNSS UTC timespace (valid when timeValid==true) bc can only get timestamp from GNSS
  bool timeValid;
  uint16_t year;
  uint8_t month, day, hour, minute, second;

  //constructor for initilizing
  GnssSnapshot()
    : fixType(-1), siv(-1), lat(0), lon(0), altMm(0), hAccMm(0), speedMmps(0), surfaceFix(false),
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
//This function tries to mount SD card and ensure RUN### folder + CSV exists
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

  //per-run folder + write info.txt ONLY ONCE per boot
  if (!runFolderCreated) { 
    if (!createNextRunFolder()) { 
      sdOK = false;              
      return;                     
    }
    writeRunInfoFile(); 
    runFolderCreated = true; 
  } 

  //creates track.csv inside run folder and writes header
  if (!SD.exists(g_csvPath)) { 
    File file = SD.open(g_csvPath, FILE_WRITE); 
    if (file) {
      file.println("utc_iso,latitude,longitude,altitude_m,hAcc_m,speed_mps,surfaceFix,fixType,siv,uptime_min,batt_v,batt_pct");
      file.close();
      delay(50); //let SD settle after creating file
      Serial.println("✅ Created track.csv with header");
    } else {
      Serial.println("❌ Could not create track.csv");
      sdOK = false;
    }
  }
}

// ===================== GNSS =====================
//This function initializes GNSS over UART (Serial1) like RAK official example
bool initGNSS() {
#if USE_GNSS
  Serial.println("Starting Init of GNSS..."); //print status to Serial
  delay(500);

#if GNSS_POWER_RESET_AT_BOOT
  //power reset GNSS rail like RAK official code (helps when GNSS gets “stuck”)
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, LOW);
  delay(1000);
  digitalWrite(WB_IO2, HIGH);
  delay(1000);
#endif

#if USE_GNSS_UART
  bool ok = false;
#if GNSS_UART_SCAN_BAUDS
  int baud[7] = {9600,14400,19200,38400,56000,57600,115200};
  for (int i = 0; i < (int)(sizeof(baud)/sizeof(int)); i++) {
    Serial1.begin(baud[i]);
    delay(50);
    if (myGNSS.begin(Serial1) == true) {
      Serial.print("✅ GNSS baud rate: ");
      Serial.println(baud[i]);
      ok = true;
      break;
    }
    Serial1.end();
    delay(200);
  }
#else
  Serial1.begin(GNSS_UART_BAUD_DEFAULT);
  delay(50);
  ok = (myGNSS.begin(Serial1) == true);
#endif

  if (!ok) {
    Serial.println("❌ GNSS not detected on UART (Serial1)");
    return false;
  }

  Serial.println("✅ GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfiguration();

  myGNSS.powerSaveMode(false);
  delay(200);

  Serial.println("✅ GNSS ready (UART)");
  return true;
#else
  return false;
#endif

#else
  return true;
#endif
}

// -------- GNSS power wrappers --------
void wakeGNSS() {
#if USE_GNSS
  if (!gnssOK) return;
  Serial.println("GNSS WAKE");
  myGNSS.powerSaveMode(false);
  delay(200);
#endif
}

void sleepGNSS() {
#if USE_GNSS
  if (!gnssOK) return;
  Serial.println("GNSS SLEEP");
  myGNSS.powerSaveMode(true);
  delay(50);
#endif
}

// ===================== GNSS ATTEMPT WINDOW =====================
GnssSnapshot getGnssSnapshot() {
  GnssSnapshot snapshot;

#if USE_GNSS
  if (!gnssOK) return snapshot;

  unsigned long tWakeCmd = millis(); //start measuring awake time
  wakeGNSS();

  unsigned long start = millis();

  while (millis() - start < GNSS_ATTEMPT_MS) {

    static unsigned long lastDot = 0;
    if (millis() - lastDot >= 1000) {
      Serial.print(".");
      lastDot = millis();
    }

    int fix = (int)myGNSS.getFixType();
    int siv = (int)myGNSS.getSIV();
    snapshot.fixType = fix;
    snapshot.siv = siv;

    uint16_t y = myGNSS.getYear();
    uint8_t mo = myGNSS.getMonth();
    uint8_t d = myGNSS.getDay();
    uint8_t h = myGNSS.getHour();
    uint8_t mi = myGNSS.getMinute();
    uint8_t s = myGNSS.getSecond();
    if (y >= 2020 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
      snapshot.timeValid = true;
      snapshot.year = y;
      snapshot.month = mo;
      snapshot.day = d;
      snapshot.hour = h;
      snapshot.minute = mi;
      snapshot.second = s;
    }

    if (fix >= 2 && siv >= MIN_SIV_FOR_VALID) {
      snapshot.lat = myGNSS.getLatitude();
      snapshot.lon = myGNSS.getLongitude();
      snapshot.altMm = myGNSS.getAltitude();
      snapshot.hAccMm = myGNSS.getHorizontalAccuracy(); //NOTE: if compile error, tell me function name mismatch
      snapshot.speedMmps = myGNSS.getGroundSpeed();
      snapshot.surfaceFix = true;
      break;
    }

    delay(250);
  }

  Serial.println();

  unsigned long duration = millis() - start;
  Serial.print("GNSS attempt duration ms = ");
  Serial.println(duration);

#if GNSS_KEEP_AWAKE_BETWEEN_CYCLES
  //do NOT sleep GNSS (test mode)
  g_lastGnssAwakeMs = millis() - tWakeCmd; //awake so far (since wake command)
#else
  unsigned long tSleepCmd = millis(); 
  sleepGNSS();
  g_lastGnssAwakeMs = tSleepCmd - tWakeCmd; //approx awake time until sleep command
#endif

#endif
  return snapshot;
}

//===================== SETUP =====================
void setup() {

  unsigned long timeout = millis();

  Serial.begin(115200);

  while (!Serial) {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }
  Serial.println("\n ✅ Firmware starting: Battery + GNSS + SD logger");

  enableWisBlockSensorRails();

  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(2);
  analogRead(WB_A0);

  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  tryInitSD();

#if USE_GNSS
  gnssOK = initGNSS();
  if (!gnssOK) {
    Serial.println("⚠️ GNSS init failed (will log battery + no-fix)");
  }
#endif

  lastLog = millis() - LOG_INTERVAL_MS;
  lastSDRetry = millis();
  lastGNSSRetry = millis();
}

// ===================== LOOP =====================
void loop() {
  if (!sdOK && (millis() - lastSDRetry >= SD_RETRY_MS)) {
    lastSDRetry = millis();
    tryInitSD();
  }

#if USE_GNSS
  if (!gnssOK && (millis() - lastGNSSRetry >= GNSS_RETRY_MS)) {
    lastGNSSRetry = millis();
    Serial.println("Retrying GNSS init...");
    gnssOK = initGNSS();
  }
#endif

  if (millis() - lastLog >= LOG_INTERVAL_MS) {

#if SCHED_FROM_NOW
    lastLog = millis();
#else
    lastLog += LOG_INTERVAL_MS;
#endif

    float battV = readBatteryV();
    int battPct = batteryPercentFromVoltage(battV);
    float uptimeMin = millis() / 60000.0f;

    GnssSnapshot gSnapshot = getGnssSnapshot();

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
    if (gSnapshot.surfaceFix) Serial.print(gSnapshot.lat / 10000000.0f, 7);
    Serial.print(" lon=");
    if (gSnapshot.surfaceFix) Serial.print(gSnapshot.lon / 10000000.0f, 7);

    Serial.print(" alt_m=");
    if (gSnapshot.surfaceFix) Serial.print(gSnapshot.altMm / 1000.0f, 2);

    Serial.print(" hAcc_m=");
    if (gSnapshot.surfaceFix) Serial.print(gSnapshot.hAccMm / 1000.0f, 2);

    Serial.print(" speed_mps=");
    if (gSnapshot.surfaceFix) Serial.print(gSnapshot.speedMmps / 1000.0f, 3);

    Serial.print(" awake_ms=");            //print GNSS awake time for this cycle
    Serial.print(g_lastGnssAwakeMs);      
#endif
    Serial.println();

    if (sdOK) {
      File file = SD.open(g_csvPath, FILE_WRITE);
      if (!file) {                 //soft retry once (prevents false SD-down + extra RUN folders)
        delay(50);                 
        file = SD.open(g_csvPath, FILE_WRITE); 
      }

      if (file) {
        if (gSnapshot.timeValid) {
          char buf[25];
          snprintf(buf, sizeof(buf), "%04u-%02u-%02uT%02u:%02u:%02uZ",
                   gSnapshot.year, gSnapshot.month, gSnapshot.day,
                   gSnapshot.hour, gSnapshot.minute, gSnapshot.second);
          file.print(buf);
        }
        file.print(",");

        if (gSnapshot.surfaceFix) {
          float latDeg = gSnapshot.lat / 10000000.0f;
          float lonDeg = gSnapshot.lon / 10000000.0f;
          float altM = gSnapshot.altMm / 1000.0f;
          float hAccM = gSnapshot.hAccMm / 1000.0f;
          float speedMps = gSnapshot.speedMmps / 1000.0f;

          file.print(latDeg, 7); file.print(",");
          file.print(lonDeg, 7); file.print(",");
          file.print(altM, 2);   file.print(",");
          file.print(hAccM, 2);  file.print(",");
          file.print(speedMps, 3);file.print(",");
        } else {
          file.print(","); file.print(","); file.print(","); file.print(","); file.print(",");
        }

        file.print(gSnapshot.surfaceFix ? 1 : 0); file.print(",");
        file.print(gSnapshot.fixType); file.print(",");
        file.print(gSnapshot.siv); file.print(",");
        file.print(uptimeMin, 2); file.print(",");
        file.print(battV, 3); file.print(",");
        file.println(battPct);

        file.close();
        Serial.println("✅ Logged 1 row to track.csv");
      } else {
        Serial.println("❌ Could not open track.csv (will retry later)"); //
        //do NOT set sdOK=false on a single open failure (prevents re-init -> new RUN folder)
      }
    } else {
      Serial.println("⚠️ SD not available, skipping log");
    }
  }

  delay(50);
}