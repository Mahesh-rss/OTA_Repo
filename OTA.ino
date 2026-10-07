#include <ModbusMaster.h>

// Explicit function prototypes
void enableGPS();
String readEEPROM(int startAddress, int endAddress);


#include <ArduinoJson.h>
#include <Preferences.h>
#include <vector>
#include <string>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <base64.h>
#include <SPI.h>
#include <SD.h>
#include <AES.h>
#include <Crypto.h>  // For SHA-256
#include <SHA256.h>
#include "esp_heap_caps.h"
#include <Wire.h>
#include "RTClib.h"

byte hashedKey[32];
byte iv[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
byte ivDecrypt[16];
String encryptionKey = "qtKgwYMEsukW2bqUtGZd6eKJPxtGtN1Fn13xS0gb6DhSi0WPdBsjCh973d8eqSk";
bool securedMode = true;

bool warningOccured = false;

RTC_DS3231 rtc;

float EPSILON = 0.001;

const char *myDirPath = "/CCMS/Log";
char myFilePath[32];
String globalDateTime;

String gpsData;
String latitude;
String longitude;

unsigned long baseMillis = 0;
time_t baseEpoch = 0;

String deviceType = "1P-Basic";

// SD Card Pin Definitions
#define SD_CS_PIN 48  // SD Card uses HSPI pins
#define SD_CLK_PIN 39
#define SD_MOSI_PIN 47
#define SD_MISO_PIN 38

#define CHUNK_SIZE 512

// SD Card File Path Definitions
const char *dirPath = "/CCMS";
const char *filePath = "/CCMS/Meter_Readings.txt";

// Initialize SPI for SD Card
SPIClass spi2(HSPI);  //SD Card

/*Version URL*/
const char *version_url = "https://raw.githubusercontent.com/Mahesh-rss/OTA_Repo/main/version.txt";
/*Firmware URL*/
const char *firmware_url = "https://raw.githubusercontent.com/Mahesh-rss/OTA_Repo/main/build/esp32.esp32.esp32s3/firmware.ino.bin";
const char *server = "124.40.247.18";              // Your Spring Boot server
const char *versionResource = "/dblayer/version";  // Version check endpoint
const char *firmwareResource = "/dblayer/ota";     // Firmware update endpoint
const int port = 214;                              // Your server port

String MAC_ID;

unsigned long motorOffTime = 0;
unsigned long motorRestartInterval = 600000;
bool offByThreshold = false;

bool publishedReadingsOnce = false;

const char *keys[] = {
  "VRL", "VRU", "VYL", "VYU", "VBL", "VBU",
  "VRYL", "VRYU", "VYBL", "VYBU", "VBRL", "VBRU",
  "WATTSL", "WATTSU", "IRU", "IYU", "IBU",
  "VRLW", "VRUW", "VYLW", "VYUW", "VBLW", "VBUW",
  "VRYLW", "VRYUW", "VYBLW", "VYBUW", "VBRLW", "VBRUW",
  "WATTSLW", "WATTSUW", "IRUW", "IYUW", "IBUW", "INTERVAL", "SAMPRATE",
  "V1PL", "V1PU", "V1PLW", "V1PUW", "V2PL", "V2PU", "V2PLW", "V2PUW", "IU", "IUW",
  "PFL", "PFLW", "FRQU", "FRQL", "FRQUW", "FRQLW",
  "VTHDU", "ITHDU",
  "VDPU", "IDPU"
};

float VR_ = 0.0;
float VY_ = 0.0;
float VB_ = 0.0;
float VRY_ = 0.0;
float VYB_ = 0.0;
float VBR_ = 0.0;
float IR_ = 0.0;
float IY_ = 0.0;
float IB_ = 0.0;
float WT_ = 0.0;
float IAVG_ = 0.0;
float VLLAVG_ = 0.0;
float VLNAVG_ = 0.0;
float PF_ = 0.0;
float FRQ_ = 0.0;
float VHR_ = 0.0;
float VHY_ = 0.0;
float VHB_ = 0.0;
float IHR_ = 0.0;
float IHY_ = 0.0;
float IHB_ = 0.0;

float PE_ = 0.0;
float NE_ = 0.0;

float CrestVR_ = 0.0;
float CrestVY_ = 0.0;
float CrestVB_ = 0.0;
float CrestIR_ = 0.0;
float CrestIY_ = 0.0;
float CrestIB_ = 0.0;
float KFactorVR_ = 0.0;
float KFactorVY_ = 0.0;
float KFactorVB_ = 0.0;
float KFactorIR_ = 0.0;
float KFactorIY_ = 0.0;
float KFactorIB_ = 0.0;
float HarR3V_ = 0.0;
float HarY3V_ = 0.0;
float HarB3V_ = 0.0;
float HarR3I_ = 0.0;
float HarY3I_ = 0.0;
float HarB3I_ = 0.0;
float HarR5V_ = 0.0;
float HarY5V_ = 0.0;
float HarB5V_ = 0.0;
float HarR5I_ = 0.0;
float HarY5I_ = 0.0;
float HarB5I_ = 0.0;
float HarR7V_ = 0.0;
float HarY7V_ = 0.0;
float HarB7V_ = 0.0;
float HarR7I_ = 0.0;
float HarY7I_ = 0.0;
float HarB7I_ = 0.0;

float KVA_MAX_DEM_ = 0.0;
float VA_TOTAL_ = 0.0;

long count = 0;

/*Current firmware version*/
const String currentVersion = "2.6";

unsigned long previouslyPublishedMillis = 0;
unsigned long readingsPublishingInterval = 10;

unsigned long previouslySampledMillis = 0;
unsigned long readingsSamplingInterval = 100;

/* Readings related variables */
#define RXD2 2  // RS485 or RS232, Hardware Serial2
#define TXD2 1  // RS485 or RS232, Hardware Serial2
uint16_t au16data[16];
ModbusMaster node;
float wattstotal, disp_pf_avg, vatotal, varphase, vayphase, vabphase, vllavg, vryphase, vybphase, vbrphase, vlnavg, vrphase, vyphase, vbphase, iavg, irphase, iyphase, ibphase, freq, iewh, ievah, ilh, co2, tpfavg, wheb, vaheb, whdg, vahdg, vhrphase, vhyphase, vhbphase, ihrphase, ihyphase, ihbphase, pfavg;

#define EEPROM_SIZE 1024
Preferences preferences;

#define TINY_GSM_MODEM_BG96  //worked for Quectel EC200
#define SerialAT Serial1
#define TINY_GSM_USE_GPRS true


#include <TinyGsmClient.h>  //https://github.com/vshymanskyy/TinyGSM
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>

// Pin Definitions
#define RXD1 40      //4G MODULE RXD INTERNALLY CONNECTED, Hardware Serial 1
#define TXD1 41      //4G MODULE TXD INTERNALLY CONNECTED, Hardware Serial 1
#define powerPin 42  ////4G MODULE ESP32 PIN D4 CONNECTED TO POWER PIN OF EC200 CHIPSET, INTERNALLY CONNECTED
#define RXD2 2       // RS485 or RS232, Hardware Serial2
#define TXD2 1       // RS485 or RS232, Hardware Serial2

const int statusLED = 18;  // Onboard LED

char brokerBuffer[64];
char usernameBuffer[64];
char passwordBuffer[64];
char *broker;
char *username;
char *password;
uint16_t mqtt_port;

const char apn[] = "";  //APN automatically detects for 4G SIM IN MOST CASES, IF NOT DETECTED, THEN ENTER APN

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);
HttpClient http(client, server, port);

String readBuffer;
String doorStatus = "Open";

const char *phoneNumber = "+919041704179";  // Format: +91 followed by 10-digit Indian mobile number

StaticJsonDocument<1024> eventIds;
StaticJsonDocument<1024> deviceTypes;

void setDeviceTypes() {
  deviceTypes["1P-Basic"] = "1";
  deviceTypes["1P-Adv"] = "2";
  deviceTypes["3P-Basic"] = "3";
  deviceTypes["3P-Adv"] = "4";
}

void setEventIds() {
  eventIds["VR_HIGH"] = 1;
  eventIds["VY_HIGH"] = 1;
  eventIds["VB_HIGH"] = 1;
  eventIds["VRY_HIGH"] = 1;
  eventIds["VYB_HIGH"] = 1;
  eventIds["VBR_HIGH"] = 1;

  eventIds["VR_LOW"] = 2;
  eventIds["VY_LOW"] = 2;
  eventIds["VB_LOW"] = 2;
  eventIds["VRY_LOW"] = 2;
  eventIds["VYB_LOW"] = 2;
  eventIds["VBR_LOW"] = 2;

  eventIds["IR_HIGH"] = 5;
  eventIds["IY_HIGH"] = 5;
  eventIds["IB_HIGH"] = 5;

  eventIds["WATTS_HIGH"] = 6;

  eventIds["IHR_HIGH"] = 9;
  eventIds["IHY_HIGH"] = 9;
  eventIds["IHB_HIGH"] = 9;

  eventIds["VHR_HIGH"] = 10;
  eventIds["VHY_HIGH"] = 10;
  eventIds["VHB_HIGH"] = 10;

  eventIds["PF_LOW"] = 11;

  eventIds["FRQ_LOW"] = 12;
  eventIds["FRQ_HIGH"] = 13;
}

void syncTimeFromSIM() {
  if (modem.isNetworkConnected()) {

    String dt = modem.getGSMDateTime(DATE_FULL);

    struct tm t = {};

    sscanf(dt.c_str(),
           "%d/%d/%d,%d:%d:%d",
           &t.tm_year,
           &t.tm_mon,
           &t.tm_mday,
           &t.tm_hour,
           &t.tm_min,
           &t.tm_sec);

    t.tm_year -= 1900;
    t.tm_mon -= 1;

    baseEpoch = mktime(&t);
    baseMillis = millis();

    // FORMAT DATETIME
    char buffer[50];

    sprintf(buffer,
            "%02d-%02d-%04d %02d:%02d:%02d",
            t.tm_mday,
            t.tm_mon + 1,
            t.tm_year + 1900,
            t.tm_hour,
            t.tm_min,
            t.tm_sec);

    Serial.println("Formatted DateTime from SIM:");
    Serial.println(buffer);
  }
}

void setup() {

  // Enable the 4G chipset
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);  //turn statusLED On or Off as per your application scenario
  delay(3000);

  // Initialize Serial Communication
  Serial.begin(115200);                            //Default Serial Monitor
  SerialAT.begin(115200, SERIAL_8N1, RXD1, TXD1);  //Serial 1 for 4G
  // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);     //Serial 2 for RS485 / RS232
  Serial0.begin(9600);  // Data Acquisition of Analog/Digital Channels
  delay(10);
  Serial.println("Vajravegha....");

  Wire.begin(5, 4);  // this is essential
  delay(100);
  // Fetch RTC Data
  if (!rtc.begin()) {
    Serial.println("RTC initialization failed!");
    //return; //CHECK RETURN FOR BOTH SD AND RTC
  } else {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(100);
    Serial.println("RTC initialized.");
    // DateTime now = rtc.now();
    // String timestamp = now.timestamp(DateTime::TIMESTAMP_FULL);
    // Serial.print("RTC Time: ");
    // Serial.println(timestamp);
  }

  check4Gmodule();
  //enableGPS();

  /////////////////////////////
  syncTimeFromSIM();
  /////////////////////////////

  mqtt.setServer(broker, mqtt_port);
  mqtt.setCallback(mqttCallback);

  Serial0.begin(9600);  // Data Acquisition of Analog/Digital Channels
  delay(100);
  Serial.println("Reset MS51 controller");
  Serial0.println("RSTDATA");  //sending RESET command to MS51 controller
  delay(1000);
  readBuffer = Serial0.readStringUntil('\n');
  Serial.println(readBuffer);

  /* Readings Related */
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Serial2.setTimeout(500);  // 500ms timeout - gives PN8700 enough time to respond to all registers
  node.begin(Serial2);

  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
    return;
  }

  WiFi.begin();
  delay(1000);
  MAC_ID = WiFi.macAddress();
  Serial.print("ESP32 MAC Address: ");
  Serial.println(MAC_ID);

  bool deviceTypefetched;
  do {
    deviceTypefetched = fetchDeviceType();
  } while (!deviceTypefetched);

  // fetchDeviceType();

  preferences.begin("storage", false);
  /***************************************************/
  preferences.putInt("VRL_FROM", 0);
  preferences.putInt("VRL_TO", 9);

  preferences.putInt("VRU_FROM", 10);
  preferences.putInt("VRU_TO", 19);

  preferences.putInt("VYL_FROM", 20);
  preferences.putInt("VYL_TO", 29);

  preferences.putInt("VYU_FROM", 30);
  preferences.putInt("VYU_TO", 39);

  preferences.putInt("VBL_FROM", 40);
  preferences.putInt("VBL_TO", 49);

  preferences.putInt("VBU_FROM", 50);
  preferences.putInt("VBU_TO", 59);

  preferences.putInt("VRYL_FROM", 60);
  preferences.putInt("VRYL_TO", 69);

  preferences.putInt("VRYU_FROM", 70);
  preferences.putInt("VRYU_TO", 79);

  preferences.putInt("VYBL_FROM", 80);
  preferences.putInt("VYBL_TO", 89);

  preferences.putInt("VYBU_FROM", 90);
  preferences.putInt("VYBU_TO", 99);

  preferences.putInt("VBRL_FROM", 100);
  preferences.putInt("VBRL_TO", 109);

  preferences.putInt("VBRU_FROM", 110);
  preferences.putInt("VBRU_TO", 119);

  preferences.putInt("WATTSL_FROM", 120);
  preferences.putInt("WATTSL_TO", 129);

  preferences.putInt("WATTSU_FROM", 130);
  preferences.putInt("WATTSU_TO", 139);

  preferences.putInt("IRU_FROM", 140);
  preferences.putInt("IRU_TO", 149);

  preferences.putInt("IYU_FROM", 150);
  preferences.putInt("IYU_TO", 159);

  preferences.putInt("IBU_FROM", 160);
  preferences.putInt("IBU_TO", 169);
  /***************************************************/
  preferences.putInt("VRLW_FROM", 170);
  preferences.putInt("VRLW_TO", 179);

  preferences.putInt("VRUW_FROM", 180);
  preferences.putInt("VRUW_TO", 189);

  preferences.putInt("VYLW_FROM", 190);
  preferences.putInt("VYLW_TO", 199);

  preferences.putInt("VYUW_FROM", 200);
  preferences.putInt("VYUW_TO", 209);

  preferences.putInt("VBLW_FROM", 210);
  preferences.putInt("VBLW_TO", 219);

  preferences.putInt("VBUW_FROM", 220);
  preferences.putInt("VBUW_TO", 229);

  preferences.putInt("VRYLW_FROM", 230);
  preferences.putInt("VRYLW_TO", 239);

  preferences.putInt("VRYUW_FROM", 240);
  preferences.putInt("VRYUW_TO", 249);

  preferences.putInt("VYBLW_FROM", 250);
  preferences.putInt("VYBLW_TO", 259);

  preferences.putInt("VYBUW_FROM", 260);
  preferences.putInt("VYBUW_TO", 269);

  preferences.putInt("VBRLW_FROM", 270);
  preferences.putInt("VBRLW_TO", 279);

  preferences.putInt("VBRUW_FROM", 280);
  preferences.putInt("VBRUW_TO", 289);

  preferences.putInt("WATTSLW_FROM", 290);
  preferences.putInt("WATTSLW_TO", 299);

  preferences.putInt("WATTSUW_FROM", 300);
  preferences.putInt("WATTSUW_TO", 309);

  preferences.putInt("IRUW_FROM", 310);
  preferences.putInt("IRUW_TO", 319);

  preferences.putInt("IYUW_FROM", 320);
  preferences.putInt("IYUW_TO", 329);

  preferences.putInt("IBUW_FROM", 330);
  preferences.putInt("IBUW_TO", 339);

  preferences.putInt("INTERVAL_FROM", 340);
  preferences.putInt("INTERVAL_TO", 349);

  preferences.putInt("SAMPRATE_FROM", 350);
  preferences.putInt("SAMPRATE_TO", 359);

  preferences.putInt("V1PU_FROM", 360);
  preferences.putInt("V1PU_TO", 369);
  preferences.putInt("V1PL_FROM", 370);
  preferences.putInt("V1PL_TO", 379);

  preferences.putInt("V1PUW_FROM", 380);
  preferences.putInt("V1PUW_TO", 389);
  preferences.putInt("V1PLW_FROM", 390);
  preferences.putInt("V1PLW_TO", 399);

  preferences.putInt("V2PU_FROM", 400);
  preferences.putInt("V2PU_TO", 409);
  preferences.putInt("V2PL_FROM", 410);
  preferences.putInt("V2PL_TO", 419);

  preferences.putInt("V2PUW_FROM", 420);
  preferences.putInt("V2PUW_TO", 429);
  preferences.putInt("V2PLW_FROM", 430);
  preferences.putInt("V2PLW_TO", 439);

  preferences.putInt("IU_FROM", 440);
  preferences.putInt("IU_TO", 449);

  preferences.putInt("IUW_FROM", 450);
  preferences.putInt("IUW_TO", 459);

  preferences.putInt("PFL_FROM", 460);
  preferences.putInt("PFL_TO", 469);
  preferences.putInt("PFLW_FROM", 470);
  preferences.putInt("PFLW_TO", 479);

  preferences.putInt("FRQU_FROM", 480);
  preferences.putInt("FRQU_TO", 489);
  preferences.putInt("FRQL_FROM", 490);
  preferences.putInt("FRQL_TO", 499);
  preferences.putInt("FRQUW_FROM", 500);
  preferences.putInt("FRQUW_TO", 509);
  preferences.putInt("FRQLW_FROM", 510);
  preferences.putInt("FRQLW_TO", 519);

  preferences.putInt("VTHDU_FROM", 520);
  preferences.putInt("VTHDU_TO", 529);

  preferences.putInt("ITHDU_FROM", 530);
  preferences.putInt("ITHDU_TO", 539);

  preferences.putInt("VDPU_FROM", 540);
  preferences.putInt("VDPU_TO", 549);

  preferences.putInt("IDPU_FROM", 550);
  preferences.putInt("IDPU_TO", 559);

  /***************************************************/

  preferences.putString("VRL", readEEPROM(0, 9));
  preferences.putString("VRU", readEEPROM(10, 19));
  preferences.putString("VYL", readEEPROM(20, 29));
  preferences.putString("VYU", readEEPROM(30, 39));
  preferences.putString("VBL", readEEPROM(40, 49));
  preferences.putString("VBU", readEEPROM(50, 59));
  preferences.putString("VRYL", readEEPROM(60, 69));
  preferences.putString("VRYU", readEEPROM(70, 79));
  preferences.putString("VYBL", readEEPROM(80, 89));
  preferences.putString("VYBU", readEEPROM(90, 99));
  preferences.putString("VBRL", readEEPROM(100, 109));
  preferences.putString("VBRU", readEEPROM(110, 119));
  preferences.putString("WATTSL", readEEPROM(120, 129));
  preferences.putString("WATTSU", readEEPROM(130, 139));
  preferences.putString("IRU", readEEPROM(140, 149));
  preferences.putString("IYU", readEEPROM(150, 159));
  preferences.putString("IBU", readEEPROM(160, 169));

  preferences.putString("VRLW", readEEPROM(170, 179));
  preferences.putString("VRUW", readEEPROM(180, 189));
  preferences.putString("VYLW", readEEPROM(190, 199));
  preferences.putString("VYUW", readEEPROM(200, 209));
  preferences.putString("VBLW", readEEPROM(210, 219));
  preferences.putString("VBUW", readEEPROM(220, 229));
  preferences.putString("VRYLW", readEEPROM(230, 239));
  preferences.putString("VRYUW", readEEPROM(240, 249));
  preferences.putString("VYBLW", readEEPROM(250, 259));
  preferences.putString("VYBUW", readEEPROM(260, 269));
  preferences.putString("VBRLW", readEEPROM(270, 279));
  preferences.putString("VBRUW", readEEPROM(280, 289));
  preferences.putString("WATTSLW", readEEPROM(290, 299));
  preferences.putString("WATTSUW", readEEPROM(300, 309));
  preferences.putString("IRUW", readEEPROM(310, 319));
  preferences.putString("IYUW", readEEPROM(320, 329));
  preferences.putString("IBUW", readEEPROM(330, 339));

  preferences.putString("INTERVAL", readEEPROM(340, 349));
  preferences.putString("SAMPRATE", readEEPROM(350, 359));

  preferences.putString("V1PU", readEEPROM(360, 369));
  preferences.putString("V1PL", readEEPROM(370, 379));
  preferences.putString("V1PUW", readEEPROM(380, 389));
  preferences.putString("V1PLW", readEEPROM(390, 399));

  preferences.putString("V2PU", readEEPROM(400, 409));
  preferences.putString("V2PL", readEEPROM(410, 419));
  preferences.putString("V2PUW", readEEPROM(420, 429));
  preferences.putString("V2PLW", readEEPROM(430, 439));

  preferences.putString("IU", readEEPROM(440, 449));
  preferences.putString("IUW", readEEPROM(450, 459));

  preferences.putString("PFL", readEEPROM(460, 469));
  preferences.putString("PFLW", readEEPROM(470, 479));

  preferences.putString("FRQU", readEEPROM(480, 489));
  preferences.putString("FRQL", readEEPROM(490, 499));
  preferences.putString("FRQUW", readEEPROM(500, 509));
  preferences.putString("FRQLW", readEEPROM(510, 519));

  preferences.putString("VTHDU", readEEPROM(520, 529));

  preferences.putString("ITHDU", readEEPROM(530, 539));

  preferences.putString("VDPU", readEEPROM(540, 549));

  preferences.putString("IDPU", readEEPROM(550, 559));

  setEventIds();
  setDeviceTypes();

  // Initialize SD Card
  if (initSDCard()) {
    String dtString = globalDateTime = getDateTime();
    const char *dt = dtString.c_str();
    Serial.print("deleting date time : ");
    Serial.println(dt);
    if (getFileNameFromDateTime(dt, myFilePath, sizeof(myFilePath))) {
      // Serial.println(myFilePath);
      createDir(SD, myDirPath);
      deleteOldLogFiles(SD, myDirPath, dt);
    }
  } else {
    Serial.println("SD Card initialization failed!");
  }
  // Read the file
  // readFile(SD, filePath);
  // Append to the file
  // appendFile(SD, filePath, "This is appended text.");
  // Read the file again
  // readFile(SD, filePath);
  // List directory contents
  // listDir(SD, "/", 0);

  hashKey(encryptionKey.c_str(), hashedKey, encryptionKey.length());
  memcpy(ivDecrypt, iv, 16);
}

// ------------------ SET RTC FROM STRING ------------------
void setRTCFromString(String dt) {
  int year, month, day, hour, minute, second;

  int parsed = sscanf(dt.c_str(), "%d/%d/%d %d:%d:%d",
                      &year, &month, &day,
                      &hour, &minute, &second);

  if (parsed == 6) {

    // Basic validation
    if (year < 2023 || month > 12 || day > 31 || hour > 23 || minute > 59 || second > 59) {
      Serial.println("Invalid Date-Time Values!");
      return;
    }

    rtc.adjust(DateTime(year, month, day, hour, minute, second));

    Serial.println("RTC Updated Successfully!");
    Serial.println(dt);

  } else {
    Serial.println("Invalid Date-Time Format!");
  }
}

void loop() {
  // Write your code here to run Repaetedly.
  // Serial.println("Version " + String(currentVersion));

  /* Make sure GPRS/EPS is still connected */
  checkGPRS();

  if (!mqtt.connected()) {
    connectToMQTT();
  } else {
    // Serial.println("=== MQTT CONNECTED ===");
    mqtt.loop();
  }
  // delay(1000);

  readAndPublishParams();
  autoRestart();
  // getGPSLocation();
  // printMemoryInfo();
}

void autoRestart() {
  if (offByThreshold && millis() - motorOffTime >= motorRestartInterval) {
    // Serial0.println("RL2ON");
    // Serial.println("RL2ON");
    // delay(500);
    Serial0.println("RL1OFF");
    Serial.println("RL1OFF");
    delay(500);
    disableAutoRestart();
  }
}

void checkGPRS() {
  if (!modem.isGprsConnected()) {
    Serial.println("GPRS disconnected!");
    Serial.print(F("Connecting to "));
    Serial.print(apn);
    if (!modem.gprsConnect(apn)) {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isGprsConnected()) {
      Serial.println("GPRS reconnected");
    }
  }
}

void check4Gmodule() {
  Serial.println("\nconfiguring 4G Module. Kindly wait");
  delay(5000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem...");
  if (!modem.init()) {
    DBG("Failed to restart modem");
    return;
  }
  // Restart takes quite some time
  // To skip it, call init() instead of restart()

  String name = modem.getModemName();
  DBG("Modem Name:", name);  // Quectel EC200U


  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);  // Quectel EC200U Revision: EC200UCNAAR03A03M08




  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");
  delay(15000);
  if (modem.isNetworkConnected()) {
    Serial.println("Connected to Network");
  } else
    Serial.println("No Network");


  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to 4G"));
  Serial.print(apn);
  if (!modem.gprsConnect(apn)) {
    Serial.println(" failed");
    return;
  }
  Serial.println(" success");

  if (modem.isGprsConnected()) {
    Serial.println("LTE Internet connected");
    // if (
    // checkForUpdate();
    //   ) {
    //   performOTA();
    // } else {
    //   Serial.println("Already running the latest firmware.");
    // }
    // Check version if modem is connected
    checkVersion();
    setRTCFromString(getDateTime());
    // fetchMqttCreds();
    setMqttCredsHost();
    // fetchDeviceType();
  } else {
    Serial.println("No LTE Internet");
  }
}

void fetchMqttCreds() {
  Serial.println("Fetching MQTT credentials...");

  const char *credsHost = "ccmsurl-env.eba-ubemb93t.ap-south-1.elasticbeanstalk.com";
  const int credsPort = 80;               // your server port
  const char *credsPath = "/creds/mqtt";  // API endpoint

  HttpClient credsHttp(client, credsHost, credsPort);

  credsHttp.beginRequest();
  credsHttp.get(credsPath);
  credsHttp.endRequest();

  int statusCode = credsHttp.responseStatusCode();
  Serial.print("Status code: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    String response = credsHttp.responseBody();  // <-- only the body
    response.trim();
    Serial.print("Received MQTT creds: ");
    Serial.println(response);
    setMqttCreds(response);
  } else {
    Serial.println("Failed to fetch MQTT creds");
  }
}

bool fetchDeviceType() {
  Serial.println("Fetching Device Type...");

  const char *host = "124.40.247.18";
  const int port = 214;
  const String macAddress = MAC_ID;

  String path = "/dblayer/GetDeviceType/" + macAddress;

  Serial.print("Host: ");
  Serial.println(host);

  Serial.print("Port: ");
  Serial.println(port);

  Serial.print("Path: ");
  Serial.println(path);

  HttpClient http(client, host, port);
  http.setHttpResponseTimeout(5000);  // 5 seconds
  loopMqttWithoutSerialLog();

  String auth = "max24:max24";
  String encodedAuth = base64::encode(auth);

  http.beginRequest();
  http.get(path);
  http.sendHeader("Authorization", "Basic " + encodedAuth);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  String response = http.responseBody();

  Serial.print("Response: ");
  Serial.println(response);

  Serial.print("Status code: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    // String response = http.responseBody();
    // response.trim();

    // Serial.print("Device Type: ");
    // Serial.println(response);
    if (response.equals("1")) {
      deviceType = "1P-Basic";
      Serial.println("1P-Basic device type");
    } else if (response.equals("2")) {
      deviceType = "1P-Adv";
      Serial.println("1P-Adv device type");
    } else if (response.equals("3")) {
      deviceType = "3P-Basic";
      Serial.println("3P-Basic device type");
    } else if (response.equals("4")) {
      deviceType = "3P-Adv";
      Serial.println("3P-Adv device type");
    } else {
      Serial.println("Unknown device type");
    }
    return true;
  } else {
    Serial.println("Failed to fetch device type");
    publishError(String(statusCode), "Fetch Device Type Api Error");
    return false;
  }
}

void publishError(String errorCode, String errorType) {
  DynamicJsonDocument error_doc(512);
  error_doc["id"] = MAC_ID;
  error_doc["info"] = "Error";
  error_doc["errorType"] = errorType;
  error_doc["errorCode"] = errorCode;
  error_doc["dateTime"] = getDateTime();
  String error_Str;
  serializeJson(error_doc, error_Str);
  Serial.print(errorType);
  Serial.println(error_Str);
  if (securedMode) {
    error_Str = initiateEncryption(error_Str);
  }
  if (!mqtt.connected()) {
    connectToMQTT();
  } else {
    mqtt.loop();
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm_error", error_Str.c_str());
  } else {
    Serial.print("MQTT not connected. Cannot publish ");
    Serial.println(errorType);
  }
}

void loopMqttWithoutSerialLog() {
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

void setMqttCreds(String response) {
  int firstComma = response.indexOf(',');
  int secondComma = response.indexOf(',', firstComma + 1);
  int thirdComma = response.indexOf(',', secondComma + 1);

  if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
    String brokerStr = response.substring(0, firstComma);
    String portStr = response.substring(firstComma + 1, secondComma);
    String usernameStr = response.substring(secondComma + 1, thirdComma);
    String passwordStr = response.substring(thirdComma + 1);

    brokerStr.toCharArray(brokerBuffer, sizeof(brokerBuffer));
    usernameStr.toCharArray(usernameBuffer, sizeof(usernameBuffer));
    passwordStr.toCharArray(passwordBuffer, sizeof(passwordBuffer));

    broker = brokerBuffer;
    mqtt_port = (uint16_t)portStr.toInt();
    username = usernameBuffer;
    password = passwordBuffer;

    Serial.print("Broker: ");
    Serial.println(broker);
    Serial.print("Port: ");
    Serial.println(mqtt_port);
    Serial.print("Username: ");
    Serial.println(username);
    Serial.print("Password: ");
    Serial.println(password);
  } else {
    Serial.println("Invalid response format!");
  }
}

void setMqttCredsHost() {
  String brokerStr = "mqtt.remsmartsystems.com";
  String portStr = "215";
  String usernameStr = "ttbs";
  String passwordStr = "ttbs@123";

  brokerStr.toCharArray(brokerBuffer, sizeof(brokerBuffer));
  usernameStr.toCharArray(usernameBuffer, sizeof(usernameBuffer));
  passwordStr.toCharArray(passwordBuffer, sizeof(passwordBuffer));

  broker = brokerBuffer;
  mqtt_port = (uint16_t)portStr.toInt();
  username = usernameBuffer;
  password = passwordBuffer;

  Serial.print("Broker: ");
  Serial.println(broker);
  Serial.print("Port: ");
  Serial.println(mqtt_port);
  Serial.print("Username: ");
  Serial.println(username);
  Serial.print("Password: ");
  Serial.println(password);
}

void connectToMQTT() {
  /* Loop until connected to MQTT server */
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");

    boolean status = mqtt.connect(MAC_ID.c_str(), username, password);

    if (status == false) {
      Serial.println(" fail");
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }

    else {

      Serial.println("Success");
      Serial.println("Topic to subscribe is : ");
      Serial.println(MAC_ID);
      mqtt.subscribe(MAC_ID.c_str());
      mqtt.subscribe("ESPPQM");
    }
  }
}

bool containsKey(const String &data) {
  for (auto &key : keys) {
    if (data.indexOf(key) >= 0) {  // just substring check
      Serial.print("Found key: ");
      Serial.println(key);
      return true;  // stop at first match
    }
  }
  return false;
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
  Serial.println();
  Serial.println("***************** Message arrived **************");
  Serial.println("Topic : " + String(topic));

  String message;
  for (int i = 0; i < len; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message : " + message);

  if (securedMode) {
    message = initiateDecryption(message);
    Serial.println("Decrypted Message : " + message);
  }

  if (String(topic).equals("ESPPQM")) {
    if (!(message.equals("FETCH_GPS_COORDINATES") || message.equals("ENABLE_THRESHOLD_CHECK") || message.equals("DISABLE_THRESHOLD_CHECK") || containsKey(message))) {
      Serial.println("Message not allowed on topic : ESPPQM, ignoring.");
      return;  // stop processing
    }
  }

  if (message.equals("PUMPON")) {
    Serial0.println("RL2ON");
    Serial.println("RL2ON");
    delay(500);
    disableAutoRestart();
  } else if (message.equals("PUMPOFF")) {
    Serial0.println("RL2OFF");
    Serial.println("RL2OFF");
    delay(500);
    disableAutoRestart();
  } else if (message.equals("TIMEROFF")) {
    Serial0.println("RL1ON");
    Serial.println("RL1ON");
    delay(500);
  } else if (message.equals("TIMERON")) {
    Serial0.println("RL1OFF");
    Serial.println("RL1OFF");
    delay(500);
  } else if (message.equals("VERSION_CHECK")) {
    publishVerion();
  } else if (message.equals("FETCH_THRESHOLD_DATA")) {
    Serial.println("----- :: Reading EEPROM :: ----- ");
    initiateEEPROMread();
    Serial.println();
  } else if (message.equals("Status")) {
    publishStatus();
  } else if (message.equals("RESET_CONTROLLER")) {
    Serial.println("Restarting the MicroController...");
    ESP.restart();
  } else if (message.equals("ENABLE_THRESHOLD_CHECK")) {
    writeThresholdCheckStateIntoEEPROM(message);
  } else if (message.equals("DISABLE_THRESHOLD_CHECK")) {
    writeThresholdCheckStateIntoEEPROM(message);
  } else if (message.equals("FETCH_GPS_COORDINATES")) {
    publishGPSCoordinates();
  } else if (message.indexOf("getSDCardData:") != -1) {
    String date = message.substring(message.indexOf(":") + 1);
    date.trim();  // Removes \r \n spaces
    String path = "/CCMS/Log/" + date + ".txt";
    sendFileMQTT(SD, path.c_str());
  } else {
    parseData(message);
  }
  Serial.println("***********************************************");
}

void sendFileMQTT(fs::FS &fs, const char *path) {

  if (SD.cardType() == CARD_NONE) {
    Serial.println("❌ SD card not available. Trying to reinitialize.");
    if (!initSDCard()) return;
  }

  File file = fs.open(path);
  if (!file) {
    Serial.println("❌ Failed to open file");
    return;
  }

  // ⭐ GET TOTAL FILE SIZE
  size_t totalSize = file.size();
  Serial.printf("📦 File Size: %u bytes\n", totalSize);

  Serial.println("📄 Reading file in chunks...\n");

  char buffer[CHUNK_SIZE + 1];  // +1 for string termination
  int chunkNo = 0;

  unsigned long prevLoopMillis = 0;
  unsigned long currLoopMillis = 0;

  while (file.available()) {

    int len = file.readBytes(buffer, CHUNK_SIZE);
    buffer[len] = '\0';  // Make printable
    publishSDCardDataChunk(buffer, ++chunkNo);

    // Serial.printf("---- Chunk %d (%d bytes) ----\n", chunkNo, len);
    // Serial.println(buffer);
    Serial.println("----------------------------\n");
    currLoopMillis = millis();
    if (currLoopMillis - prevLoopMillis >= 500) {
      loopMQTT();
      prevLoopMillis = currLoopMillis;
    }

    delay(100);  // readability in Serial Monitor
  }

  file.close();
  Serial.println("✅ File read complete");
}

void publishSDCardDataChunk(const char *chunkData, int chunkNo) {
  StaticJsonDocument<2048> sd_chunk_doc;
  sd_chunk_doc["id"] = MAC_ID;
  sd_chunk_doc["info"] = "SD-Chunk";
  sd_chunk_doc["chunkNumber"] = chunkNo;
  sd_chunk_doc["chunkData"] = chunkData;
  sd_chunk_doc["deviceType"] = deviceTypes[deviceType];

  String chunk_Str;
  serializeJson(sd_chunk_doc, chunk_Str);
  // Serial.printf("Payload size before encryption: %u bytes\n", strlen(chunk_Str.c_str()));
  if (securedMode) {
    chunk_Str = initiateEncryption(chunk_Str);
    Serial.print("Encrypted Payload: ");
    Serial.println(chunk_Str);
  }
  // Serial.printf("Payload size after encryption: %u bytes\n", strlen(chunk_Str.c_str()));
  if (mqtt.connected()) {
    mqtt.publish("pqm", chunk_Str.c_str());
  }
}

void publishGPSCoordinates() {
  if (!getGPSLocation()) return;
  DynamicJsonDocument gps_doc(512);
  gps_doc["id"] = MAC_ID;
  gps_doc["info"] = "GPS Coordinates";
  gps_doc["latitude"] = latitude;
  gps_doc["longitude"] = longitude;
  gps_doc["deviceType"] = deviceTypes[deviceType];
  String gps_Str;
  serializeJson(gps_doc, gps_Str);
  if (securedMode) {
    gps_Str = initiateEncryption(gps_Str);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", gps_Str.c_str());
  }
}

void publishVerion() {
  DynamicJsonDocument ver_doc(512);
  ver_doc["id"] = MAC_ID;
  ver_doc["info"] = "Version";
  ver_doc["version"] = currentVersion;
  ver_doc["deviceType"] = deviceTypes[deviceType];
  String ver_Str;
  serializeJson(ver_doc, ver_Str);
  if (securedMode) {
    ver_Str = initiateEncryption(ver_Str);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", ver_Str.c_str());
  }
}

void publishStatus() {
  DynamicJsonDocument status_doc(512);
  status_doc["id"] = MAC_ID;
  status_doc["info"] = "Status";
  status_doc["status"] = "Active";
  status_doc["deviceType"] = deviceTypes[deviceType];
  String status_Str;
  serializeJson(status_doc, status_Str);
  if (securedMode) {
    status_Str = initiateEncryption(status_Str);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", status_Str.c_str());
  }
}

void writeThresholdCheckStateIntoEEPROM(String state) {
  if (state.equals("ENABLE_THRESHOLD_CHECK")) {
    writeEEPROM(400, 429, String(state));
  } else {
    writeEEPROM(400, 429, String(state));
  }
}

String getDoorState() {
  String currentDoorStatus = "Unknown";
  loopMQTT();
  Serial0.println("GETDATA");  //sending GETDATA command to MS51 controller
  delay(1000);
  readBuffer = Serial0.readStringUntil('\n');
  // Serial.println(readBuffer);
  DynamicJsonDocument door(512);
  DeserializationError error = deserializeJson(door, readBuffer);
  loopMQTT();
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return currentDoorStatus;
  }
  // Extract the value of DI1
  int di1 = door["DI1"];
  // Serial.print("DI1 = ");
  // Serial.println(di1);

  currentDoorStatus = di1 == 0 ? "Closed" : "Open";
  // currentDoorStatus = di1 == 0 ? "Open" : "Closed";

  if (!currentDoorStatus.equals(doorStatus)) {
    char *message = "";
    if (currentDoorStatus.equals("Open")) {
      message = "ALERT: The DOOR is OPEN.";
      publishDoorStatus();
    } else if (currentDoorStatus.equals("Closed")) {
      message = "NOTICE: The DOOR is CLOSED.";
    }
    // sendSMSUsingTinyGSM(message);
    doorStatus = currentDoorStatus;
  }
  loopMQTT();
  Serial.println("Door_Status : " + String(currentDoorStatus));
  return currentDoorStatus;
}

void publishDoorStatus() {
  DynamicJsonDocument door_doc(512);
  door_doc["id"] = MAC_ID;
  door_doc["info"] = "Door_Status";
  door_doc["status"] = "Open";
  door_doc["deviceType"] = deviceTypes[deviceType];
  String door_Str;
  serializeJson(door_doc, door_Str);
  if (securedMode) {
    door_Str = initiateEncryption(door_Str);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", door_Str.c_str());
  }
}

String getDateTime() {
  String dateAndTime = "Unknown";
  loopMQTT();
  if (modem.isNetworkConnected()) {
    String dateTime = modem.getGSMDateTime(DATE_FULL);
    // Parse the date and time from the full string
    // Format is typically "YYYY/MM/DD,HH:MM:SS+TZ"
    int commaIndex = dateTime.indexOf(',');
    if (commaIndex != -1) {
      String date = dateTime.substring(0, commaIndex);
      String time = dateTime.substring(commaIndex + 1, commaIndex + 9);  // Get HH:MM:SS

      Serial.print("Current Date and Time: ");
      Serial.print(date);
      Serial.print(" ");
      Serial.println(time);
      dateAndTime = date + " " + time;
    } else {
      Serial.println("Error parsing date and time");
    }
    // Serial.println("------------------------");
  } else {
    Serial.println("Network not connected - cannot get time");
  }
  loopMQTT();
  return dateAndTime;
}

// ------------------ GET RTC TIMESTAMP ------------------
String getRTCTimestamp() {
  DateTime now = rtc.now();

  char buffer[25];
  sprintf(buffer, "%04d/%02d/%02d %02d:%02d:%02d",
          now.year(),
          now.month(),
          now.day(),
          now.hour(),
          now.minute(),
          now.second());

  return String(buffer);
}

bool readParamFromMeter(uint16_t paramAddress, float &targetVar) {
  uint16_t answer = 0;
  uint16_t DATA1[10];
  answer = node.readHoldingRegisters(1, paramAddress, 6);
  if (answer == node.ku8MBSuccess) {
    for (int Aj = 0; Aj < 6; Aj++) {
      DATA1[Aj] = node.getResponseBuffer(Aj);
    }
    uint32_t combined = DATA1[1];
    combined = (combined << 16) | DATA1[0];
    memcpy(&targetVar, &combined, 4);
    delay(25);
    loopMQTT();
    return true;
  }
  loopMQTT();
  return false;
}

String getCurrentTime() {
  time_t currentEpoch = baseEpoch + ((millis() - baseMillis) / 1000);

  struct tm *t = localtime(&currentEpoch);

  char buffer[25];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          t->tm_year + 1900,
          t->tm_mon + 1,
          t->tm_mday,
          t->tm_hour,
          t->tm_min,
          t->tm_sec);

  Serial.println("Date time from get current time  : ");
  Serial.println(buffer);
  return String(buffer);
}

void readAndPublishParams() {

  bool publish = false;
  bool published = false;
  warningOccured = false;
  uint8_t result;

  unsigned long currentMillis = millis();

  readingsSamplingInterval = preferences.getString("SAMPRATE").toInt() * 1000;

  if (currentMillis - previouslySampledMillis >= readingsSamplingInterval) {

    DynamicJsonDocument docu(4096);
    docu["id"] = MAC_ID;
    docu["info"] = "Readings";
    String dateTime;
    // docu["dateTime"] = dateTime = globalDateTime = getDateTime();
    // docu["dateTime"] = dateTime = globalDateTime = getRTCTimestamp();
    docu["dateTime"] = dateTime = globalDateTime = getCurrentTime();
    Serial.println("Date time to publish is : ");
    Serial.println(dateTime.c_str());
    const char *dt = dateTime.c_str();
    // docu["Door_Status"] = getDoorState();
    // getDoorState();

    float VRL = preferences.getString("VRL").toFloat();
    float VRU = preferences.getString("VRU").toFloat();
    float VYL = preferences.getString("VYL").toFloat();
    float VYU = preferences.getString("VYU").toFloat();
    float VBL = preferences.getString("VBL").toFloat();
    float VBU = preferences.getString("VBU").toFloat();
    float VRYL = preferences.getString("VRYL").toFloat();
    float VRYU = preferences.getString("VRYU").toFloat();
    float VYBL = preferences.getString("VYBL").toFloat();
    float VYBU = preferences.getString("VYBU").toFloat();
    float VBRL = preferences.getString("VBRL").toFloat();
    float VBRU = preferences.getString("VBRU").toFloat();
    float WATTSL = preferences.getString("WATTSL").toFloat();
    float WATTSU = preferences.getString("WATTSU").toFloat();
    float IRU = preferences.getString("IRU").toFloat();
    float IYU = preferences.getString("IYU").toFloat();
    float IBU = preferences.getString("IBU").toFloat();

    float VRLW = preferences.getString("VRLW").toFloat();
    float VRUW = preferences.getString("VRUW").toFloat();
    float VYLW = preferences.getString("VYLW").toFloat();
    float VYUW = preferences.getString("VYUW").toFloat();
    float VBLW = preferences.getString("VBLW").toFloat();
    float VBUW = preferences.getString("VBUW").toFloat();
    float VRYLW = preferences.getString("VRYLW").toFloat();
    float VRYUW = preferences.getString("VRYUW").toFloat();
    float VYBLW = preferences.getString("VYBLW").toFloat();
    float VYBUW = preferences.getString("VYBUW").toFloat();
    float VBRLW = preferences.getString("VBRLW").toFloat();
    float VBRUW = preferences.getString("VBRUW").toFloat();
    float WATTSLW = preferences.getString("WATTSLW").toFloat();
    float WATTSUW = preferences.getString("WATTSUW").toFloat();
    float IRUW = preferences.getString("IRUW").toFloat();
    float IYUW = preferences.getString("IYUW").toFloat();
    float IBUW = preferences.getString("IBUW").toFloat();
    //readingsPublishingInterval = preferences.getString("INTERVAL").toInt() * 1000;

    float V1PL = preferences.getString("V1PL").toFloat();
    float V1PU = preferences.getString("V1PU").toFloat();
    float V1PLW = preferences.getString("V1PLW").toFloat();
    float V1PUW = preferences.getString("V1PUW").toFloat();

    float V2PL = preferences.getString("V2PL").toFloat();
    float V2PU = preferences.getString("V2PU").toFloat();
    float V2PLW = preferences.getString("V2PLW").toFloat();
    float V2PUW = preferences.getString("V2PUW").toFloat();

    float IU = preferences.getString("IU").toFloat();
    float IUW = preferences.getString("IUW").toFloat();

    float PFL = preferences.getString("PFL").toFloat();
    float PFLW = preferences.getString("PFLW").toFloat();

    float FRQU = preferences.getString("FRQU").toFloat();
    float FRQL = preferences.getString("FRQL").toFloat();
    float FRQUW = preferences.getString("FRQUW").toFloat();
    float FRQLW = preferences.getString("FRQLW").toFloat();

    float VTHDU = preferences.getString("VTHDU").toFloat();
    float ITHDU = preferences.getString("ITHDU").toFloat();

    float VDPU = preferences.getString("VDPU").toFloat();
    float IDPU = preferences.getString("IDPU").toFloat();

    String checkThresholds = readEEPROM(400, 429);

    // Serial.print("Start -->");
    // getDateTime();

    // =========================================================
    // JSY-MK-1039 SINGLE-PHASE METER - MODBUS READ
    // Register 72 = Voltage
    // Register 73 = Current
    // Register 74 = Active Power
    // Registers 75-76 = Forward Energy
    // Registers 77-78 = Reverse Energy
    // Register 79 = Power Factor
    // Register 80 = Frequency
    // =========================================================

    clearBuffer();

   



    docu["deviceType"] = deviceTypes[deviceType];

    String jsonString;
    serializeJson(docu, jsonString);

    // Serial.print("End -->");
    // getDateTime();

    // IAVG_ = 100.00;

    // VR_ = 230.00;
    // VY_ = 230.00;
    // VB_ = 230.00;
    // VRY_ = 230.00;
    // VYB_ = 230.00;
    // VBR_ = 230.00;
    // WT_ = 4000.00;
    // IR_ = 80.00;
    // IY_ = 80.00;
    // IB_ = 80.00;

    // if (IAVG_ > 0) {
    if (currentMillis - previouslyPublishedMillis >= readingsPublishingInterval) {

      if (mqtt.connected()) {
        /* Publish the data */

        if (deviceType.equals("1P-Basic") || deviceType.equals("3P-Basic")) {
          String readingsData = String(jsonString);
          Serial.print("Data to be published: ");
          Serial.println(readingsData);
          if (securedMode) {
            readingsData = initiateEncryption(readingsData);
          }
          if (mqtt.publish("pqm", readingsData.c_str())) {
            published = true;
            Serial.println("Done.");
            Serial.println(++count);
            previouslyPublishedMillis = currentMillis;
            publishedReadingsOnce = false;
          } else {
            Serial.println("Data publish failed: ");
          }
        } else if (deviceType.equals("3P-Adv")) {
          if (publish3PAdvChunks(jsonString)) {
            published = true;
            Serial.println("All chunks publishing done.");
            Serial.println(++count);
            previouslyPublishedMillis = currentMillis;
            publishedReadingsOnce = false;
          } else {
            Serial.println("Data publish failed: ");
          }
        }
      } else {
        Serial.println("MQTT not connected, can't publish Readings.");
      }
    }
    // } else {
    //   if (!publishedReadingsOnce) {
    //     Serial.print("Data to be published once after Turning OFF Motor: ");
    //     Serial.println(jsonString);
    //     if(securedMode){
    //       jsonString = initiateEncryption(jsonString);
    //     }
    //     mqtt.publish("pqm", jsonString.c_str());
    //     Serial.println("Done.");
    //     publishedReadingsOnce = true;
    //   }
    // }

    // storeReadingsIntoSDCard(jsonString);
    // storeDataIntoparticularDatedSdCardFile(dt, jsonString);

    previouslySampledMillis = currentMillis;

    if (!checkThresholds.equals("DISABLE_THRESHOLD_CHECK")) {
      // if (IAVG_ > 0) {
      do {
        if (deviceType.equals("3P-Adv") || deviceType.equals("3P-Basic")) {
          if (checkThresholdBreach(VR_, V1PLW, V1PUW, V1PL, V1PU, "VR", jsonString, dt)) { publish = true; }
          if (checkThresholdBreach(VY_, V1PLW, V1PUW, V1PL, V1PU, "VY", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(VB_, V1PLW, V1PUW, V1PL, V1PU, "VB", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(VRY_, V2PLW, V2PUW, V2PL, V2PU, "VRY", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(VYB_, V2PLW, V2PUW, V2PL, V2PU, "VYB", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(VBR_, V2PLW, V2PUW, V2PL, V2PU, "VBR", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VR_, VRLW, VRUW, VRL, VRU, "VR", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VY_, VYLW, VYUW, VYL, VYU, "VY", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VB_, VBLW, VBUW, VBL, VBU, "VB", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VRY_, VRYLW, VRYUW, VRYL, VRYU, "VRY", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VYB_, VYBLW, VYBUW, VYBL, VYBU, "VYB", jsonString, dt)) { publish = true; };
          // if (checkThresholdBreach(VBR_, VBRLW, VBRUW, VBRL, VBRU, "VBR", jsonString, dt)) { publish = true; };

          if (checkSingleThresholdBreach(IR_, IUW, IU, "IR", jsonString, dt)) { publish = true; };
          if (checkSingleThresholdBreach(IY_, IUW, IU, "IY", jsonString, dt)) { publish = true; };
          if (checkSingleThresholdBreach(IB_, IUW, IU, "IB", jsonString, dt)) { publish = true; };
          // if (checkSingleThresholdBreach(IR_, IRUW, IRU, "IR", jsonString, dt)) { publish = true; };
          // if (checkSingleThresholdBreach(IY_, IYUW, IYU, "IY", jsonString, dt)) { publish = true; };
          // if (checkSingleThresholdBreach(IB_, IBUW, IBU, "IB", jsonString, dt)) { publish = true; };

          if (checkSingleLowerThresholdBreach(PF_, PFLW, PFL, "PF", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(FRQ_, FRQLW, FRQUW, FRQL, FRQU, "FRQ", jsonString, dt)) { publish = true; };

          if (checkSingleUpperCriticalThresholdBreach(VHR_, VTHDU, "VHR", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(VHY_, VTHDU, "VHY", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(VHB_, VTHDU, "VHB", jsonString, dt)) { publish = true; };

          if (checkSingleUpperCriticalThresholdBreach(IHR_, ITHDU, "IHR", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(IHY_, ITHDU, "IHY", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(IHB_, ITHDU, "IHB", jsonString, dt)) { publish = true; };

          // checkThresholdBreach(WT_, WATTSLW, WATTSUW, WATTSL, WATTSU, "WATTS", jsonString);
          // if (checkThresholdBreach(WT_, WATTSLW, WATTSUW, WATTSL, WATTSU, "WATTS")) { publish = true; };

          if (checkSingleThresholdBreach(WT_, WATTSUW, WATTSU, "WATTS", jsonString, dt)) { publish = true; };
        }
        if (deviceType.equals("1P-Basic")) {
          if (checkThresholdBreach(VR_, V1PLW, V1PUW, V1PL, V1PU, "VR", jsonString, dt)) { publish = true; }
          if (checkSingleThresholdBreach(IR_, IUW, IU, "IR", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(VHR_, VTHDU, "VHR", jsonString, dt)) { publish = true; };
          if (checkSingleUpperCriticalThresholdBreach(IHR_, ITHDU, "IHR", jsonString, dt)) { publish = true; };
          if (checkSingleLowerThresholdBreach(PF_, PFLW, PFL, "PF", jsonString, dt)) { publish = true; };
          if (checkThresholdBreach(FRQ_, FRQLW, FRQUW, FRQL, FRQU, "FRQ", jsonString, dt)) { publish = true; };
          if (checkSingleThresholdBreach(WT_, WATTSUW, WATTSU, "WATTS", jsonString, dt)) { publish = true; };
        }

        // } else {
        //   Serial.println("Current passing is Zero.");
        // }
      } while (false);
    } else {
      Serial.println("Thresholds checking disabled.");
    }

    if (warningOccured == true) {
      publish = true;
    }

    // VRY_ = 20.00;
    // IR_ = 20.00;

    // checkVoltageDeviation(VR_, VLNAVG_, "Voltage Imbalance - VR", jsonString, dt, 7);
    // checkVoltageDeviation(VY_, VLNAVG_, "Voltage Imbalance - VY", jsonString, dt, 7);
    // checkVoltageDeviation(VB_, VLNAVG_, "Voltage Imbalance - VB", jsonString, dt, 7);
    // checkVoltageDeviation(VRY_, VLLAVG_, "Voltage Imbalance - VRY", jsonString, dt, 7);
    // checkVoltageDeviation(VYB_, VLLAVG_, "Voltage Imbalance - VYB", jsonString, dt, 7);
    // checkVoltageDeviation(VBR_, VLLAVG_, "Voltage Imbalance - VBR", jsonString, dt, 7);
    // checkCurrentDeviation(IR_, IAVG_, "Current Imbalance - IR", jsonString, dt, 8);
    // checkCurrentDeviation(IY_, IAVG_, "Current Imbalance - IY", jsonString, dt, 8);
    // checkCurrentDeviation(IB_, IAVG_, "Current Imbalance - IB", jsonString, dt, 8);

    float vrd = getDeviationPercentage(VR_, VLNAVG_);
    float vyd = getDeviationPercentage(VY_, VLNAVG_);
    float vbd = getDeviationPercentage(VB_, VLNAVG_);

    // vrd = -10.21;
    // vyd = 5.20;
    // vbd = 5.19;
    if (deviceType.equals("3P-Adv") || deviceType.equals("3P-Basic")) {
      if (getMaxDeviatedPhaseAndInitiatePublishDeviation(vrd, vyd, vbd,
                                                         "Voltage Imbalance",
                                                         "Voltage Imbalance",
                                                         "Voltage Imbalance",
                                                         VR_, VY_, VB_,
                                                         VLNAVG_, jsonString,
                                                         dt, VDPU, 7)) { publish = true; };

      float vryd = getDeviationPercentage(VRY_, VLLAVG_);
      float vybd = getDeviationPercentage(VYB_, VLLAVG_);
      float vbrd = getDeviationPercentage(VBR_, VLLAVG_);

      // vryd = 5.21;
      // vybd = 5.20;
      // vbrd = 5.19;
      if (getMaxDeviatedPhaseAndInitiatePublishDeviation(vryd, vybd, vbrd,
                                                         "Voltage Imbalance",
                                                         "Voltage Imbalance",
                                                         "Voltage Imbalance",
                                                         VRY_, VYB_, VBR_,
                                                         VLLAVG_, jsonString,
                                                         dt, VDPU, 7)) { publish = true; };

      float ird = getDeviationPercentage(IR_, IAVG_);
      float iyd = getDeviationPercentage(IY_, IAVG_);
      float ibd = getDeviationPercentage(IB_, IAVG_);

      // ird = 10.21;
      // iyd = 10.20;
      // ibd = 10.19;
      if (getMaxDeviatedPhaseAndInitiatePublishDeviation(ird, iyd, ibd,
                                                         "Current Imbalance",
                                                         "Current Imbalance",
                                                         "Current Imbalance",
                                                         IR_, IY_, IB_,
                                                         IAVG_, jsonString,
                                                         dt, IDPU, 8)) { publish = true; };
    }

    if (publish == true && published == false) {
      if (mqtt.connected()) {

        /* Publish the data */
        if (deviceType.equals("1P-Basic") || deviceType.equals("3P-Basic")) {
          String readingsData = String(jsonString);
          Serial.print("Data to be published: ");
          Serial.println(readingsData);
          if (securedMode) {
            readingsData = initiateEncryption(readingsData);
          }
          if (mqtt.publish("pqm", readingsData.c_str())) {
            Serial.println("Done.");
            // Serial.println(++count);
            // previouslyPublishedMillis = currentMillis;
            // publishedReadingsOnce = false;
          } else {
            Serial.println("Data publish failed: ");
          }
        } else if (deviceType.equals("3P-Adv")) {
          if (publish3PAdvChunks(jsonString)) {
            Serial.println("All chunks publishing done.");
            // Serial.println(++count);
            // previouslyPublishedMillis = currentMillis;
            // publishedReadingsOnce = false;
          } else {
            Serial.println("Data publish failed: ");
          }
        }

      } else {
        Serial.println("MQTT not connected, can't publish Readings.");
      }
    }
    Serial.println("*****************************************************************");
  } else {
    delay(50);
  }
}

bool publish3PAdvChunks(const String &fullJson) {
  DynamicJsonDocument src(4096);

  if (deserializeJson(src, fullJson)) {
    Serial.println("Failed to parse JSON");
    return false;
  }

  bool chunk1Bool = publishChunk(src, 1);
  bool chunk2Bool = publishChunk(src, 2);
  bool chunk3Bool = publishChunk(src, 3);

  return chunk1Bool && chunk2Bool && chunk3Bool;
}

bool publishChunk(JsonDocument &src, int chunkNo) {
  DynamicJsonDocument chunk(1024);

  chunk["id"] = src["id"];
  chunk["info"] = src["info"];
  chunk["dateTime"] = src["dateTime"];
  chunk["deviceType"] = src["deviceType"];
  chunk["chunkNo"] = chunkNo;
  chunk["totalChunks"] = 3;

  switch (chunkNo) {
    case 1:
      chunk["VR"] = src["VR"];
      chunk["VY"] = src["VY"];
      chunk["VB"] = src["VB"];
      chunk["VRY"] = src["VRY"];
      chunk["VYB"] = src["VYB"];
      chunk["VBR"] = src["VBR"];

      chunk["IR"] = src["IR"];
      chunk["IY"] = src["IY"];
      chunk["IB"] = src["IB"];

      chunk["MAX_VDP_LN"] = src["MAX_VDP_LN"];
      chunk["MAX_VDP_LL"] = src["MAX_VDP_LL"];
      chunk["MAX_VDP_I"] = src["MAX_VDP_I"];

      chunk["VLL_AVG"] = src["VLL_AVG"];
      chunk["VLN_AVG"] = src["VLN_AVG"];
      chunk["IAVG"] = src["IAVG"];

      chunk["PF"] = src["PF"];
      chunk["FREQ"] = src["FREQ"];
      chunk["WT"] = src["WT"];

      chunk["VHR"] = src["VHR"];
      chunk["VHY"] = src["VHY"];
      chunk["VHB"] = src["VHB"];
      chunk["IHR"] = src["IHR"];
      chunk["IHY"] = src["IHY"];
      chunk["IHB"] = src["IHB"];
      break;

    case 2:
      chunk["KWHR"] = src["KWHR"];
      chunk["KWHY"] = src["KWHY"];
      chunk["KWHB"] = src["KWHB"];

      chunk["CrestVR"] = src["CrestVR"];
      chunk["CrestVY"] = src["CrestVY"];
      chunk["CrestVB"] = src["CrestVB"];
      chunk["CrestIR"] = src["CrestIR"];
      chunk["CrestIY"] = src["CrestIY"];
      chunk["CrestIB"] = src["CrestIB"];

      chunk["KFactorVR"] = src["KFactorVR"];
      chunk["KFactorVY"] = src["KFactorVY"];
      chunk["KFactorVB"] = src["KFactorVB"];
      chunk["KFactorIR"] = src["KFactorIR"];
      chunk["KFactorIY"] = src["KFactorIY"];
      chunk["KFactorIB"] = src["KFactorIB"];
      break;

    case 3:
      chunk["HarR3V"] = src["HarR3V"];
      chunk["HarY3V"] = src["HarY3V"];
      chunk["HarB3V"] = src["HarB3V"];
      chunk["HarR3I"] = src["HarR3I"];
      chunk["HarY3I"] = src["HarY3I"];
      chunk["HarB3I"] = src["HarB3I"];

      chunk["HarR5V"] = src["HarR5V"];
      chunk["HarY5V"] = src["HarY5V"];
      chunk["HarB5V"] = src["HarB5V"];
      chunk["HarR5I"] = src["HarR5I"];
      chunk["HarY5I"] = src["HarY5I"];
      chunk["HarB5I"] = src["HarB5I"];

      chunk["HarR7V"] = src["HarR7V"];
      chunk["HarY7V"] = src["HarY7V"];
      chunk["HarB7V"] = src["HarB7V"];
      chunk["HarR7I"] = src["HarR7I"];
      chunk["HarY7I"] = src["HarY7I"];
      chunk["HarB7I"] = src["HarB7I"];
      break;

    case 4:
      chunk["VHR"] = src["VHR"];
      chunk["IHR"] = src["IHR"];
      break;
  }

  String payload;
  serializeJson(chunk, payload);

  Serial.printf("Chunk %d size=%d\n", chunkNo, payload.length());

  loopMQTT();
  Serial.print("Data to be published: ");
  Serial.println(payload);
  if (securedMode) {
    payload = initiateEncryption(payload);
  }
  delay(50);
  if (mqtt.publish("pqm", payload.c_str())) {
    Serial.println("Done.");
    return true;
  } else {
    Serial.println("Chunk publish failed: ");
    return false;
  }
}

void checkVoltageDeviation(float reading, float average, String readingName, String readings, const char *dt, int eventId) {

  if (average == 0.0) {
    Serial.println("Average is zero, can't calculate deviation percentage");
    return;
  }

  // Serial.printf("Reading of %s phase is %.2f\n", readingName.c_str(), reading);
  // Serial.printf("Average of %s phase is %.2f\n", readingName.c_str(), average);

  float deviation = average - reading;
  // Serial.printf("Deviation of %s phase is %.2f\n", readingName.c_str(), deviation);

  float deviationPercentage = (deviation / average) * 100.0;
  // Serial.printf("Percentage deviation of %s phase is %.2f%%\n", readingName.c_str(), deviationPercentage);

  if (deviationPercentage > 3) {
    // if (true) {
    publishDeviation(String(reading), String(readingName), "Critical", String(average), String(deviationPercentage), readings, dt, eventId);
  }
}

void checkCurrentDeviation(float reading, float average, String readingName, String readings, const char *dt, int eventId) {

  if (average == 0.0) {
    Serial.println("Average is zero, can't calculate deviation percentage");
    return;
  }

  // Serial.printf("Reading of %s phase is %.2f\n", readingName.c_str(), reading);
  // Serial.printf("Average of %s phase is %.2f\n", readingName.c_str(), average);

  float deviation = average - reading;
  // Serial.printf("Deviation of %s phase is %.2f\n", readingName.c_str(), deviation);

  float deviationPercentage = (deviation / average) * 100.0;
  // Serial.printf("Percentage deviation of %s phase is %.2f%%\n", readingName.c_str(), deviationPercentage);

  if (deviationPercentage > 20) {
    // if (true) {
    publishDeviation(String(reading), String(readingName), "Critical", String(average), String(deviationPercentage), readings, dt, eventId);
  }
}

float getDeviationPercentage(float reading, float average) {

  if (average == 0.0) {
    // Serial.println("Average is zero, so deviation percentage is 0.0");
    return 0.0;
  }

  float deviation = average - reading;
  float deviationPercentage = (deviation / average) * 100.0;
  return deviationPercentage;
}

float getMaxDeviatedPercentage(float percentage1, float percentage2, float percentage3) {

  float maxDev = max(percentage1, max(percentage2, percentage3));

  bool p1Max = abs(percentage1 - maxDev) < EPSILON;
  bool p2Max = abs(percentage2 - maxDev) < EPSILON;
  bool p3Max = abs(percentage3 - maxDev) < EPSILON;

  if (abs(percentage1 - maxDev) < EPSILON) {
    return percentage1;
  } else if (abs(percentage2 - maxDev) < EPSILON) {
    return percentage2;
  } else {
    return percentage3;
  }
}

bool getMaxDeviatedPhaseAndInitiatePublishDeviation(
  float percentage1, float percentage2, float percentage3,
  String paramName1, String paramName2, String paramName3,
  float value1, float value2, float value3,
  float average,
  String readings,
  const char *dt,
  float percentageLimit,
  int eventId) {

  bool deviated = false;

  float maxDev = max(percentage1, max(percentage2, percentage3));

  bool p1Max = abs(percentage1 - maxDev) < EPSILON;
  bool p2Max = abs(percentage2 - maxDev) < EPSILON;
  bool p3Max = abs(percentage3 - maxDev) < EPSILON;

  float readValue;
  String paramDeviated;
  float percentage;

  if (abs(percentage1 - maxDev) < EPSILON) {
    readValue = value1;
    paramDeviated = paramName1;
    percentage = percentage1;
  } else if (abs(percentage2 - maxDev) < EPSILON) {
    readValue = value2;
    paramDeviated = paramName2;
    percentage = percentage2;
  } else {
    readValue = value3;
    paramDeviated = paramName3;
    percentage = percentage3;
  }

  Serial.print("Maximum deviated phase - ");
  Serial.println(paramDeviated);

  if (percentage > percentageLimit) {
    publishDeviation(String(readValue), paramDeviated, "Critical", String(average), String(percentage), readings, dt, eventId);
    deviated = true;
  }

  return deviated;
}

void publishDeviation(String readValue, String paramDeviated, String severity, String average, String percentage, String readings, const char *dt, int eventId) {
  DynamicJsonDocument doc(1024);
  doc["id"] = MAC_ID;
  doc["info"] = "Deviation detected";
  doc["severity"] = severity;
  doc["parameter"] = paramDeviated;
  doc["readingFromMeter"] = readValue;
  doc["averageFromMeter"] = average;
  doc["deviationPercentage"] = percentage;
  doc["eventId"] = String(eventId);
  doc["deviceType"] = deviceTypes[deviceType];
  String deviation_doc;
  serializeJson(doc, deviation_doc);
  // storeDataIntoparticularDatedSdCardFile(dt, deviation_doc);
  if (securedMode) {
    deviation_doc = initiateEncryption(deviation_doc);
    readings = initiateEncryption(readings);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", deviation_doc.c_str());
    // Serial.println(++count);
    // mqtt.publish("pqm", readings.c_str());
  }
}

template<typename T>
void serialLog(T value) {
  Serial.println(value);

  static String varMemoryAllocatedOnce;
  varMemoryAllocatedOnce = "";
  varMemoryAllocatedOnce += value;

  // storeLog(varMemoryAllocatedOnce);
}

void storeLog(String data) {
  const char *dt = globalDateTime.c_str();
  storeDataIntoparticularDatedSdCardFile(dt, data);
}

void storeDataIntoparticularDatedSdCardFile(const char *dt, String jsonString) {

  getFileNameFromDateTime(dt, myFilePath, sizeof(myFilePath));

  if (safeAppendFile(SD, myFilePath, jsonString.c_str())) {
  } else {
    // If write failed, try to reinitialize SD card
    if (!initSDCard()) {
      Serial.println("Failed to reinitialize SD card");
    }
  }
}

String getONTime(uint32_t totalSeconds) {
  uint32_t hours = totalSeconds / 3600;
  uint32_t minutes = (totalSeconds % 3600) / 60;
  uint32_t seconds = totalSeconds % 60;

  String h = String(hours);
  String m = (minutes < 10 ? "0" : "") + String(minutes);
  String s = (seconds < 10 ? "0" : "") + String(seconds);

  return h + ":" + m + ":" + s;
}

float getFloatFromBuffer(uint8_t index) {
  uint16_t lo = node.getResponseBuffer(index);
  uint16_t hi = node.getResponseBuffer(index + 1);

  uint32_t raw = ((uint32_t)hi << 16) | lo;
  float val;
  memcpy(&val, &raw, sizeof(float));
  return val;
}

uint32_t getLongInverseFromBuffer(uint8_t index) {
  uint16_t hi = node.getResponseBuffer(index);
  uint16_t lo = node.getResponseBuffer(index + 1);

  return ((uint32_t)hi << 16) | lo;
}

void clearBuffer() {
  delay(25);
  node.clearResponseBuffer();
}

void storeReadingsIntoSDCard(String jsonString) {
  if (safeAppendFile(SD, filePath, jsonString.c_str())) {
    // Read the file again
    // readFile(SD, filePath);
  } else {
    // If write failed, try to reinitialize SD card
    if (!initSDCard()) {
      Serial.println("Failed to reinitialize SD card");
    }
  }
}

bool checkThresholdBreach(float value, float lowerWarning, float upperWarning, float lowerCritical, float upperCritical, String phase, String readings, const char *dt) {
  loopMQTT();
  bool flag = false;
  int eventId = -1;

  if (value < lowerWarning || value < lowerCritical) {
    eventId = eventIds.containsKey(phase + String("_LOW")) ? eventIds[phase + String("_LOW")] : eventId;
  } else if (value > upperWarning || value > upperCritical) {
    eventId = eventIds.containsKey(phase + String("_HIGH")) ? eventIds[phase + String("_HIGH")] : eventId;
  }

  if (value < lowerWarning || value > upperWarning) {
    if (value < lowerCritical || value > upperCritical) {
      publishBreach(String(lowerCritical), String(upperCritical), String(value), phase, "Critical", readings, dt, eventId);
      flag = true;
    } else {
      warningOccured = true;
      publishBreach(String(lowerWarning), String(upperWarning), String(value), phase, "Warning", readings, dt, eventId);
    }
  }

  // if (value < lowerCritical || value > upperCritical) {
  //   publishBreach(String(lowerCritical), String(upperCritical), String(value), phase, "Critical", readings, dt, eventId);
  //   flag = true;
  // }

  return flag;
}

bool checkSingleThresholdBreach(float value, float upperWarning, float upperCritical, String phase, String readings, const char *dt) {
  loopMQTT();
  bool flag = false;
  int eventId = -1;

  if (value > upperWarning || value > upperCritical) {
    eventId = eventIds.containsKey(phase + String("_HIGH")) ? eventIds[phase + String("_HIGH")] : eventId;
  }

  if (value > upperWarning) {
    if (value > upperCritical) {
      publishBreach1(String(upperCritical), String(value), phase, "Critical", readings, dt, eventId);
      flag = true;
    } else {
      warningOccured = true;
      publishBreach1(String(upperWarning), String(value), phase, "Warning", readings, dt, eventId);
    }
  }

  // if (value > upperCritical) {
  //   publishBreach1(String(upperCritical), String(value), phase, "Critical", readings, dt, eventId);
  //   flag = true;
  // }

  return flag;
}

bool checkSingleLowerThresholdBreach(float value, float lowerWarning, float lowerCritical, String phase, String readings, const char *dt) {
  loopMQTT();
  bool flag = false;

  if (value < lowerWarning) {
    if (value < lowerCritical) {
      publishLowerBreach1(String(lowerCritical), String(value), phase, "Critical", readings, dt);
      flag = true;
    } else {
      warningOccured = true;
      publishLowerBreach1(String(lowerWarning), String(value), phase, "Warning", readings, dt);
    }
  }

  // if (value < lowerCritical) {
  //   publishLowerBreach1(String(lowerCritical), String(value), phase, "Critical", readings, dt);
  //   flag = true;
  // }

  return flag;
}

bool checkSingleUpperCriticalThresholdBreach(float value, float upperCritical, String phase, String readings, const char *dt) {
  loopMQTT();
  bool flag = false;
  int eventId = -1;

  if (value > upperCritical) {
    eventId = eventIds.containsKey(phase + String("_HIGH")) ? eventIds[phase + String("_HIGH")] : eventId;
  }

  if (value > upperCritical) {
    publishUpperCriticalBreach1(String(upperCritical), String(value), phase, "Critical", readings, dt, eventId);
    flag = true;
  }

  return flag;
}

void publishBreach(String thresholdLower, String thresholdUpper, String readValue, String paramBreached, String severity, String readings, const char *dt, int eventId) {
  // if (severity.equals("Critical") && !paramBreached.equals("WATTS")) {
  //   Serial.println("*********** Turning OFF Motor due to " + paramBreached + " mismatch***********");
  //   Serial0.println("RL2OFF");
  //   delay(500);
  //   Serial0.println("RL1ON");
  //   Serial.println("RL1ON");
  //   delay(500);
  //   enableAutoRestart();
  // }
  DynamicJsonDocument doc(1024);
  doc["id"] = MAC_ID;
  doc["info"] = "Threshold has been breached";
  doc["severity"] = severity;
  doc["parameter"] = paramBreached;
  doc["thresholdLower"] = thresholdLower;
  doc["thresholdUpper"] = thresholdUpper;
  doc["readingFromMeter"] = readValue;
  doc["eventId"] = String(eventId);
  doc["deviceType"] = deviceTypes[deviceType];
  String alert;
  serializeJson(doc, alert);
  // storeDataIntoparticularDatedSdCardFile(dt, alert);
  if (securedMode) {
    alert = initiateEncryption(alert);
    readings = initiateEncryption(readings);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", alert.c_str());
    // Serial.println(++count);
    // mqtt.publish("pqm", readings.c_str());
  }
}

void publishBreach1(String thresholdUpper, String readValue, String paramBreached, String severity, String readings, const char *dt, int eventId) {
  // if (severity.equals("Critical")) {
  //   Serial.println("*********** Turning OFF Motor due to " + paramBreached + " mismatch***********");
  //   Serial0.println("RL2OFF");
  //   delay(500);
  //   Serial0.println("RL1ON");
  //   Serial.println("RL1ON");
  //   delay(500);
  //   enableAutoRestart();
  // }
  DynamicJsonDocument doc(1024);
  doc["id"] = MAC_ID;
  doc["info"] = "Threshold has been breached";
  doc["severity"] = severity;
  doc["parameter"] = paramBreached;
  doc["thresholdUpper"] = thresholdUpper;
  doc["readingFromMeter"] = readValue;
  doc["eventId"] = String(eventId);
  doc["deviceType"] = deviceTypes[deviceType];
  String alert;
  serializeJson(doc, alert);
  // storeDataIntoparticularDatedSdCardFile(dt, alert);
  if (securedMode) {
    alert = initiateEncryption(alert);
    readings = initiateEncryption(readings);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", alert.c_str());
    // Serial.println(++count);
    // mqtt.publish("pqm", readings.c_str());
  }
}

void publishLowerBreach1(String thresholdLower, String readValue, String paramBreached, String severity, String readings, const char *dt) {
  // if (severity.equals("Critical")) {
  //   Serial.println("*********** Turning OFF Motor due to " + paramBreached + " mismatch***********");
  //   Serial0.println("RL2OFF");
  //   delay(500);
  //   Serial0.println("RL1ON");
  //   Serial.println("RL1ON");
  //   delay(500);
  //   enableAutoRestart();
  // }
  DynamicJsonDocument doc(1024);
  doc["id"] = MAC_ID;
  doc["info"] = "Threshold has been breached";
  doc["severity"] = severity;
  doc["parameter"] = paramBreached;
  doc["thresholdLower"] = thresholdLower;
  doc["readingFromMeter"] = readValue;
  doc["deviceType"] = deviceTypes[deviceType];
  String alert;
  serializeJson(doc, alert);
  // storeDataIntoparticularDatedSdCardFile(dt, alert);
  if (securedMode) {
    alert = initiateEncryption(alert);
    readings = initiateEncryption(readings);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", alert.c_str());
    // Serial.println(++count);
    // mqtt.publish("pqm", readings.c_str());
  }
}

void publishUpperCriticalBreach1(String thresholdUpper, String readValue, String paramBreached, String severity, String readings, const char *dt, int eventId) {
  // if (severity.equals("Critical")) {
  //   Serial.println("*********** Turning OFF Motor due to " + paramBreached + " mismatch***********");
  //   Serial0.println("RL2OFF");
  //   delay(500);
  //   Serial0.println("RL1ON");
  //   Serial.println("RL1ON");
  //   delay(500);
  //   enableAutoRestart();
  // }
  DynamicJsonDocument doc(1024);
  doc["id"] = MAC_ID;
  doc["info"] = "Threshold has been breached";
  doc["severity"] = severity;
  doc["parameter"] = paramBreached;
  doc["thresholdUpper"] = thresholdUpper;
  doc["readingFromMeter"] = readValue;
  doc["eventId"] = String(eventId);
  doc["deviceType"] = deviceTypes[deviceType];
  String alert;
  serializeJson(doc, alert);
  // storeDataIntoparticularDatedSdCardFile(dt, alert);
  if (securedMode) {
    alert = initiateEncryption(alert);
    readings = initiateEncryption(readings);
  }
  if (mqtt.connected()) {
    mqtt.publish("pqm", alert.c_str());
    // Serial.println(++count);
    // mqtt.publish("pqm", readings.c_str());
  }
}

void enableAutoRestart() {
  offByThreshold = true;
  motorOffTime = millis();  // remember when it was turned OFF
}

void disableAutoRestart() {
  offByThreshold = false;
}

void loopMQTT() {
  if (mqtt.connected()) {
    mqtt.loop();
    // Serial.println("===== MQTT CONNECTED =====");
  } else {
    Serial.println("===== MQTT DISCONNECTED =====");
  }
}

void parseData(String input) {
  std::string data = input.c_str();                                // Input data
  std::vector<std::pair<std::string, std::string>> keyValuePairs;  // Dynamic storage

  size_t start = 0, end;
  while ((end = data.find(',', start)) != std::string::npos) {
    std::string pair = data.substr(start, end - start);
    size_t separator = pair.find(':');
    if (separator != std::string::npos) {
      keyValuePairs.emplace_back(pair.substr(0, separator), pair.substr(separator + 1));
    }
    start = end + 1;
  }

  // Handle the last key-value pair
  size_t separator = data.find(':', start);
  if (separator != std::string::npos) {
    keyValuePairs.emplace_back(data.substr(start, separator - start), data.substr(separator + 1));
  }

  // Print parsed key-value pairs
  Serial.println("Parsed Key-Value Pairs:");
  for (const auto &pair : keyValuePairs) {
    String key = String(pair.first.c_str());
    key.trim();
    String value = String(pair.second.c_str());
    value.trim();
    Serial.println(key + " : " + value);
    initiateEEPROMwrite(key, value);
  }
}

void initiateEEPROMwrite(String key, String value) {
  String msgKey = String(key);
  if (!msgKey.equals("VRL") && !msgKey.equals("VRU") && !msgKey.equals("VYL") && !msgKey.equals("VYU") && !msgKey.equals("VBL") && !msgKey.equals("VBU") && !msgKey.equals("VRYL") && !msgKey.equals("VRYU") && !msgKey.equals("VYBL") && !msgKey.equals("VYBU") && !msgKey.equals("VBRL") && !msgKey.equals("VBRU") && !msgKey.equals("WATTSL") && !msgKey.equals("WATTSU") && !msgKey.equals("IRU") && !msgKey.equals("IYU") && !msgKey.equals("IBU") && !msgKey.equals("VRLW") && !msgKey.equals("VRUW") && !msgKey.equals("VYLW") && !msgKey.equals("VYUW") && !msgKey.equals("VBLW") && !msgKey.equals("VBUW") && !msgKey.equals("VRYLW") && !msgKey.equals("VRYUW") && !msgKey.equals("VYBLW") && !msgKey.equals("VYBUW") && !msgKey.equals("VBRLW") && !msgKey.equals("VBRUW") && !msgKey.equals("WATTSLW") && !msgKey.equals("WATTSUW") && !msgKey.equals("IRUW") && !msgKey.equals("IYUW") && !msgKey.equals("IBUW") && !msgKey.equals("INTERVAL") && !msgKey.equals("SAMPRATE") && !msgKey.equals("V1PU") && !msgKey.equals("V1PL") && !msgKey.equals("V1PUW") && !msgKey.equals("V1PLW") && !msgKey.equals("V2PU") && !msgKey.equals("V2PL") && !msgKey.equals("V2PUW") && !msgKey.equals("V2PLW") && !msgKey.equals("IU") && !msgKey.equals("IUW") && !msgKey.equals("PFL") && !msgKey.equals("PFLW") && !msgKey.equals("FRQU") && !msgKey.equals("FRQL") && !msgKey.equals("FRQUW") && !msgKey.equals("FRQLW") && !msgKey.equals("VTHDU") && !msgKey.equals("ITHDU") && !msgKey.equals("VDPU") && !msgKey.equals("IDPU")) {
    Serial.println("ERROR.");
    return;
  }

  int from = preferences.getInt(String(String(key) + "_FROM").c_str(), 0);  // Default to 0 if not found
  int to = preferences.getInt(String(String(key) + "_TO").c_str(), 0);      // Default to 0 if not found
  Serial.println("From : " + String(from) + ", " + "To : " + String(to));
  preferences.putString(String(key).c_str(), String(value));
  writeEEPROM(from, to, String(value));
}

void initiateEEPROMread() {
  Serial.println("************ Threshold-Data ************");
  DynamicJsonDocument thresholds(1024);
  thresholds["id"] = MAC_ID;
  thresholds["info"] = "Thresholds";
  // thresholds["VRL"] = String(preferences.getString("VRL"));
  // thresholds["VRU"] = String(preferences.getString("VRU"));
  // thresholds["VYL"] = String(preferences.getString("VYL"));
  // thresholds["VYU"] = String(preferences.getString("VYU"));
  // thresholds["VBL"] = String(preferences.getString("VBL"));
  // thresholds["VBU"] = String(preferences.getString("VBU"));
  // thresholds["VRYL"] = String(preferences.getString("VRYL"));
  // thresholds["VRYU"] = String(preferences.getString("VRYU"));
  // thresholds["VYBL"] = String(preferences.getString("VYBL"));
  // thresholds["VYBU"] = String(preferences.getString("VYBU"));
  // thresholds["VBRL"] = String(preferences.getString("VBRL"));
  // thresholds["VBRU"] = String(preferences.getString("VBRU"));
  thresholds["WATTSL"] = String(preferences.getString("WATTSL"));
  thresholds["WATTSU"] = String(preferences.getString("WATTSU"));
  // thresholds["IRU"] = String(preferences.getString("IRU"));
  // thresholds["IYU"] = String(preferences.getString("IYU"));
  // thresholds["IBU"] = String(preferences.getString("IBU"));
  // thresholds["VRLW"] = String(preferences.getString("VRLW"));
  // thresholds["VRUW"] = String(preferences.getString("VRUW"));
  // thresholds["VYLW"] = String(preferences.getString("VYLW"));
  // thresholds["VYUW"] = String(preferences.getString("VYUW"));
  // thresholds["VBLW"] = String(preferences.getString("VBLW"));
  // thresholds["VBUW"] = String(preferences.getString("VBUW"));
  // thresholds["VRYLW"] = String(preferences.getString("VRYLW"));
  // thresholds["VRYUW"] = String(preferences.getString("VRYUW"));
  // thresholds["VYBLW"] = String(preferences.getString("VYBLW"));
  // thresholds["VYBUW"] = String(preferences.getString("VYBUW"));
  // thresholds["VBRLW"] = String(preferences.getString("VBRLW"));
  // thresholds["VBRUW"] = String(preferences.getString("VBRUW"));
  thresholds["WATTSLW"] = String(preferences.getString("WATTSLW"));
  thresholds["WATTSUW"] = String(preferences.getString("WATTSUW"));
  // thresholds["IRUW"] = String(preferences.getString("IRUW"));
  // thresholds["IYUW"] = String(preferences.getString("IYUW"));
  // thresholds["IBUW"] = String(preferences.getString("IBUW"));

  thresholds["V1PL"] = String(preferences.getString("V1PL"));
  thresholds["V1PU"] = String(preferences.getString("V1PU"));
  thresholds["V1PLW"] = String(preferences.getString("V1PLW"));
  thresholds["V1PUW"] = String(preferences.getString("V1PUW"));
  thresholds["V2PL"] = String(preferences.getString("V2PL"));
  thresholds["V2PU"] = String(preferences.getString("V2PU"));
  thresholds["V2PLW"] = String(preferences.getString("V2PLW"));
  thresholds["V2PUW"] = String(preferences.getString("V2PUW"));
  thresholds["IU"] = String(preferences.getString("IU"));
  thresholds["IUW"] = String(preferences.getString("IUW"));

  thresholds["PFL"] = String(preferences.getString("PFL"));
  thresholds["PFLW"] = String(preferences.getString("PFLW"));
  thresholds["FRQU"] = String(preferences.getString("FRQU"));
  thresholds["FRQL"] = String(preferences.getString("FRQL"));
  thresholds["FRQUW"] = String(preferences.getString("FRQUW"));
  thresholds["FRQLW"] = String(preferences.getString("FRQLW"));

  thresholds["VTHDU"] = String(preferences.getString("VTHDU"));
  thresholds["ITHDU"] = String(preferences.getString("ITHDU"));

  thresholds["VDPU"] = String(preferences.getString("VDPU"));
  thresholds["IDPU"] = String(preferences.getString("IDPU"));
  thresholds["deviceType"] = deviceTypes[deviceType];

  Serial.println("*****************************************");
  String jsonString;
  serializeJson(thresholds, jsonString);
  // Publish the data
  if (mqtt.connected()) {
    Serial.println("Published Thresholds: " + String(jsonString));
    String thresholdData = String(jsonString);
    if (securedMode) {
      //thresholdData = initiateEncryption(thresholdData);
    }
    if (mqtt.publish("pqm", thresholdData.c_str())) {
      Serial.println("Thresholds published successfully.");
    } else {
      Serial.println("Error publishing Thresholds.");
    }
  }
}

// Function to write new data to EEPROM within a specified range
void writeEEPROM(int startAddress, int endAddress, String data) {
  int dataLength = data.length();

  if (dataLength > (endAddress - startAddress + 1)) {
    Serial.println("Error: Data is too long for the specified EEPROM range.");
    return;
  }

  // First, clear the EEPROM range
  clearEEPROMAt(startAddress, endAddress);

  // Write new data
  for (int i = 0; i < dataLength; i++) {
    EEPROM.write(startAddress + i, data[i]);
  }
  EEPROM.write(startAddress + dataLength, '\0');  // Null-terminate the string
  EEPROM.commit();                                // Save changes
  Serial.println("Data written to EEPROM successfully.");
}

// Function to clear EEPROM data from startAddress to endAddress
void clearEEPROMAt(int startAddress, int endAddress) {
  for (int i = startAddress; i <= endAddress; i++) {
    EEPROM.write(i, 0xFF);  // Write 0xFF to indicate erased state
  }
  EEPROM.commit();  // Save changes
  Serial.println("EEPROM cleared from " + String(startAddress) + " to " + String(endAddress) + ".");
}

// Function to read and print stored data from EEPROM
String readEEPROM(int startAddress, int endAddress) {
  String data = "";
  for (int i = startAddress; i <= endAddress; i++) {
    char value = EEPROM.read(i);
    if (value == '\0' || value == 0xFF) {  // Stop if null or empty data
      break;
    }
    // Serial.print(value);
    data += String(value);
  }
  return data;
}

// Check for update by comparing versions
bool checkForUpdate() {
  Serial.println("Start");
  HTTPClient http;
  http.begin(version_url);
  // http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);  // Enable following redirects
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) {
    String newLocation = http.header("Location");
    Serial.println("Redirected to: " + newLocation);
    http.end();
    http.begin(newLocation);  // Follow the new URL
    httpCode = http.GET();
  }
  Serial.println("httpCode: " + String(httpCode));
  if (httpCode == HTTP_CODE_OK) {
    String latestVersion = http.getString();
    latestVersion.trim();  // Remove whitespace or newline characters
    Serial.println("Latest Version: " + latestVersion);
    Serial.println("Current Version: " + currentVersion);
    return (latestVersion != currentVersion);
  } else {
    Serial.println("Failed to check for updates. HTTP Code: " + String(httpCode));
    return false;
  }
}

// Perform OTA update
void performOTA() {
  HTTPClient http;
  http.begin(firmware_url);
  // http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);  // Enable following redirects
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) {
    String newLocation = http.header("Location");
    Serial.println("Redirected to: " + newLocation);
    http.end();
    http.begin(newLocation);  // Follow the new URL
    httpCode = http.GET();
  }
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin) {
      Serial.println("Starting OTA...");
      WiFiClient &client = http.getStream();
      size_t written = Update.writeStream(client);

      if (written == contentLength && Update.end()) {
        Serial.println("OTA Update Success. Restarting...");
        ESP.restart();
      } else {
        Serial.println("OTA Update Failed. Error #: " + String(Update.getError()));
      }
    } else {
      Serial.println("Not enough space for OTA update.");
    }
  } else {
    Serial.println("Failed to fetch firmware. HTTP Code: " + String(httpCode));
  }

  http.end();
}

// Check version from server
void checkVersion() {
  Serial.println("\nChecking for updates...");
  Serial.print("Current Version: ");
  Serial.println(currentVersion);

  Serial.print("URL: http://");
  Serial.print(server);
  Serial.println(versionResource);

  http.beginRequest();
  http.get(versionResource);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Status code: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    String latestVersion = "";
    while (http.available()) {
      latestVersion = http.readStringUntil('\n');
      latestVersion.trim();  // Remove any whitespace or newlines
    }

    Serial.print("Latest Version: ");
    Serial.println(latestVersion);

    if (latestVersion != currentVersion) {
      Serial.println("Update available!");
      performUpdate();  // Start the update process
    } else {
      Serial.println("Already running the latest version.");
    }
  } else {
    Serial.print("Version check failed with status code: ");
    Serial.println(statusCode);

    String errorResponse = "";
    while (http.available()) {
      errorResponse += http.readString();
    }

    if (errorResponse.length() > 0) {
      Serial.println("Error response: ");
      Serial.println(errorResponse);
    }
  }
}

// Perform OTA update
void performUpdate() {
  Serial.println("\nStarting firmware update...");
  Serial.print("URL: http://");
  Serial.print(server);
  Serial.println(firmwareResource);

  // Set longer timeout for large file download
  http.setTimeout(30000);  // 30 second timeout

  http.beginRequest();
  http.get(firmwareResource);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Status code: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    int contentLength = http.contentLength();
    Serial.print("Content length: ");
    Serial.println(contentLength);

    if (contentLength <= 0) {
      Serial.println("Invalid content length");
      return;
    }

    if (!Update.begin(contentLength)) {
      Serial.println("Not enough space for update");
      Serial.println("Update error: " + String(Update.getError()));
      return;
    }

    Serial.println("Starting update...");

    // Use a larger buffer for faster updates
    const size_t bufferSize = 1024;  // 1KB buffer
    uint8_t buffer[bufferSize];
    size_t written = 0;
    size_t lastProgress = 0;

    while (written < contentLength) {
      // Calculate and show progress
      int progress = (written * 100) / contentLength;
      if (progress != lastProgress) {
        Serial.print("Progress: ");
        Serial.print(progress);
        Serial.println("%");
        lastProgress = progress;
      }

      // Read a chunk of data
      size_t bytesToRead = min(bufferSize, (size_t)(contentLength - written));
      size_t bytesRead = http.readBytes((char *)buffer, bytesToRead);

      if (bytesRead > 0) {
        // Write the chunk to update
        size_t bytesWritten = Update.write(buffer, bytesRead);
        if (bytesWritten == bytesRead) {
          written += bytesWritten;
        } else {
          Serial.println("Error writing to update");
          Update.abort();
          return;
        }
      } else {
        delay(100);  // Wait a bit if no data
      }
    }

    if (written == contentLength && Update.end()) {
      Serial.println("Update successful! Restarting...");
      ESP.restart();
    } else {
      Serial.println("Update failed!");
      Serial.print("Error: ");
      Serial.println(Update.getError());
      Serial.print("Written: ");
      Serial.print(written);
      Serial.print(" of ");
      Serial.println(contentLength);
    }
  } else {
    Serial.print("Failed to download firmware. Status code: ");
    Serial.println(statusCode);
  }
}

/**********************************************  SD-Card Functions ***************************************************/
// Function to check if SD card is present and accessible
bool isSDCardAccessible() {
  if (SD.cardType() == CARD_NONE) {
    Serial.println("SD Card not detected!");
    return false;
  }
  return true;
}

// Function to ensure directory and file exist
bool ensureFileExists(fs::FS &fs, const char *path) {
  // Extract directory path from file path
  String dirPath = String(path);
  int lastSlash = dirPath.lastIndexOf('/');
  if (lastSlash > 0) {
    dirPath = dirPath.substring(0, lastSlash);

    // Check if directory exists
    if (!fs.exists(dirPath.c_str())) {
      Serial.println("Directory does not exist, creating directory");
      if (!fs.mkdir(dirPath.c_str())) {
        Serial.println("Failed to create directory");
        return false;
      }
    }
  }

  // Check if file exists
  if (!fs.exists(path)) {
    Serial.println("File does not exist, creating new file");
    // Create the file first
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to create new file");
      return false;
    }
    file.close();
  }
  return true;
}

// Function to safely append data with error handling
bool safeAppendFile(fs::FS &fs, const char *path, const char *message) {
  if (!isSDCardAccessible()) {
    Serial.println("Cannot append: SD Card not accessible");
    return false;
  }
  // Ensure directory and file exist
  if (!ensureFileExists(fs, path)) {
    return false;
  }

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to write Data into SD-Card");
    return false;
  }

  if (file.print("\n")) {
    if (file.print(message)) {
      Serial.println("✔");
      file.close();
      return true;
    } else {
      Serial.println("Append failed");
      file.close();
      return false;
    }
  } else {
    Serial.println("Failed to add newline");
    file.close();
    return false;
  }
}

// Initialize SD Card
bool initSDCard() {
  Serial.println("Initializing SD Card...");

  // Initialize SD Card
  spi2.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spi2, 80000000)) {
    Serial.println("Card Mount Failed");
    return false;
  }

  Serial.println("SD card initialized.");
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  return true;
}

// SD Card Functions
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);

  // Check if directory already exists
  if (fs.exists(path)) {
    Serial.println("Directory already exists");
    return;
  }

  if (fs.mkdir(path)) {
    Serial.println("Dir created");
    // Write a test file
    // writeFile(SD, filePath, "--------------------- < START > ---------------------");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println();
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  // Check if file exists
  if (fs.exists(path)) {
    Serial.println("File already exists, will be overwritten");
  } else {
    Serial.println("Creating new file");
  }

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }

  // Add newline, timestamp, and message
  if (file.print("\n")) {
    if (file.printf("%s", message)) {
      Serial.println("Message appended");
    } else {
      Serial.println("Append failed");
    }
  } else {
    Serial.println("Failed to add newline");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

bool getFileNameFromDateTime(const char *dateTime, char *fileName, size_t fileNameSize) {
  int y, m, d, hh, mm, ss;

  // Expected format: YYYY/MM/DD HH:MM:SS
  if (sscanf(dateTime, "%d/%d/%d %d:%d:%d", &y, &m, &d, &hh, &mm, &ss) != 6) {
    return false;
  }

  snprintf(fileName, fileNameSize, "/CCMS/Log/%04d-%02d-%02d.txt", y, m, d);
  return true;
}

bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

bool parseDateTime(const char *dateTime, int &y, int &m, int &d) {
  // Expected format: YYYY/MM/DD HH:MM:SS
  return sscanf(dateTime, "%4d/%2d/%2d %*d:%*d:%*d", &y, &m, &d) == 3;
}

long dateToDays(int y, int m, int d) {
  static const int daysBeforeMonth[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

  long days = 0;

  // Days from previous years
  for (int year = 1970; year < y; year++) {
    days += isLeapYear(year) ? 366 : 365;
  }

  // Days from previous months in current year
  days += daysBeforeMonth[m - 1];

  // Add leap day if past Feb in leap year
  if (m > 2 && isLeapYear(y)) {
    days += 1;
  }

  // Days in current month
  days += d - 1;

  return days;
}

bool extractDateFromFileName(const char *fileName, int &y, int &m, int &d) {
  // Expected format: YYYY-MM-DD.txt
  if (strlen(fileName) != 14) return false;

  return sscanf(fileName, "%4d-%2d-%2d.txt", &y, &m, &d) == 3;
}

void deleteOldLogFiles(fs::FS &fs, const char *dirPath, const char *currentDateTime) {
  int cy, cm, cd;

  if (!parseDateTime(currentDateTime, cy, cm, cd)) {
    Serial.println("Invalid current datetime format");
    return;
  }

  long currentDays = dateToDays(cy, cm, cd);

  File dir = fs.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    Serial.println("Failed to open log directory");
    return;
  }

  File file;
  while ((file = dir.openNextFile())) {

    if (!file.isDirectory()) {
      const char *name = file.name();

      int fy, fm, fd;
      if (extractDateFromFileName(name, fy, fm, fd)) {

        long fileDays = dateToDays(fy, fm, fd);
        long diffDays = currentDays - fileDays;

        Serial.printf("Checking %s → %ld days old\n", name, diffDays);

        if (diffDays > 7) {
          Serial.printf("Deleting old file: %s\n", name);
          char fullPath[64];
          snprintf(fullPath, sizeof(fullPath), "%s/%s", myDirPath, name);
          fs.remove(fullPath);
        }
      }
    }

    file.close();
  }

  dir.close();
}
/**********************************************  ----------------- ***************************************************/
/**********************************************  < SMS-Functions > ***************************************************/
void sendSMS(char *messageText) {
  Serial.println("Sending SMS...");

  // Set SMS text mode
  SerialAT.println("AT+CMGF=1");
  delay(500);

  // Send SMS command with phone number
  Serial.print("Sending to: ");
  Serial.println(phoneNumber);
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(phoneNumber);
  SerialAT.println("\"");
  delay(500);

  // Check for ">" prompt
  if (SerialAT.find(">")) {
    // Send message content
    SerialAT.print(messageText);
    SerialAT.write(26);  // Ctrl+Z to send message
    // Serial.println("SMS sent!");

    // Wait for response
    delay(5000);

    // Read and print any response
    while (SerialAT.available()) {
      Serial.write(SerialAT.read());
    }

  } else {
    Serial.println("Failed to get prompt");
  }
}

// Alternative way to send SMS using TinyGSM library functions
void sendSMSUsingTinyGSM(char *messageText) {
  Serial.println("Sending SMS using TinyGSM...");

  // Send SMS
  if (modem.sendSMS(phoneNumber, messageText)) {
    Serial.println("SMS sent successfully");
    blinkLED(1);
  } else {
    Serial.println("SMS failed to send");
    blinkLED(3);
  }
}
/**********************************************  ----------------- ***************************************************/
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(statusLED, LOW);
    delay(200);
    digitalWrite(statusLED, HIGH);
    delay(200);
  }
}
/**********************************************  ----------------- ***************************************************/
/* -------------------------- Encryption - Decryption Methods  -----------------------*/

String initiateEncryption(String plainText) {
  return encryptString(plainText, hashedKey, iv);
}

String initiateDecryption(String encryptedText) {
  return decryptString(encryptedText, hashedKey, ivDecrypt);
}

void hashKey(const char *key, byte *hashedKey, size_t keyLength) {
  SHA256 sha256;
  sha256.update((const uint8_t *)key, keyLength);  // Corrected method
  sha256.finalize(hashedKey, 32);                  // Generate a 32-byte key
}

// Function to pad the input string to a multiple of 16 bytes
void padString(String &input) {
  int paddingLength = 16 - (input.length() % 16);
  for (int i = 0; i < paddingLength; i++) {
    input += (char)paddingLength;
  }
}

// Function to remove padding from the decrypted string
void unpadString(String &input) {
  int paddingLength = input[input.length() - 1];
  if (paddingLength > 0 && paddingLength <= 16) {
    input.remove(input.length() - paddingLength);
  }
}

// Function to encrypt a string using AES-256-CBC
String encryptString(const String &plainText, const byte *key, const byte *iv) {
  AES aes;
  aes.set_key(key, 32);  // Set 256-bit key

  // Pad the plaintext
  String paddedText = plainText;
  padString(paddedText);

  int paddedLength = paddedText.length();
  byte encryptedBytes[paddedLength];

  // Create a mutable copy of IV since CBC mode modifies it
  byte ivCopy[16];
  memcpy(ivCopy, iv, 16);

  // Encrypt the padded text
  aes.cbc_encrypt((byte *)paddedText.c_str(), encryptedBytes, paddedLength / 16, ivCopy);

  // Convert encrypted bytes to hex string
  String encryptedText;
  for (int i = 0; i < paddedLength; i++) {
    char hex[3];
    sprintf(hex, "%02X", encryptedBytes[i]);
    encryptedText += hex;
  }

  return encryptedText;
}

// Function to decrypt a string using AES-256-CBC
String decryptString(const String &encryptedText, const byte *key, const byte *iv) {
  AES aes;
  aes.set_key(key, 32);  // Set 256-bit key

  // Convert hex string to byte array
  int encryptedLength = encryptedText.length() / 2;
  byte encryptedBytes[encryptedLength];

  for (int i = 0; i < encryptedLength; i++) {
    char hex[3] = { encryptedText[i * 2], encryptedText[i * 2 + 1], '\0' };
    encryptedBytes[i] = (byte)strtol(hex, NULL, 16);
  }

  // Decrypt the bytes
  byte decryptedBytes[encryptedLength];

  // Create a fresh IV copy for decryption
  byte ivCopy[16];
  memcpy(ivCopy, iv, 16);

  aes.cbc_decrypt(encryptedBytes, decryptedBytes, encryptedLength / 16, ivCopy);

  // Convert decrypted bytes to string
  String decryptedText;
  for (int i = 0; i < encryptedLength; i++) {
    decryptedText += (char)decryptedBytes[i];
  }

  // Remove padding
  unpadString(decryptedText);

  return decryptedText;
}

// ---------------- GPS METHODS ---------------- //

void enableGPS() {
  Serial.println("Enabling GPS module...");
  SerialAT.println("AT+QGPS=1");
  delay(2000);
  gpsData = SerialAT.readString();
  Serial.print("GPS Enable Response: ");
  Serial.println(gpsData);
}

bool getGPSLocation() {
  latitude = "";
  longitude = "";
  Serial.println("Requesting GPS coordinates...");
  SerialAT.println("AT+QGPSLOC=0");
  delay(2000);
  gpsData = SerialAT.readString();
  Serial.print("GPS Location: ");
  Serial.println(gpsData);
  if (gpsData.indexOf("+QGPSLOC:") != -1) {
    latitude = extractLatitude(gpsData);
    longitude = extractLongitude(gpsData);
    Serial.println("Latitude: " + String(latitude));
    Serial.println("Longitude: " + String(longitude));
    return true;
  } else {
    Serial.println("Failed to read Co-ordinates");
    return false;
  }
}

String extractLatitude(String gpsData) {
  int firstComma = gpsData.indexOf(',');
  int secondComma = gpsData.indexOf(',', firstComma + 1);
  String rawLat = gpsData.substring(firstComma + 1, secondComma);
  char latDir = gpsData.charAt(secondComma + 1);

  // Convert NMEA format (ddmm.mmmm) to decimal degrees
  float deg = rawLat.substring(0, 2).toFloat();
  float minutes = rawLat.substring(2).toFloat();
  float latitude = deg + (minutes / 60.0);

  if (latDir == 'S') latitude = -latitude;
  return String(latitude, 6);
}

String extractLongitude(String gpsData) {
  int secondComma = gpsData.indexOf(',', gpsData.indexOf(',') + 1);
  int thirdComma = gpsData.indexOf(',', secondComma + 1);
  String rawLon = gpsData.substring(secondComma + 1, thirdComma);
  char lonDir = gpsData.charAt(thirdComma + 1);

  // Convert NMEA format (dddmm.mmmm) to decimal degrees
  float deg = rawLon.substring(0, 3).toFloat();
  float minutes = rawLon.substring(3).toFloat();
  float longitude = deg + (minutes / 60.0);

  if (lonDir == 'W') longitude = -longitude;
  return String(longitude, 6);
}

/* ---------------------------- Memory Related Functions --------------------------------*/

void printMemoryInfo() {
  Serial.println("\n====== ESP32 MEMORY INFO ======");

  // -------- RAM (HEAP) --------
  uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
  uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  uint32_t minHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

  Serial.print("Total Heap (RAM): ");
  Serial.print(totalHeap / 1024);
  Serial.println(" KB");

  Serial.print("Free Heap (RAM): ");
  Serial.print(freeHeap / 1024);
  Serial.println(" KB");

  Serial.print("Minimum Free Heap Ever: ");
  Serial.print(minHeap / 1024);
  Serial.println(" KB");

  // -------- INTERNAL RAM DETAILS --------
  Serial.print("Free Internal RAM: ");
  Serial.print(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
  Serial.println(" KB");

  // -------- PSRAM (if available) --------
  if (psramFound()) {
    Serial.print("Total PSRAM: ");
    Serial.print(ESP.getPsramSize() / 1024);
    Serial.println(" KB");

    Serial.print("Free PSRAM: ");
    Serial.print(ESP.getFreePsram() / 1024);
    Serial.println(" KB");
  } else {
    Serial.println("PSRAM: Not available");
  }

  // -------- FLASH MEMORY --------
  Serial.print("Flash Chip Size: ");
  Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println(" MB");

  Serial.print("Sketch Size: ");
  Serial.print(ESP.getSketchSize() / 1024);
  Serial.println(" KB");

  Serial.print("Free Sketch Space (OTA): ");
  Serial.print(ESP.getFreeSketchSpace() / 1024);
  Serial.println(" KB");

  Serial.println("================================\n");
}