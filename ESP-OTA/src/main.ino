#include "driver/rtc_io.h"
#include <HardwareSerial.h>
#include <Preferences.h>


#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ===== OTA LIB =====
#include <ESP32S3_SIM7670_OTA.h>
#define CURRENT_FIRMWARE_VERSION "1.0.0"

// ===================== Pin Definitions =====================





// LoRa SX1278
#define PIN_LORA_SCK   13
#define PIN_LORA_MISO  14
#define PIN_LORA_MOSI  15
#define PIN_LORA_CS    10
#define PIN_LORA_RST   11
#define PIN_LORA_DIO0  12
#define LORA_FREQUENCY 433E6



// GSM Serial (SIM7670X)
#define GSM_RX 17
#define GSM_TX 18

// Sleep Timing
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 600

// ===================== Object Definitions =====================

ESP32S3_SIM7670_OTA ota(
    SerialAt,
    GSM_RX, GSM_TX,
    "network provider.com",
    "",
    "",
    "apisite.com",
    80
);

Preferences prefs;

// ===================== Globals =====================
String tId;
sensors_event_t a, g, temp;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR volatile unsigned long rainCount = 0;
RTC_DATA_ATTR volatile unsigned long motionCount = 0;

bool rainWake = false;
bool loRaAvailable = false;   //  ADDED

// ===================== Function Declarations =====================
String sendGSMData(String url);
bool extractOTA(String payload, String &version, String &url);
void print_wakeup_reason();
void readGSM();


// ===================== ISR =====================


// ===================== Setup =====================
void setup() {

  esp_sleep_enable_ext0_wakeup((gpio_num_t)INT_PIN, 1);
  rtc_gpio_pullup_dis((gpio_num_t)PIN);
  rtc_gpio_pulldown_en((gpio_num_tPIN);

  Serial.begin(115200);
  SerialAt.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);

  // ===== NVS Triplet ID =====
  prefs.begin("device", true);   // READ-ONLY MODE
  tId = prefs.getString("tId", "");
  prefs.end();
  Serial.println("Triplet ID (from flash): " + tId);

  pinMode( INPUT);
  pinMode( INPUT);
  pinMode(INPUT_PULLUP);
  pinMode( INPUT);

  attachInterrupt(digitalPinToInterrupt( CHANGE);
  attachInterrupt(digitalPinToInterrupt( RISING);

  ++bootCount;



  // ===================== LoRa INIT (ADDED ONLY) =====================
  Serial.println("Initializing LoRa...");
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

  if (LoRa.begin(LORA_FREQUENCY)) {
    loRaAvailable = true;
    Serial.println("✅ LoRa init successful");
  } else {
    loRaAvailable = false;
    Serial.println("⚠️ LoRa not detected");
  }
  // ===============================================================

  // ===== OTA INIT =====
  ota.begin();
  ota.setVersion(CURRENT_FIRMWARE_VERSION);


  print_wakeup_reason();

  // ===================== LoRa SEND ON EXT0 (ADDED ONLY) =====================
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 && loRaAvailable) {
    Serial.println("📡 Sending LoRa packet (motion wake)");
    LoRa.beginPacket();
    LoRa.print("Motion detected at " + tId);
    LoRa.endPacket();
  }
  // ========================================================================

 

  String url =
    "http://yourapiaddress"
    "&triplet=" + tId +
    

  String response = sendGSMData(url);
  String otaVersion;
  String otaUrl;

  // ===== OTA DECISION (UNCHANGED) =====
  if (!extractOTA(response, otaVersion, otaUrl)) {
  Serial.println("OTA PARSE FAILED");
  return;
}

Serial.println(" OTA PARSE OK");
Serial.println("Version : " + otaVersion);
Serial.println("URL (raw): " + otaUrl);

//  FIX: unescape JSON URL (\/ → /)
otaUrl.replace("\\/", "/");

Serial.println("URL (fixed): " + otaUrl);

// Version decision
if (otaVersion == CURRENT_FIRMWARE_VERSION) {
  Serial.println("Firmware already up to date");
  return;
}

  Serial.println("STARTING OTA FLASH");
  ota.performOTA(otaUrl);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(1000);
  esp_deep_sleep_start();
}

void loop() {}

// ===================== OTA JSON Parser =====================
bool extractOTA(String payload, String &version, String &url) {
  int v = payload.indexOf("\"ota_version\"");
  int u = payload.indexOf("\"ota_url\"");
  if (v == -1 || u == -1) return false;

  int vs = payload.indexOf("\"", v + 13) + 1;
  int ve = payload.indexOf("\"", vs);
  version = payload.substring(vs, ve);

  int us = payload.indexOf("\"", u + 9) + 1;
  int ue = payload.indexOf("\"", us);
  url = payload.substring(us, ue);

  return true;
}

// ===================== GSM =====================
String sendGSMData(String url) {

  String response = "";
  String urcLine = "";

  // --- Basic init ---
  SerialAt.println("AT");
  delay(200);

  SerialAt.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"");
  delay(200);

  SerialAt.println("AT+CGACT=1,1");
  delay(1000);

  SerialAt.println("AT+HTTPINIT");
  delay(1000);

  SerialAt.print("AT+HTTPPARA=\"URL\",\"");
  SerialAt.print(url);
  SerialAt.println("\"");
  delay(500);

  // --- Start HTTP GET ---
  SerialAt.println("AT+HTTPACTION=0");

  unsigned long start = millis();
  int contentLength = -1;

  // --- WAIT FOR FULL +HTTPACTION LINE ---
  while (millis() - start < 20000) {

    while (SerialAt.available()) {
      char c = SerialAt.read();

      // accumulate line
      if (c == '\n') {
        urcLine.trim();

        // Debug print
        if (urcLine.length()) {
          Serial.println(urcLine);
        }

        // Check for HTTPACTION line
        if (urcLine.startsWith("+HTTPACTION:")) {

          // Format: +HTTPACTION: 0,200,450
          int lastComma = urcLine.lastIndexOf(',');
          if (lastComma != -1) {
            contentLength = urcLine.substring(lastComma + 1).toInt();
          }
          goto READ_BODY;
        }

        urcLine = ""; // reset for next line
      }
      else {
        urcLine += c;
      }
    }
  }

READ_BODY:

  if (contentLength <= 0) {
    Serial.println("❌ HTTPACTION failed or empty response");
    SerialAt.println("AT+HTTPTERM");
    return "";
  }

  // --- READ BODY ---
  SerialAt.print("AT+HTTPREAD=0,");
  SerialAt.println(contentLength);

  start = millis();
  while (millis() - start < 20000) {
    while (SerialAt.available()) {
      char c = SerialAt.read();
      response += c;
    }
    if (response.indexOf("}") != -1) break;
  }

  SerialAt.println("AT+HTTPTERM");
  delay(500);

  // --- Extract JSON only ---
  int js = response.indexOf("{");
  int je = response.lastIndexOf("}");
  if (js != -1 && je != -1 && je > js) {
    response = response.substring(js, je + 1);
  } else {
    response = "";
  }

  Serial.println("===== RAW RESPONSE =====");
  Serial.println(response);

  return response;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("🔔 ");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("🌧️");
      rainWake = true;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("⏰ Wakeup");
      break;
    default:
      Serial.printf("Wakeup cause: %d\n", reason);
      break;
  }
}

void readGSM() {
  delay(100);
  while (SerialAt.available()) Serial.write(SerialAt.read());
}

// ===================== Sensors (UNCHANGED) =====================

String getGNSSLocation() {
  SerialAt.println("AT+CGNSINF");
  delay(1000);
  String response = "";
  while (SerialAt.available()) response += (char)SerialAt.read();
  int latStart = response.indexOf(",", 28);
  int lonStart = response.indexOf(",", latStart + 1);
  int lonEnd = response.indexOf(",", lonStart + 1);
  if (latStart > 0 && lonStart > 0 && lonEnd > 0) {
    String lat = response.substring(latStart + 1, lonStart);
    String lon = response.substring(lonStart + 1, lonEnd);
    lat.trim(); lon.trim();
    return lat + "," + lon;
  }
  return "0.0,0.0";
}

String getS5Reading() {
  mpu.getEvent(&a, &g, &temp);
  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float yaw   = atan2(g.gyro.z, g.gyro.x) * 180.0 / PI;

  String gnssData = getGNSSLocation();
 
}
