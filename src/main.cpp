#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiUdp.h>
#include "time.h"

// -----------------------------
// Talkie Audio Configuration
// -----------------------------
#include <Talkie.h>
#include <Vocab_Special.h>
#include <Vocab_US_Large.h>
#include <Vocab_US_Clock.h>
#include <Vocab_US_Acorn.h>

Talkie voice; // Initialize Talkie instance

// DAC pin for audio output
static const int AUDIO_PIN = 25;

/**
 * Brief voice prompt assembled from the Talkie vocabulary libraries.
 * Phrases have adjusted pauses/delays for human speech simulation.
 * Audio Output: "Action time for service"
 */
void speakDispense()
{
  // Ensure DAC pin is ready for Talkie output
  dacWrite(AUDIO_PIN, 0);
  voice.say(sp4_ACTION);
  delay(120);
  voice.say(sp3_TIME);
  delay(120);
  voice.say(sp4_FOR);
  delay(140);
  voice.say(sp4_SERVICE);
  delay(140);
  dacWrite(AUDIO_PIN, 0); // Reset DAC after speech
}

// -----------------------------
// WiFi and Cloud Configuration
// -----------------------------
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "12345678";
const char *DEVICE_ID = "DEVICE123";
const char *DEVICE_SECRET = "ABC123";
const char *BASE = "https://us-central1-medilock-f614d.cloudfunctions.net";

// -----------------------------
// LCD Configuration
// -----------------------------
#define USE_LCD 1
static const uint8_t I2C_ADDR = 0x27;
static const int I2C_SDA = 21, I2C_SCL = 22;
LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

// -----------------------------
// Button Configuration
// -----------------------------
static const int BTN_PIN = 36;  // dispense button
static const int BTN_DOWN = 34; // scroll down
static const int BTN_UP = 35;   // scroll up
static const unsigned long DEBOUNCE_MS = 200;

volatile bool btnIRQ = false;
volatile unsigned long btnIRQTime = 0;

// -----------------------------
// Pill Parameters
// -----------------------------
int freqv[6] = {0, 1, 1, 1, 1, 1};
int thresh[6] = {0, 5, 5, 5, 5, 5};
int stockv[6] = {0, 20, 20, 20, 20, 20};

enum Mode
{
  SETUP_MODE,
  MAIN_MODE
};
Mode currentMode = SETUP_MODE;

// -----------------------------
// Time and Scheduling
// -----------------------------
WiFiUDP ntpUDP;
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600; // Puerto Rico timezone
const int daylightOffset_sec = 0;
time_t nextDispense[6];       // next scheduled dispense time per pill
bool dispensing[6] = {false}; // flag to indicate alarm active

// -----------------------------
// Logging (Circular Buffer)
// -----------------------------
struct LogEntry
{
  int pill;
  time_t ts;
};
LogEntry logs[5];
int logCount = 0;
int logIndex = 0; // Track current log index

// -----------------------------
// Utility Functions
// -----------------------------
String makeUrl(const char *path)
{
  String u = BASE;
  u += path;
  u += "?deviceId=";
  u += DEVICE_ID;
  u += "&secret=";
  u += DEVICE_SECRET;
  return u;
}

bool httpsPostJson(const String &url, const String &json)
{
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url))
    return false;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  Serial.printf("[HTTP POST] %s -> %d\n", url.c_str(), code);
  http.end();
  return (code >= 200 && code < 300);
}

bool httpsGet(const String &url, String &out)
{
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url))
    return false;
  int code = http.GET();
  if (code > 0)
    out = http.getString();
  Serial.printf("[HTTP GET] %s -> %d\n", url.c_str(), code);
  http.end();
  return (code >= 200 && code < 300);
}

void lcdShow(const String &l1, const String &l2)
{
#if USE_LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(l1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(l2.substring(0, 16));
#endif
}

// -----------------------------
// Cloud Interaction
// -----------------------------
void pushTelemetry(int pill)
{
  DynamicJsonDocument d(256);
  d["pill"] = pill;
  d["freq"] = freqv[pill];
  d["threshold"] = thresh[pill];
  d["stock"] = stockv[pill];
  d["ts"] = (uint32_t)millis();
  String body;
  serializeJson(d, body);
  httpsPostJson(makeUrl("/pushTelemetry"), body);
}

void pushAlert(const char *type, int pill, const char *msg, int stockOpt = -1)
{
  DynamicJsonDocument d(256);
  d["type"] = type;
  d["pill"] = pill;
  d["msg"] = msg;
  if (stockOpt >= 0)
    d["stock"] = stockOpt;
  d["ts"] = (uint32_t)millis();
  String body;
  serializeJson(d, body);
  httpsPostJson(makeUrl("/pushAlert"), body);
}

// -----------------------------
// Threshold (Low Stock) Alerts
// -----------------------------
void maybeLowStockAlert(int pill, int prevStock, int prevThresh)
{
  if (prevStock > prevThresh && stockv[pill] <= thresh[pill])
  {
    pushAlert("pill_low", pill, "Pill stock is low", stockv[pill]);
  }
}

// -----------------------------
// Command Application
// -----------------------------
void applyCommand(int pill, const char *field, int value)
{
  int prevStock = stockv[pill];
  int prevThresh = thresh[pill];

  if (strcmp(field, "freq") == 0)
  {
    freqv[pill] = constrain(value, 1, 4);
    lcdShow("P" + String(pill) + " freq ->", String(freqv[pill]));
  }
  else if (strcmp(field, "threshold") == 0)
  {
    thresh[pill] = max(0, value);
    lcdShow("P" + String(pill) + " threshold", "-> " + String(thresh[pill]));
    maybeLowStockAlert(pill, prevStock, prevThresh);
  }
  else if (strcmp(field, "stock") == 0)
  {
    stockv[pill] = max(0, value);
    lcdShow("P" + String(pill) + " stock ->", String(stockv[pill]));
    maybeLowStockAlert(pill, prevStock, prevThresh);
  }

  pushTelemetry(pill);
}

void pullCommandsDrain()
{
  for (int i = 0; i < 5; ++i)
  {
    String resp;
    if (!httpsGet(makeUrl("/pullCommand"), resp))
      return;
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, resp))
      return;
    if (!doc["ok"] || doc["cmd"].isNull())
      return;

    int pill = doc["cmd"]["pill"] | 1;
    const char *field = doc["cmd"]["field"] | "";
    int value = doc["cmd"]["value"] | 0;

    Serial.printf("[CMD] pill=%d field=%s value=%d\n", pill, field, value);
    applyCommand(pill, field, value);
    delay(100);
  }
}

// -----------------------------
// Scheduling & Logging
// -----------------------------
void scheduleNextDispense(int pill)
{
  time_t now;
  time(&now);
  nextDispense[pill] = now + freqv[pill] * 60; // freq * 1 min demo
  dispensing[pill] = false;                    // reset alarm flag
  Serial.printf("[SCHED] Pill %d next at %ld\n", pill, nextDispense[pill]);
}

void addLog(int pill)
{
  time_t now;
  time(&now);
  logs[logCount % 5] = {pill, now};
  logCount++;
  logIndex = (logCount - 1) % 5;
}

void displayLogAtIndex(int index)
{
  if (logCount == 0)
  {
    lcdShow("No logs yet", "");
    return;
  }
  int validCount = min(logCount, 5);
  index = (index + validCount) % validCount;
  struct tm timeinfo;
  localtime_r(&logs[index].ts, &timeinfo);
  char buf[16];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  lcdShow("P" + String(logs[index].pill) + " Dispensed", buf);
}

void handleScrollButtons()
{
  static unsigned long lastPress = 0;
  if (millis() - lastPress < DEBOUNCE_MS)
    return;

  if (digitalRead(BTN_DOWN))
  {
    lastPress = millis();
    if (logCount > 0)
    {
      logIndex = (logIndex + 1) % min(logCount, 5);
      displayLogAtIndex(logIndex);
    }
    else
      lcdShow("No logs yet", "");
  }
  else if (digitalRead(BTN_UP))
  {
    lastPress = millis();
    if (logCount > 0)
    {
      int validCount = min(logCount, 5);
      logIndex = (logIndex - 1 + validCount) % validCount;
      displayLogAtIndex(logIndex);
    }
    else
      lcdShow("No logs yet", "");
  }
}

// -----------------------------
// Dispense Scheduler
// -----------------------------
void checkDispenseSchedule()
{
  time_t now;
  time(&now);

  for (int p = 1; p <= 5; ++p)
  {
    if (freqv[p] > 0 && now >= nextDispense[p])
    {
      // Trigger alarm only once per scheduled dispense
      if (!dispensing[p])
      {
        dispensing[p] = true;
        pushAlert("time_to_take", p, "Time to take your pill");
        lcdShow("Time to take", "Pill " + String(p));
        speakDispense();
      }

      // Handle button press to confirm dispense
      if (btnIRQ)
      {
        noInterrupts();
        btnIRQ = false;
        interrupts();

        if (stockv[p] > 0)
          stockv[p]--;

        pushTelemetry(p);
        addLog(p);
        pushAlert("dispense", p, "User dispensed");
        lcdShow("Dispensed P" + String(p), "Stock=" + String(stockv[p]));
        maybeLowStockAlert(p, stockv[p] + 1, thresh[p]);

        scheduleNextDispense(p);
      }
    }
  }
}

// -----------------------------
// ISR + Setup + Loop
// -----------------------------
void IRAM_ATTR btnISR()
{
  btnIRQ = true;
  btnIRQTime = millis();
}

void setup()
{
  Serial.begin(115200);
#if USE_LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcdShow("Booting...", "WiFi connect");
#endif

  pinMode(BTN_PIN, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(BTN_UP, INPUT);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnISR, RISING);

  pinMode(AUDIO_PIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK, IP=%s\n", WiFi.localIP().toString().c_str());
  lcdShow("WiFi connected", WiFi.localIP().toString());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  lcdShow("Setup Mode", "Waiting...");
}

void loop()
{
  static unsigned long tPoll = 0, tWiFiCheck = 0;

  if (currentMode == SETUP_MODE)
  {
    if (millis() - tPoll > 1000)
    {
      tPoll = millis();
      pullCommandsDrain();
    }

    if (btnIRQ)
    {
      noInterrupts();
      btnIRQ = false;
      interrupts();
      currentMode = MAIN_MODE;
      lcdShow("Main Mode", "Starting...");
      delay(1500);
      lcdShow("No logs yet", "");
      for (int p = 1; p <= 5; ++p)
        scheduleNextDispense(p);
    }
  }
  else if (currentMode == MAIN_MODE)
  {
    checkDispenseSchedule();
    handleScrollButtons();

    if (millis() - tWiFiCheck > 5000)
    {
      tWiFiCheck = millis();
      if (WiFi.status() != WL_CONNECTED)
      {
        lcdShow("WiFi lost", "reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
    }

    delay(50); // faster loop for responsive buttons
  }
}
