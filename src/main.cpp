#include <Arduino.h>
#include <cstdio>     // snprintf
#include <time.h>     // time_t, struct tm, localtime_r, strftime

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiUdp.h>

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
 * Audio Output: "Action time for service" (keep original delays)
 */
void speakDispense()
{
  dacWrite(AUDIO_PIN, 0);
  // voice.say(sp4_ACTION); delay(120);  // uncomment if you want "Action"
  voice.say(sp3_TIME);
  delay(120);
  voice.say(sp4_FOR);
  delay(140);
  voice.say(sp4_SERVICE);
  delay(140);
  dacWrite(AUDIO_PIN, 0);
}

void speakLevelLow()
{
  dacWrite(AUDIO_PIN, 0);
  voice.say(sp4_WARNING);
  delay(200);
  voice.say(sp4_LEVEL);
  delay(200);
  voice.say(sp3_LOW);
  delay(200);
  dacWrite(AUDIO_PIN, 0);
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
static const int BTN_PIN  = 36;  // confirm/dispense
static const int BTN_DOWN = 34;  // scroll down
static const int BTN_UP   = 35;  // scroll up
static const unsigned long DEBOUNCE_MS = 200;

// Edge-queued button press counter (captures rapid taps)
volatile uint8_t btnCount = 0;
volatile unsigned long lastISR = 0;

// -----------------------------
// Pill Parameters
// -----------------------------
int freqv[6]  = {0, 1, 1, 1, 1, 1};
int thresh[6] = {0, 5, 5, 5, 5, 5};
int stockv[6] = {0, 20, 20, 20, 20, 20};

enum Mode { SETUP_MODE, MAIN_MODE };
Mode currentMode = SETUP_MODE;

// -----------------------------
// Time and Scheduling
// -----------------------------
WiFiUDP ntpUDP;
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -4 * 3600; // Puerto Rico timezone
const int daylightOffset_sec = 0;
time_t nextDispense[6];       // next scheduled dispense time per pill
bool dispensing[6] = {false}; // pill is due & waiting for confirm

// Group/burst handling
static bool dueGroupActive = false;          // true while there are unconfirmed due pills

// -----------------------------
// Logging (Circular Buffer)
// -----------------------------
struct LogEntry { int pill; time_t ts; };
LogEntry logs[5];
int logCount = 0;
int logIndex = 0;

// -----------------------------
// Utility
// -----------------------------
String makeUrl(const char *path) {
  String u = BASE;
  u += path;
  u += "?deviceId="; u += DEVICE_ID;
  u += "&secret=";   u += DEVICE_SECRET;
  return u;
}

void lcdShow(const String &l1, const String &l2) {
#if USE_LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(l1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(l2.substring(0, 16));
#endif
}

// ---- TS formatting helpers ----
// Line-2 fits 16 chars exactly: "DD/MM/YY HH:MMAM"
void formatDateLine(char* out, size_t n, time_t t) {
  struct tm tmv; localtime_r(&t, &tmv);
  char datebuf[9];   // "DD/MM/YY" -> 8
  char timebuf[6];   // "HH:MM"    -> 5 (12-hour)
  char pbuf[3];      // "%p" -> "AM"/"PM"

  strftime(datebuf, sizeof(datebuf), "%d/%m/%y", &tmv);
  strftime(timebuf, sizeof(timebuf), "%I:%M",   &tmv);
  strftime(pbuf,    sizeof(pbuf),    "%p",      &tmv);

  // Normalize to 'AM' or 'PM' regardless of locale
  char AP = (pbuf[0] == 'p' || pbuf[0] == 'P') ? 'P' : 'A';

  // Compose with NO space before AM/PM: "DD/MM/YY HH:MMAM"
  // 8 + 1 + 5 + 2 = 16 chars total
  snprintf(out, n, "%s %s%cM", datebuf, timebuf, AP);
}

// -----------------------------
// Non-blocking Network Queue
// -----------------------------
struct NetJob { String url; String body; };
static NetJob jobs[12];
static int qh = 0, qt = 0;

inline bool qEmpty() { return qh == qt; }
inline bool qFull()  { return ((qt + 1) % (int)(sizeof(jobs)/sizeof(jobs[0]))) == qh; }

void enqueueJob(const String& url, const String& body) {
  if (qFull()) { Serial.println("[NET] queue full, dropping job"); return; }
  jobs[qt] = {url, body};
  qt = (qt + 1) % (int)(sizeof(jobs)/sizeof(jobs[0]));
}

// Reused TLS client to reduce handshake overhead
static WiFiClientSecure sharedClient;
static HTTPClient sharedHttp;

void serviceNetQueue() {
  static unsigned long lastKick = 0;
  if (millis() - lastKick < 150) return;
  lastKick = millis();
  if (qEmpty()) return;

  sharedClient.setInsecure();
  NetJob &job = jobs[qh];

  if (sharedHttp.begin(sharedClient, job.url)) {
    sharedHttp.addHeader("Content-Type", "application/json");
    int code = sharedHttp.POST(job.body);
    Serial.printf("[HTTP POST] %s -> %d\n", job.url.c_str(), code);
    sharedHttp.end();
  } else {
    Serial.printf("[HTTP BEGIN FAIL] %s\n", job.url.c_str());
  }
  qh = (qh + 1) % (int)(sizeof(jobs)/sizeof(jobs[0]));
}

// -----------------------------
// Cloud Interaction (enqueued)
// -----------------------------
void pushTelemetry(int pill) {
  JsonDocument d;
  d["pill"] = pill;
  d["freq"] = freqv[pill];
  d["threshold"] = thresh[pill];
  d["stock"] = stockv[pill];
  d["ts"] = (uint32_t)millis();
  String body; serializeJson(d, body);
  enqueueJob(makeUrl("/pushTelemetry"), body);
}

void pushAlert(const char *type, int pill, const char *msg, int stockOpt = -1) {
  JsonDocument d;
  d["type"] = type;
  d["pill"] = pill;
  d["msg"] = msg;
  if (stockOpt >= 0) d["stock"] = stockOpt;
  d["ts"] = (uint32_t)millis();
  String body; serializeJson(d, body);
  enqueueJob(makeUrl("/pushAlert"), body);
}

// Forward declaration used in applyCommand()
void maybeLowStockAlert(int pill, int prevStock, int prevThresh);

// -----------------------------
// Commands (pull stays blocking; only in SETUP)
// -----------------------------
bool httpsGet(const String &url, String &out) {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code > 0) out = http.getString();
  Serial.printf("[HTTP GET] %s -> %d\n", url.c_str(), code);
  http.end();
  return (code >= 200 && code < 300);
}

void applyCommand(int pill, const char *field, int value) {
  int prevStock = stockv[pill];
  int prevThresh = thresh[pill];

  if (strcmp(field, "freq") == 0) {
    freqv[pill] = constrain(value, 1, 4);
    lcdShow("P" + String(pill) + " freq ->", String(freqv[pill]));
  } else if (strcmp(field, "threshold") == 0) {
    thresh[pill] = max(0, value);
    lcdShow("P" + String(pill) + " threshold", "-> " + String(thresh[pill]));
    maybeLowStockAlert(pill, prevStock, prevThresh);
  } else if (strcmp(field, "stock") == 0) {
    stockv[pill] = max(0, value);
    lcdShow("P" + String(pill) + " stock ->", String(stockv[pill]));
    maybeLowStockAlert(pill, prevStock, prevThresh);
  }

  pushTelemetry(pill); // enqueued
}

void pullCommandsDrain() {
  for (int i = 0; i < 5; ++i) {
    String resp;
    if (!httpsGet(makeUrl("/pullCommand"), resp)) return;
    JsonDocument doc;
    if (deserializeJson(doc, resp)) return;
    if (!doc["ok"] || doc["cmd"].isNull()) return;

    int pill = doc["cmd"]["pill"] | 1;
    const char *field = doc["cmd"]["field"] | "";
    int value = doc["cmd"]["value"] | 0;

    Serial.printf("[CMD] pill=%d field=%s value=%d\n", pill, field, value);
    applyCommand(pill, field, value);
    delay(100);
  }
}

// -----------------------------
// Threshold (Low Stock) Alerts
// -----------------------------
void maybeLowStockAlert(int pill, int prevStock, int prevThresh) {
  // Fires on threshold CROSSING (one-time)
  if (prevStock > prevThresh && stockv[pill] <= thresh[pill]) {
    pushAlert("pill_low", pill, "Pill stock is low", stockv[pill]);
    speakLevelLow();
  }
}

// New: fires on ALARM ACTIVATION (every time pill becomes due)
// Use this to replay the low-stock audio each time the alarm triggers.
void lowStockAlertOnAlarm(int pill) {
  if (stockv[pill] <= thresh[pill]) {
    pushAlert("pill_low", pill, "Pill stock is low", stockv[pill]);
    speakLevelLow();
  }
}

// -----------------------------
// Scheduling & Logging
// -----------------------------
void scheduleNextDispense(int pill) {
  time_t now; time(&now);
  nextDispense[pill] = now + freqv[pill] * 60; // freq * 1 min demo
  dispensing[pill]   = false;                  // reset wait-for-confirm
  Serial.printf("[SCHED] Pill %d next at %ld\n", pill, nextDispense[pill]);
}

void addLog(int pill) {
  time_t now; time(&now);
  logs[logCount % 5] = {pill, now};
  logCount++;
  logIndex = (logCount - 1) % 5;
}

void displayLogAtIndex(int index) {
  if (logCount == 0) { lcdShow("No logs yet", ""); return; }
  int validCount = min(logCount, 5);
  index = (index + validCount) % validCount;

  char line2[17];
  formatDateLine(line2, sizeof(line2), logs[index].ts);  // "DD/MM/YY HH:MMAM"
  lcdShow("P" + String(logs[index].pill) + " Dispensed", line2);
}

// -----------------------------
// Confirm ALL currently-due pills (one press clears the burst)
// -----------------------------
int confirmAllDuePills() {
  time_t now; time(&now);
  int cleared = 0;

  for (int p = 1; p <= 5; ++p) {
    if (freqv[p] > 0 && now >= nextDispense[p] && dispensing[p]) {
      if (stockv[p] > 0) stockv[p]--;

      char line2[17];
      formatDateLine(line2, sizeof(line2), now); // "DD/MM/YY HH:MMAM"
      lcdShow("Dispensed P" + String(p), String(line2));

      addLog(p);
      pushTelemetry(p);                           // enqueued
      pushAlert("dispense", p, "User dispensed"); // enqueued

      // Keep one-time crossing alert after decrement
      maybeLowStockAlert(p, stockv[p] + 1, thresh[p]);

      scheduleNextDispense(p); // push to future & clear dispensing[p]
      cleared++;
    }
  }
  return cleared;
}

// -----------------------------
// Dispense Scheduler (burst-aware)
// -----------------------------
void checkDispenseSchedule() {
  time_t tnow; time(&tnow);

  // 1) Mark pills that just became due
  bool anyDue = false;
  for (int p = 1; p <= 5; ++p) {
    if (freqv[p] > 0 && tnow >= nextDispense[p]) {
      if (!dispensing[p]) {
        dispensing[p] = true;  // arm once when it becomes due

        // New: immediately check low stock for THIS pill at alarm activation
        lowStockAlertOnAlarm(p);
      }
      anyDue = true;
    }
  }

  // 2) If a new burst just started, speak ONCE and show UI; send PER-PILL notifications
  if (anyDue && !dueGroupActive) {
    int dueCount = 0, onlyPill = -1;
    for (int p = 1; p <= 5; ++p) {
      if (dispensing[p]) { dueCount++; if (onlyPill == -1) onlyPill = p; }
    }

    if (dueCount == 1) {
      // Exactly one pill due → single notification with its number
      pushAlert("time_to_take", onlyPill, ("Time to take pill " + String(onlyPill)).c_str());
      lcdShow("Time to take", "Pill " + String(onlyPill));
    } else {
      // Multiple pills due → send ONE notification PER pill (no commas)
      for (int p = 1; p <= 5; ++p) {
        if (dispensing[p]) {
          pushAlert("time_to_take", p, ("Time to take pill " + String(p)).c_str());
        }
      }
      lcdShow("Time to take", String(dueCount) + " pill(s) due");
    }

    speakDispense();      // full audio (once per burst)

    // New: after the main prompt, replay low-stock warning for EACH due low pill
    for (int p = 1; p <= 5; ++p) {
      if (dispensing[p] && stockv[p] <= thresh[p]) {
        speakLevelLow();
      }
    }

    dueGroupActive = true;
  }

  // 3) If user pressed confirm, clear ALL currently-due pills in this burst
  uint8_t got = 0;
  noInterrupts();
  if (btnCount > 0) { btnCount--; got = 1; }
  interrupts();

  if (got && dueGroupActive) {
    int cleared = confirmAllDuePills();

    // Check if anything is still due (race: something could become due *now*)
    bool stillDue = false;
    for (int p = 1; p <= 5; ++p) if (dispensing[p]) { stillDue = true; break; }

    if (!stillDue) {
      dueGroupActive = false;
      // Optional: leave last "Dispensed Pk" or show summary:
      lcdShow("All set", "Cleared " + String(cleared));
    }
    return; // keep loop snappy
  }

  // 4) No confirm; nothing else to do here this tick
}

// -----------------------------
// ISR + Setup + Loop
// -----------------------------
void IRAM_ATTR btnISR() {
  unsigned long now = millis();
  if (now - lastISR > 60) {          // ~60 ms debounce
    if (btnCount < 10) btnCount++;   // small cap
    lastISR = now;
  }
}

void setup() {
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK, IP=%s\n", WiFi.localIP().toString().c_str());
  lcdShow("WiFi connected", WiFi.localIP().toString());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  lcdShow("Setup Mode", "Waiting...");
}

void handleScrollButtons() {
  static unsigned long lastPress = 0;
  if (millis() - lastPress < DEBOUNCE_MS) return;

  if (digitalRead(BTN_DOWN)) {
    lastPress = millis();
    if (logCount > 0) {
      logIndex = (logIndex + 1) % min(logCount, 5);
      displayLogAtIndex(logIndex);
    } else lcdShow("No logs yet", "");
  } else if (digitalRead(BTN_UP)) {
    lastPress = millis();
    if (logCount > 0) {
      int validCount = min(logCount, 5);
      logIndex = (logIndex - 1 + validCount) % validCount;
      displayLogAtIndex(logIndex);
    } else lcdShow("No logs yet", "");
  }
}

void loop() {
  static unsigned long tPoll = 0, tWiFiCheck = 0;

  if (currentMode == SETUP_MODE) {
    if (millis() - tPoll > 1000) {
      tPoll = millis();
      pullCommandsDrain();  // blocking is fine here
    }

    // Enter MAIN when button pressed (use queued press)
    uint8_t enter = 0;
    noInterrupts();
    if (btnCount > 0) { btnCount--; enter = 1; }
    interrupts();

    if (enter) {
      currentMode = MAIN_MODE;
      lcdShow("Main Mode", "Starting...");
      delay(1500);
      lcdShow("No logs yet", "");
      for (int p = 1; p <= 5; ++p) scheduleNextDispense(p);
    }
  }
  else if (currentMode == MAIN_MODE) {
    checkDispenseSchedule(); // burst-aware
    handleScrollButtons();
    serviceNetQueue();       // drain HTTP queue

    if (millis() - tWiFiCheck > 5000) {
      tWiFiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) {
        lcdShow("WiFi lost", "reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
    }
    delay(30);
  }
}
