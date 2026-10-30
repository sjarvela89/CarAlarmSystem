/*
  CarAlarmSystem.ino
  Arduino Mega 2560 burglar alarm / vehicle anti-theft prototype.

  Hardware assumptions:
  - Arduino Mega 2560
  - SIM900 GSM/GPRS module on Serial1
  - GPS module (NMEA 0183) on Serial3
  - PIR sensor on pin 48
  - Immobilizer relay on pin 22 (active HIGH by default)
  - Status LEDs on pins 23 (red) and 24 (green)
  - Vehicle battery sense on A0 through a resistor divider

  This sketch intentionally avoids the thesis prototype issues:
  - Uses SMS-based control instead of relying on incoming caller number.
  - Uses Mega hardware serial ports instead of SoftwareSerial.
  - Uses a non-blocking state machine instead of the blocking/buggy loop logic.
*/

#include <Arduino.h>

// -------------------- User configuration --------------------

static const char OWNER_NUMBER[] = "+358XXXXXXXXX"; // Replace with the holder's phone number.

static const bool ENABLE_ALARM_CALL = true;      // Outgoing call on alarm, if the SIM/network allows it.
static const bool ENABLE_LOW_BATTERY_SMS = true; // Send a one-time SMS when battery voltage drops.

static const uint32_t GSM_BAUD = 115200;         // SIM900 factory baud used in the thesis.
static const uint32_t GPS_BAUD = 9600;           // Typical NMEA baud.

static const uint8_t PIR_PIN = 48;
static const uint8_t RELAY_PIN = 22;
static const uint8_t RED_LED_PIN = 23;
static const uint8_t GREEN_LED_PIN = 24;
static const uint8_t BATTERY_PIN = A0;

// Relay polarity: many relay boards are active LOW. Change if needed.
static const uint8_t RELAY_ACTIVE_LEVEL = HIGH;
static const uint8_t RELAY_INACTIVE_LEVEL = LOW;

// Battery monitor: adjust to your divider.
// Example: if 12.6V on the battery becomes 3.15V on the analog pin, ratio = 4.0.
static const float ADC_REF_VOLTAGE = 5.0f;
static const float BATTERY_DIVIDER_RATIO = 4.0f;
static const float LOW_BATTERY_THRESHOLD_V = 11.8f;

// Timing
static const uint32_t PIR_ARM_DELAY_MS = 30000UL;     // Thesis: wait ~30 seconds after arming.
static const uint32_t BATTERY_CHECK_INTERVAL_MS = 60000UL;
static const uint32_t GPS_FIX_STALE_MS = 5000UL;
static const uint32_t SMS_NOTIFY_COOLDOWN_MS = 120000UL;
static const uint32_t ALARM_CALL_DURATION_MS = 30000UL;
static const uint32_t LED_BLINK_MS = 180UL;

// -------------------- Serial aliases --------------------

#define GSM_SERIAL Serial1
#define GPS_SERIAL Serial3

// -------------------- State --------------------

enum AlarmState : uint8_t {
  DISARMED = 0,
  ARMING_DELAY,
  ARMED,
  ALARM_TRIGGERED
};

static AlarmState alarmState = DISARMED;
static bool immobilizerLocked = false;

static uint32_t armRequestedAtMs = 0;
static uint32_t lastBatteryCheckMs = 0;
static uint32_t lastSmsNotifyMs = 0;
static uint32_t alarmStartedAtMs = 0;
static uint32_t ledBlinkUntilMs = 0;
static bool ledBlinkState = false;
static uint8_t ledBlinkCount = 0;
static uint8_t ledBlinkTarget = 0;

static bool lowBatteryWarned = false;
static bool pendingLocationReply = false;
static uint32_t locationRequestAtMs = 0;

// GPS fix
static bool gpsHasFix = false;
static float gpsLatitude = 0.0f;
static float gpsLongitude = 0.0f;
static uint32_t lastGpsFixMs = 0;

// GSM parsing
static char gsmLine[180];
static uint8_t gsmLinePos = 0;
static char lastSmsSender[32] = {0};
static bool awaitingSmsBody = false;

// GPS parsing
static char gpsLine[120];
static uint8_t gpsLinePos = 0;

// Forward declarations
void blinkLed(uint8_t pin, uint8_t times);
void setImmobilizer(bool locked);
void setAlarmState(AlarmState newState);
void sendSms(const char *number, const char *text);
void callOwner();
void hangUp();
void notifyAlarm(const char *reason);
void notifyStatus();
void requestLocation();
void handleSmsCommand(const char *sender, const char *body);
void handleGsmLine(const char *line);
void handleGpsLine(const char *line);
void pollGsm();
void pollGps();
void initGsm();
bool gsmWaitForToken(const char *token, uint32_t timeoutMs);
void gsmFlushInput();
bool startsWith(const char *s, const char *prefix);
void trimInPlace(char *s);
void toUpperInPlace(char *s);
bool senderMatchesOwner(const char *sender);
bool parseRmcSentence(const char *sentence, float &lat, float &lon);
bool parseNmeaCoord(const char *value, char hemisphere, float &out);
float readBatteryVoltage();
void checkBattery();
void updateLeds();
void scheduleLedBlink(uint8_t pin, uint8_t times);
void reportAliveOnce();

// -------------------- Utility --------------------

bool startsWith(const char *s, const char *prefix) {
  while (*prefix) {
    if (*s++ != *prefix++) return false;
  }
  return true;
}

void trimInPlace(char *s) {
  if (!s) return;
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n' || s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[--len] = '\0';
  }
  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\t') start++;
  if (start > 0) {
    memmove(s, s + start, strlen(s + start) + 1);
  }
}

void toUpperInPlace(char *s) {
  for (; *s; ++s) {
    if (*s >= 'a' && *s <= 'z') *s = char(*s - 'a' + 'A');
  }
}

bool senderMatchesOwner(const char *sender) {
  if (!sender || !sender[0]) return false;
  // Accept exact match or the same number with leading '+' variations handled minimally.
  return strcmp(sender, OWNER_NUMBER) == 0;
}

void gsmFlushInput() {
  while (GSM_SERIAL.available()) {
    GSM_SERIAL.read();
  }
}

bool gsmWaitForToken(const char *token, uint32_t timeoutMs) {
  char buffer[64] = {0};
  uint8_t pos = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (GSM_SERIAL.available()) {
      char c = (char)GSM_SERIAL.read();
      if (pos < sizeof(buffer) - 1) {
        buffer[pos++] = c;
        buffer[pos] = '\0';
      } else {
        memmove(buffer, buffer + 1, sizeof(buffer) - 2);
        buffer[sizeof(buffer) - 2] = c;
        buffer[sizeof(buffer) - 1] = '\0';
      }
      if (strstr(buffer, token)) {
        return true;
      }
      // Allow processing asynchronous lines during waits.
      if (c == '\n') {
        handleGsmLine(buffer);
        pos = 0;
        buffer[0] = '\0';
      }
    }
  }
  return false;
}

// -------------------- Hardware actions --------------------

void blinkLed(uint8_t pin, uint8_t times) {
  for (uint8_t i = 0; i < times; ++i) {
    digitalWrite(pin, HIGH);
    delay(LED_BLINK_MS);
    digitalWrite(pin, LOW);
    delay(LED_BLINK_MS);
  }
}

void scheduleLedBlink(uint8_t pin, uint8_t times) {
  (void)pin; // Pin is implicit in target color.
  ledBlinkTarget = times * 2; // on+off cycles
  ledBlinkCount = 0;
  ledBlinkUntilMs = millis();
  ledBlinkState = false;
}

void updateLeds() {
  // If a blink sequence is active, toggle between red and green LEDs.
  if (ledBlinkTarget == 0) return;
  if (millis() < ledBlinkUntilMs) return;

  ledBlinkUntilMs = millis() + LED_BLINK_MS;
  ledBlinkState = !ledBlinkState;
  ledBlinkCount++;

  if (alarmState == DISARMED) {
    digitalWrite(GREEN_LED_PIN, ledBlinkState ? HIGH : LOW);
  } else {
    digitalWrite(RED_LED_PIN, ledBlinkState ? HIGH : LOW);
  }

  if (ledBlinkCount >= ledBlinkTarget) {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    ledBlinkTarget = 0;
    ledBlinkCount = 0;
  }
}

void setImmobilizer(bool locked) {
  immobilizerLocked = locked;
  digitalWrite(RELAY_PIN, locked ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
}

void setAlarmState(AlarmState newState) {
  alarmState = newState;

  if (newState == DISARMED) {
    setImmobilizer(false);
    scheduleLedBlink(GREEN_LED_PIN, 3);
  } else if (newState == ARMING_DELAY || newState == ARMED || newState == ALARM_TRIGGERED) {
    setImmobilizer(true);
    scheduleLedBlink(RED_LED_PIN, 3);
  }
}

// -------------------- GSM / SMS / call --------------------

void initGsm() {
  delay(5000); // Give the SIM900 time to power up.
  gsmFlushInput();

  for (uint8_t attempt = 0; attempt < 6; ++attempt) {
    GSM_SERIAL.println(F("AT"));
    if (gsmWaitForToken("OK", 2000)) break;
    delay(1000);
  }

  GSM_SERIAL.println(F("ATE0"));           // Echo off
  gsmWaitForToken("OK", 2000);

  GSM_SERIAL.println(F("AT+CMGF=1"));      // SMS text mode
  gsmWaitForToken("OK", 2000);

  GSM_SERIAL.println(F("AT+CNMI=2,2,0,0,0")); // Push incoming SMS directly to serial
  gsmWaitForToken("OK", 2000);

  GSM_SERIAL.println(F("AT+CLIP=1"));      // Caller ID for diagnostics if supported
  gsmWaitForToken("OK", 2000);
}

void sendSms(const char *number, const char *text) {
  if (!number || !number[0] || !text) return;

  GSM_SERIAL.print(F("AT+CMGS=\""));
  GSM_SERIAL.print(number);
  GSM_SERIAL.println(F("\""));

  if (!gsmWaitForToken(">", 5000)) {
    return;
  }

  GSM_SERIAL.print(text);
  GSM_SERIAL.write((char)26);
  gsmWaitForToken("OK", 15000);
}

void callOwner() {
  if (!ENABLE_ALARM_CALL) return;
  GSM_SERIAL.print(F("ATD"));
  GSM_SERIAL.print(OWNER_NUMBER);
  GSM_SERIAL.println(F(";"));
}

void hangUp() {
  GSM_SERIAL.println(F("ATH"));
}

void notifyAlarm(const char *reason) {
  uint32_t now = millis();
  if (now - lastSmsNotifyMs < SMS_NOTIFY_COOLDOWN_MS) return;
  lastSmsNotifyMs = now;

  char message[160];
  snprintf(message, sizeof(message), "ALARM: %s. Immobilizer ON.", reason);
  sendSms(OWNER_NUMBER, message);

  callOwner();
  alarmStartedAtMs = now;
  alarmState = ALARM_TRIGGERED;
}

void notifyStatus() {
  char message[180];
  const char *stateText =
      (alarmState == DISARMED) ? "DISARMED" :
      (alarmState == ARMING_DELAY) ? "ARMING" :
      (alarmState == ARMED) ? "ARMED" :
      "ALARM";

  char latBuf[16] = {0};
  char lonBuf[16] = {0};
  if (gpsHasFix) {
    dtostrf(gpsLatitude, 0, 6, latBuf);
    dtostrf(gpsLongitude, 0, 6, lonBuf);
  } else {
    strcpy(latBuf, "NOFIX");
    strcpy(lonBuf, "NOFIX");
  }

  float batteryV = readBatteryVoltage();
  snprintf(message, sizeof(message),
           "STATUS:%s IMMOB:%s BAT:%.2fV GPS:%s,%s",
           stateText,
           immobilizerLocked ? "LOCKED" : "OPEN",
           batteryV,
           latBuf,
           lonBuf);
  sendSms(OWNER_NUMBER, message);
}

void requestLocation() {
  pendingLocationReply = true;
  locationRequestAtMs = millis();
  sendSms(OWNER_NUMBER, "GPS: locating...");
}

void sendLocationReply() {
  char message[180];
  if (!gpsHasFix || (millis() - lastGpsFixMs > GPS_FIX_STALE_MS)) {
    snprintf(message, sizeof(message), "GPS: no valid fix yet");
  } else {
    char latBuf[16] = {0};
    char lonBuf[16] = {0};
    dtostrf(gpsLatitude, 0, 6, latBuf);
    dtostrf(gpsLongitude, 0, 6, lonBuf);
    snprintf(message, sizeof(message), "GPS: %s,%s", latBuf, lonBuf);
  }
  sendSms(OWNER_NUMBER, message);
}

// -------------------- SMS command handling --------------------

void handleSmsCommand(const char *sender, const char *body) {
  if (!senderMatchesOwner(sender)) {
    // Ignore unknown numbers; optionally notify owner.
    return;
  }

  char cmd[96];
  strncpy(cmd, body, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';
  trimInPlace(cmd);
  toUpperInPlace(cmd);

  if (strcmp(cmd, "ACTIVATE") == 0 || strcmp(cmd, "ARM") == 0) {
    armRequestedAtMs = millis();
    alarmState = ARMING_DELAY;
    setImmobilizer(true);
    sendSms(OWNER_NUMBER, "System activated. PIR calibration in progress.");
    return;
  }

  if (strcmp(cmd, "DEACTIVATE") == 0 || strcmp(cmd, "DISARM") == 0) {
    setAlarmState(DISARMED);
    sendSms(OWNER_NUMBER, "System deactivated.");
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    setImmobilizer(true);
    sendSms(OWNER_NUMBER, "Immobilizer: ON");
    return;
  }

  if (strcmp(cmd, "START") == 0) {
    setImmobilizer(false);
    sendSms(OWNER_NUMBER, "Immobilizer: OFF");
    return;
  }

  if (strcmp(cmd, "LOCATION") == 0) {
    requestLocation();
    return;
  }

  if (strcmp(cmd, "STATUS") == 0) {
    notifyStatus();
    return;
  }

  if (strcmp(cmd, "HANGUP") == 0 || strcmp(cmd, "STOPCALL") == 0) {
    hangUp();
    return;
  }

  sendSms(OWNER_NUMBER, "Unknown command.");
}

void handleGsmLine(const char *line) {
  if (!line || !line[0]) return;

  // Incoming SMS format from CNMI: +CMT: "number",,...
  if (startsWith(line, "+CMT:")) {
    char sender[32] = {0};
    const char *firstQuote = strchr(line, '"');
    const char *secondQuote = firstQuote ? strchr(firstQuote + 1, '"') : nullptr;
    if (firstQuote && secondQuote && secondQuote > firstQuote + 1) {
      size_t n = (size_t)(secondQuote - (firstQuote + 1));
      if (n >= sizeof(sender)) n = sizeof(sender) - 1;
      memcpy(sender, firstQuote + 1, n);
      sender[n] = '\0';
      strncpy(lastSmsSender, sender, sizeof(lastSmsSender) - 1);
      lastSmsSender[sizeof(lastSmsSender) - 1] = '\0';
      awaitingSmsBody = true;
    }
    return;
  }

  if (awaitingSmsBody) {
    char body[160];
    strncpy(body, line, sizeof(body) - 1);
    body[sizeof(body) - 1] = '\0';
    trimInPlace(body);
    handleSmsCommand(lastSmsSender, body);
    awaitingSmsBody = false;
    return;
  }

  // Optional CLIP handling for diagnostics only.
  if (startsWith(line, "+CLIP:")) {
    // Example: +CLIP: "+358...",145,"",0,"",0
    return;
  }
}

void handleGpsLine(const char *line) {
  if (!line || !line[0]) return;

  if (strstr(line, "RMC") != nullptr) {
    float lat = 0.0f, lon = 0.0f;
    if (parseRmcSentence(line, lat, lon)) {
      gpsLatitude = lat;
      gpsLongitude = lon;
      gpsHasFix = true;
      lastGpsFixMs = millis();
    }
  }
}

void pollGsm() {
  while (GSM_SERIAL.available()) {
    char c = (char)GSM_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      gsmLine[gsmLinePos] = '\0';
      if (gsmLinePos > 0) {
        handleGsmLine(gsmLine);
      }
      gsmLinePos = 0;
      continue;
    }

    if (gsmLinePos < sizeof(gsmLine) - 1) {
      gsmLine[gsmLinePos++] = c;
    } else {
      gsmLinePos = 0;
    }
  }
}

void pollGps() {
  while (GPS_SERIAL.available()) {
    char c = (char)GPS_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      gpsLine[gpsLinePos] = '\0';
      if (gpsLinePos > 0) {
        handleGpsLine(gpsLine);
      }
      gpsLinePos = 0;
      continue;
    }

    if (gpsLinePos < sizeof(gpsLine) - 1) {
      gpsLine[gpsLinePos++] = c;
    } else {
      gpsLinePos = 0;
    }
  }
}

// -------------------- GPS parsing --------------------

bool parseNmeaCoord(const char *value, char hemisphere, float &out) {
  if (!value || !value[0]) return false;
  float raw = (float)atof(value);
  if (raw <= 0.0f) return false;

  int degrees = (int)(raw / 100.0f);
  float minutes = raw - (degrees * 100.0f);
  out = (float)degrees + (minutes / 60.0f);

  if (hemisphere == 'S' || hemisphere == 'W') {
    out = -out;
  }
  return true;
}

bool parseRmcSentence(const char *sentence, float &lat, float &lon) {
  // Expected fields:
  // $GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,...
  char buf[120];
  strncpy(buf, sentence, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *save = nullptr;
  char *token = strtok_r(buf, ",", &save);
  int field = 0;
  char status = 'V';
  char latStr[16] = {0};
  char lonStr[16] = {0};
  char ns = 'N';
  char ew = 'E';

  while (token != nullptr) {
    switch (field) {
      case 2: status = token[0]; break;
      case 3: strncpy(latStr, token, sizeof(latStr) - 1); break;
      case 4: ns = token[0]; break;
      case 5: strncpy(lonStr, token, sizeof(lonStr) - 1); break;
      case 6: ew = token[0]; break;
      default: break;
    }
    token = strtok_r(nullptr, ",", &save);
    field++;
  }

  if (status != 'A') return false;
  if (!parseNmeaCoord(latStr, ns, lat)) return false;
  if (!parseNmeaCoord(lonStr, ew, lon)) return false;
  return true;
}

// -------------------- Battery monitor --------------------

float readBatteryVoltage() {
  const uint16_t raw = analogRead(BATTERY_PIN);
  const float pinVoltage = (raw * ADC_REF_VOLTAGE) / 1023.0f;
  return pinVoltage * BATTERY_DIVIDER_RATIO;
}

void checkBattery() {
  if (!ENABLE_LOW_BATTERY_SMS) return;

  const uint32_t now = millis();
  if (now - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) return;
  lastBatteryCheckMs = now;

  float v = readBatteryVoltage();
  if (v < LOW_BATTERY_THRESHOLD_V) {
    if (!lowBatteryWarned) {
      char msg[80];
      snprintf(msg, sizeof(msg), "Low battery: %.2fV", v);
      sendSms(OWNER_NUMBER, msg);
      lowBatteryWarned = true;
    }
  } else {
    lowBatteryWarned = false;
  }
}

// -------------------- Lifecycle --------------------

void reportAliveOnce() {
  static bool alreadySent = false;
  if (alreadySent) return;
  alreadySent = true;
  sendSms(OWNER_NUMBER, "Car alarm system online.");
}

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  setImmobilizer(false);

  Serial.begin(115200);
  GSM_SERIAL.begin(GSM_BAUD);
  GPS_SERIAL.begin(GPS_BAUD);

  initGsm();

  // Start in disarmed mode.
  setAlarmState(DISARMED);

  reportAliveOnce();
}

void loop() {
  pollGsm();
  pollGps();
  updateLeds();
  checkBattery();

  // Handle arming delay.
  if (alarmState == ARMING_DELAY) {
    if (millis() - armRequestedAtMs >= PIR_ARM_DELAY_MS) {
      alarmState = ARMED;
      sendSms(OWNER_NUMBER, "System armed.");
    }
  }

  // Detect motion only when armed.
  if (alarmState == ARMED) {
    if (digitalRead(PIR_PIN) == HIGH) {
      notifyAlarm("motion detected");
    }
  }

  // Alarm mode: keep notifications and allow recovery by SMS.
  if (alarmState == ALARM_TRIGGERED) {
    // Periodically reassert the relay.
    setImmobilizer(true);
    if (ENABLE_ALARM_CALL && (millis() - alarmStartedAtMs > ALARM_CALL_DURATION_MS)) {
      hangUp();
    }
  }

  // Location request handling: send a reply once a valid fix arrives or timeout.
  if (pendingLocationReply) {
    if (gpsHasFix && (millis() - lastGpsFixMs <= GPS_FIX_STALE_MS)) {
      sendLocationReply();
      pendingLocationReply = false;
    } else if (millis() - locationRequestAtMs > 60000UL) {
      sendSms(OWNER_NUMBER, "GPS: no fix within 60s");
      pendingLocationReply = false;
    }
  }

  // Keep the immobilizer locked while armed.
  if (alarmState == ARMED || alarmState == ALARM_TRIGGERED || alarmState == ARMING_DELAY) {
    setImmobilizer(true);
  }
}
