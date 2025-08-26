/*
  Mutfak LED + Alarm + RTC(DS1307) + Bluetooth + LCD (Software I2C)
  Improved version with better Bluetooth handling and LED control
*/

#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <SoftwareWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===================== PINLER =====================
const uint8_t PIN_PIR1   = 2;
const uint8_t PIN_PIR2   = 3;
const uint8_t PIN_FLAME  = 4;   // Aktif LOW
const uint8_t PIN_GAS    = 5;   // Aktif LOW (MQ-135 dijital çıkış)
const uint8_t PIN_BUZZER = 6;
const uint8_t PIN_RELAY  = 7;
const uint8_t PIN_SYSLED = 13;
const uint8_t PIN_LDR    = A0;
const uint8_t LCD_SDA = 8;
const uint8_t LCD_SCL = 9;
const uint8_t BT_RX = 11;
const uint8_t BT_TX = 10;
const uint8_t PIN_BT_STATE = 12;
const uint8_t PIN_DS18B20 = A1;

// ===================== NESNELER =====================
RTC_DS1307 rtc;
SoftwareWire swI2C(LCD_SDA, LCD_SCL);
SoftwareSerial BTSerial(BT_RX, BT_TX);
OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);

// ===================== SABITLER =====================
const int LDR_DARK_THRESHOLD = 600;
const int LDR_LIGHT_THRESHOLD = 520;
const unsigned long NO_MOTION_OFF_DELAY = 5000UL;
const unsigned long LCD_UPDATE_MS = 200UL;
const unsigned long BT_TOAST_MS = 2000UL;
const unsigned long ALERT_BLINK_MS = 500UL;
const unsigned long TEMP_UPDATE_MS = 1000UL;
const unsigned long GAS_WARMUP_MS = 60000UL; // 1 dakika ısınma süresi
const unsigned long BT_IDLE_COMMIT_MS = 250;

// Alarm parametreleri
const unsigned int SIREN_MIN_FREQ = 600;
const unsigned int SIREN_MAX_FREQ = 3200;
const unsigned long SIREN_SWEEP_MS = 1200UL;
const unsigned long SIREN_STEP_MS = 20UL;
const unsigned int NUCLEA_F1 = 520;
const unsigned int NUCLEA_F2 = 880;
const unsigned long NUCLEA_TONE_MS = 700UL;
const unsigned long NUCLEA_SIL_MS = 300UL;
const unsigned long BOTH_SWITCH_MS = 1000UL;

// Sensör yönleri
const bool INVERT_PIR = false;
const bool INVERT_FLAME = true;
const bool INVERT_GAS = true;

// ===================== GLOBAL DEĞIŞKENLER =====================
bool isNight = false;
bool relayOn = false;
bool manualLedControl = false; // Manuel LED kontrolü aktif mi?
bool manualLedState = false;   // Manuel LED durumu
unsigned long lastMotionMs = 0;
unsigned long lastLcdUpdate = 0;
unsigned long bootMs = 0;

// Sıcaklık değişkenleri
float lastTempC = NAN;
unsigned long lastTempReqMs = 0;
bool tempRequestInFlight = false;

// Alarm durumları
enum AlarmMode { ALARM_NONE, ALARM_FLAME, ALARM_GAS, ALARM_BOTH };
AlarmMode alarmMode = ALARM_NONE;
unsigned long sirenStartMs = 0;
unsigned long lastSirenStep = 0;

// Nuclear alarm durumları
const int NUCLEA_SEQ_LEN = 4;
unsigned int nucleoFreqSeq[NUCLEA_SEQ_LEN] = { NUCLEA_F1, 0, NUCLEA_F2, 0 };
unsigned long nucleoDurSeq[NUCLEA_SEQ_LEN] = { NUCLEA_TONE_MS, NUCLEA_SIL_MS, NUCLEA_TONE_MS, NUCLEA_SIL_MS };
int nucleoIdx = 0;
unsigned long nucleoNextChange = 0;

// Both alarm durumları
bool bothUseFlamePhase = true;
unsigned long bothNextSwitch = 0;

// Boot melody
struct Note { unsigned int freq; unsigned long dur; };
const Note BOOT_MELODY[] = {
  {784,150},{0,60},{880,150},{0,60},{988,150},{0,60},{1046,220},{0,120},
  {1318,180},{0,80},{1175,180},{0,80},{1046,220}
};
const int BOOT_LEN = sizeof(BOOT_MELODY)/sizeof(BOOT_MELODY[0]);
bool bootPlaying = true;
int bootIdx = 0;
unsigned long bootNextChange = 0;

// Bluetooth değişkenleri
String btBuf;
bool btConnected = false;
bool prevBtConnected = false;
unsigned long btToastUntil = 0;
unsigned long btLastCharMs = 0;

// LCD değişkenleri
const uint8_t LCD_RS = 0x01;
const uint8_t LCD_RW = 0x02;
const uint8_t LCD_EN = 0x04;
const uint8_t LCD_BL = 0x08;
uint8_t lcdAddr = 0;
bool lcdReady = false;
bool lcdBacklightOn = true;
unsigned long lastAlertBlink = 0;
bool alertVisible = true;

// ===================== YARDIMCI FONKSIYONLAR =====================
void swDelayUS(unsigned int us) {
  unsigned long t0 = micros();
  while((unsigned long)(micros() - t0) < us) {}
}

inline bool readActive(uint8_t pin, bool invert) {
  int v = digitalRead(pin);
  return invert ? (v == LOW) : (v == HIGH);
}

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  digitalWrite(PIN_SYSLED, on ? HIGH : LOW);
}

void updateIsNight(int ldrValue) {
  if (ldrValue >= LDR_DARK_THRESHOLD) isNight = true;
  else if (ldrValue <= LDR_LIGHT_THRESHOLD) isNight = false;
}

// ===================== LCD FONKSIYONLARI =====================
void pcfWrite(uint8_t val) {
  swI2C.beginTransmission(lcdAddr);
  swI2C.write(val);
  swI2C.endTransmission();
}

void lcdPulseEnable(uint8_t data) {
  pcfWrite(data | LCD_EN);
  swDelayUS(2);
  pcfWrite(data & ~LCD_EN);
  swDelayUS(50);
}

void lcdWrite4bits(uint8_t nibble, uint8_t ctrl) {
  uint8_t out = (nibble & 0xF0) | ctrl;
  if (lcdBacklightOn) out |= LCD_BL;
  pcfWrite(out);
  lcdPulseEnable(out);
}

void lcdSend(uint8_t value, bool isData) {
  uint8_t ctrl = isData ? LCD_RS : 0x00;
  lcdWrite4bits(value & 0xF0, ctrl);
  lcdWrite4bits((value << 4) & 0xF0, ctrl);
  if (!isData && (value == 0x01 || value == 0x02)) swDelayUS(2000);
}

void lcdCommand(uint8_t cmd) { lcdSend(cmd, false); }
void lcdWriteChar(char c) { lcdSend((uint8_t)c, true); }

void lcdSetCursor(uint8_t col, uint8_t row) {
  static const uint8_t rowAddr[] = {0x00, 0x40, 0x14, 0x54};
  if (row > 1) row = 1;
  lcdCommand(0x80 | (col + rowAddr[row]));
}

void lcdClear() { lcdCommand(0x01); }
void lcdPrint(const char* s) { while(*s) lcdWriteChar(*s++); }

void lcdPrintLine(uint8_t row, const char* s) {
  char buf[17];
  memset(buf, ' ', 16);
  buf[16] = '\0';
  size_t n = strnlen(s, 16);
  memcpy(buf, s, n);
  lcdSetCursor(0, row);
  lcdPrint(buf);
}

uint8_t detectLcdAddr_SW() {
  uint8_t pref[] = {0x27, 0x3F};
  for (uint8_t i = 0; i < sizeof(pref); i++) {
    swI2C.beginTransmission(pref[i]);
    if (swI2C.endTransmission() == 0) return pref[i];
  }
  for (uint8_t a = 0x20; a <= 0x27; a++) {
    swI2C.beginTransmission(a);
    if (swI2C.endTransmission() == 0) return a;
  }
  return 0;
}

void lcdInit() {
  swDelayUS(50000);
  uint8_t base = (lcdBacklightOn ? LCD_BL : 0);
  pcfWrite(base);
  lcdWrite4bits(0x30, 0);
  swDelayUS(4500);
  lcdWrite4bits(0x30, 0);
  swDelayUS(150);
  lcdWrite4bits(0x30, 0);
  swDelayUS(150);
  lcdWrite4bits(0x20, 0);
  swDelayUS(150);
  lcdCommand(0x28);
  lcdCommand(0x0C);
  lcdClear();
  lcdCommand(0x06);
  lcdReady = true;
}

// ===================== AYDINLATMA LOGİĞİ =====================
void handleLightingLogic(int ldrValue, bool pir1, bool pir2, unsigned long now) {
  updateIsNight(ldrValue);
  bool motion = (pir1 || pir2);
  
  // Manuel kontrol aktifse, otomatik sistemi devre dışı bırak
  if (manualLedControl) {
    setRelay(manualLedState);
    return;
  }
  
  // Otomatik kontrol
  if (isNight) {
    if (motion) {
      if (!relayOn) setRelay(true);
      lastMotionMs = now;
    } else if (relayOn && (now - lastMotionMs >= NO_MOTION_OFF_DELAY)) {
      setRelay(false);
    }
  } else {
    if (relayOn) setRelay(false);
  }
}

// ===================== ALARM SİSTEMİ =====================
void playFlameSiren(unsigned long now) {
  if (sirenStartMs == 0) {
    sirenStartMs = now;
    lastSirenStep = 0;
  }
  unsigned long period = SIREN_SWEEP_MS * 2UL;
  unsigned long t = (now - sirenStartMs) % period;
  float ratio = (t <= SIREN_SWEEP_MS) ? 
                (float)t / SIREN_SWEEP_MS : 
                1.0f - (float)(t - SIREN_SWEEP_MS) / SIREN_SWEEP_MS;
  unsigned int freq = (unsigned int)(SIREN_MIN_FREQ + (SIREN_MAX_FREQ - SIREN_MIN_FREQ) * ratio);
  if (now - lastSirenStep >= SIREN_STEP_MS) {
    tone(PIN_BUZZER, freq);
    lastSirenStep = now;
  }
}

void playNuclearSeq(unsigned long now) {
  if (now >= nucleoNextChange) {
    unsigned int f = nucleoFreqSeq[nucleoIdx];
    if (f == 0) noTone(PIN_BUZZER);
    else tone(PIN_BUZZER, f);
    nucleoNextChange = now + nucleoDurSeq[nucleoIdx];
    nucleoIdx = (nucleoIdx + 1) % NUCLEA_SEQ_LEN;
  }
}

void playBothAlarms(unsigned long now) {
  if (now >= bothNextSwitch) {
    bothUseFlamePhase = !bothUseFlamePhase;
    bothNextSwitch = now + BOTH_SWITCH_MS;
    sirenStartMs = now;
    lastSirenStep = 0;
    nucleoIdx = 0;
    nucleoNextChange = now;
  }
  if (bothUseFlamePhase) playFlameSiren(now);
  else playNuclearSeq(now);
}

void stopAllAlarmSounds() {
  noTone(PIN_BUZZER);
  sirenStartMs = 0;
  lastSirenStep = 0;
  nucleoIdx = 0;
  nucleoNextChange = 0;
}

void handleBootMelody(unsigned long now) {
  if (!bootPlaying) return;
  if (alarmMode != ALARM_NONE) {
    bootPlaying = false;
    noTone(PIN_BUZZER);
    return;
  }
  if (bootNextChange == 0 || now >= bootNextChange) {
    if (bootIdx >= BOOT_LEN) {
      bootPlaying = false;
      noTone(PIN_BUZZER);
      return;
    }
    unsigned int f = BOOT_MELODY[bootIdx].freq;
    unsigned long d = BOOT_MELODY[bootIdx].dur;
    if (f == 0) noTone(PIN_BUZZER);
    else tone(PIN_BUZZER, f);
    bootNextChange = now + d;
    bootIdx++;
  }
}

void handleAlarms(bool flameActive, bool gasActiveEffective, unsigned long now) {
  AlarmMode newMode = ALARM_NONE;
  if (flameActive && gasActiveEffective) newMode = ALARM_BOTH;
  else if (flameActive) newMode = ALARM_FLAME;
  else if (gasActiveEffective) newMode = ALARM_GAS;

  if (newMode != alarmMode) {
    alarmMode = newMode;
    stopAllAlarmSounds();
    bothUseFlamePhase = true;
    bothNextSwitch = now + BOTH_SWITCH_MS;
  }

  switch (alarmMode) {
    case ALARM_FLAME: playFlameSiren(now); break;
    case ALARM_GAS: playNuclearSeq(now); break;
    case ALARM_BOTH: playBothAlarms(now); break;
    case ALARM_NONE: handleBootMelody(now); break;
  }
}

// ===================== BLUETOOTH İŞLEME =====================
void btSendHelp() {
  BTSerial.println(F("Komutlar:"));
  BTSerial.println(F("  SET YYYY-MM-DD HH:MM:SS"));
  BTSerial.println(F("  TIME YYYY-MM-DD HH:MM:SS"));
  BTSerial.println(F("  SET_TIME [YYYY-MM-DD HH:MM:SS]"));
  BTSerial.println(F("  SET_EPOCH <unix_s>"));
  BTSerial.println(F("  SYNC_BUILD"));
  BTSerial.println(F("  GET"));
  BTSerial.println(F("  LED_ON  - LED'i manuel ac"));
  BTSerial.println(F("  LED_OFF - LED'i manuel kapat"));
  BTSerial.println(F("  LED_AUTO - Otomatik moda don"));
  BTSerial.println(F("  HELP"));
}

bool parseDateTime(const String& s, int &Y, int &M, int &D, int &h, int &m, int &sec) {
  String t = s;
  t.trim();
  t.replace('/', '-');
  t.replace(',', ' ');
  
  // Birden fazla boşluk varsa tek boşluğa düşür
  while (t.indexOf("  ") >= 0) {
    t.replace("  ", " ");
  }
  
  Serial.print(F("Parsing date/time: '"));
  Serial.print(t);
  Serial.println(F("'"));
  
  // Tarih ve saat kısmını ayır
  int spacePos = t.indexOf(' ');
  if (spacePos < 0) {
    Serial.println(F("Error: No space found between date and time"));
    return false;
  }
  
  String datePart = t.substring(0, spacePos);
  String timePart = t.substring(spacePos + 1);
  
  datePart.trim();
  timePart.trim();
  
  Serial.print(F("Date part: '"));
  Serial.print(datePart);
  Serial.print(F("', Time part: '"));
  Serial.print(timePart);
  Serial.println(F("'"));
  
  // Tarih kısmını ayrıştır (YYYY-MM-DD)
  int dash1 = datePart.indexOf('-');
  int dash2 = datePart.lastIndexOf('-');
  
  if (dash1 < 0 || dash2 <= dash1) {
    Serial.println(F("Error: Invalid date format"));
    return false;
  }
  
  // Saat kısmını ayrıştır (HH:MM:SS)
  int colon1 = timePart.indexOf(':');
  int colon2 = timePart.lastIndexOf(':');
  
  if (colon1 < 0 || colon2 < colon1) {
    Serial.println(F("Error: Invalid time format"));
    return false;
  }
  
  // Değerleri ayrıştır
  String yearStr = datePart.substring(0, dash1);
  String monthStr = datePart.substring(dash1 + 1, dash2);
  String dayStr = datePart.substring(dash2 + 1);
  
  String hourStr = timePart.substring(0, colon1);
  String minuteStr = timePart.substring(colon1 + 1, colon2);
  String secondStr = timePart.substring(colon2 + 1);
  
  // String'leri sayılara çevir
  Y = yearStr.toInt();
  M = monthStr.toInt();
  D = dayStr.toInt();
  h = hourStr.toInt();
  m = minuteStr.toInt();
  sec = secondStr.toInt();
  
  // Değerleri kontrol et
  if (Y < 2000 || Y > 2099) {
    Serial.print(F("Error: Invalid year: "));
    Serial.println(Y);
    return false;
  }
  if (M < 1 || M > 12) {
    Serial.print(F("Error: Invalid month: "));
    Serial.println(M);
    return false;
  }
  if (D < 1 || D > 31) {
    Serial.print(F("Error: Invalid day: "));
    Serial.println(D);
    return false;
  }
  if (h < 0 || h > 23) {
    Serial.print(F("Error: Invalid hour: "));
    Serial.println(h);
    return false;
  }
  if (m < 0 || m > 59) {
    Serial.print(F("Error: Invalid minute: "));
    Serial.println(m);
    return false;
  }
  if (sec < 0 || sec > 59) {
    Serial.print(F("Error: Invalid second: "));
    Serial.println(sec);
    return false;
  }
  
  Serial.print(F("Parsed successfully: "));
  Serial.print(Y); Serial.print(F("-"));
  Serial.print(M); Serial.print(F("-"));
  Serial.print(D); Serial.print(F(" "));
  Serial.print(h); Serial.print(F(":"));
  Serial.print(m); Serial.print(F(":"));
  Serial.println(sec);
  
  return true;
}

void commitBtCommand(const String& raw) {
  String cmd = raw;
  cmd.trim();
  String upper = cmd;
  upper.toUpperCase();
  
  BTSerial.print(F("#CMD: "));
  BTSerial.println(upper);
  
  if (upper.startsWith("SET_TIME ")) {
    String arg = cmd.substring(9);
    int Y, M, D, h, m, s;
    if (parseDateTime(arg, Y, M, D, h, m, s)) {
      rtc.adjust(DateTime(Y, M, D, h, m, s));
      BTSerial.println(F("OK: Saat ayarlandi (SET_TIME)."));
    } else {
      BTSerial.println(F("HATA: 'SET_TIME YYYY-MM-DD HH:MM:SS'"));
    }
  } else if (upper == "SET_TIME") {
    BTSerial.println(F("TIME_REQ"));
    BTSerial.println(F("INFO: 'TIME YYYY-MM-DD HH:MM:SS' gonderin."));
  } else if (upper.startsWith("SET ")) {
    String arg = cmd.substring(4);
    int Y, M, D, h, m, s;
    if (parseDateTime(arg, Y, M, D, h, m, s)) {
      rtc.adjust(DateTime(Y, M, D, h, m, s));
      BTSerial.println(F("OK: Saat ayarlandi."));
    } else {
      BTSerial.println(F("HATA: 'SET YYYY-MM-DD HH:MM:SS'"));
    }
  } else if (upper.startsWith("TIME ")) {
    String arg = cmd.substring(5);
    int Y, M, D, h, m, s;
    if (parseDateTime(arg, Y, M, D, h, m, s)) {
      rtc.adjust(DateTime(Y, M, D, h, m, s));
      BTSerial.println(F("OK: TIME setildi."));
    } else {
      BTSerial.println(F("HATA: TIME formati."));
    }
  } else if (upper.startsWith("SET_EPOCH ")) {
    String arg = cmd.substring(10);
    String t = arg;
    t.trim();
    unsigned long secs = (unsigned long)t.toInt();
    if (secs > 946684800UL) {
      rtc.adjust(DateTime(secs));
      BTSerial.println(F("OK: Epoch ayarlandi."));
    } else {
      BTSerial.println(F("HATA: Gecersiz epoch."));
    }
  } else if (upper == "SYNC_BUILD") {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    BTSerial.println(F("OK: Derleme zamanina esitlendi."));
  } else if (upper == "GET") {
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", 
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    BTSerial.print(F("TIME "));
    BTSerial.println(buf);
  } else if (upper == "LED_ON") {
    manualLedControl = true;
    manualLedState = true;
    BTSerial.println(F("OK: LED manuel ACIK"));
  } else if (upper == "LED_OFF") {
    manualLedControl = true;
    manualLedState = false;
    BTSerial.println(F("OK: LED manuel KAPALI"));
  } else if (upper == "LED_AUTO") {
    manualLedControl = false;
    BTSerial.println(F("OK: LED otomatik moda gecti"));
  } else if (upper == "HELP") {
    btSendHelp();
  } else {
    BTSerial.println(F("Bilinmeyen komut. HELP yazin."));
  }
}

void handleBluetooth() {
  while (BTSerial.available()) {
    char ch = BTSerial.read();
    Serial.write(ch); // Bridge to Serial Monitor
    
    btLastCharMs = millis();
    if (ch == '\r' || ch == '\n') {
      if (btBuf.length() == 0) continue;
      String cmd = btBuf;
      btBuf = "";
      commitBtCommand(cmd);
    } else {
      if (btBuf.length() < 64) btBuf += ch;
    }
  }
  
  // Timeout için komut işleme
  if (btBuf.length() > 0 && (millis() - btLastCharMs) > BT_IDLE_COMMIT_MS) {
    String cmd = btBuf;
    btBuf = "";
    commitBtCommand(cmd);
  }
}

// ===================== SICAKLIK OKUMA =====================
void updateTemperature(unsigned long nowMs) {
  if (!tempRequestInFlight) {
    if (nowMs - lastTempReqMs >= TEMP_UPDATE_MS) {
      tempSensors.requestTemperatures();
      tempRequestInFlight = true;
      lastTempReqMs = nowMs;
    }
  } else {
    float t = tempSensors.getTempCByIndex(0);
    if (t > -55 && t < 125) {
      lastTempC = t;
    }
    tempRequestInFlight = false;
  }
}

// ===================== EKRAN ÇIZIMLERI =====================
void drawNormalScreen(const DateTime& dt) {
  char line1[17], line2[17];
  memset(line1, ' ', 16); line1[16] = '\0';
  memset(line2, ' ', 16); line2[16] = '\0';
  
  // 1. satır: Tarih + BT durumu
  char dateStr[11];
  snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d", dt.day(), dt.month(), dt.year());
  memcpy(line1, dateStr, 10);
  if (btConnected) {
    const char* btok = " BT:OK";
    memcpy(line1 + 10, btok, 6);
  }
  
  // 2. satır: Saat + Sıcaklık + Manuel LED göstergesi
  char timeStr[9];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
  memcpy(line2, timeStr, 8);
  
  // Sıcaklık
  if (!isnan(lastTempC)) {
    char tempStr[6];
    int t = (int)round(lastTempC);
    snprintf(tempStr, sizeof(tempStr), "%d%c", t, (char)223);
    size_t len = strnlen(tempStr, 5);
    int start = 16 - (int)len;
    if (manualLedControl) start -= 2; // Manual LED indicator için yer aç
    if (start < 9) start = 9;
    memcpy(line2 + start, tempStr, len);
  }
  
  // Manuel LED kontrolü göstergesi
  if (manualLedControl) {
    line2[14] = 'M';
    line2[15] = manualLedState ? '1' : '0';
  }
  
  lcdPrintLine(0, line1);
  lcdPrintLine(1, line2);
}

void drawAlertScreen(bool flameActive, bool gasActiveEff) {
  unsigned long now = millis();
  if (now - lastAlertBlink >= ALERT_BLINK_MS) {
    lastAlertBlink = now;
    alertVisible = !alertVisible;
  }
  if (!alertVisible) {
    lcdPrintLine(0, "                ");
    lcdPrintLine(1, "                ");
    return;
  }
  
  if (flameActive && gasActiveEff) {
    lcdPrintLine(0, "*** TEHLIKE *** ");
    lcdPrintLine(1, "ALEV + GAZ      ");
  } else if (flameActive) {
    lcdPrintLine(0, "*** TEHLIKE *** ");
    lcdPrintLine(1, "ALEV            ");
  } else if (gasActiveEff) {
    lcdPrintLine(0, "*** TEHLIKE *** ");
    lcdPrintLine(1, "GAZ             ");
  }
}

void drawBtToast() {
  lcdPrintLine(0, "Bluetooth baglandi");
}

// ===================== STATUS LOG =====================
void printStatusIfChanged(int ldrValue, bool pir1, bool pir2, bool flameActive, bool gasActiveEff, unsigned long now, const DateTime& dt) {
  static bool init = false;
  static bool pIsNight, pRelayOn, pPir1, pPir2, pFlame, pGas, pBt, pManual;
  static AlarmMode pAlarm = ALARM_NONE;
  
  bool changed = (!init) ||
                 pIsNight != isNight || pRelayOn != relayOn ||
                 pPir1 != pir1 || pPir2 != pir2 ||
                 pFlame != flameActive || pGas != gasActiveEff ||
                 pBt != btConnected || pAlarm != alarmMode ||
                 pManual != manualLedControl;
  
  if (!changed) return;
  
  char line[250];
  snprintf(line, sizeof(line),
    "[ms=%lu] %02d-%02d-%04d %02d:%02d:%02d | LDR=%d->%s | PIR1=%s | PIR2=%s | LED=%s%s | Alev=%s | GazEff=%s | BT=%s | Alarm=%s | Temp=%.1fC",
    now,
    dt.day(), dt.month(), dt.year(), dt.hour(), dt.minute(), dt.second(),
    ldrValue, isNight ? "Gece" : "Gunduz",
    pir1 ? "Hareket" : "Yok",
    pir2 ? "Hareket" : "Yok",
    relayOn ? "Acik" : "Kapali",
    manualLedControl ? "(Manuel)" : "",
    flameActive ? "TEHLIKE" : "YOK",
    gasActiveEff ? "TEHLIKE" : "YOK",
    btConnected ? "Bagli" : "Kopuk",
    alarmMode == ALARM_NONE ? "Yok" : (alarmMode == ALARM_FLAME ? "ALEV" : (alarmMode == ALARM_GAS ? "GAZ" : "BOTH")),
    isnan(lastTempC) ? NAN : lastTempC
  );
  
  Serial.println(line);
  
  init = true;
  pIsNight = isNight; pRelayOn = relayOn; pPir1 = pir1; pPir2 = pir2;
  pFlame = flameActive; pGas = gasActiveEff; pBt = btConnected; pAlarm = alarmMode;
  pManual = manualLedControl;
}

// ===================== SETUP =====================
void setup() {
  // Pin modları
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_GAS, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_SYSLED, OUTPUT);
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_BT_STATE, INPUT);
  
  setRelay(false);
  
  // Seri haberleşme
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // RTC başlatma
  Wire.begin();
  if (!rtc.begin()) {
    Serial.println(F("RTC bulunamadi! (0x68)"));
  } else {
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println(F("RTC derleme zamani ile AYARLANDI (ilk kurulum)."));
    } else {
      Serial.println(F("RTC calisiyor; saat korunuyor."));
    }
  }
  
  // LCD başlatma
  swI2C.begin();
  lcdAddr = detectLcdAddr_SW();
  if (lcdAddr == 0) {
    Serial.println(F("LCD bulunamadi (SW I2C)."));
  } else {
    lcdInit();
    if (lcdReady) {
      lcdPrintLine(0, "Sistema Baslatiliyor");
      lcdPrintLine(1, "MQ135 Isinma: 60s");
    }
  }
  
  // DS18B20 başlatma
  tempSensors.begin();
  
  // Boot melody
  bootPlaying = true;
  bootIdx = 0;
  bootNextChange = 0;
  
  BTSerial.println(F("Sistema baslatildi. HELP yazin."));
  
  bootMs = millis(); // Gaz sensörü ısınma sayacı
}

// ===================== LOOP =====================
void loop() {
  unsigned long nowMs = millis();
  
  // BT durumu kontrol
  prevBtConnected = btConnected;
  btConnected = (digitalRead(PIN_BT_STATE) == HIGH);
  if (btConnected && !prevBtConnected) {
    btToastUntil = nowMs + BT_TOAST_MS;
    if (lcdReady) drawBtToast();
    BTSerial.println(F("TIME_REQ"));
  }
  
  // Sensör okumaları
  int ldrValue = analogRead(PIN_LDR);
  bool pir1Active = readActive(PIN_PIR1, INVERT_PIR);
  bool pir2Active = readActive(PIN_PIR2, INVERT_PIR);
  bool flameActive = readActive(PIN_FLAME, INVERT_FLAME);
  bool gasRaw = readActive(PIN_GAS, INVERT_GAS);
  
  // MQ-135 warmup: ilk 60sn gazı yok say
  bool gasActiveEffective = (nowMs - bootMs >= GAS_WARMUP_MS) ? gasRaw : false;
  
  // Aydınlatma kontrolü
  handleLightingLogic(ldrValue, pir1Active, pir2Active, nowMs);
  
  // Alarm sistemi
  handleAlarms(flameActive, gasActiveEffective, nowMs);
  
  // Bluetooth komut işleme
  handleBluetooth();
  
  // Serial ↔ BT köprüsü
  while (Serial.available()) {
    BTSerial.write(Serial.read());
  }
  
  // Sıcaklık güncelleme
  updateTemperature(nowMs);
  
  // LCD güncelleme
  if (lcdReady && (nowMs - lastLcdUpdate >= LCD_UPDATE_MS)) {
    lastLcdUpdate = nowMs;
    
    if (flameActive || gasActiveEffective) {
      drawAlertScreen(flameActive, gasActiveEffective);
    } else if (btConnected && nowMs < btToastUntil) {
      drawBtToast();
    } else if (rtc.isrunning()) {
      DateTime dt = rtc.now();
      drawNormalScreen(dt);
    } else {
      lcdPrintLine(0, "RTC ayar bekliyor");
      lcdPrintLine(1, "BT: SET/TIME     ");
    }
  }
  
  // Status log (sadece değişimde)
  if (rtc.isrunning()) {
    DateTime dt = rtc.now();
    printStatusIfChanged(ldrValue, pir1Active, pir2Active, flameActive, gasActiveEffective, nowMs, dt);
  }
}