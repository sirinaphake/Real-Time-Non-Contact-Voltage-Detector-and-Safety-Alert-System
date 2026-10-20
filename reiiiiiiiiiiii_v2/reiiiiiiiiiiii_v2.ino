#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/wdt.h>

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int csPin = 10;
const int resetPin = 9;
const int irqPin = 2;

const int buzzerPin = A3;
const int ledPin1 = A0;
const int ledPin2 = A1;
const int ledPin3 = A2;
const int sw = 3;

bool lastSwState = 1;
unsigned long lastSwChangeMs = 0;
const unsigned long swDebounceMs = 50;

bool phase1On = false;
bool phase2On = false;
bool phase3On = false;

// ================== โหมดทดสอบ (TEST) ==================
bool testActive = false;
unsigned long testUntil = 0;
const unsigned long testDurationMs = 3000;   // ค้างไว้ 3 วินาที

unsigned long lastPacketTime = 0;
const unsigned long linkTimeout = 8000;

bool linkConnected = false;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateMs = 200;
bool hasDisplay = false;

int lastRssi = 0;
float lastSnr = 0;

// ================== Boot Animation ==================
void showBootAnimation() {
  if (!hasDisplay) return;

  for (int i = 0; i <= 28; i += 4) {
    display.clearDisplay();
    display.drawRoundRect(64 - (i * 2), 32 - i, i * 4, i * 2, 4, SSD1306_WHITE);
    display.display();
    delay(15);
  }

  display.clearDisplay();
  display.drawRoundRect(2, 2, 124, 60, 4, SSD1306_WHITE);

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(46, 12);
  display.print(F("PEA"));

  display.setTextSize(1);
  display.setCursor(18, 32);
  display.print(F("SAWANG DAEN DIN"));

  for (int w = 0; w <= 96; w += 8) {
    display.fillRect(16, 52, w, 3, SSD1306_WHITE);
    display.display();
    delay(30);
  }
  delay(200);
}

void drawSignalIcon(int x, int y, int bars, uint16_t color) {
  const int barW = 2, gap = 1, maxH = 8;
  if (bars <= 0) {
    display.drawLine(x, y, x + 8, y + 8, color);
    display.drawLine(x, y + 8, x + 8, y, color);
    return;
  }
  for (int i = 0; i < 4; i++) {
    int h = (i + 1) * 2;
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < bars) display.fillRect(bx, by, barW, h, color);
    else          display.drawRect(bx, by, barW, h, color);
  }
}

int rssiToBars(int rssi) {
  if (!linkConnected) return 0;
  if (rssi >= -70)  return 4;
  if (rssi >= -85)  return 3;
  if (rssi >= -100) return 2;
  return 1;
}

bool extractPhaseState(const String &data, const String &key) {
  int idx = data.indexOf(key);
  if (idx == -1) return false;
  int start = idx + key.length();
  return data.substring(start, start + 2) == "ON";
}

// ไฟ + บัซเซอร์ (โหมดทดสอบ = ติดทุกดวง)
void updateOutputs() {
  bool p1 = testActive || phase1On;
  bool p2 = testActive || phase2On;
  bool p3 = testActive || phase3On;
  digitalWrite(ledPin1, p1 ? 1 : 0);
  digitalWrite(ledPin2, p2 ? 1 : 0);
  digitalWrite(ledPin3, p3 ? 1 : 0);
  digitalWrite(buzzerPin, (p1 || p2 || p3) ? 1 : 0);
}

// ================== หน้าจอ Dashboard ==================
void updateDisplay() {
  if (!hasDisplay) return;

  display.clearDisplay();

  // ---------------- HEADER ----------------
  display.setTextSize(1);

  if (linkConnected) {
    display.fillCircle(3, 3, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(8, 0);
    display.print(F("ON"));

    display.setCursor(72, 0);
    display.print(lastRssi);
    display.print(F("dBm"));

    drawSignalIcon(SCREEN_WIDTH - 12, 0, rssiToBars(lastRssi), SSD1306_WHITE);
  } else {
    display.drawCircle(3, 3, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(8, 0);
    display.print(F("OFF"));

    drawSignalIcon(SCREEN_WIDTH - 12, 0, 0, SSD1306_WHITE);
  }

  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);

  // ---------------- PHASE CARDS ----------------
  const bool phases[3] = { testActive || phase1On,
                           testActive || phase2On,
                           testActive || phase3On };
  const char* labels[3] = { "L1", "L2", "L3" };

  for (int i = 0; i < 3; i++) {
    int x = i * 43;
    int y = 14;
    int w = 40;
    int h = 48;
    int radius = 4;

    if (phases[i]) {
      display.fillRoundRect(x, y, w, h, radius, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);

      display.setTextSize(1);
      display.setCursor(x + 14, y + 6);
      display.print(labels[i]);

      display.setTextSize(2);
      display.setCursor(x + 8, y + 24);
      display.print(F("ON"));
    } else {
      display.drawRoundRect(x, y, w, h, radius, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);

      display.setTextSize(1);
      display.setCursor(x + 14, y + 6);
      display.print(labels[i]);

      display.setCursor(x + 11, y + 26);
      display.print(F("OFF"));
    }
  }

  display.display();
  lastDisplayUpdate = millis();
}

// ================== รับคำสั่ง TEST ==================
void runLinkTestFeedback() {
  testActive = true;
  testUntil = millis() + testDurationMs;
  updateOutputs();
  updateDisplay();
}

void handlePacket(const String &data) {
  if (data.startsWith("TEST")) {
    runLinkTestFeedback();
    return;
  }
  if (data.startsWith("STATUS")) {
    phase1On = extractPhaseState(data, "V1:");
    phase2On = extractPhaseState(data, "V2:");
    phase3On = extractPhaseState(data, "V3:");
  }
  else if (data == "VOLTAGE1_ON")  phase1On = true;
  else if (data == "VOLTAGE1_OFF") phase1On = false;
  else if (data == "VOLTAGE2_ON")  phase2On = true;
  else if (data == "VOLTAGE2_OFF") phase2On = false;
  else if (data == "VOLTAGE3_ON")  phase3On = true;
  else if (data == "VOLTAGE3_OFF") phase3On = false;

  updateOutputs();
  updateDisplay();
}

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(9600);
  pinMode(sw, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  digitalWrite(buzzerPin, 0);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    hasDisplay = false;
  } else {
    hasDisplay = true;
    showBootAnimation();
  }

  LoRa.setPins(csPin, resetPin, irqPin);

  bool started = false;
  for (int i = 0; i < 5; i++) {
    if (LoRa.begin(433E6)) {
      started = true;
      break;
    }
    delay(300);
  }

  if (!started) {
    while (1);
  }

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  updateDisplay();
}

void loop() {
  // ---------------- ปุ่ม RESET ----------------
  bool swReading = digitalRead(sw);
  if (swReading != lastSwState) {
    lastSwChangeMs = millis();
  }

  if (millis() - lastSwChangeMs > swDebounceMs) {
    if (swReading == LOW) {
      wdt_enable(WDTO_15MS);
      while (1);
    }
  }
  lastSwState = swReading;

  // ---------------- รับแพ็กเก็ต LoRa ----------------
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String data = "";
    while (LoRa.available()) {
      data += (char)LoRa.read();
    }

    lastRssi = LoRa.packetRssi();
    lastSnr = LoRa.packetSnr();
    lastPacketTime = millis();
    linkConnected = true;

    handlePacket(data);
  }

  // ---------------- Link Timeout ----------------
  if (lastPacketTime != 0 && millis() - lastPacketTime > linkTimeout) {
    if (linkConnected) {
      linkConnected = false;
    }
    if (phase1On || phase2On || phase3On) {
      phase1On = phase2On = phase3On = false;
      updateOutputs();
    }
  }

  // ---------------- จบโหมดทดสอบ ----------------
  if (testActive && millis() > testUntil) {
    testActive = false;
    updateOutputs();
    updateDisplay();
  }

  // ---------------- อัปเดตหน้าจอเป็นระยะ ----------------
  if (millis() - lastDisplayUpdate >= displayUpdateMs) {
    updateDisplay();
  }
}