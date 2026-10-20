#include <SPI.h>
#include <LoRa.h>

const int sensorPin  = 6;   // PD6 -> PCINT22
const int sensorPin2 = 5;   // PD5 -> PCINT21
const int sensorPin3 = 4;   // PD4 -> PCINT20
const int redLedPin  = A0;
const int redLedPin2 = A1;
const int redLedPin3 = A2;
const int swtest     = A3;  // ปุ่มต่อระหว่าง A3 กับ GND (INPUT_PULLUP)

// ================== LoRa RA-02 (SX1278) ==================
const int csPin = 10;
const int resetPin = 9;
const int irqPin = 2;

unsigned long lastSendTime = 0;
const unsigned long sendIntervalMs = 2500;

// ---------------- ช่อง 1 (pin 6) ----------------
volatile unsigned long lastEdgeMicros = 0;
volatile unsigned long capturedPeriod = 0;
volatile bool pulseFlag = false;
unsigned long lastValidPulseTime = 0;
int consecutiveValid = 0;
unsigned long lastAcceptedPeriod = 0;
bool isVoltageDetected = false;
String detectedMode = "";

// ---------------- ช่อง 2 (pin 5) ----------------
volatile unsigned long lastEdgeMicros2 = 0;
volatile unsigned long capturedPeriod2 = 0;
volatile bool pulseFlag2 = false;
unsigned long lastValidPulseTime2 = 0;
int consecutiveValid2 = 0;
unsigned long lastAcceptedPeriod2 = 0;
bool isVoltageDetected2 = false;
String detectedMode2 = "";

// ---------------- ช่อง 3 (pin 4) ----------------
volatile unsigned long lastEdgeMicros3 = 0;
volatile unsigned long capturedPeriod3 = 0;
volatile bool pulseFlag3 = false;
unsigned long lastValidPulseTime3 = 0;
int consecutiveValid3 = 0;
unsigned long lastAcceptedPeriod3 = 0;
bool isVoltageDetected3 = false;
String detectedMode3 = "";

volatile uint8_t lastPortDState = 0;

const unsigned long noVoltageTimeout = 300;

const unsigned long half_minUs = 14000;
const unsigned long half_maxUs = 23000;
const unsigned long full_minUs = 7000;
const unsigned long full_maxUs = 11500;

const unsigned long debounceUs = 1500;
const int requiredConsecutive = 2;

// ---------------- ปุ่ม swtest (A3) ----------------
bool lastSwState = HIGH;
unsigned long lastSwChangeTime = 0;
const unsigned long swDebounceMs = 50;

void runSelfTest() {
  Serial.println("[SWTEST] Self-test triggered!");

  // ไฟฝั่งส่งติดทั้ง 3 ดวงทันทีที่กด
  digitalWrite(redLedPin,  HIGH);
  digitalWrite(redLedPin2, HIGH);
  digitalWrite(redLedPin3, HIGH);

  // ส่ง TEST แบบ blocking (รอส่งจบเอง ~2 วินาทีที่ SF12)
  // ถ้าวิทยุยังส่งแพ็กเก็ตก่อนหน้าอยู่ beginPacket() จะคืน false -> รอแล้วลองใหม่
  bool sent = false;
  for (int i = 0; i < 20 && !sent; i++) {
    if (LoRa.beginPacket()) {
      LoRa.print("TEST,SWTEST_PRESSED");
      LoRa.endPacket();          // blocking: รอจนส่งเสร็จ
      sent = true;
      Serial.println("[LoRa TX] TEST,SWTEST_PRESSED");
    } else {
      delay(200);
    }
  }
  if (!sent) Serial.println("[LoRa TX] TEST failed");

  // ให้ไฟฝั่งส่งค้างต่ออีกเล็กน้อย
  delay(1000);

  digitalWrite(redLedPin,  isVoltageDetected  ? HIGH : LOW);
  digitalWrite(redLedPin2, isVoltageDetected2 ? HIGH : LOW);
  digitalWrite(redLedPin3, isVoltageDetected3 ? HIGH : LOW);

  lastSendTime = millis();   // กันส่ง STATUS ซ้อนทันทีหลังทดสอบ
}

void checkSwTest() {
  bool reading = digitalRead(swtest);

  if (reading != lastSwState) {
    lastSwChangeTime = millis();
  }

  if ((millis() - lastSwChangeTime) > swDebounceMs) {
    static bool stableState = HIGH;
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        runSelfTest();
      }
    }
  }

  lastSwState = reading;
}

void setupPCINT() {
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT22);
  PCMSK2 |= (1 << PCINT21);
  PCMSK2 |= (1 << PCINT20);
  lastPortDState = PIND;
}

ISR(PCINT2_vect) {
  uint8_t currentPortD = PIND;
  uint8_t changed = currentPortD ^ lastPortDState;

  // ช่อง 1 (PD6)
  if (changed & (1 << PORTD6)) {
    bool cur  = (currentPortD  >> PORTD6) & 1;
    bool prev = (lastPortDState >> PORTD6) & 1;
    if (cur == HIGH && prev == LOW) {
      unsigned long now = micros();
      unsigned long diff = now - lastEdgeMicros;
      if (diff >= debounceUs) {
        lastEdgeMicros = now;
        capturedPeriod = diff;
        pulseFlag = true;
      }
    }
  }

  // ช่อง 2 (PD5)  <-- แก้จาก PORTD3 เป็น PORTD5
  if (changed & (1 << PORTD5)) {
    bool cur  = (currentPortD  >> PORTD5) & 1;
    bool prev = (lastPortDState >> PORTD5) & 1;
    if (cur == HIGH && prev == LOW) {
      unsigned long now = micros();
      unsigned long diff = now - lastEdgeMicros2;
      if (diff >= debounceUs) {
        lastEdgeMicros2 = now;
        capturedPeriod2 = diff;
        pulseFlag2 = true;
      }
    }
  }

  // ช่อง 3 (PD4)
  if (changed & (1 << PORTD4)) {
    bool cur  = (currentPortD  >> PORTD4) & 1;
    bool prev = (lastPortDState >> PORTD4) & 1;
    if (cur == HIGH && prev == LOW) {
      unsigned long now = micros();
      unsigned long diff = now - lastEdgeMicros3;
      if (diff >= debounceUs) {
        lastEdgeMicros3 = now;
        capturedPeriod3 = diff;
        pulseFlag3 = true;
      }
    }
  }

  lastPortDState = currentPortD;
}

bool checkPeriod(unsigned long period, String &modeOut) {
  if (period >= half_minUs && period <= half_maxUs) { modeOut = "1 pulse/cycle"; return true; }
  if (period >= full_minUs && period <= full_maxUs) { modeOut = "2 pulses/cycle"; return true; }
  return false;
}

bool isConsistent(unsigned long period, unsigned long lastAccepted, float tolerancePct) {
  if (lastAccepted == 0) return true;
  bool curHalf  = (period       >= half_minUs && period       <= half_maxUs);
  bool prevHalf = (lastAccepted >= half_minUs && lastAccepted <= half_maxUs);
  if (curHalf != prevHalf) return true;
  long diff = (long)period - (long)lastAccepted;
  if (diff < 0) diff = -diff;
  long allowed = (long)((float)lastAccepted * tolerancePct);
  return diff <= allowed;
}

void sendStatusPacket() {
  String packet = "STATUS,V1:";
  packet += isVoltageDetected  ? "ON" : "OFF";
  packet += ",V2:";
  packet += isVoltageDetected2 ? "ON" : "OFF";
  packet += ",V3:";
  packet += isVoltageDetected3 ? "ON" : "OFF";

  if (!LoRa.beginPacket()) return;
  LoRa.print(packet);
  LoRa.endPacket(true);

  Serial.print("[LoRa TX] ");
  Serial.println(packet);
}

void setupLoRa() {
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
    Serial.println("LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  Serial.println("LoRa Ready!");
}

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(redLedPin2, OUTPUT);
  pinMode(redLedPin3, OUTPUT);
  pinMode(swtest, INPUT_PULLUP);
  Serial.begin(9600);
  while (!Serial) delay(10);
  setupPCINT();
  setupLoRa();
  Serial.println("Transmitter Ready! Monitoring line voltage...");
}

void loop() {
  checkSwTest();

  // ---------- ช่อง 1 ----------
  if (pulseFlag) {
    noInterrupts();
    unsigned long period = capturedPeriod;
    pulseFlag = false;
    interrupts();

    Serial.print("[CH1] Period(us): ");
    Serial.println(period);

    String mode;
    if (checkPeriod(period, mode) && isConsistent(period, lastAcceptedPeriod, 0.30)) {
      consecutiveValid++;
      lastAcceptedPeriod = period;
      lastValidPulseTime = millis();
      detectedMode = mode;
      if (consecutiveValid >= requiredConsecutive && !isVoltageDetected) {
        isVoltageDetected = true;
        digitalWrite(redLedPin, HIGH);
        Serial.print("[CH1] Voltage Detected! Mode: ");
        Serial.println(detectedMode);
      }
    } else {
      if (consecutiveValid > 0) consecutiveValid--;
    }
  }
  if (millis() - lastValidPulseTime > noVoltageTimeout) {
    if (isVoltageDetected) {
      isVoltageDetected = false;
      digitalWrite(redLedPin, LOW);
      Serial.println("[CH1] No Voltage");
    }
    consecutiveValid = 0;
    lastAcceptedPeriod = 0;
  }

  // ---------- ช่อง 2 ----------
  if (pulseFlag2) {
    noInterrupts();
    unsigned long period2 = capturedPeriod2;
    pulseFlag2 = false;
    interrupts();

    Serial.print("[CH2] Period(us): ");
    Serial.println(period2);

    String mode2;
    if (checkPeriod(period2, mode2) && isConsistent(period2, lastAcceptedPeriod2, 0.30)) {
      consecutiveValid2++;
      lastAcceptedPeriod2 = period2;
      lastValidPulseTime2 = millis();
      detectedMode2 = mode2;
      if (consecutiveValid2 >= requiredConsecutive && !isVoltageDetected2) {
        isVoltageDetected2 = true;
        digitalWrite(redLedPin2, HIGH);
        Serial.print("[CH2] Voltage Detected! Mode: ");
        Serial.println(detectedMode2);
      }
    } else {
      if (consecutiveValid2 > 0) consecutiveValid2--;
    }
  }
  if (millis() - lastValidPulseTime2 > noVoltageTimeout) {
    if (isVoltageDetected2) {
      isVoltageDetected2 = false;
      digitalWrite(redLedPin2, LOW);
      Serial.println("[CH2] No Voltage");
    }
    consecutiveValid2 = 0;
    lastAcceptedPeriod2 = 0;
  }

  // ---------- ช่อง 3 ----------
  if (pulseFlag3) {
    noInterrupts();
    unsigned long period3 = capturedPeriod3;
    pulseFlag3 = false;
    interrupts();

    Serial.print("[CH3] Period(us): ");
    Serial.println(period3);

    String mode3;
    if (checkPeriod(period3, mode3) && isConsistent(period3, lastAcceptedPeriod3, 0.30)) {
      consecutiveValid3++;
      lastAcceptedPeriod3 = period3;
      lastValidPulseTime3 = millis();
      detectedMode3 = mode3;
      if (consecutiveValid3 >= requiredConsecutive && !isVoltageDetected3) {
        isVoltageDetected3 = true;
        digitalWrite(redLedPin3, HIGH);
        Serial.print("[CH3] Voltage Detected! Mode: ");
        Serial.println(detectedMode3);
      }
    } else {
      if (consecutiveValid3 > 0) consecutiveValid3--;
    }
  }
  if (millis() - lastValidPulseTime3 > noVoltageTimeout) {
    if (isVoltageDetected3) {
      isVoltageDetected3 = false;
      digitalWrite(redLedPin3, LOW);
      Serial.println("[CH3] No Voltage");
    }
    consecutiveValid3 = 0;
    lastAcceptedPeriod3 = 0;
  }

  // ---------- ส่งสถานะเป็นระยะ ----------
  if (millis() - lastSendTime >= sendIntervalMs) {
    lastSendTime = millis();
    sendStatusPacket();
  }
}