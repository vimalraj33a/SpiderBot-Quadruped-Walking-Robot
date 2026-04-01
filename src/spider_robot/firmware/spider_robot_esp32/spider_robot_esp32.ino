/*
 * spider_robot_esp32.ino  — FIXED VERSION with debug
 * ====================================================
 * Board : DOIT ESP32 DEVKIT V1
 * SDA   : GPIO 21
 * SCL   : GPIO 22
 * Baud  : 115200
 *
 * Receives CSV from ROS2 hardware_bridge.py:
 *   fr1,fr2,fl1,fl2,rr1,rr2,rl1,rl2  (degrees, -90 to +90)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ── Servo pulse calibration ────────────────────────────────
// Tune SERVO_MIN / SERVO_MAX if servos don't reach full range
#define SERVO_MIN  102    // -90 deg  ~0.5ms pulse
#define SERVO_MID  307    //   0 deg  ~1.5ms pulse
#define SERVO_MAX  512    // +90 deg  ~2.5ms pulse

// ── Channel → joint mapping ───────────────────────────────
// CH0=fr1  CH1=fr2  CH2=fl1  CH3=fl2
// CH4=rr1  CH5=rr2  CH6=rl1  CH7=rl2

// ── State ─────────────────────────────────────────────────
String  rxBuf       = "";
bool    pca_ready   = false;
uint32_t lastRx     = 0;
uint32_t pktCount   = 0;
const uint32_t WATCHDOG_MS = 3000;   // return to stand if no data

// Standing pose in degrees
const float STAND[8] = {0, 17.2, 0, 17.2, 0, 17.2, 0, 17.2};
float current[8]     = {0, 0, 0, 0, 0, 0, 0, 0};

// ── Helpers ───────────────────────────────────────────────
int degToPulse(float deg) {
  deg = constrain(deg, -90.0f, 90.0f);
  float t = (deg + 90.0f) / 180.0f;          // 0.0 → 1.0
  return (int)(SERVO_MIN + t * (SERVO_MAX - SERVO_MIN));
}

void setAllServos(const float deg[8]) {
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(i, 0, degToPulse(deg[i]));
    current[i] = deg[i];
  }
}

// Smooth move to target over ~ms milliseconds
void smoothMove(const float target[8], int ms = 800, int steps = 20) {
  float start[8];
  for (int i = 0; i < 8; i++) start[i] = current[i];
  int d = ms / steps;
  for (int s = 1; s <= steps; s++) {
    float t = (float)s / steps;
    float interp[8];
    for (int i = 0; i < 8; i++)
      interp[i] = start[i] + t * (target[i] - start[i]);
    setAllServos(interp);
    delay(d);
  }
}

// Parse "a,b,c,d,e,f,g,h\n" into float[8]
// Returns true if exactly 8 values parsed
bool parseCSV(const String &line, float out[8]) {
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  int idx = 0;
  int pos = 0;
  while (idx < 8 && pos <= (int)s.length()) {
    int comma = s.indexOf(',', pos);
    String token;
    if (comma == -1) {
      token = s.substring(pos);
      pos   = s.length() + 1;
    } else {
      token = s.substring(pos, comma);
      pos   = comma + 1;
    }
    token.trim();
    if (token.length() == 0) return false;
    out[idx++] = token.toFloat();
  }
  return (idx == 8);
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(500);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.println("\n==============================");
  Serial.println(" SpiderBot ESP32 — Fixed FW");
  Serial.println("==============================");

  // Init I2C + PCA9685
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(100);
  pca_ready = true;
  Serial.println("[OK] PCA9685 ready at 0x40");

  // Startup: zero then smooth to stand
  float zeros[8] = {0,0,0,0,0,0,0,0};
  setAllServos(zeros);
  delay(300);
  Serial.println("[OK] Moving to stand pose...");
  smoothMove(STAND, 1200, 30);
  Serial.println("[OK] Standing. Waiting for ROS2 data on Serial...");
  Serial.println("     Format: fr1,fr2,fl1,fl2,rr1,rr2,rl1,rl2 (degrees)");
  Serial.println("==============================\n");

  lastRx = millis();
  digitalWrite(2, HIGH);
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  // ── Read incoming serial bytes ────────────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      // Got a complete line — try to parse
      String line = rxBuf;
      rxBuf = "";

      if (line.length() < 5) continue;   // ignore empty/short lines

      float angles[8];
      if (parseCSV(line, angles)) {
        // Valid packet received
        setAllServos(angles);
        lastRx  = millis();
        pktCount++;

        // Blink LED on every packet
        digitalWrite(2, LOW);
        delayMicroseconds(800);
        digitalWrite(2, HIGH);

        // Print debug every 20 packets (~1 second at 20Hz)
        if (pktCount % 20 == 0) {
          Serial.print("[RX #"); Serial.print(pktCount);
          Serial.print("] fr1="); Serial.print(angles[0], 1);
          Serial.print(" fr2="); Serial.print(angles[1], 1);
          Serial.print(" fl1="); Serial.print(angles[2], 1);
          Serial.print(" fl2="); Serial.print(angles[3], 1);
          Serial.print(" rr1="); Serial.print(angles[4], 1);
          Serial.print(" rr2="); Serial.print(angles[5], 1);
          Serial.print(" rl1="); Serial.print(angles[6], 1);
          Serial.print(" rl2="); Serial.println(angles[7], 1);
        }
      } else {
        // Bad packet — print for debug
        Serial.print("[ERR] Bad packet: '");
        Serial.print(line);
        Serial.println("'");
      }

    } else if (c != '\r') {
      rxBuf += c;
      if (rxBuf.length() > 200) rxBuf = "";  // overflow guard
    }
  }

  // ── Watchdog ─────────────────────────────────────────
  if ((millis() - lastRx) > WATCHDOG_MS && pktCount > 0) {
    static bool warned = false;
    if (!warned) {
      Serial.println("[WARN] No data for 3s — returning to stand");
      smoothMove(STAND, 800, 20);
      warned = true;
    }
    // Slow blink = waiting
    digitalWrite(2, (millis() / 600) % 2);
  } else if ((millis() - lastRx) < WATCHDOG_MS) {
    static bool warned = false;
    warned = false;
  }
}