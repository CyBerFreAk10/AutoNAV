#include <Wire.h>

#define ICM_ADDR 0x69

// ================= ICM-20948 =================

#define REG_BANK_SEL        0x7F
#define WHO_AM_I            0x00
#define PWR_MGMT_1          0x06
#define PWR_MGMT_2          0x07
#define INT_ENABLE_1        0x11
#define INT_STATUS_1        0x1A
#define ACCEL_XOUT_H        0x2D

#define GYRO_SMPLRT_DIV     0x00
#define GYRO_CONFIG_1       0x01
#define ACCEL_SMPLRT_DIV_1  0x10
#define ACCEL_SMPLRT_DIV_2  0x11
#define ACCEL_CONFIG        0x14

const float ACC_SCALE = 1.0f / 16384.0f;
const float GYR_SCALE = 1.0f / 65.5f;
const int   CAL_SAMPLES = 2000;

float gxBias = 0, gyBias = 0, gzBias = 0;

unsigned long lastMicros = 0;
float yaw = 0.0f;

bool imuOk    = false;
bool streamOn = true;


// ================= L298N =================
//
// D25 -> ENA   D26 -> IN1   D27 -> IN2    (motor A = left side)
// D33 -> IN3   D32 -> IN4   D14 -> ENB    (motor B = right side)
//
// ENA/ENB jumpers on the L298N board must be removed for speed control.

#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 33
#define IN4 32
#define ENB 14

int driveSpeed = 200;              // PWM 0-255 used by w/a/s/d and test
int curL = 0, curR = 0;            // last applied signed PWM per side

unsigned long moveStart = 0;       // timed-move bookkeeping
unsigned long moveDur   = 0;       // 0 = run until another command

unsigned long wdTimeout    = 0;    // ms, 0 = watchdog off
unsigned long lastMotorCmd = 0;

bool testMode = false;
int  testStep = -1;
unsigned long testStepStart = 0;


// ---- low level ----

// spd: -255..255. Positive = forward. 0 = coast.
void setChannel(uint8_t en, uint8_t inA, uint8_t inB, int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(inA, spd > 0 ? HIGH : LOW);
  digitalWrite(inB, spd < 0 ? HIGH : LOW);
  analogWrite(en, abs(spd));
}

void drive(int left, int right) {
  curL = constrain(left,  -255, 255);
  curR = constrain(right, -255, 255);
  setChannel(ENA, IN1, IN2, curL);
  setChannel(ENB, IN3, IN4, curR);
}

void motorStop() {
  drive(0, 0);
  moveDur = 0;
}

void startMove(int l, int r, long ms, const char *name) {
  testMode = false;
  drive(l, r);
  moveStart    = millis();
  moveDur      = ms > 0 ? (unsigned long)ms : 0;
  lastMotorCmd = moveStart;

  if (moveDur)
    Serial.printf("# MOTOR %s L=%d R=%d for %lu ms\n", name, curL, curR, moveDur);
  else
    Serial.printf("# MOTOR %s L=%d R=%d\n", name, curL, curR);
}


// ================= MOTOR TEST (same sequence as before) =================

struct TestStep { int l, r; unsigned long ms; const char *name; };

const TestStep TEST_SEQ[] = {
  {  1,  1, 3000, "FORWARD"  },
  {  0,  0, 2000, "STOP"     },
  { -1, -1, 3000, "BACKWARD" },
  {  0,  0, 2000, "STOP"     },
  { -1,  1, 2000, "LEFT"     },
  {  0,  0, 2000, "STOP"     },
  {  1, -1, 2000, "RIGHT"    },
  {  0,  0, 3000, "STOP"     },
};
const int TEST_COUNT = sizeof(TEST_SEQ) / sizeof(TEST_SEQ[0]);

void motorTest() {
  unsigned long now = millis();

  if (testStep < 0 || now - testStepStart >= TEST_SEQ[testStep].ms) {
    testStep = (testStep + 1) % TEST_COUNT;
    const TestStep &s = TEST_SEQ[testStep];
    drive(s.l * driveSpeed, s.r * driveSpeed);
    testStepStart = now;
    Serial.printf("# TEST %s\n", s.name);
  }
}

void updateMotors() {
  if (testMode) {
    motorTest();
    return;
  }

  unsigned long now = millis();

  if (moveDur && now - moveStart >= moveDur) {
    motorStop();
    Serial.println("# MOTOR timed move done -> STOP");
  }

  // Timed moves stop themselves, so the watchdog only guards open-ended ones
  if (wdTimeout && !moveDur && (curL || curR) && now - lastMotorCmd >= wdTimeout) {
    motorStop();
    Serial.println("# MOTOR watchdog -> STOP");
  }
}


// ================= ICM-20948 FUNCTIONS =================

void wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t rd(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void bank(uint8_t b) {
  wr(REG_BANK_SEL, (b & 0x03) << 4);
}

bool initIMU() {
  bank(0);

  uint8_t id = rd(WHO_AM_I);
  if (id != 0xEA) {
    Serial.printf("# WHO_AM_I = 0x%02X, expected 0xEA\n", id);
    return false;
  }

  wr(PWR_MGMT_1, 0x80);
  delay(100);

  bank(0);
  wr(PWR_MGMT_1, 0x01);
  delay(50);

  wr(PWR_MGMT_2, 0x00);

  bank(2);
  wr(GYRO_SMPLRT_DIV, 10);
  wr(GYRO_CONFIG_1, 0x1B);
  wr(ACCEL_SMPLRT_DIV_1, 0);
  wr(ACCEL_SMPLRT_DIV_2, 10);
  wr(ACCEL_CONFIG, 0x19);

  bank(0);
  wr(INT_ENABLE_1, 0x01);

  delay(100);
  return true;
}

void dumpConfig() {
  bank(2);
  Serial.printf(
    "# GYRO_SMPLRT_DIV=%u GYRO_CONFIG_1=0x%02X "
    "ACCEL_SMPLRT_DIV_2=%u ACCEL_CONFIG=0x%02X\n",
    rd(GYRO_SMPLRT_DIV), rd(GYRO_CONFIG_1),
    rd(ACCEL_SMPLRT_DIV_2), rd(ACCEL_CONFIG));
  bank(0);
  Serial.println("# expect: 10, 0x1B, 10, 0x19");
}

bool dataReady() {
  return rd(INT_STATUS_1) & 0x01;
}

void readAGT(float *a, float *g, float *t) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)14);

  if (Wire.available() < 14)
    return;

  int16_t raw[7];
  for (int i = 0; i < 7; i++)
    raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());

  for (int i = 0; i < 3; i++) a[i] = raw[i] * ACC_SCALE;
  for (int i = 0; i < 3; i++) g[i] = raw[i + 3] * GYR_SCALE;

  *t = (raw[6] / 333.87f) + 21.0f;
}

void calibrateGyro() {
  double sx = 0, sy = 0, sz = 0;
  float a[3] = {0}, g[3] = {0}, t = 0;

  unsigned long deadline = millis() + 40000UL;
  Serial.printf("# calibrating %d samples, keep still\n", CAL_SAMPLES);

  int n = 0;
  while (n < CAL_SAMPLES && millis() < deadline) {
    if (dataReady()) {
      readAGT(a, g, &t);
      sx += g[0];
      sy += g[1];
      sz += g[2];
      n++;
    }
  }

  if (n == 0) {
    Serial.println("# no samples - dataReady never fired, check INT_ENABLE_1");
    return;
  }
  if (n < CAL_SAMPLES)
    Serial.printf("# timeout, only %d samples\n", n);

  gxBias = sx / n;
  gyBias = sy / n;
  gzBias = sz / n;

  Serial.printf("# bias dps: %.4f, %.4f, %.4f\n", gxBias, gyBias, gzBias);
}


// ================= SERIAL COMMANDS =================

void printHelp() {
  Serial.println(
    "# ---- commands (terminate with Enter) ----\n"
    "# w|f [ms]    forward        s|b [ms]   backward\n"
    "# a|l [ms]    spin left      d|r [ms]   spin right\n"
    "# x|stop      stop (coast)\n"
    "# m <L> <R>   raw signed PWM per side, -255..255\n"
    "# v [0-255]   set/show speed used by w/a/s/d/test\n"
    "# wd <ms>     stop if no motor cmd within ms (0 = off)\n"
    "# test        start/stop scripted motor test\n"
    "# q           toggle IMU CSV stream\n"
    "# z           zero yaw\n"
    "# cal         stop motors + recalibrate gyro\n"
    "# h|?         this help");
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t,");
  if (!cmd) return;
  for (char *p = cmd; *p; ++p) *p = tolower((unsigned char)*p);

  char *a1 = strtok(NULL, " \t,");
  char *a2 = strtok(NULL, " \t,");
  long n1 = a1 ? strtol(a1, NULL, 10) : 0;
  long n2 = a2 ? strtol(a2, NULL, 10) : 0;

  int v = driveSpeed;
  auto is = [cmd](const char *s) { return strcmp(cmd, s) == 0; };

  if      (is("w") || is("f")) startMove( v,  v, n1, "FORWARD");
  else if (is("s") || is("b")) startMove(-v, -v, n1, "BACKWARD");
  else if (is("a") || is("l")) startMove(-v,  v, n1, "LEFT");
  else if (is("d") || is("r")) startMove( v, -v, n1, "RIGHT");

  else if (is("x") || is("stop")) {
    testMode = false;
    motorStop();
    lastMotorCmd = millis();
    Serial.println("# MOTOR STOP");
  }

  else if (is("m")) {
    if (!a1 || !a2) Serial.println("# usage: m <left> <right>  (-255..255)");
    else            startMove((int)n1, (int)n2, 0, "RAW");
  }

  else if (is("v")) {
    if (a1) driveSpeed = constrain((int)n1, 0, 255);
    Serial.printf("# speed = %d (applies to next w/a/s/d)\n", driveSpeed);
  }

  else if (is("wd")) {
    if (a1) wdTimeout = n1 > 0 ? (unsigned long)n1 : 0;
    lastMotorCmd = millis();
    Serial.printf("# watchdog = %lu ms%s\n", wdTimeout, wdTimeout ? "" : " (off)");
  }

  else if (is("test")) {
    testMode = !testMode;
    if (testMode) {
      testStep = -1;
      Serial.println("# TEST start (any motion cmd or x ends it)");
    } else {
      motorStop();
      Serial.println("# TEST stop");
    }
  }

  else if (is("q")) {
    streamOn = !streamOn;
    Serial.printf("# stream %s\n", streamOn ? "ON" : "OFF");
  }

  else if (is("z")) {
    yaw = 0;
    Serial.println("# yaw zeroed");
  }

  else if (is("cal")) {
    if (!imuOk) { Serial.println("# no IMU"); return; }
    testMode = false;
    motorStop();
    calibrateGyro();
    yaw = 0;
    lastMicros = micros();   // avoid a huge dt on the next sample
  }

  else if (is("h") || is("?") || is("help")) printHelp();

  else Serial.printf("# unknown command '%s' - type h for help\n", cmd);
}

char   cmdBuf[64];
size_t cmdLen = 0;

// Non-blocking line reader. Accepts CR, LF or CRLF endings.
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}


// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  motorStop();

  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  Serial.println("\n# ESP32 + ICM-20948 + L298N");

  imuOk = initIMU();
  if (imuOk) {
    dumpConfig();
    calibrateGyro();
    Serial.println("# IMU calibration complete");
  } else {
    Serial.println("# IMU init failed - motor commands still available");
  }

  printHelp();
  Serial.println("# t_ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,temp_C,yaw_deg");

  lastMicros = micros();
}


// ================= LOOP =================

void loop() {
  pollSerial();

  if (imuOk && dataReady()) {
    float a[3] = {0}, g[3] = {0}, t = 0;
    readAGT(a, g, &t);

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    float gx = g[0] - gxBias;
    float gy = g[1] - gyBias;
    float gz = g[2] - gzBias;

    yaw += gz * dt;   // keeps integrating even with stream off

    if (streamOn) {
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.3f\n",
                    now / 1000,
                    a[0] * 1000.0f, a[1] * 1000.0f, a[2] * 1000.0f,
                    gx, gy, gz, t, yaw);
    }
  }

  updateMotors();
}
