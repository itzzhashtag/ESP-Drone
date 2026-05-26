// ================================================================
//  Q450 — Motor & MPU Test Sketch  (STANDALONE)
//  Flash this separately to test hardware before flying.
//  This sketch has NO flight logic — it only lets you spin
//  motors one at a time and read the MPU-6050.
//
//  !! REMOVE ALL PROPELLERS BEFORE USING !!
//
//  Pins used:
//    GPIO 25 → ESC M1  (Front-Left)
//    GPIO 26 → ESC M2  (Front-Right)
//    GPIO 27 → ESC M3  (Back-Right)
//    GPIO 14 → ESC M4  (Back-Left)
//    GPIO 21 → MPU-6050 SDA
//    GPIO 22 → MPU-6050 SCL
//    GPIO  2 → LED
//
//  HOW TO USE:
//    1. Flash this sketch (NOT the flight controller sketch).
//    2. Open Serial Monitor at 115200 baud.
//    3. Type commands and press Enter.
//    4. When done, re-flash the flight controller sketch.
//
//  COMMANDS:
//    m1 <µs>      Spin Motor 1 at given µs  (1000–2000)
//    m2 <µs>      Spin Motor 2
//    m3 <µs>      Spin Motor 3
//    m4 <µs>      Spin Motor 4
//    all <µs>     All motors to same speed
//    stop         All motors → 1000 µs  (off)
//    ramp <m> <start> <end> <step>   Slowly ramp one motor
//                 e.g.  ramp m1 1000 1400 10
//    mpu          Stream 100 MPU-6050 readings
//    orient       Orientation check — tilt craft, see which way
//    imu reset    Re-capture ground reference (average 50 reads)
//    status       Print current motor µs and ground reference
//    help         Show this command list
// ================================================================

#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>   // "MPU6050_light" by rfetick

// ── Pin definitions ───────────────────────────────────────────
#define PIN_M1   25
#define PIN_M2   26
#define PIN_M3   27
#define PIN_M4   14
#define PIN_LED   2
#define MPU_SDA  21
#define MPU_SCL  22

// ── ESC limits ────────────────────────────────────────────────
#define ESC_MIN  1000
#define ESC_MAX  2000

// ── IMU orientation flips ─────────────────────────────────────
// Change to -1.0 if an axis reads backwards on your mount.
#define PITCH_FLIP  1.0f
#define ROLL_FLIP   1.0f
#define YAW_FLIP    1.0f

// ── Objects ───────────────────────────────────────────────────
Servo   esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// ── Current motor values (displayed by "status") ──────────────
int curM1 = ESC_MIN, curM2 = ESC_MIN;
int curM3 = ESC_MIN, curM4 = ESC_MIN;

// ── IMU ground reference ──────────────────────────────────────
float groundRoll = 0, groundPitch = 0;


// ================================================================
//  MOTOR HELPERS
// ================================================================

void setMotor(int motor, int us)
{
  us = constrain(us, ESC_MIN, ESC_MAX);
  switch (motor)
  {
    case 1: esc1.writeMicroseconds(us); curM1 = us; break;
    case 2: esc2.writeMicroseconds(us); curM2 = us; break;
    case 3: esc3.writeMicroseconds(us); curM3 = us; break;
    case 4: esc4.writeMicroseconds(us); curM4 = us; break;
  }
}

void stopAll()
{
  esc1.writeMicroseconds(ESC_MIN); curM1 = ESC_MIN;
  esc2.writeMicroseconds(ESC_MIN); curM2 = ESC_MIN;
  esc3.writeMicroseconds(ESC_MIN); curM3 = ESC_MIN;
  esc4.writeMicroseconds(ESC_MIN); curM4 = ESC_MIN;
  Serial.println("[TEST] All motors → 1000 µs (STOPPED)");
}


// ================================================================
//  IMU HELPERS
// ================================================================

float adjRoll()    { return ROLL_FLIP  * (mpu.getAngleX() - groundRoll);  }
float adjPitch()   { return PITCH_FLIP * (mpu.getAngleY() - groundPitch); }
float adjYawRate() { return YAW_FLIP   * mpu.getGyroZ(); }

void captureGround()
{
  float sr = 0, sp = 0;
  Serial.println("[IMU] Capturing ground reference — keep craft still ...");
  for (int i = 0; i < 50; i++) { mpu.update(); sr += mpu.getAngleX(); sp += mpu.getAngleY(); delay(10); }
  groundRoll  = sr / 50.0f;
  groundPitch = sp / 50.0f;
  Serial.printf("[IMU] Ground ref → Roll=%.2f°  Pitch=%.2f°\n", groundRoll, groundPitch);
  Serial.println("[IMU] Adjusted readings will now be 0.0 at this angle.");
}


// ================================================================
//  COMMAND PARSER
// ================================================================

void printHelp()
{
  Serial.println("\n[HELP] ── Q450 Motor & MPU Test ────────────────────────");
  Serial.println("[HELP]  m1 <µs>             Spin M1  e.g. m1 1300");
  Serial.println("[HELP]  m2 <µs>             Spin M2");
  Serial.println("[HELP]  m3 <µs>             Spin M3");
  Serial.println("[HELP]  m4 <µs>             Spin M4");
  Serial.println("[HELP]  all <µs>            All motors same speed");
  Serial.println("[HELP]  stop                All motors OFF (1000 µs)");
  Serial.println("[HELP]  ramp <m> <s> <e> <step>  Slow ramp one motor");
  Serial.println("[HELP]                      e.g. ramp m2 1000 1400 5");
  Serial.println("[HELP]  mpu                 100 live IMU readings");
  Serial.println("[HELP]  orient              Orientation check + guide");
  Serial.println("[HELP]  imu reset           Re-capture ground level");
  Serial.println("[HELP]  status              Show current µs + ground ref");
  Serial.println("[HELP]  help                This list");
  Serial.println("[HELP] ────────────────────────────────────────────────\n");
}

void handleCommand(String cmd)
{
  cmd.trim();
  String lc = cmd;
  lc.toLowerCase();

  // ── m1/m2/m3/m4 <µs> ─────────────────────────────────────
  if (lc.startsWith("m") && lc.length() >= 4 && lc.charAt(2) == ' ')
  {
    int motor = lc.charAt(1) - '0';
    if (motor < 1 || motor > 4) { Serial.println("[ERR] Motor must be 1–4"); return; }
    int us = lc.substring(3).toInt();
    if (us < ESC_MIN || us > ESC_MAX)
    {
      Serial.printf("[ERR] µs must be %d–%d\n", ESC_MIN, ESC_MAX); return;
    }
    setMotor(motor, us);
    Serial.printf("[TEST] M%d → %d µs\n", motor, us);
    return;
  }

  // ── all <µs> ─────────────────────────────────────────────
  if (lc.startsWith("all "))
  {
    int us = lc.substring(4).toInt();
    us = constrain(us, ESC_MIN, ESC_MAX);
    setMotor(1, us); setMotor(2, us); setMotor(3, us); setMotor(4, us);
    Serial.printf("[TEST] All motors → %d µs\n", us);
    return;
  }

  // ── stop ─────────────────────────────────────────────────
  if (lc == "stop") { stopAll(); return; }

  // ── ramp <motor> <start> <end> <step> ─────────────────────
  // e.g.  ramp m1 1000 1400 10
  if (lc.startsWith("ramp "))
  {
    // Tokenise: ramp  mX  start  end  step
    int p1 = lc.indexOf(' ');         // after "ramp"
    int p2 = lc.indexOf(' ', p1+1);   // after motor name
    int p3 = lc.indexOf(' ', p2+1);   // after start
    int p4 = lc.indexOf(' ', p3+1);   // after end

    if (p4 < 0) { Serial.println("[ERR] Usage: ramp m1 1000 1400 10"); return; }

    String mStr  = lc.substring(p1+1, p2);
    int motor    = mStr.charAt(1) - '0';
    int startUs  = lc.substring(p2+1, p3).toInt();
    int endUs    = lc.substring(p3+1, p4).toInt();
    int stepUs   = lc.substring(p4+1).toInt();

    if (motor < 1 || motor > 4) { Serial.println("[ERR] Motor must be m1–m4"); return; }
    startUs = constrain(startUs, ESC_MIN, ESC_MAX);
    endUs   = constrain(endUs,   ESC_MIN, ESC_MAX);
    if (stepUs < 1) stepUs = 5;

    Serial.printf("[RAMP] M%d: %d → %d  step %d µs\n", motor, startUs, endUs, stepUs);

    int dir = (endUs >= startUs) ? 1 : -1;
    for (int us = startUs; dir > 0 ? us <= endUs : us >= endUs; us += dir * stepUs)
    {
      setMotor(motor, us);
      Serial.printf("[RAMP] M%d = %d µs\n", motor, us);
      delay(80);   // 80 ms per step — slow enough to hear the motor
    }
    setMotor(motor, endUs);
    Serial.printf("[RAMP] Done — M%d held at %d µs. Type 'stop' to cut.\n", motor, endUs);
    return;
  }

  // ── mpu ──────────────────────────────────────────────────
  if (lc == "mpu")
  {
    Serial.println("[MPU] 100 readings — tilt craft to observe angles:");
    Serial.println("[MPU] Roll(adj)  Pitch(adj)  YawRate  Raw AngleX  Raw AngleY");
    for (int i = 0; i < 100; i++)
    {
      mpu.update();
      Serial.printf("[MPU] %+7.2f°  %+7.2f°  %+7.2f°/s  | raw: AX=%+7.2f  AY=%+7.2f\n",
                    adjRoll(), adjPitch(), adjYawRate(),
                    mpu.getAngleX(), mpu.getAngleY());
      delay(50);
    }
    Serial.println("[MPU] Done.");
    return;
  }

  // ── orient ───────────────────────────────────────────────
  if (lc == "orient")
  {
    Serial.println("\n[ORIENT] ── Orientation Check ───────────────────────────");
    Serial.println("[ORIENT]  Tilt NOSE DOWN   → AdjPitch should go NEGATIVE");
    Serial.printf("[ORIENT]  (If it goes positive, set PITCH_FLIP -1.0 in code)\n");
    Serial.println("[ORIENT]  Tilt RIGHT DOWN  → AdjRoll  should go POSITIVE");
    Serial.printf("[ORIENT]  (If it goes negative, set ROLL_FLIP  -1.0 in code)\n");
    Serial.printf("[ORIENT]  Current: PITCH_FLIP=%.1f  ROLL_FLIP=%.1f\n\n",
                  PITCH_FLIP, ROLL_FLIP);

    for (int i = 0; i < 100; i++)
    {
      mpu.update();
      Serial.printf("[ORIENT] Roll:%+7.2f°  Pitch:%+7.2f°  YawRate:%+6.2f°/s\n",
                    adjRoll(), adjPitch(), adjYawRate());
      delay(60);
    }
    Serial.println("[ORIENT] ── Done ──\n");
    return;
  }

  // ── imu reset ────────────────────────────────────────────
  if (lc == "imu reset")
  {
    for (int i = 0; i < 10; i++) { mpu.update(); delay(10); }
    captureGround();
    return;
  }

  // ── status ───────────────────────────────────────────────
  if (lc == "status")
  {
    Serial.println("\n[STATUS] ── Current State ──────────────────────────────");
    Serial.printf("[STATUS]  M1: %d µs   M2: %d µs   M3: %d µs   M4: %d µs\n",
                  curM1, curM2, curM3, curM4);
    Serial.printf("[STATUS]  Ground ref → Roll=%.2f°  Pitch=%.2f°\n",
                  groundRoll, groundPitch);
    mpu.update();
    Serial.printf("[STATUS]  Live adj   → Roll=%+.2f°  Pitch=%+.2f°  YawRate=%+.2f°/s\n",
                  adjRoll(), adjPitch(), adjYawRate());
    Serial.println("[STATUS] ────────────────────────────────────────────────\n");
    return;
  }

  // ── help ─────────────────────────────────────────────────
  if (lc == "help") { printHelp(); return; }

  // ── unknown ──────────────────────────────────────────────
  Serial.printf("[ERR] Unknown command: '%s'  — type 'help'\n", cmd.c_str());
}


// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  // ── LED ─────────────────────────────────────────────────────
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);   // on during init

  Serial.println("\n[TEST] ═══════════════════════════════════════════");
  Serial.println("[TEST]  Q450 Motor & MPU Test Sketch");
  Serial.println("[TEST]  !! REMOVE PROPS BEFORE MOTOR COMMANDS !!");
  Serial.println("[TEST] ═══════════════════════════════════════════\n");

  // ── ESCs ────────────────────────────────────────────────────
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  esc1.setPeriodHertz(50); esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50); esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50); esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50); esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);
  stopAll();   // send 1000 µs to all ESCs immediately
  Serial.println("[TEST] ESCs attached — all at 1000 µs");

  // ── Wait for ESCs to initialise ────────────────────────────
  // SimonK needs ~2 s of continuous low signal to fully arm.
  Serial.println("[TEST] Waiting 3 s for ESCs to initialise ...");
  for (int i = 3; i >= 1; i--)
  {
    Serial.printf("[TEST]  %d ...\n", i);
    delay(1000);
  }
  Serial.println("[TEST] ESCs ready");

  // ── MPU-6050 ────────────────────────────────────────────────
  Wire.begin(MPU_SDA, MPU_SCL);
  byte status = mpu.begin();
  if (status != 0)
  {
    Serial.printf("[TEST] MPU-6050 NOT FOUND (err %d) — check wiring\n", status);
    // Blink rapidly to signal IMU error — motor tests still work
    for (int i = 0; i < 10; i++) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(150); }
  }
  else
  {
    Serial.println("[TEST] MPU-6050 OK");
    delay(200);
    // Warm-up reads
    for (int i = 0; i < 20; i++) { mpu.update(); delay(10); }
    captureGround();   // capture boot orientation as zero reference
  }

  // ── Ready ───────────────────────────────────────────────────
  digitalWrite(PIN_LED, LOW);
  printHelp();
  Serial.println("[TEST] Ready — type a command:\n> ");
}


// ================================================================
//  MAIN LOOP
// ================================================================
void loop()
{
  // Read a full line from Serial Monitor then parse it
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      Serial.printf("> %s\n", line.c_str());   // echo input
      handleCommand(line);
    }
    Serial.print("> ");   // prompt for next command
  }

  // Keep sending current motor values every 20 ms so ESCs don't time out
  esc1.writeMicroseconds(curM1);
  esc2.writeMicroseconds(curM2);
  esc3.writeMicroseconds(curM3);
  esc4.writeMicroseconds(curM4);

  delay(20);
}
