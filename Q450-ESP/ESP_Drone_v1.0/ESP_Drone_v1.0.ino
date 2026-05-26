// ================================================================
//  Q450 ESP32 Flight Controller v1.0
//  Frame   : Q450 Quadcopter (X-Config)
//  Motors  : A2212/13T 1000KV
//  ESCs    : SimonK 30A (Standard PWM 1000–2000µs)
//  IMU     : MPU-6050 (I2C)
//  Radio   : ESP-NOW via ReceiverModule.h
//  Mode    : Mode 2 (Left=Throttle/Yaw, Right=Pitch/Roll)
//
//  FLIGHT CHANNEL MAPPING:
//    Left  Y  → Throttle  (up/down)
//    Left  X  → Yaw       (rotate CW/CCW)
//    Right Y  → Pitch     (forward/back)
//    Right X  → Roll      (left/right)
//    LBt      → ARM / DISARM toggle
//    RBt      → LED ON / OFF toggle
//
//  MOTOR LAYOUT (X-Config, top view):
//
//       Front
//    M1(CW)  M2(CCW)
//       \    /
//        \  /
//        /  \
//       /    \
//    M4(CCW) M3(CW)
//       Back
//
//  PIN ASSIGNMENTS:
//    GPIO 25 → ESC Motor 1 (Front-Left,  CW)
//    GPIO 26 → ESC Motor 2 (Front-Right, CCW)
//    GPIO 27 → ESC Motor 3 (Back-Right,  CW)
//    GPIO 14 → ESC Motor 4 (Back-Left,   CCW)
//    GPIO 21 → MPU-6050 SDA
//    GPIO 22 → MPU-6050 SCL
//    GPIO  2 → Built-in LED
//
// ================================================================

#include "ReceiverModule.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>   // Install: "MPU6050_light" by rfetick

// ================================================================
//  ██████╗ ██╗███╗   ██╗    ██████╗ ██╗███╗   ██╗███████╗
//  ██╔══██╗██║████╗  ██║    ██╔══██╗██║████╗  ██║██╔════╝
//  ██████╔╝██║██╔██╗ ██║    ██████╔╝██║██╔██╗ ██║███████╗
//  ██╔═══╝ ██║██║╚██╗██║    ██╔═══╝ ██║██║╚██╗██║╚════██║
//  ██║     ██║██║ ╚████║    ██║     ██║██║ ╚████║███████║
//  ╚═╝     ╚═╝╚═╝  ╚═══╝   ╚═╝     ╚═╝╚═╝  ╚═══╝╚══════╝
//  — All tunable values in ONE PLACE — tweak here, not below —
// ================================================================

// ── Hardware Pins ─────────────────────────────────────────────
#define PIN_M1      25    // Front-Left  (CW)
#define PIN_M2      26    // Front-Right (CCW)
#define PIN_M3      27    // Back-Right  (CW)
#define PIN_M4      14    // Back-Left   (CCW)
#define PIN_LED      2    // Built-in LED (active HIGH on most ESP32 devkits)
#define MPU_SDA     21
#define MPU_SCL     22

// ── ESC PWM Range ─────────────────────────────────────────────
#define ESC_MIN    1000   // µs — motor off / minimum throttle
#define ESC_MAX    2000   // µs — full throttle
#define ESC_ARM    1000   // µs — sent during arm sequence
#define ESC_IDLE   1100   // µs — idle when armed but stick at zero
                          //       raise slightly if motors stutter at idle

// ── Stick Input Deadband ──────────────────────────────────────
// Any stick value within ±DEADBAND of zero is treated as exactly 0.
// Helps with cheap analog sticks that don't return to perfect center.
#define DEADBAND      5   // units: raw stick (-99..99 scale)

// ── Expo Curve ────────────────────────────────────────────────
// 0.0 = linear (no expo)
// Higher = gentler around center, more responsive at extremes.
// Roll/Pitch expo makes hovering smoother with twitchy sticks.
// Yaw expo keeps slow pans buttery.
// Throttle expo is usually kept 0 (linear).
#define EXPO_ROLL    0.35f   // 0.0 – 0.6 recommended
#define EXPO_PITCH   0.35f
#define EXPO_YAW     0.25f
#define EXPO_THROTTLE 0.0f

// ── Rate / Gain ───────────────────────────────────────────────
// Controls how many µs of PWM the full stick deflection commands.
// Smaller = calmer, more manageable. Larger = more aggressive.
// Start LOW and raise carefully.
#define RATE_ROLL    150   // µs authority for full roll deflection
#define RATE_PITCH   150   // µs authority for full pitch deflection
#define RATE_YAW     120   // µs authority for full yaw deflection

// ── PID Gains ─────────────────────────────────────────────────
// Start with all gains at 0 EXCEPT P, tune one axis at a time.
// Increase P until oscillation → back off 20% → add a little D.
// I corrects slow drift; keep it tiny at first.
//
//            P         I         D
float KP[3] = { 1.20f,  1.20f,  0.80f };  // [0]=Roll [1]=Pitch [2]=Yaw
float KI[3] = { 0.02f,  0.02f,  0.01f };
float KD[3] = { 0.80f,  0.80f,  0.30f };

// ── PID Integral Clamp ────────────────────────────────────────
// Prevents integrator windup during throttle extremes / flips.
#define ITERM_MAX   50.0f   // µs — max I-term contribution per axis

// ── Failsafe Descent ──────────────────────────────────────────
// When signal is lost the craft slowly lowers throttle to land.
// DESCENT_RATE: µs per update tick to reduce throttle.
// DESCENT_MIN:  stop descending when throttle reaches this value
//               (prevents motors cutting out mid-air at low alt).
#define DESCENT_RATE   2     // µs per loop (~40µs/s at 50Hz)
#define DESCENT_FLOOR  ESC_IDLE  // cut below idle is unsafe

// ── Loop timing ───────────────────────────────────────────────
// 50 Hz is plenty for a beginner build. Raise to 100–250 Hz later.
#define LOOP_HZ      50
#define LOOP_US      (1000000 / LOOP_HZ)

// ── Arm/Disarm Guard ─────────────────────────────────────────
// Craft must stay DISARMED for at least this many ms after a
// disarm event before it can re-arm (prevents accidental re-arm).
#define REARM_GUARD_MS  2000

// ── Test / Debug Mode ─────────────────────────────────────────
// Set TEST_MODE to 1 to enable serial telemetry without actually
// driving ESCs (safe bench testing). Set to 0 for real flight.
#define TEST_MODE   1        // 1 = bench test (ESCs frozen at ESC_MIN)
                             // 0 = live flight

// ── Calibration Offsets (set after IMU calibration run) ───────
// Run the sketch once with CALIBRATE_IMU 1, note the printed offsets,
// paste them here, then set CALIBRATE_IMU back to 0.
#define CALIBRATE_IMU  0     // 1 = print offsets and halt, 0 = normal
float IMU_OFFSET_AX = 0.0f; // paste from calibration output
float IMU_OFFSET_AY = 0.0f;
float IMU_OFFSET_AZ = 0.0f;
float IMU_OFFSET_GX = 0.0f;
float IMU_OFFSET_GY = 0.0f;
float IMU_OFFSET_GZ = 0.0f;

// ================================================================
//  END OF TUNING SECTION
//  ↓  Do not edit below unless you know what you're changing  ↓
// ================================================================


// ── Objects ───────────────────────────────────────────────────
Servo esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// ── State ─────────────────────────────────────────────────────
bool  armed         = false;
bool  ledState      = false;
bool  lastLBt       = false;   // previous LBt for edge detection
bool  lastRBt       = false;   // previous RBt for edge detection
int   failThrottle  = ESC_MIN; // descends from last throttle on loss
bool  signalLost    = false;
unsigned long lastDisarmMs = 0;

// ── PID state per axis [0=Roll 1=Pitch 2=Yaw] ─────────────────
float pidPrev[3]  = {0, 0, 0}; // previous error
float pidIacc[3]  = {0, 0, 0}; // accumulated integral

// ── Timing ────────────────────────────────────────────────────
unsigned long loopTimer = 0;

// ── Telemetry counter (prints every N loops) ──────────────────
int telTick = 0;
#define TEL_EVERY   10   // print every 10 loops (5 times/sec at 50Hz)


// ================================================================
//  HELPER: applyExpo(value, expo)
//  Applies cubic expo curve. value and output are -1.0..1.0.
//  expo = 0 → linear.  expo = 1 → full cubic.
// ================================================================
float applyExpo(float v, float expo)
{
  // formula: out = expo*v³ + (1-expo)*v
  return expo * v * v * v + (1.0f - expo) * v;
}

// ================================================================
//  HELPER: applyDeadband(raw)
//  Returns 0 if |raw| <= DEADBAND, otherwise shifts the value
//  inward so there's no jump at the deadband edge.
// ================================================================
int applyDeadband(int raw)
{
  if (raw >  DEADBAND) return raw - DEADBAND;
  if (raw < -DEADBAND) return raw + DEADBAND;
  return 0;
}

// ================================================================
//  HELPER: writeMotors(m1, m2, m3, m4)
//  Clamps each value to ESC_MIN..ESC_MAX then writes PWM.
//  In TEST_MODE motors are frozen at ESC_MIN.
// ================================================================
void writeMotors(int m1, int m2, int m3, int m4)
{
#if TEST_MODE
  // In test mode just print, don't spin
  return;
#endif
  esc1.writeMicroseconds(constrain(m1, ESC_MIN, ESC_MAX));
  esc2.writeMicroseconds(constrain(m2, ESC_MIN, ESC_MAX));
  esc3.writeMicroseconds(constrain(m3, ESC_MIN, ESC_MAX));
  esc4.writeMicroseconds(constrain(m4, ESC_MIN, ESC_MAX));
}

// ================================================================
//  HELPER: motorsOff()
//  Immediately cuts all motors to ESC_MIN (disarmed / failsafe).
// ================================================================
void motorsOff()
{
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

// ================================================================
//  ARM SEQUENCE
//  SimonK / BLHeli ESCs arm with a short low-throttle pulse.
//  We send ESC_MIN for 2 seconds, then ESC_ARM+50 briefly.
// ================================================================
void armSequence()
{
  Serial.println("[FC] ── ARM SEQUENCE starting ──");
  // Step 1: confirm low throttle (2 s)
  for (int i = 0; i < 200; i++)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(10);
  }
  // Step 2: brief pulse to confirm arm (some SimonK needs this)
  for (int i = 0; i < 50; i++)
  {
    esc1.writeMicroseconds(ESC_ARM + 50);
    esc2.writeMicroseconds(ESC_ARM + 50);
    esc3.writeMicroseconds(ESC_ARM + 50);
    esc4.writeMicroseconds(ESC_ARM + 50);
    delay(10);
  }
  // Back to low
  motorsOff();
  Serial.println("[FC] ── ARMED ──");
}

// ================================================================
//  PID COMPUTE
//  axis:   0=Roll, 1=Pitch, 2=Yaw
//  setpt:  desired rate   (−1.0 .. +1.0 normalised)
//  actual: gyro rate      (degrees/s)
//  dt:     loop period    (seconds)
//  returns µs correction  (can be negative)
// ================================================================
float computePID(int axis, float setpt, float actual, float dt)
{
  float err  = setpt - actual;
  pidIacc[axis] += err * dt;
  pidIacc[axis]  = constrain(pidIacc[axis], -ITERM_MAX, ITERM_MAX);
  float deriv    = (err - pidPrev[axis]) / dt;
  pidPrev[axis]  = err;
  return KP[axis] * err + KI[axis] * pidIacc[axis] + KD[axis] * deriv;
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[FC] ════════════════════════════════════");
  Serial.println("[FC]  Q450 Flight Controller Boot");
  Serial.printf ("[FC]  TEST_MODE : %s\n", TEST_MODE ? "ON (no ESC output)" : "OFF (LIVE)");
  Serial.println("[FC] ════════════════════════════════════");

  // ── LED ───────────────────────────────────────────────────────
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // ── ESC init — write low BEFORE attaching to avoid glitch ────
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  esc1.setPeriodHertz(50); esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50); esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50); esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50); esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);
  motorsOff();
  Serial.println("[FC] ESCs attached — sending min throttle (1000µs)");

  // ── MPU-6050 init ─────────────────────────────────────────────
  Wire.begin(MPU_SDA, MPU_SCL);
  byte status = mpu.begin();
  if (status != 0)
  {
    Serial.printf("[FC] MPU-6050 INIT FAILED (err %d) — check wiring!\n", status);
    // Blink LED rapidly and halt
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  Serial.println("[FC] MPU-6050 OK");

#if CALIBRATE_IMU
  Serial.println("[FC] ── IMU CALIBRATION MODE ──");
  Serial.println("[FC] Place craft FLAT and STILL. Calibrating in 3s ...");
  delay(3000);
  mpu.calcOffsets(true, true); // gyro + accel
  Serial.println("[FC] ── Paste these into the PIN CONFIGURATION section ──");
  Serial.printf("[FC] IMU_OFFSET_AX = %.4f;\n", mpu.getAccXoffset());
  Serial.printf("[FC] IMU_OFFSET_AY = %.4f;\n", mpu.getAccYoffset());
  Serial.printf("[FC] IMU_OFFSET_AZ = %.4f;\n", mpu.getAccZoffset());
  Serial.printf("[FC] IMU_OFFSET_GX = %.4f;\n", mpu.getGyroXoffset());
  Serial.printf("[FC] IMU_OFFSET_GY = %.4f;\n", mpu.getGyroYoffset());
  Serial.printf("[FC] IMU_OFFSET_GZ = %.4f;\n", mpu.getGyroZoffset());
  Serial.println("[FC] ── Set CALIBRATE_IMU 0 then re-flash ──");
  while (1); // halt
#endif

  // Apply stored offsets
  mpu.setAccOffsets(IMU_OFFSET_AX, IMU_OFFSET_AY, IMU_OFFSET_AZ);
  mpu.setGyroOffsets(IMU_OFFSET_GX, IMU_OFFSET_GY, IMU_OFFSET_GZ);

  // ── ESP-NOW Receiver ──────────────────────────────────────────
  receiver.timeoutMs = 1000;
  receiver.begin("A4:F0:0F:90:34:28", true);

  // ── Blink LED 3× to signal ready ─────────────────────────────
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_LED, HIGH); delay(150);
    digitalWrite(PIN_LED, LOW);  delay(150);
  }

  Serial.println("[FC] ── Ready. Toggle LBt to ARM ──\n");
  loopTimer = micros();
}

// ================================================================
//  LOOP
// ================================================================
void loop()
{
  // ── Fixed-rate loop gate ──────────────────────────────────────
  // Busy-wait until LOOP_US µs have passed since last iteration.
  // This gives a stable, predictable PID sample rate.
  while (micros() - loopTimer < LOOP_US);
  float dt = (micros() - loopTimer) / 1e6f; // actual dt in seconds
  loopTimer = micros();

  // ── 1. Receiver update (failsafe + connected_flag) ───────────
  receiver.update();
  bool linkOK = receiver.connected();

  // ── 2. IMU update ─────────────────────────────────────────────
  mpu.update();
  float gyroX = mpu.getGyroX(); // roll  rate  deg/s
  float gyroY = mpu.getGyroY(); // pitch rate  deg/s
  float gyroZ = mpu.getGyroZ(); // yaw   rate  deg/s

  // ── 3. Signal-lost detection (with suppress-spam latch) ──────
  if (!linkOK && !signalLost)
  {
    signalLost    = true;
    failThrottle  = (armed) ? ESC_IDLE + 100 : ESC_MIN;
    // Only print once when signal is FIRST lost
    Serial.println("[FC] ⚠  SIGNAL LOST — failsafe descent active");
    // Reset PID integrals to avoid windup during coast
    memset(pidIacc, 0, sizeof(pidIacc));
    memset(pidPrev, 0, sizeof(pidPrev));
  }
  else if (linkOK && signalLost)
  {
    signalLost = false;
    Serial.println("[FC] ✔  Signal restored");
  }

  // ── 4. Button edge detection (arm/disarm + LED) ───────────────
  bool curLBt = receiver.data.LBt;
  bool curRBt = receiver.data.RBt;

  // ─ ARM / DISARM on LBt rising edge ────────────────────────────
  if (curLBt && !lastLBt)
  {
    if (!armed)
    {
      // Guard: don't re-arm too soon after disarm
      if (millis() - lastDisarmMs >= REARM_GUARD_MS)
      {
        if (linkOK)
        {
          armed = true;
          memset(pidIacc, 0, sizeof(pidIacc)); // clear integrals on arm
          memset(pidPrev, 0, sizeof(pidPrev));
#if !TEST_MODE
          armSequence();
#else
          Serial.println("[FC] [TEST] Arm requested (no ESC output)");
#endif
        }
        else
        {
          Serial.println("[FC] ✖ Cannot arm — no TX signal!");
        }
      }
      else
      {
        Serial.println("[FC] ✖ Rearm guard active — wait before re-arming");
      }
    }
    else
    {
      armed        = false;
      lastDisarmMs = millis();
      motorsOff();
      Serial.println("[FC] ── DISARMED ──");
    }
  }
  lastLBt = curLBt;

  // ─ LED toggle on RBt rising edge ─────────────────────────────
  if (curRBt && !lastRBt)
  {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    Serial.printf("[FC] LED %s\n", ledState ? "ON" : "OFF");
  }
  lastRBt = curRBt;

  // ── 5. Stick processing ───────────────────────────────────────
  // Mode 2: Left Y = Throttle, Left X = Yaw, Right Y = Pitch, Right X = Roll
  int rawThrottle = receiver.data.Ly;  // -99..99
  int rawYaw      = receiver.data.Lx;
  int rawPitch    = receiver.data.Ry;  // forward = positive
  int rawRoll     = receiver.data.Rx;

  // Deadband
  rawYaw   = applyDeadband(rawYaw);
  rawPitch = applyDeadband(rawPitch);
  rawRoll  = applyDeadband(rawRoll);
  // Throttle: only lower deadband (near zero), keep full positive range
  if (rawThrottle < DEADBAND && rawThrottle > -DEADBAND) rawThrottle = 0;

  // Normalise to -1.0..+1.0 for expo
  float nThrottle = rawThrottle / 99.0f;
  float nYaw      = rawYaw      / 99.0f;
  float nPitch    = rawPitch    / 99.0f;
  float nRoll     = rawRoll     / 99.0f;

  // Expo (no expo on throttle by default)
  nThrottle = applyExpo(nThrottle, EXPO_THROTTLE);
  nYaw      = applyExpo(nYaw,      EXPO_YAW);
  nPitch    = applyExpo(nPitch,    EXPO_PITCH);
  nRoll     = applyExpo(nRoll,     EXPO_ROLL);

  // Map throttle -1..1 → ESC_IDLE..ESC_MAX  (negative = clamp to idle)
  int throttle = (int)map((long)(nThrottle * 100), -100, 100, ESC_IDLE, ESC_MAX);
  throttle = constrain(throttle, ESC_IDLE, ESC_MAX);

  // Rate setpoints in deg/s (expo output × max rate)
  float spRoll  = nRoll  * RATE_ROLL;
  float spPitch = nPitch * RATE_PITCH;
  float spYaw   = nYaw   * RATE_YAW;

  // ── 6. PID ────────────────────────────────────────────────────
  float pidRoll  = computePID(0, spRoll,  gyroX, dt);
  float pidPitch = computePID(1, spPitch, gyroY, dt);
  float pidYaw   = computePID(2, spYaw,   gyroZ, dt);

  // ── 7. Motor mixing (X-Config) ────────────────────────────────
  //  M1 Front-Left  CW  : Throttle + Pitch + Roll - Yaw
  //  M2 Front-Right CCW : Throttle + Pitch - Roll + Yaw
  //  M3 Back-Right  CW  : Throttle - Pitch - Roll - Yaw
  //  M4 Back-Left   CCW : Throttle - Pitch + Roll + Yaw
  int m1 = throttle + (int)pidPitch + (int)pidRoll  - (int)pidYaw;
  int m2 = throttle + (int)pidPitch - (int)pidRoll  + (int)pidYaw;
  int m3 = throttle - (int)pidPitch - (int)pidRoll  - (int)pidYaw;
  int m4 = throttle - (int)pidPitch + (int)pidRoll  + (int)pidYaw;

  // ── 8. Output decision ────────────────────────────────────────
  if (!armed)
  {
    // Disarmed — always off
    motorsOff();
  }
  else if (signalLost)
  {
    // ── FAILSAFE: gradual descent ──────────────────────────────
    // Slowly reduce throttle toward ESC_IDLE. PID still active
    // so craft stays level (gyro still works).
    failThrottle -= DESCENT_RATE;
    failThrottle  = max(failThrottle, DESCENT_FLOOR);

    int fm1 = failThrottle - (int)pidYaw;
    int fm2 = failThrottle + (int)pidYaw;
    int fm3 = failThrottle - (int)pidYaw;
    int fm4 = failThrottle + (int)pidYaw;
    writeMotors(fm1, fm2, fm3, fm4);
  }
  else
  {
    // ── Normal flight ─────────────────────────────────────────
    writeMotors(m1, m2, m3, m4);
  }

  // ── 9. Serial telemetry (throttled, not every loop) ──────────
  telTick++;
  if (telTick >= TEL_EVERY)
  {
    telTick = 0;
    Serial.printf(
      "[FC] ARM:%s LINK:%s | THR:%4d | Y:%+6.1f P:%+6.1f R:%+6.1f | "
      "gX:%+6.1f gY:%+6.1f gZ:%+6.1f | "
      "pidR:%+6.1f pidP:%+6.1f pidY:%+6.1f | "
      "M1:%4d M2:%4d M3:%4d M4:%4d | BAT:%3d%%\n",
      armed     ? "Y" : "N",
      linkOK    ? "Y" : "N",
      throttle,
      spYaw, spPitch, spRoll,
      gyroX, gyroY, gyroZ,
      pidRoll, pidPitch, pidYaw,
      constrain(m1,ESC_MIN,ESC_MAX), constrain(m2,ESC_MIN,ESC_MAX),
      constrain(m3,ESC_MIN,ESC_MAX), constrain(m4,ESC_MIN,ESC_MAX),
      receiver.data.BAT
    );
  }
}
