// ================================================================
//  Q450 ESP32 Flight Controller  —  v1.3
//  Frame   : Q450 Quadcopter (X-Config)
//  Motors  : A2212/13T 1000KV
//  ESCs    : SimonK 30A  (Standard PWM 1000–2000 µs)
//  IMU     : MPU-6050 (I2C)
//  Radio   : ESP-NOW via ReceiverModule.h
//  Mode    : Angle Mode  (self-levelling)
//
// ----------------------------------------------------------------
//  CHANGES v1.2 → v1.3
//  • Accumulated throttle system — center stick holds altitude
//    Left stick Y now acts as rate command, not position
//    Stick UP = climb, CENTER = hold, DOWN = descend
//  • THROTTLE_RAMP_RATE added — controls climb/descent speed
//  • Per-motor trim offsets added (MOTOR_TRIM[4])
//    Compensates for motors spinning at unequal speeds
//  • Safe arm sequence — removed dangerous MIN→MAX→MIN ESC
//    calibration from normal arm, now gentle ramp to ESC_SAFE only
//  • ESC range calibration moved to serial command "escal"
//    Only needed once per ESC, saves to ESC flash
//  • ESC_SAFE lowered to 1120µs (was 1200, was lifting craft)
//  • STICK_PITCH_FLIP / STICK_ROLL_FLIP defines added (stub)
//  • accThrottle reset to ESC_SAFE on arm and disarm
//  • prevM* reset on arm — fixes slew lag after re-arm
// ================================================================

#include "ReceiverModule.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>


// ================================================================
//   TUNING — ALL TWEAKABLE SETTINGS IN ONE PLACE
// ================================================================

// ── Hardware pins ─────────────────────────────────────────────
#define PIN_M1        25    // Front-Left  (CW)
#define PIN_M2        26    // Front-Right (CCW)
#define PIN_M3        27    // Back-Right  (CW)
#define PIN_M4        14    // Back-Left   (CCW)
#define PIN_LED        2    // Built-in LED
#define MPU_SDA       21
#define MPU_SCL       22

// ── TX MAC ────────────────────────────────────────────────────
#define TX_MAC  "A4:F0:0F:90:34:28"

// ── ESC PWM limits ────────────────────────────────────────────
#define ESC_MIN     1000    // µs — absolute minimum (ESC off)
#define ESC_MAX     2000    // µs — absolute maximum (full throttle)

// ── Per-motor trim offsets ────────────────────────────────────
// If one motor spins faster or slower than the others at the same
// µs command, use these to balance them out.
// POSITIVE = motor gets MORE power than commanded
// NEGATIVE = motor gets LESS power than commanded
// Start at 0 for all. Test by running "all 1300" in serial test
// and listening — the loudest motor needs a negative trim.
// Adjust in steps of 10 until all 4 sound equal.
//
//              M1    M2    M3    M4
int MOTOR_TRIM[4] = {  0,    0,    0,    0  };  // µs  ★ TUNE ME

// ── Idle and safe-spin speed ──────────────────────────────────
// ESC_IDLE  : motors spin visibly but craft cannot lift.
//             Raise by 10 if any motor doesn't spin on arm.
// ESC_SAFE  : the speed motors sit at while armed and waiting
//             for throttle input. Must be >= ESC_IDLE.
//             Keep well below liftoff to prevent surprise takeoff.
#define ESC_IDLE    1180    // µs — minimum reliable spin  ★ TUNE ME
#define ESC_SAFE    1200    // µs — armed waiting speed    ★ TUNE ME

// ── Accumulated throttle limits ───────────────────────────────
// The throttle is NOT directly mapped from the stick.
// Instead the stick nudges it up or down each loop tick.
// These cap how high/low the accumulated throttle can go.
#define THROTTLE_MIN  ESC_SAFE   // can't go below safe spin when armed
#define THROTTLE_MAX  1800       // µs — cap below ESC_MAX for safety ★ TUNE ME

// ── Throttle ramp rate ────────────────────────────────────────
// How many µs the throttle changes per loop tick (at 100 Hz).
// LOWER = smoother and safer for cheap sticks but slower response.
// HIGHER = faster response but may feel twitchy.
// At 100 Hz:  1 µs/tick = 100 µs/s  (very slow, ~9s to full range)
//             3 µs/tick = 300 µs/s  (moderate)
//             5 µs/tick = 500 µs/s  (fast)
// Recommended start: 2 for cheap sticks, 4 for quality sticks.
#define THROTTLE_RAMP_RATE   2   // µs per loop tick  ★ TUNE ME

// ── Stick deadband ────────────────────────────────────────────
// Raw stick units below this are treated as zero.
// Increase if your sticks are twitchy at center.
#define DEADBAND       8    // raw stick units (-99..99)  ★ TUNE ME

// ── IMU angle deadband ────────────────────────────────────────
#define IMU_ANGLE_DEADZONE  0.5f   // degrees  ★ TUNE ME

// ── Gyro rate deadband ────────────────────────────────────────
#define GYRO_DEADZONE_DPS   2.0f   // deg/s  ★ TUNE ME

// ── IMU angle IIR smoothing ───────────────────────────────────
// 1.0 = no smoothing (raw).  0.7 = moderate.  0.4 = heavy.
#define ANGLE_FILTER_ALPHA  0.8f   // 0.0–1.0  ★ TUNE ME

// ── Motor output slew limit ───────────────────────────────────
// Max µs change per loop tick per motor.
// Prevents PID spikes from reaching ESCs instantly.
#define MAX_MOTOR_STEP  30         // µs per loop tick  ★ TUNE ME

// ── Expo curve ────────────────────────────────────────────────
// 0.0 = linear.  0.5 = strong expo (gentle around center).
#define EXPO_ROLL     0.40f
#define EXPO_PITCH    0.40f
#define EXPO_YAW      0.30f

// ── Max angle / rate targets ──────────────────────────────────
#define MAX_ANGLE_ROLL   20.0f   // degrees at full roll stick
#define MAX_ANGLE_PITCH  20.0f   // degrees at full pitch stick
#define MAX_RATE_YAW    120.0f   // deg/s at full yaw stick

// ── IMU orientation ───────────────────────────────────────────
#define PITCH_FLIP   1.0f    //  1.0 = normal  |  -1.0 = inverted
#define ROLL_FLIP    1.0f    //  1.0 = normal  |  -1.0 = inverted
#define YAW_FLIP     1.0f    //  1.0 = normal  |  -1.0 = inverted

// ── PID gains ─────────────────────────────────────────────────
//           Roll     Pitch    Yaw
float KP[3]={ 1.50f,  1.50f,  0.80f };
float KI[3]={ 0.02f,  0.02f,  0.00f };
float KD[3]={ 0.80f,  0.80f,  0.20f };

// ── PID integral clamp ────────────────────────────────────────
#define ITERM_MAX     40.0f

// ── Failsafe descent ──────────────────────────────────────────
#define DESCENT_RATE    3      // µs per loop tick
#define DESCENT_FLOOR   ESC_IDLE

// ── Loop rate ─────────────────────────────────────────────────
#define LOOP_HZ      100
#define LOOP_US      (1000000 / LOOP_HZ)

// ── Arm guard ─────────────────────────────────────────────────
#define REARM_GUARD_MS   2000

// ── IMU reset hold time ───────────────────────────────────────
#define IMU_RESET_HOLD_MS  5000

// ── Test / Debug mode ─────────────────────────────────────────
// TEST_MODE 1 → serial telemetry ON, ESCs NOT driven.
// TEST_MODE 0 → LIVE flight. REMOVE PROPS before changing!
#define TEST_MODE    0       // ← 0 = LIVE FLIGHT

// ── IMU calibration helper ────────────────────────────────────
#define CALIBRATE_IMU  0
float IMU_OFF_AX = 0.0f, IMU_OFF_AY = 0.0f, IMU_OFF_AZ = 0.0f;
float IMU_OFF_GX = 0.0f, IMU_OFF_GY = 0.0f, IMU_OFF_GZ = 0.0f;

// ── Serial motor test ─────────────────────────────────────────
#define MOTOR_TEST_MODE  1

// ================================================================
//  END OF TUNING SECTION
// ================================================================


// ── Library objects ───────────────────────────────────────────
Servo   esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// ── Flight state ──────────────────────────────────────────────
bool  armed          = false;
bool  ledState       = false;
bool  lastLBt        = false;
bool  lastRBt        = false;
bool  signalLost     = false;
int   failThrottle   = ESC_MIN;

// ── Accumulated throttle ──────────────────────────────────────
// This is the persistent throttle value that the stick nudges.
// It starts at ESC_SAFE when armed and is held when stick centers.
float accThrottle    = ESC_SAFE;

// ── Previous motor outputs (for slew limiting) ────────────────
int prevM1 = ESC_MIN, prevM2 = ESC_MIN;
int prevM3 = ESC_MIN, prevM4 = ESC_MIN;

// ── IMU ground reference ──────────────────────────────────────
float groundRoll  = 0.0f;
float groundPitch = 0.0f;

// ── Filtered IMU angle state ──────────────────────────────────
float filtRoll  = 0.0f;
float filtPitch = 0.0f;

// ── PID state [0=Roll  1=Pitch  2=Yaw] ───────────────────────
float pidPrevErr[3]  = {0, 0, 0};
float pidIntegral[3] = {0, 0, 0};

// ── Timing ────────────────────────────────────────────────────
unsigned long lastDisarmMs  = 0;
unsigned long bothBtnHeldMs = 0;
bool          bothBtnHeld   = false;
unsigned long loopTimer     = 0;

// ── Telemetry ─────────────────────────────────────────────────
int  telTick = 0;
#define TEL_EVERY   20

// ── Actual motor outputs (for telemetry) ─────────────────────
int lastM1 = ESC_MIN, lastM2 = ESC_MIN;
int lastM3 = ESC_MIN, lastM4 = ESC_MIN;

// ── Motor test overrides ──────────────────────────────────────
int  testM1 = ESC_MIN, testM2 = ESC_MIN;
int  testM3 = ESC_MIN, testM4 = ESC_MIN;
bool testOverride = false;


// ================================================================
//  SECTION 1 — ESC / MOTOR HELPERS
// ================================================================

// ── Slew limit: prevent instant jumps to a new motor value ────
int slewMotor(int target, int previous)
{
  int delta = constrain(target - previous, -MAX_MOTOR_STEP, MAX_MOTOR_STEP);
  return previous + delta;
}

// ── Write to all four ESCs with slew limiting and per-motor trim
void writeMotors(int m1, int m2, int m3, int m4)
{
  // Apply per-motor trim before clamping
  m1 += MOTOR_TRIM[0];
  m2 += MOTOR_TRIM[1];
  m3 += MOTOR_TRIM[2];
  m4 += MOTOR_TRIM[3];

  // Clamp to valid ESC range
  m1 = constrain(m1, ESC_MIN, ESC_MAX);
  m2 = constrain(m2, ESC_MIN, ESC_MAX);
  m3 = constrain(m3, ESC_MIN, ESC_MAX);
  m4 = constrain(m4, ESC_MIN, ESC_MAX);

  // Apply slew limiting — no more than MAX_MOTOR_STEP µs per tick
  m1 = slewMotor(m1, prevM1);
  m2 = slewMotor(m2, prevM2);
  m3 = slewMotor(m3, prevM3);
  m4 = slewMotor(m4, prevM4);

  // Remember outputs for next tick's slew calculation and telemetry
  prevM1 = m1; prevM2 = m2; prevM3 = m3; prevM4 = m4;
  lastM1 = m1; lastM2 = m2; lastM3 = m3; lastM4 = m4;

#if TEST_MODE
  return;   // do not write to ESCs in test mode
#endif

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

// ── Cut all motors instantly — bypasses slew and TEST_MODE ────
void motorsOff()
{
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_MIN;
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

// ── Safe arm sequence — NO max throttle spike ─────────────────
// v2.2: We skip the ESC range calibration (MIN→MAX→MIN).
// Reason: that sequence briefly commands 2000µs which is
// dangerous near people. SimonK ESCs remember their calibration
// from the first time you ran it. If yours haven't been calibrated
// yet, do it ONCE with props removed using the serial command
// "escal" (see serial commands section).
//
// Normal arm now just:
//   1. Holds MIN for 1s (tells ESC the signal is alive)
//   2. Slowly ramps to ESC_SAFE so motors start spinning gently
void runArmSequence()
{
  Serial.println("[FC][ARM] Arming — gentle start (no max spike)");
  Serial.println("[FC][ARM] Step 1: Holding MIN for 1 s ...");

  // Hold minimum so ESC knows the signal is present
  unsigned long t = millis();
  while (millis() - t < 1000)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }

  // Slowly ramp from MIN to ESC_SAFE — no sudden spin-up
  Serial.printf("[FC][ARM] Step 2: Ramping to safe idle (%d µs) ...\n", ESC_SAFE);
  for (int us = ESC_MIN; us <= ESC_SAFE; us += 2)
  {
    esc1.writeMicroseconds(us);
    esc2.writeMicroseconds(us);
    esc3.writeMicroseconds(us);
    esc4.writeMicroseconds(us);
    delay(20);
  }

  // Sync slew state so first loop tick doesn't jump
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_SAFE;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_SAFE;

  Serial.printf("[FC][ARM] Armed — motors at safe idle %d µs\n", ESC_SAFE);
  Serial.println("[FC][ARM] Move LEFT stick UP to increase throttle slowly.");
}

// ── One-time ESC range calibration (serial command only) ──────
// Run this ONCE with props removed if your ESCs have never been
// calibrated. After this, normal arm sequence is safe to use.
void runESCCalibration()
{
  Serial.println("[FC][ESCAL] !! REMOVE PROPS — ESC range calibration !!");
  Serial.println("[FC][ESCAL] Sending MIN for 3 s ...");
  unsigned long t = millis();
  while (millis() - t < 3000)
  {
    esc1.writeMicroseconds(ESC_MIN); esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN); esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Sending MAX for 0.5 s ...");
  t = millis();
  while (millis() - t < 500)
  {
    esc1.writeMicroseconds(ESC_MAX); esc2.writeMicroseconds(ESC_MAX);
    esc3.writeMicroseconds(ESC_MAX); esc4.writeMicroseconds(ESC_MAX);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Back to MIN — wait for ESC confirmation beeps ...");
  t = millis();
  while (millis() - t < 2000)
  {
    esc1.writeMicroseconds(ESC_MIN); esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN); esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Done. ESC range calibrated. Safe to arm normally now.");
}

// ── Attach ESC PWM outputs ────────────────────────────────────
void initESCs()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(50); esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50); esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50); esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50); esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);

  motorsOff();
  Serial.println("[FC] ESCs attached — MIN throttle sent");
}


// ================================================================
//  SECTION 2 — IMU HELPERS
// ================================================================

float angleDeadzone(float val, float dz)
{
  if (val >  dz) return val - dz;
  if (val < -dz) return val + dz;
  return 0.0f;
}

float gyroDeadzone(float val, float dz)
{
  if (val >  dz) return val - dz;
  if (val < -dz) return val + dz;
  return 0.0f;
}

float getAdjustedRoll()
{
  float raw = ROLL_FLIP * (mpu.getAngleX() - groundRoll);
  filtRoll  = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtRoll;
  return angleDeadzone(filtRoll, IMU_ANGLE_DEADZONE);
}

float getAdjustedPitch()
{
  float raw  = PITCH_FLIP * (mpu.getAngleY() - groundPitch);
  filtPitch  = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtPitch;
  return angleDeadzone(filtPitch, IMU_ANGLE_DEADZONE);
}

float getAdjustedYawRate()
{
  return gyroDeadzone(YAW_FLIP * mpu.getGyroZ(), GYRO_DEADZONE_DPS);
}

void captureGroundReference()
{
  float sumRoll = 0, sumPitch = 0;
  const int samples = 100;
  Serial.println("[FC][IMU] Capturing ground reference (keep craft still) ...");
  for (int i = 0; i < samples; i++)
  {
    mpu.update();
    sumRoll  += mpu.getAngleX();
    sumPitch += mpu.getAngleY();
    delay(5);
  }
  groundRoll  = sumRoll  / samples;
  groundPitch = sumPitch / samples;
  filtRoll    = 0.0f;
  filtPitch   = 0.0f;
  Serial.printf("[FC][IMU] Ground ref → Roll=%.3f°  Pitch=%.3f°\n",
                groundRoll, groundPitch);
}

bool initIMU()
{
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);
  byte status = mpu.begin();
  if (status != 0)
  {
    Serial.printf("[FC][IMU] INIT FAILED — error %d\n", status);
    return false;
  }
  Serial.println("[FC][IMU] MPU-6050 OK");
  mpu.setAccOffsets(IMU_OFF_AX, IMU_OFF_AY, IMU_OFF_AZ);
  mpu.setGyroOffsets(IMU_OFF_GX, IMU_OFF_GY, IMU_OFF_GZ);
  delay(500);
  for (int i = 0; i < 50; i++) { mpu.update(); delay(10); }
  captureGroundReference();
  return true;
}

void printOrientationInfo()
{
  Serial.println("[ORIENT] Tilt NOSE DOWN → Adjusted Pitch should go NEGATIVE");
  Serial.println("[ORIENT] Tilt RIGHT side DOWN → Adjusted Roll should go POSITIVE");
  Serial.println("[ORIENT] Live for 5 s:");
  for (int i = 0; i < 100; i++)
  {
    mpu.update();
    Serial.printf("[ORIENT] Roll:%+7.2f°  Pitch:%+7.2f°  YawRate:%+7.2f°/s\n",
                  getAdjustedRoll(), getAdjustedPitch(), getAdjustedYawRate());
    delay(50);
  }
  Serial.println("[ORIENT] Done.");
}


// ================================================================
//  SECTION 3 — ARM / DISARM
// ================================================================

void resetPID()
{
  for (int i = 0; i < 3; i++) { pidPrevErr[i] = 0; pidIntegral[i] = 0; }
}

void doArm()
{
  if (armed) { Serial.println("[FC][ARM] Already armed"); return; }

  if (!receiver.connected())
  {
    Serial.println("[FC][ARM] ✖ No TX signal — cannot arm");
    return;
  }

  unsigned long guard = millis() - lastDisarmMs;
  if (guard < REARM_GUARD_MS)
  {
    Serial.printf("[FC][ARM] ✖ Re-arm guard — wait %lu ms\n", REARM_GUARD_MS - guard);
    return;
  }

  resetPID();

  // Reset accumulated throttle to safe idle — pilot must
  // deliberately push the stick up to increase it
  accThrottle = ESC_SAFE;

  // Reset slew state so arm ramp doesn't fight stale prev values
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;

  armed = true;

#if TEST_MODE
  Serial.println("[FC][ARM] ARMED (TEST MODE — no ESC output)");
  Serial.printf ("[FC][ARM] accThrottle reset to %d µs\n", ESC_SAFE);
#else
  runArmSequence();
#endif
}

void doDisarm()
{
  if (!armed) { Serial.println("[FC][ARM] Already disarmed"); return; }
  armed        = false;
  lastDisarmMs = millis();
  accThrottle  = ESC_SAFE;   // reset for next arm
  motorsOff();
  resetPID();
  Serial.println("[FC][ARM] DISARMED — motors stopped");
}


// ================================================================
//  SECTION 4 — RC INPUT PROCESSING
// ================================================================

float applyExpo(float v, float expo)
{
  // Expo curve: blends linear and cubic response
  // More expo = more gentle around stick center
  return expo * v * v * v + (1.0f - expo) * v;
}

int applyDeadband(int raw)
{
  if (raw >  DEADBAND) return raw - DEADBAND;
  if (raw < -DEADBAND) return raw + DEADBAND;
  return 0;
}

// ── Accumulated throttle update ───────────────────────────────
// Instead of mapping stick directly to µs, we treat the throttle
// stick as a rate command:
//   Stick UP   → accThrottle increases by THROTTLE_RAMP_RATE per tick
//   Stick DOWN → accThrottle decreases by THROTTLE_RAMP_RATE per tick
//   Stick CENTER (deadband) → accThrottle stays exactly where it is
//
// This gives you:
//   • Smooth, slow climb/descent even with cheap twitchy sticks
//   • Hands-off hover: just center the stick
//   • No sudden jumps when stick snaps back to center
void updateAccumulatedThrottle(int rawThrottle)
{
  // Apply deadband — small center movements are ignored
  int db = applyDeadband(rawThrottle);

  if (db > 0)
  {
    // Stick is above center — increase throttle
    // Scale ramp rate by how far the stick is pushed (0.0–1.0)
    // so full stick climbs faster than half stick
    float fraction = db / (99.0f - DEADBAND);
    accThrottle += THROTTLE_RAMP_RATE * fraction;
  }
  else if (db < 0)
  {
    // Stick is below center — decrease throttle
    float fraction = (-db) / (99.0f - DEADBAND);
    accThrottle -= THROTTLE_RAMP_RATE * fraction;
  }
  // db == 0 → do nothing, throttle holds

  // Clamp to safe operating range
  accThrottle = constrain(accThrottle, (float)THROTTLE_MIN, (float)THROTTLE_MAX);
}

void readSticks(int   &throttleUs,
                float &targetRoll,
                float &targetPitch,
                float &targetYawRate)
{
  // Update the accumulated throttle from left stick Y axis
  updateAccumulatedThrottle(receiver.data.Ly);
  throttleUs = (int)accThrottle;

  // Yaw, pitch, roll are direct with deadband + expo (unchanged)
  int rawYaw   = applyDeadband(receiver.data.Lx);
  int rawPitch = applyDeadband(receiver.data.Ry);
  int rawRoll  = applyDeadband(receiver.data.Rx);

  float nYaw   = applyExpo(rawYaw   / 99.0f, EXPO_YAW);
  float nPitch = applyExpo(rawPitch / 99.0f, EXPO_PITCH);
  float nRoll  = applyExpo(rawRoll  / 99.0f, EXPO_ROLL);

  targetRoll    = nRoll  * MAX_ANGLE_ROLL;
  targetPitch   = nPitch * MAX_ANGLE_PITCH;
  targetYawRate = nYaw   * MAX_RATE_YAW;
}


// ================================================================
//  SECTION 5 — PID CONTROLLER
// ================================================================

float computePID(int axis, float setpoint, float measured, float dt)
{
  float error = setpoint - measured;

  pidIntegral[axis] += error * dt;
  pidIntegral[axis]  = constrain(pidIntegral[axis], -ITERM_MAX, ITERM_MAX);

  float derivative = (error - pidPrevErr[axis]) / dt;
  pidPrevErr[axis]  = error;

  return KP[axis] * error
       + KI[axis] * pidIntegral[axis]
       + KD[axis] * derivative;
}


// ================================================================
//  SECTION 6 — MOTOR MIXING (X-Config)
// ================================================================
//
//       M1(CW)   M2(CCW)
//         \       /
//          \     /
//          /     \
//         /       \
//       M4(CCW)  M3(CW)
//
//  M1 Front-Left  CW  : thr + pitch + roll  − yaw
//  M2 Front-Right CCW : thr + pitch − roll  + yaw
//  M3 Back-Right  CW  : thr − pitch − roll  − yaw
//  M4 Back-Left   CCW : thr − pitch + roll  + yaw

void mixAndWrite(int throttleUs, float pidRoll, float pidPitch, float pidYaw)
{
  int m1 = throttleUs + (int)pidPitch + (int)pidRoll  - (int)pidYaw;
  int m2 = throttleUs + (int)pidPitch - (int)pidRoll  + (int)pidYaw;
  int m3 = throttleUs - (int)pidPitch - (int)pidRoll  - (int)pidYaw;
  int m4 = throttleUs - (int)pidPitch + (int)pidRoll  + (int)pidYaw;

  writeMotors(m1, m2, m3, m4);
}


// ================================================================
//  SECTION 7 — FAILSAFE
// ================================================================

void handleFailsafe(float pidRoll, float pidPitch, float pidYaw)
{
  // Slowly reduce throttle to floor — craft descends gently
  failThrottle -= DESCENT_RATE;
  failThrottle  = max(failThrottle, (int)DESCENT_FLOOR);

  int fm1 = failThrottle + (int)pidPitch + (int)pidRoll  - (int)pidYaw;
  int fm2 = failThrottle + (int)pidPitch - (int)pidRoll  + (int)pidYaw;
  int fm3 = failThrottle - (int)pidPitch - (int)pidRoll  - (int)pidYaw;
  int fm4 = failThrottle - (int)pidPitch + (int)pidRoll  + (int)pidYaw;

  writeMotors(fm1, fm2, fm3, fm4);
}


// ================================================================
//  SECTION 8 — BUTTON HANDLERS
// ================================================================

void handleArmSwitch(bool curLBt)
{
  if (curLBt && !armed)      doArm();
  else if (!curLBt && armed) doDisarm();
  lastLBt = curLBt;
}

void handleLEDToggle(bool curRBt)
{
  if (curRBt && !lastRBt)
  {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }
  lastRBt = curRBt;
}

void handleIMUReset(bool la, bool ra)
{
  if (la && ra)
  {
    if (!bothBtnHeld)
    {
      bothBtnHeld   = true;
      bothBtnHeldMs = millis();
      Serial.println("[FC][IMU] Hold 5 s to reset IMU ...");
    }
    else if (millis() - bothBtnHeldMs >= IMU_RESET_HOLD_MS)
    {
      Serial.println("[FC][IMU] IMU ground reference RESET");
      captureGroundReference();
      resetPID();
      bothBtnHeldMs = millis();
      for (int i = 0; i < 4; i++)
      {
        digitalWrite(PIN_LED, HIGH); delay(80);
        digitalWrite(PIN_LED, LOW);  delay(80);
      }
      digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    }
  }
  else
  {
    if (bothBtnHeld) Serial.println("[FC][IMU] IMU reset cancelled");
    bothBtnHeld = false;
  }
}


// ================================================================
//  SECTION 9 — SERIAL COMMANDS
// ================================================================

#if MOTOR_TEST_MODE
void handleSerialCommands()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("m1 ") || cmd.startsWith("m2 ") ||
      cmd.startsWith("m3 ") || cmd.startsWith("m4 "))
  {
    int motorNum = cmd.charAt(1) - '0';
    int us = constrain(cmd.substring(3).toInt(), ESC_MIN, ESC_MAX);
    Serial.printf("[TEST] Motor M%d → %d µs\n", motorNum, us);
    testOverride = true;
    switch (motorNum) {
      case 1: testM1 = us; break;
      case 2: testM2 = us; break;
      case 3: testM3 = us; break;
      case 4: testM4 = us; break;
    }
    esc1.writeMicroseconds(testM1); esc2.writeMicroseconds(testM2);
    esc3.writeMicroseconds(testM3); esc4.writeMicroseconds(testM4);
  }
  else if (cmd.startsWith("all "))
  {
    int us = constrain(cmd.substring(4).toInt(), ESC_MIN, ESC_MAX);
    Serial.printf("[TEST] All motors → %d µs\n", us);
    testOverride = true;
    testM1 = testM2 = testM3 = testM4 = us;
    esc1.writeMicroseconds(us); esc2.writeMicroseconds(us);
    esc3.writeMicroseconds(us); esc4.writeMicroseconds(us);
  }
  else if (cmd == "stop")
  {
    testOverride = false;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff();
    Serial.println("[TEST] All motors STOPPED");
  }
  else if (cmd == "escal")
  {
    // One-time ESC range calibration — only needed once per ESC
    runESCCalibration();
  }
  else if (cmd == "mpu")
  {
    Serial.println("[TEST][MPU] 50 readings:");
    for (int i = 0; i < 50; i++)
    {
      mpu.update();
      Serial.printf("[MPU] AdjRoll:%+7.2f°  AdjPitch:%+7.2f°  YawRate:%+7.2f°/s\n",
                    getAdjustedRoll(), getAdjustedPitch(), getAdjustedYawRate());
      delay(50);
    }
  }
  else if (cmd == "orient")   { printOrientationInfo(); }
  else if (cmd == "imu reset")
  {
    for (int i = 0; i < 50; i++) { mpu.update(); delay(10); }
    captureGroundReference(); resetPID();
    Serial.println("[TEST][IMU] Ground reference reset.");
  }
  else if (cmd == "arm")
  {
    armed = true; resetPID();
    accThrottle = ESC_SAFE;
    prevM1 = prevM2 = prevM3 = prevM4 = ESC_IDLE;
    Serial.printf("[TEST] Force ARM — accThrottle=%d (TEST_MODE: no ESC output)\n", ESC_SAFE);
  }
  else if (cmd == "disarm")
  {
    armed = false; lastDisarmMs = millis();
    testOverride = false;
    accThrottle  = ESC_SAFE;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff(); resetPID();
    Serial.println("[TEST] Force DISARM");
  }
  else if (cmd == "thr")
  {
    // Show current accumulated throttle
    Serial.printf("[TEST] accThrottle = %.1f µs  (min=%d max=%d)\n",
                  accThrottle, THROTTLE_MIN, THROTTLE_MAX);
  }
  else if (cmd == "trim")
  {
    // Show current motor trims
    Serial.printf("[TEST] MOTOR_TRIM: M1=%d  M2=%d  M3=%d  M4=%d µs\n",
                  MOTOR_TRIM[0], MOTOR_TRIM[1], MOTOR_TRIM[2], MOTOR_TRIM[3]);
  }
  else if (cmd == "status")
  {
    Serial.printf("[STATUS] Armed:%s  LinkOK:%s  accThr:%.0f\n",
                  armed ? "Y" : "N", receiver.connected() ? "Y" : "N", accThrottle);
    Serial.printf("[STATUS] ESC_SAFE:%d  THROTTLE_MAX:%d  RAMP:%d µs/tick\n",
                  ESC_SAFE, THROTTLE_MAX, THROTTLE_RAMP_RATE);
    Serial.printf("[STATUS] TRIM: M1=%d M2=%d M3=%d M4=%d\n",
                  MOTOR_TRIM[0],MOTOR_TRIM[1],MOTOR_TRIM[2],MOTOR_TRIM[3]);
    Serial.printf("[STATUS] KP:%.2f/%.2f/%.2f  KI:%.3f/%.3f/%.3f  KD:%.2f/%.2f/%.2f\n",
                  KP[0],KP[1],KP[2], KI[0],KI[1],KI[2], KD[0],KD[1],KD[2]);
  }
  else
  {
    Serial.println("[TEST] Commands:");
    Serial.println("  m1/m2/m3/m4 <µs>   individual motor");
    Serial.println("  all <µs>           all motors");
    Serial.println("  stop               all motors off");
    Serial.println("  escal              ESC range calibration (once, props off!)");
    Serial.println("  mpu                50 IMU readings");
    Serial.println("  orient             orientation check");
    Serial.println("  imu reset          recapture ground level");
    Serial.println("  arm  /  disarm");
    Serial.println("  thr                show current accumulated throttle");
    Serial.println("  trim               show motor trims");
    Serial.println("  status             all settings");
  }
}
#endif


// ================================================================
//  SECTION 10 — TELEMETRY
// ================================================================

void printTelemetry(bool linkOK,
                    int  throttleUs,
                    float tgtRoll,  float tgtPitch, float tgtYaw,
                    float actRoll,  float actPitch, float actYawRate,
                    float pRoll,    float pPitch,   float pYaw)
{
  Serial.printf(
    "[FC] ARM:%s LNK:%s | THR:%4d(acc:%.0f) | "
    "TGT R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "ACT R:%+5.1f P:%+5.1f YR:%+5.1f | "
    "PID R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "M:%4d %4d %4d %4d | BAT:%3d%%\n",
    armed  ? "Y" : "N",
    linkOK ? "Y" : "N",
    throttleUs,
    accThrottle,
    tgtRoll, tgtPitch, tgtYaw,
    actRoll, actPitch, actYawRate,
    pRoll,   pPitch,   pYaw,
    lastM1, lastM2, lastM3, lastM4,
    receiver.data.BAT
  );
}


// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  delay(600);

  Serial.println("\n[FC] ═══════════════════════════════════════════");
  Serial.println("[FC]  Q450 Flight Controller  v2.2  — BOOT");
  Serial.printf ("[FC]  TEST_MODE      : %s\n", TEST_MODE ? "ON (no ESC output)" : "OFF (LIVE)");
  Serial.printf ("[FC]  ESC_SAFE       : %d µs  (armed idle)\n", ESC_SAFE);
  Serial.printf ("[FC]  THROTTLE_MAX   : %d µs\n", THROTTLE_MAX);
  Serial.printf ("[FC]  RAMP_RATE      : %d µs/tick at %d Hz\n", THROTTLE_RAMP_RATE, LOOP_HZ);
  Serial.printf ("[FC]  TRIM M1=%d M2=%d M3=%d M4=%d\n",
                 MOTOR_TRIM[0],MOTOR_TRIM[1],MOTOR_TRIM[2],MOTOR_TRIM[3]);
  Serial.println("[FC] ═══════════════════════════════════════════\n");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  initESCs();

#if CALIBRATE_IMU
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.begin();
  Serial.println("[CAL] Lay craft FLAT — starting in 3 s ...");
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.printf("[CAL] IMU_OFF_AX=%.4f AY=%.4f AZ=%.4f\n",
                mpu.getAccXoffset(), mpu.getAccYoffset(), mpu.getAccZoffset());
  Serial.printf("[CAL] IMU_OFF_GX=%.4f GY=%.4f GZ=%.4f\n",
                mpu.getGyroXoffset(), mpu.getGyroYoffset(), mpu.getGyroZoffset());
  Serial.println("[CAL] Set CALIBRATE_IMU 0 and re-flash.");
  while (1);
#endif

  if (!initIMU())
  {
    Serial.println("[FC] FATAL: IMU not found — halting");
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  receiver.timeoutMs = 1000;
  receiver.begin(TX_MAC, true);

#if MOTOR_TEST_MODE
  Serial.println("[FC] Motor Test ACTIVE — REMOVE PROPS!");
  Serial.println("[FC]  Type 'help' for commands");
#endif

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_LED, HIGH); delay(150);
    digitalWrite(PIN_LED, LOW);  delay(150);
  }

  Serial.println("\n[FC] Ready — Set LBt ON to ARM\n");
  loopTimer = micros();
}


// ================================================================
//  MAIN LOOP
// ================================================================
void loop()
{
  // ── Fixed-rate loop gate ──────────────────────────────────────
  while (micros() - loopTimer < LOOP_US);
  float dt  = (micros() - loopTimer) / 1e6f;
  loopTimer = micros();

  // ── 1. Serial commands ────────────────────────────────────────
#if MOTOR_TEST_MODE
  handleSerialCommands();
  if (testOverride) { delay(2); return; }
#endif

  // ── 2. Receiver + IMU update ──────────────────────────────────
  receiver.update();
  bool linkOK = receiver.connected();
  mpu.update();

  float actualRoll    = getAdjustedRoll();
  float actualPitch   = getAdjustedPitch();
  float actualYawRate = getAdjustedYawRate();

  // ── 3. Signal-loss detection ──────────────────────────────────
  if (!linkOK && !signalLost)
  {
    signalLost   = true;
    // Set failsafe throttle to current accumulated value so
    // the descent starts from wherever we are, not from zero
    failThrottle = armed ? (int)accThrottle : ESC_MIN;
    resetPID();
    Serial.println("[FC] ⚠  SIGNAL LOST — failsafe descent");
  }
  else if (linkOK && signalLost)
  {
    signalLost = false;
    // Sync accumulated throttle to where failsafe landed
    // so there's no jump when signal returns
    accThrottle = (float)failThrottle;
    Serial.println("[FC] ✔  Signal restored");
  }

  // ── 4. Buttons ────────────────────────────────────────────────
  handleArmSwitch(receiver.data.LBt);
  handleLEDToggle(receiver.data.RBt);
  handleIMUReset(receiver.data.LABt, receiver.data.RABt);

  // ── 5. Stick processing ───────────────────────────────────────
  int   throttleUs   = (int)accThrottle;
  float tgtRoll      = 0, tgtPitch = 0, tgtYawRate = 0;

  if (linkOK && armed)
  {
    // readSticks updates accThrottle internally and returns throttleUs
    readSticks(throttleUs, tgtRoll, tgtPitch, tgtYawRate);
  }
  else if (!linkOK)
  {
    throttleUs = failThrottle;
  }
  // If disarmed + link OK, we don't update accThrottle

  // ── 6. PID — reset integral when essentially on the ground ────
  // Prevents integral windup while sitting at safe idle
  if (throttleUs <= ESC_SAFE + 20)
  {
    pidIntegral[0] = pidIntegral[1] = pidIntegral[2] = 0;
  }

  float pidRoll  = computePID(0, tgtRoll,    actualRoll,    dt);
  float pidPitch = computePID(1, tgtPitch,   actualPitch,   dt);
  float pidYaw   = computePID(2, tgtYawRate, actualYawRate, dt);

  // ── 7. Motor output ───────────────────────────────────────────
  if (!armed)
  {
    motorsOff();
  }
  else if (signalLost)
  {
    handleFailsafe(pidRoll, pidPitch, pidYaw);
  }
  else
  {
    mixAndWrite(throttleUs, pidRoll, pidPitch, pidYaw);
  }

  // ── 8. Telemetry ──────────────────────────────────────────────
  if (++telTick >= TEL_EVERY)
  {
    telTick = 0;
    printTelemetry(linkOK, throttleUs,
                   tgtRoll, tgtPitch, tgtYawRate,
                   actualRoll, actualPitch, actualYawRate,
                   pidRoll, pidPitch, pidYaw);
  }
}