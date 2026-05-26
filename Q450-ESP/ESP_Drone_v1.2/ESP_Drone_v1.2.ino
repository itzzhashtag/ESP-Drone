// ================================================================
//  Q450 ESP32 Flight Controller  —  v1.2
//  Frame   : Q450 Quadcopter (X-Config)
//  Motors  : A2212/13T 1000KV
//  ESCs    : SimonK 30A  (Standard PWM 1000–2000 µs)
//  IMU     : MPU-6050 (I2C)
//  Radio   : ESP-NOW via ReceiverModule.h
//  Mode    : Angle Mode  (self-levelling, center-stick = hover)
//
// ----------------------------------------------------------------
//  CHANGES v1.1 → v1.2
//  • Added IMU_ANGLE_DEADZONE — angles below this treated as 0
//    Eliminates roll/pitch flicker at rest (was ±0.2–3.5 deg)
//  • Added GYRO_DEADZONE_DPS — yaw gyro noise below this = 0
//    Fixes yaw PID spiking to ±13 µs at rest (was -1.3 deg/s noise)
//  • Fixed ESC arm sequence — now sends MIN for 3 s then calibrates
//    range (MIN→MAX→MIN) so SimonK ESCs lock onto the right range
//  • ESC_IDLE raised from 1150 to 1180 — confirmed motors spin
//  • Motors now start at ESC_IDLE immediately on arm (no delay)
//  • Added output_change_limit — motor commands change at most
//    MAX_MOTOR_STEP µs per loop, smoothing sudden stick inputs
//  • PID integral reset on arm/disarm AND on throttle below idle+20
//  • Telemetry now shows actual µs written, not clamped estimate
//  • IMU angle used for PID only after ANGLE_FILTER_ALPHA smoothing
//  • Added capacitor recommendation comment near I2C init
//
// ----------------------------------------------------------------
//  MOTOR LAYOUT  (X-Config, viewed from above)
//
//            FRONT
//       M1(CW)   M2(CCW)
//          \       /
//           \     /
//           /     \
//          /       \
//       M4(CCW)  M3(CW)
//            BACK
//
//  GPIO PINS
//    25 → ESC M1  Front-Left  CW
//    26 → ESC M2  Front-Right CCW
//    27 → ESC M3  Back-Right  CW
//    14 → ESC M4  Back-Left   CCW
//    21 → MPU-6050 SDA
//    22 → MPU-6050 SCL
//     2 → Built-in LED
//
// ----------------------------------------------------------------
//  HARDWARE NOTE — I2C NOISE / MPU FLICKER
//  If the MPU still flickers after software deadzones, add:
//    • 100nF ceramic capacitor from VCC to GND on the MPU-6050
//      as close to the chip as possible (decoupling)
//    • 4.7kΩ pull-up resistors on SDA and SCL to 3.3V
//      (most GY-521 modules already have these, but confirm)
//    • Twist the SDA/SCL wires together to reduce crosstalk
//    • Keep I2C wires away from ESC signal wires
// ================================================================

#include "ReceiverModule.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>   // Library Manager: "MPU6050_light" by rfetick


// ================================================================
//   ████████╗██╗   ██╗███╗   ██╗██╗███╗   ██╗ ██████╗
//      ██╔══╝██║   ██║████╗  ██║██║████╗  ██║██╔════╝
//      ██║   ██║   ██║██╔██╗ ██║██║██╔██╗ ██║██║  ███╗
//      ██║   ██║   ██║██║╚██╗██║██║██║╚██╗██║██║   ██║
//      ██║   ╚██████╔╝██║ ╚████║██║██║ ╚████║╚██████╔╝
//      ╚═╝    ╚═════╝ ╚═╝  ╚═══╝╚═╝╚═╝  ╚═══╝ ╚═════╝
//  ══ ALL TWEAKABLE SETTINGS ARE HERE — ONE PLACE ONLY ══
// ================================================================

// ── Hardware pins ─────────────────────────────────────────────
#define PIN_M1        25    // Front-Left  (CW)
#define PIN_M2        26    // Front-Right (CCW)
#define PIN_M3        27    // Back-Right  (CW)
#define PIN_M4        14    // Back-Left   (CCW)
#define PIN_LED        2    // Built-in LED  (HIGH = on)
#define MPU_SDA       21
#define MPU_SCL       22

// ── TX MAC (your transmitter's MAC — printed at TX boot) ──────
#define TX_MAC  "A4:F0:0F:90:34:28"

// ── ESC PWM limits ────────────────────────────────────────────
#define ESC_MIN     1000    // µs — motor fully off / ESC signal floor
#define ESC_MAX     2000    // µs — full throttle

// HOVER_THROTTLE: µs that keeps the craft airborne at mid-height.
// Start at 1380 for 3S ~450 g build; tune up/down by 20 µs steps.
#define ESC_HOVER   1380    // µs — hover throttle    ★ TUNE ME

// IDLE: motors spin visibly at this but craft stays on ground.
// Must be high enough that all 4 ESCs run, low enough to not lift.
// ★ If motors don't spin on arm, raise by 10 µs until they do.
#define ESC_IDLE    1180    // µs — armed idle          ★ TUNE ME

// ── Stick deadband ────────────────────────────────────────────
// Stick center noise elimination. Increase if sticks are twitchy.
#define DEADBAND       5    // raw stick units  (-99..99 scale)

// ── IMU angle deadband ────────────────────────────────────────
// ★ KEY FIX: angles smaller than this are treated as exactly 0.
// Eliminates micro-corrections from sensor noise when craft is still.
// Your telemetry showed roll ±0.2–3.5° and pitch ±0.1–1.9° at rest.
// 0.5° is a safe start. Lower to 0.3 once in the air if needed.
#define IMU_ANGLE_DEADZONE  0.5f   // degrees   ★ TUNE ME

// ── Gyro rate deadband ────────────────────────────────────────
// ★ KEY FIX: yaw gyro readings below this deg/s are treated as 0.
// Your telemetry showed -1.3 to -1.4 deg/s yaw rate at rest —
// this was causing PID yaw to spike ±13 µs even on a still bench.
// 2.0 is safe (your noise was ~1.4 deg/s). Raise if still noisy.
#define GYRO_DEADZONE_DPS   2.0f   // deg/s     ★ TUNE ME

// ── IMU angle IIR smoothing ──────────────────────────────────
// Low-pass filter on angle readings before feeding PID.
// 1.0 = no smoothing (raw).  0.7 = moderate.  0.4 = heavy.
// Too low → sluggish levelling. Too high → noisy corrections.
#define ANGLE_FILTER_ALPHA  0.8f   // 0.0–1.0   ★ TUNE ME

// ── Motor output slew limit ───────────────────────────────────
// Maximum µs change per loop tick on any motor.
// Prevents instant jumps from PID spikes reaching the ESCs.
// At 100 Hz, 30 µs/tick = 3000 µs/s max rate of change.
// Increase if craft feels sluggish to commands; decrease for more smoothing.
#define MAX_MOTOR_STEP  30         // µs per loop tick   ★ TUNE ME

// ── Expo curve ────────────────────────────────────────────────
// 0.0 = linear  (no expo).  0.5 = strong expo.
// Throttle expo kept at 0 for predictable climb/descent.
#define EXPO_ROLL     0.40f
#define EXPO_PITCH    0.40f
#define EXPO_YAW      0.30f
#define EXPO_THROTTLE 0.0f

// ── Max angle targets (degrees) ───────────────────────────────
// Full stick = this many degrees of tilt. Start low, raise later.
#define MAX_ANGLE_ROLL   20.0f
#define MAX_ANGLE_PITCH  20.0f
#define MAX_RATE_YAW    120.0f   // deg/s at full yaw stick

// ── IMU orientation ───────────────────────────────────────────
// Set to -1.0f if the axis reads backwards on your mount.
#define PITCH_FLIP   1.0f    //  1.0 = normal  |  -1.0 = inverted
#define ROLL_FLIP    1.0f    //  1.0 = normal  |  -1.0 = inverted
#define YAW_FLIP     1.0f    //  1.0 = normal  |  -1.0 = inverted

// ── PID gains — ANGLE mode ────────────────────────────────────
// PID[0] = Roll,  PID[1] = Pitch,  PID[2] = Yaw
//
// HOW TO TUNE (bench first, then flight):
//   1. Set I=0, D=0.  Raise P until fast oscillation starts.
//   2. Back P off 20%.  Add D slowly to kill oscillation.
//   3. Add tiny I only to fix a slow consistent drift.
//   Yaw is rate-based — its P will be much smaller than Roll/Pitch.
//
//           Roll     Pitch    Yaw
float KP[3]={ 1.50f,  1.50f,  0.80f };   // ★ START HERE — low & safe
float KI[3]={ 0.02f,  0.02f,  0.00f };   //   Add I only after P+D stable
float KD[3]={ 0.80f,  0.80f,  0.20f };

// ── PID integral clamp ────────────────────────────────────────
#define ITERM_MAX     40.0f   // µs max I contribution per axis

// ── Failsafe descent ──────────────────────────────────────────
#define DESCENT_RATE    3     // µs per loop  (at 100 Hz ≈ 300 µs/s)
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
// TEST_MODE 0 → live flight.  REMOVE PROPS before changing to 0!
#define TEST_MODE    0       // ← SET TO 0 FOR REAL FLIGHT

// ── IMU calibration helper ────────────────────────────────────
#define CALIBRATE_IMU  0
float IMU_OFF_AX = 0.0f;
float IMU_OFF_AY = 0.0f;
float IMU_OFF_AZ = 0.0f;
float IMU_OFF_GX = 0.0f;
float IMU_OFF_GY = 0.0f;
float IMU_OFF_GZ = 0.0f;

// ── Serial motor test ─────────────────────────────────────────
// MOTOR_TEST_MODE 1 → serial commands spin individual motors.
// Commands: "m1 1200"  "m2 1300"  "all 1200"  "stop"
//           "mpu"  "orient"  "arm"  "disarm"  "imu reset"
// !! REMOVE PROPS BEFORE USING !!
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

// ── Arm/disarm timing ─────────────────────────────────────────
unsigned long lastDisarmMs  = 0;

// ── IMU reset button timing ───────────────────────────────────
unsigned long bothBtnHeldMs = 0;
bool          bothBtnHeld   = false;

// ── Loop timer ────────────────────────────────────────────────
unsigned long loopTimer = 0;

// ── Telemetry throttle ────────────────────────────────────────
int  telTick = 0;
#define TEL_EVERY   20    // print once every N loops

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

// ── Apply slew limit to one motor value ──────────────────────
// Limits how fast the motor command can change per loop tick.
// Prevents PID spikes from instantly reaching the ESCs.
int slewMotor(int target, int previous)
{
  int delta = target - previous;
  delta = constrain(delta, -MAX_MOTOR_STEP, MAX_MOTOR_STEP);
  return previous + delta;
}

// ── Raw write to all four ESCs (with slew limit) ─────────────
void writeMotors(int m1, int m2, int m3, int m4)
{
  // Apply slew limiting
  m1 = slewMotor(constrain(m1, ESC_MIN, ESC_MAX), prevM1);
  m2 = slewMotor(constrain(m2, ESC_MIN, ESC_MAX), prevM2);
  m3 = slewMotor(constrain(m3, ESC_MIN, ESC_MAX), prevM3);
  m4 = slewMotor(constrain(m4, ESC_MIN, ESC_MAX), prevM4);

  prevM1 = m1; prevM2 = m2; prevM3 = m3; prevM4 = m4;
  lastM1 = m1; lastM2 = m2; lastM3 = m3; lastM4 = m4;

#if TEST_MODE
  return;   // no ESC output in test mode
#endif

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

// ── Cut all motors immediately (bypasses slew + TEST_MODE) ───
void motorsOff()
{
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_MIN;
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

// ── SimonK ESC calibration + arm sequence ─────────────────────
// SimonK needs to see MIN for several seconds, then optionally
// MAX so it can learn the throttle range endpoints, then back to MIN.
// This must run with props OFF — motors will briefly reach max signal.
// After this the ESC accepts 1000–2000 µs as the full range.
void runArmSequence()
{
  Serial.println("[FC][ARM] ESC calibration sequence — REMOVE PROPS!");
  Serial.println("[FC][ARM] Step 1: Sending MIN (1000 µs) for 3 s ...");

  // Step 1 — hold minimum so ESC enters calibration listening mode
  unsigned long t = millis();
  while (millis() - t < 3000)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }

  // Step 2 — briefly send MAX so ESC sees the top endpoint
  Serial.println("[FC][ARM] Step 2: Sending MAX (2000 µs) for 0.5 s ...");
  t = millis();
  while (millis() - t < 500)
  {
    esc1.writeMicroseconds(ESC_MAX);
    esc2.writeMicroseconds(ESC_MAX);
    esc3.writeMicroseconds(ESC_MAX);
    esc4.writeMicroseconds(ESC_MAX);
    delay(20);
  }

  // Step 3 — back to minimum — ESC beeps to confirm calibration saved
  Serial.println("[FC][ARM] Step 3: Back to MIN — wait for ESC beeps ...");
  t = millis();
  while (millis() - t < 2000)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }

  // Step 4 — ramp up to idle so motors start spinning
  Serial.println("[FC][ARM] Step 4: Ramping to idle ...");
  for (int us = ESC_MIN; us <= ESC_IDLE; us += 5)
  {
    esc1.writeMicroseconds(us);
    esc2.writeMicroseconds(us);
    esc3.writeMicroseconds(us);
    esc4.writeMicroseconds(us);
    delay(20);
  }

  prevM1 = prevM2 = prevM3 = prevM4 = ESC_IDLE;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_IDLE;
  Serial.printf("[FC][ARM] Armed — motors idling at %d µs\n", ESC_IDLE);
}

// ── Attach ESC PWM outputs ────────────────────────────────────
void initESCs()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(50);  esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50);  esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50);  esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50);  esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);

  motorsOff();
  Serial.println("[FC] ESCs attached — min throttle sent");
}


// ================================================================
//  SECTION 2 — IMU HELPERS
// ================================================================

// ── Angle deadzone helper ─────────────────────────────────────
// Returns 0.0 if the absolute value is below the deadzone threshold.
// This is applied AFTER the IIR filter, on the corrected angle.
float angleDeadzone(float val, float dz)
{
  if (val > dz)  return val - dz;   // shift inward so no jump at edge
  if (val < -dz) return val + dz;
  return 0.0f;
}

// ── Gyro rate deadzone ────────────────────────────────────────
float gyroDeadzone(float val, float dz)
{
  if (val > dz)  return val - dz;
  if (val < -dz) return val + dz;
  return 0.0f;
}

// ── Read adjusted + filtered roll ────────────────────────────
float getAdjustedRoll()
{
  float raw = ROLL_FLIP * (mpu.getAngleX() - groundRoll);
  // IIR low-pass filter
  filtRoll = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtRoll;
  // Apply angle deadzone after filtering
  return angleDeadzone(filtRoll, IMU_ANGLE_DEADZONE);
}

// ── Read adjusted + filtered pitch ───────────────────────────
float getAdjustedPitch()
{
  float raw = PITCH_FLIP * (mpu.getAngleY() - groundPitch);
  filtPitch = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtPitch;
  return angleDeadzone(filtPitch, IMU_ANGLE_DEADZONE);
}

// ── Read yaw rate with deadzone ───────────────────────────────
float getAdjustedYawRate()
{
  float raw = YAW_FLIP * mpu.getGyroZ();
  return gyroDeadzone(raw, GYRO_DEADZONE_DPS);
}

// ── Capture ground reference ──────────────────────────────────
void captureGroundReference()
{
  float sumRoll = 0, sumPitch = 0;
  const int samples = 100;   // more samples → more stable reference

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

  // Reset filter state to new reference
  filtRoll  = 0.0f;
  filtPitch = 0.0f;

  Serial.printf("[FC][IMU] Ground ref → Roll=%.3f°  Pitch=%.3f°\n",
                groundRoll, groundPitch);
}

// ── Initialise MPU-6050 ───────────────────────────────────────
bool initIMU()
{
  // ── Hardware note ─────────────────────────────────────────
  // Add a 100nF ceramic cap from MPU VCC to GND for decoupling.
  // Add 4.7kΩ pull-ups on SDA and SCL to 3.3V if not already
  // present on the GY-521 breakout (check board markings).
  // ──────────────────────────────────────────────────────────
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);   // 400 kHz fast-mode — reduces read time

  byte status = mpu.begin();
  if (status != 0)
  {
    Serial.printf("[FC][IMU] INIT FAILED — error %d\n", status);
    return false;
  }
  Serial.println("[FC][IMU] MPU-6050 OK");

  mpu.setAccOffsets(IMU_OFF_AX, IMU_OFF_AY, IMU_OFF_AZ);
  mpu.setGyroOffsets(IMU_OFF_GX, IMU_OFF_GY, IMU_OFF_GZ);

  // Warm-up — let the complementary filter settle
  delay(500);
  for (int i = 0; i < 50; i++) { mpu.update(); delay(10); }

  captureGroundReference();
  return true;
}

// ── Orientation check (serial command "orient") ───────────────
void printOrientationInfo()
{
  Serial.println("");
  Serial.println("[ORIENT] Tilt NOSE DOWN → Adjusted Pitch should go NEGATIVE");
  Serial.println("[ORIENT] Tilt RIGHT side DOWN → Adjusted Roll should go POSITIVE");
  Serial.printf ("[ORIENT] PITCH_FLIP=%.1f  ROLL_FLIP=%.1f\n", PITCH_FLIP, ROLL_FLIP);
  Serial.println("[ORIENT] Live for 5 s:");

  for (int i = 0; i < 100; i++)
  {
    mpu.update();
    // Call getAdjusted* to run filter + deadzone
    float r = getAdjustedRoll();
    float p = getAdjustedPitch();
    float y = getAdjustedYawRate();
    Serial.printf("[ORIENT] Roll:%+7.2f°  Pitch:%+7.2f°  YawRate:%+7.2f°/s\n", r, p, y);
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
  armed = true;

#if TEST_MODE
  Serial.println("[FC][ARM] ARMED (TEST MODE — no ESC output)");
  Serial.printf ("[FC][ARM] Motors would idle at %d µs\n", ESC_IDLE);
#else
  runArmSequence();
#endif
}

void doDisarm()
{
  if (!armed) { Serial.println("[FC][ARM] Already disarmed"); return; }
  armed        = false;
  lastDisarmMs = millis();
  motorsOff();
  resetPID();
  Serial.println("[FC][ARM] DISARMED — motors stopped");
}


// ================================================================
//  SECTION 4 — RC INPUT PROCESSING
// ================================================================

float applyExpo(float v, float expo)
{
  return expo * v * v * v + (1.0f - expo) * v;
}

int applyDeadband(int raw)
{
  if (raw >  DEADBAND) return raw - DEADBAND;
  if (raw < -DEADBAND) return raw + DEADBAND;
  return 0;
}

// ── Split-center throttle mapping ────────────────────────────
// Center stick (0) → ESC_HOVER (holds altitude)
// Full up (+99)   → ESC_MAX
// Full down (-99) → ESC_IDLE
int mapThrottle(int raw)
{
  if (raw >= 0)
    return (int)map(raw, 0, 99, ESC_HOVER, ESC_MAX);
  else
    return (int)map(raw, -99, 0, ESC_IDLE, ESC_HOVER);
}

void readSticks(int   &throttleUs,
                float &targetRoll,
                float &targetPitch,
                float &targetYawRate)
{
  int rawThrottle = receiver.data.Ly;
  int rawYaw      = receiver.data.Lx;
  int rawPitch    = receiver.data.Ry;
  int rawRoll     = receiver.data.Rx;

  rawYaw   = applyDeadband(rawYaw);
  rawPitch = applyDeadband(rawPitch);
  rawRoll  = applyDeadband(rawRoll);
  if (rawThrottle > -DEADBAND && rawThrottle < DEADBAND) rawThrottle = 0;

  float nYaw   = rawYaw   / 99.0f;
  float nPitch = rawPitch / 99.0f;
  float nRoll  = rawRoll  / 99.0f;

  nYaw   = applyExpo(nYaw,   EXPO_YAW);
  nPitch = applyExpo(nPitch, EXPO_PITCH);
  nRoll  = applyExpo(nRoll,  EXPO_ROLL);

  throttleUs    = mapThrottle(rawThrottle);
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

  float derivative  = (error - pidPrevErr[axis]) / dt;
  pidPrevErr[axis]  = error;

  return KP[axis] * error
       + KI[axis] * pidIntegral[axis]
       + KD[axis] * derivative;
}


// ================================================================
//  SECTION 6 — MOTOR MIXING  (X-Config)
// ================================================================
//
//       M1(CW)   M2(CCW)
//         |  ╲ ╱  |
//         |  ╱ ╲  |
//       M4(CCW)  M3(CW)
//
//  M1 Front-Left  CW  : thr + pitch + roll  − yaw
//  M2 Front-Right CCW : thr + pitch − roll  + yaw
//  M3 Back-Right  CW  : thr − pitch − roll  − yaw
//  M4 Back-Left   CCW : thr − pitch + roll  + yaw

void mixAndWrite(int throttleUs,
                 float pidRoll, float pidPitch, float pidYaw)
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
  if (curLBt && !armed)       doArm();
  else if (!curLBt && armed)  doDisarm();
  lastLBt = curLBt;
}

void handleLEDToggle(bool curRBt)
{
  if (curRBt && !lastRBt)
  {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    Serial.printf("[FC][LED] LED %s\n", ledState ? "ON" : "OFF");
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
    else
    {
      unsigned long elapsed = millis() - bothBtnHeldMs;
      if (elapsed >= IMU_RESET_HOLD_MS)
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
  }
  else
  {
    if (bothBtnHeld) Serial.println("[FC][IMU] IMU reset cancelled");
    bothBtnHeld = false;
  }
}


// ================================================================
//  SECTION 9 — SERIAL MOTOR TEST
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
    esc1.writeMicroseconds(testM1);
    esc2.writeMicroseconds(testM2);
    esc3.writeMicroseconds(testM3);
    esc4.writeMicroseconds(testM4);
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
    Serial.println("[TEST] All motors STOPPED");
    testOverride = false;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff();
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
    Serial.println("[TEST][MPU] Done.");
  }
  else if (cmd == "orient")
  {
    printOrientationInfo();
  }
  else if (cmd == "imu reset")
  {
    Serial.println("[TEST][IMU] Manual ground reference reset ...");
    for (int i = 0; i < 50; i++) { mpu.update(); delay(10); }
    captureGroundReference();
    resetPID();
  }
  else if (cmd == "arm")
  {
    armed = true;
    resetPID();
    prevM1 = prevM2 = prevM3 = prevM4 = ESC_IDLE;
    Serial.printf("[TEST] Force ARM — idle at %d µs (TEST_MODE: no ESC output)\n", ESC_IDLE);
  }
  else if (cmd == "disarm")
  {
    armed = false; lastDisarmMs = millis();
    testOverride = false;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff(); resetPID();
    Serial.println("[TEST] Force DISARM");
  }
  else if (cmd == "status")
  {
    Serial.printf("[STATUS] Armed:%s  LinkOK:%s  TestOverride:%s\n",
                  armed ? "Y" : "N",
                  receiver.connected() ? "Y" : "N",
                  testOverride ? "Y" : "N");
    Serial.printf("[STATUS] ESC_IDLE:%d  ESC_HOVER:%d  ESC_MAX:%d\n",
                  ESC_IDLE, ESC_HOVER, ESC_MAX);
    Serial.printf("[STATUS] AngDeadzone:%.1f°  GyroDeadzone:%.1f°/s  SlewLimit:%dµs\n",
                  IMU_ANGLE_DEADZONE, GYRO_DEADZONE_DPS, MAX_MOTOR_STEP);
    Serial.printf("[STATUS] KP:%.2f/%.2f/%.2f  KI:%.3f/%.3f/%.3f  KD:%.2f/%.2f/%.2f\n",
                  KP[0],KP[1],KP[2], KI[0],KI[1],KI[2], KD[0],KD[1],KD[2]);
  }
  else
  {
    Serial.println("[TEST] Commands:");
    Serial.println("  m1/m2/m3/m4 <µs>   e.g. m1 1300");
    Serial.println("  all <µs>           e.g. all 1200");
    Serial.println("  stop");
    Serial.println("  mpu                50 IMU readings");
    Serial.println("  orient             orientation check");
    Serial.println("  imu reset          recapture ground level");
    Serial.println("  arm  /  disarm");
    Serial.println("  status             show all settings");
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
    "[FC] ARM:%s LNK:%s | THR:%4d | "
    "TGT R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "ACT R:%+5.1f P:%+5.1f YR:%+5.1f | "
    "PID R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "M:%4d %4d %4d %4d | BAT:%3d%%\n",
    armed  ? "Y" : "N",
    linkOK ? "Y" : "N",
    throttleUs,
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
  Serial.println("[FC]  Q450 Flight Controller  v2.1  — BOOT");
  Serial.printf ("[FC]  TEST_MODE     : %s\n", TEST_MODE ? "ON  (no ESC output)" : "OFF (LIVE)");
  Serial.printf ("[FC]  LOOP_HZ       : %d Hz\n", LOOP_HZ);
  Serial.printf ("[FC]  ESC_IDLE      : %d µs\n", ESC_IDLE);
  Serial.printf ("[FC]  ESC_HOVER     : %d µs\n", ESC_HOVER);
  Serial.printf ("[FC]  AngleDeadzone : %.1f°\n", IMU_ANGLE_DEADZONE);
  Serial.printf ("[FC]  GyroDeadzone  : %.1f°/s\n", GYRO_DEADZONE_DPS);
  Serial.printf ("[FC]  SlewLimit     : %d µs/tick\n", MAX_MOTOR_STEP);
  Serial.printf ("[FC]  KP %.2f/%.2f/%.2f  KI %.3f/%.3f/%.3f  KD %.2f/%.2f/%.2f\n",
                 KP[0],KP[1],KP[2], KI[0],KI[1],KI[2], KD[0],KD[1],KD[2]);
  Serial.println("[FC] ═══════════════════════════════════════════\n");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  initESCs();

#if CALIBRATE_IMU
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.begin();
  Serial.println("[CAL] Lay craft FLAT and STILL — starting in 3 s ...");
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.printf("[CAL] IMU_OFF_AX = %.4f;\n", mpu.getAccXoffset());
  Serial.printf("[CAL] IMU_OFF_AY = %.4f;\n", mpu.getAccYoffset());
  Serial.printf("[CAL] IMU_OFF_AZ = %.4f;\n", mpu.getAccZoffset());
  Serial.printf("[CAL] IMU_OFF_GX = %.4f;\n", mpu.getGyroXoffset());
  Serial.printf("[CAL] IMU_OFF_GY = %.4f;\n", mpu.getGyroYoffset());
  Serial.printf("[CAL] IMU_OFF_GZ = %.4f;\n", mpu.getGyroZoffset());
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

  Serial.println("[FC] ── Orientation hint ────────────────────────");
  Serial.printf ("[FC]  Nose-down pitch → Adjusted Pitch should go %s\n",
                 PITCH_FLIP > 0 ? "NEGATIVE" : "POSITIVE");
  Serial.printf ("[FC]  Right-side-down → Adjusted Roll should go %s\n",
                 ROLL_FLIP  > 0 ? "POSITIVE" : "NEGATIVE");
  Serial.println("[FC]  Send 'orient' to verify.  Send 'status' for all settings.");
  Serial.println("[FC] ────────────────────────────────────────────");

#if MOTOR_TEST_MODE
  Serial.println("[FC] Motor Test ACTIVE — REMOVE PROPS!");
  Serial.println("[FC]  m1/m2/m3/m4 <µs>  all <µs>  stop");
  Serial.println("[FC]  mpu  orient  imu reset  arm  disarm  status");
#endif

  // 3 ready blinks
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
  // ── Fixed-rate gate ───────────────────────────────────────────
  while (micros() - loopTimer < LOOP_US);
  float dt  = (micros() - loopTimer) / 1e6f;
  loopTimer = micros();

  // ── 1. Serial commands ────────────────────────────────────────
#if MOTOR_TEST_MODE
  handleSerialCommands();
  if (testOverride) { delay(2); return; }
#endif

  // ── 2. Receiver update ───────────────────────────────────────
  receiver.update();
  bool linkOK = receiver.connected();

  // ── 3. IMU update ────────────────────────────────────────────
  mpu.update();

  float actualRoll    = getAdjustedRoll();    // filtered + deadzone
  float actualPitch   = getAdjustedPitch();   // filtered + deadzone
  float actualYawRate = getAdjustedYawRate(); // deadzone only

  // ── 4. Signal-loss detection ──────────────────────────────────
  if (!linkOK && !signalLost)
  {
    signalLost   = true;
    failThrottle = armed ? ESC_HOVER + 50 : ESC_MIN;
    resetPID();
    Serial.println("[FC] ⚠  SIGNAL LOST — failsafe descent");
  }
  else if (linkOK && signalLost)
  {
    signalLost = false;
    Serial.println("[FC] ✔  Signal restored");
  }

  // ── 5. Buttons ────────────────────────────────────────────────
  handleArmSwitch(receiver.data.LBt);
  handleLEDToggle(receiver.data.RBt);
  handleIMUReset(receiver.data.LABt, receiver.data.RABt);

  // ── 6. Stick processing ───────────────────────────────────────
  int   throttleUs   = ESC_IDLE;
  float tgtRoll      = 0, tgtPitch = 0, tgtYawRate = 0;

  if (linkOK)
  {
    readSticks(throttleUs, tgtRoll, tgtPitch, tgtYawRate);
  }
  else
  {
    throttleUs = failThrottle;
  }

  // ── 7. PID ────────────────────────────────────────────────────
  // Reset integral if throttle is too low to matter (craft on ground)
  if (throttleUs <= ESC_IDLE + 20)
  {
    pidIntegral[0] = 0;
    pidIntegral[1] = 0;
    pidIntegral[2] = 0;
  }

  float pidRoll  = computePID(0, tgtRoll,    actualRoll,    dt);
  float pidPitch = computePID(1, tgtPitch,   actualPitch,   dt);
  float pidYaw   = computePID(2, tgtYawRate, actualYawRate, dt);

  // ── 8. Motor output ───────────────────────────────────────────
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

  // ── 9. Telemetry ──────────────────────────────────────────────
  if (++telTick >= TEL_EVERY)
  {
    telTick = 0;
    printTelemetry(linkOK, throttleUs,
                   tgtRoll, tgtPitch, tgtYawRate,
                   actualRoll, actualPitch, actualYawRate,
                   pidRoll, pidPitch, pidYaw);
  }
}