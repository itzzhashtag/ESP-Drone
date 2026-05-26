// ================================================================
//  Q450 ESP32 Flight Controller  —  v1.1
//  Frame   : Q450 Quadcopter (X-Config)
//  Motors  : A2212/13T 1000KV
//  ESCs    : SimonK 30A  (Standard PWM 1000–2000 µs)
//  IMU     : MPU-6050 (I2C)
//  Radio   : ESP-NOW via ReceiverModule.h
//  Mode    : Angle Mode  (self-levelling, center-stick = hover)
//
// ----------------------------------------------------------------
//  KEY BEHAVIOURS and CHANGES from v1.0 → v1.1
//  • Left  stick  Y = Throttle  (center = hover, up = climb)
//  • Left  stick  X = Yaw       (center = hold heading)
//  • Right stick  Y = Pitch     (center = level, up = forward)
//  • Right stick  X = Roll      (center = level, right = roll R)
//  • LBt  ON  → ARM   (motors start at idle)
//  • LBt  OFF → DISARM (motors cut instantly)
//  • RBt  toggle → LED ON / OFF
//  • LABt + RABt held 5 s → IMU ground-reference RESET
//  • Signal lost → gradual throttle descent, craft stays level
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
#define ESC_MIN     1000    // µs  — motor fully off
#define ESC_MAX     2000    // µs  — full throttle
//
// HOVER_THROTTLE: the µs value that keeps the craft airborne
// at roughly mid-height without climbing or sinking.
// Tune this first. Start at 1350 for a 3S 450g build and adjust.
#define ESC_HOVER   1380    // µs  — hover / center-stick throttle ← TUNE ME
//
// IDLE: motors spin at this when armed but stick is fully down.
// High enough to keep ESCs alive, low enough to sit on ground.
#define ESC_IDLE    1150    // µs  — armed idle (stick at bottom)

// ── Stick deadband ────────────────────────────────────────────
// Anything within ±DEADBAND of stick center is treated as zero.
// Increase if your sticks are noisy at center.
#define DEADBAND       5    // raw stick units  (-99 .. 99 scale)

// ── Expo curve ────────────────────────────────────────────────
// Makes stick response gentle near center, sharper at edges.
// 0.0 = linear  (no expo)
// 0.5 = strong expo (recommended for beginners)
// Throttle expo is kept 0 so climb/descent stays predictable.
#define EXPO_ROLL    0.40f
#define EXPO_PITCH   0.40f
#define EXPO_YAW     0.30f
#define EXPO_THROTTLE 0.0f

// ── Max angle targets (degrees) ───────────────────────────────
// Full stick deflection commands this many degrees of tilt.
// Lower = calmer craft, easier to fly.  Start at 15-20 deg.
#define MAX_ANGLE_ROLL   20.0f   // degrees at full roll stick
#define MAX_ANGLE_PITCH  20.0f   // degrees at full pitch stick
#define MAX_RATE_YAW    120.0f   // deg/s   at full yaw stick

// ── IMU orientation ───────────────────────────────────────────
// If your MPU is mounted with X pointing toward the BACK of the
// frame instead of the front, set PITCH_FLIP to -1.0f to invert.
// Same for ROLL — if rolling right makes the drone think it's
// rolling left, set ROLL_FLIP to -1.0f.
// After flying check the serial output "Orientation Check" section.
#define PITCH_FLIP   1.0f    //  1.0 = normal  |  -1.0 = inverted
#define ROLL_FLIP    1.0f    //  1.0 = normal  |  -1.0 = inverted
#define YAW_FLIP     1.0f    //  1.0 = normal  |  -1.0 = inverted

// ── PID gains — ANGLE mode ────────────────────────────────────
// PID[0] = Roll,  PID[1] = Pitch,  PID[2] = Yaw
//
// HOW TO TUNE:
//   1. Set all I and D to 0. Raise P until fast oscillations start.
//   2. Back P off ~20%. Then slowly add D to kill remaining oscillation.
//   3. Finally add tiny I to correct slow drift.
//   Yaw is rate-based so its P will be much smaller.
//
//           Roll     Pitch    Yaw
float KP[3]={ 2.50f,  2.50f,  1.20f };   // Proportional gain
float KI[3]={ 0.03f,  0.03f,  0.01f };   // Integral gain
float KD[3]={ 1.20f,  1.20f,  0.40f };   // Derivative gain

// ── PID integral clamp ────────────────────────────────────────
// Stops the integral from winding up during hard manoeuvres.
#define ITERM_MAX     60.0f   // µs maximum I contribution per axis

// ── Failsafe descent ──────────────────────────────────────────
// When TX signal is lost the craft slowly reduces throttle to land.
// DESCENT_RATE µs are removed per loop tick until DESCENT_FLOOR.
#define DESCENT_RATE    3     // µs per loop  (at 50 Hz ≈ 150 µs/s)
#define DESCENT_FLOOR   ESC_IDLE   // never cut below idle mid-air

// ── Throttle stick mapping ────────────────────────────────────
// Center stick (0) = ESC_HOVER.
// Full up (+99) = ESC_MAX.  Full down (-99) = ESC_IDLE.
// The mapping is split: below-center maps idle..hover,
//                        above-center maps hover..max.

// ── Loop rate ─────────────────────────────────────────────────
#define LOOP_HZ      100     // Hz  — raise to 250 later for snappier response
#define LOOP_US      (1000000 / LOOP_HZ)

// ── Arm guard ─────────────────────────────────────────────────
// After every disarm the craft cannot re-arm for this long.
// Prevents accidental re-arm from button bounce.
#define REARM_GUARD_MS   2000

// ── IMU reset hold time ───────────────────────────────────────
// Both A-buttons held for this many ms → recalibrate IMU ground ref.
#define IMU_RESET_HOLD_MS  5000

// ── Test / Debug mode ─────────────────────────────────────────
// TEST_MODE 1 → full serial telemetry but ESCs are NOT driven.
//               Safe for bench work — no props needed.
// TEST_MODE 0 → live flight mode.  Always remove props first!
#define TEST_MODE    1       // ← SET TO 0 FOR REAL FLIGHT

// ── IMU calibration helper ────────────────────────────────────
// Set CALIBRATE_IMU to 1, flash, open Serial, keep craft flat.
// Copy the 6 printed offsets into the fields below, then set back to 0.
#define CALIBRATE_IMU  0
float IMU_OFF_AX = 0.0f;  // ← paste from calibration
float IMU_OFF_AY = 0.0f;
float IMU_OFF_AZ = 0.0f;
float IMU_OFF_GX = 0.0f;
float IMU_OFF_GY = 0.0f;
float IMU_OFF_GZ = 0.0f;

// ── Serial motor test ─────────────────────────────────────────
// MOTOR_TEST_MODE 1 → enables Serial commands to spin individual
// motors from the Serial Monitor. TEST_MODE must also be 1.
// Commands: "m1 1200"  "m2 1300"  "m3 1400"  "m4 1500"
//           "all 1200"  "stop"  "mpu"  "orient"
// !! REMOVE PROPS BEFORE USING !!
#define MOTOR_TEST_MODE  1   // 1 = enable serial motor commands

// ================================================================
//  END OF TUNING SECTION
//  ──────────────────────────────────────────────────────────────
//  Do not change anything below unless you understand it fully.
//  Everything is organised into small labelled functions.
// ================================================================


// ── Library objects ───────────────────────────────────────────
Servo   esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// ── Flight state ──────────────────────────────────────────────
bool  armed          = false;    // true = ESCs receiving throttle
bool  ledState       = false;    // current LED state
bool  lastLBt        = false;    // previous LBt value (edge detect)
bool  lastRBt        = false;    // previous RBt value (edge detect)
bool  signalLost     = false;    // true once TX timeout fires
int   failThrottle   = ESC_MIN;  // starts descending from here on signal loss

// ── IMU ground reference ──────────────────────────────────────
// Captured at boot (craft sitting flat on ground).
// All angle readings are subtracted from this so "level" = 0.
float groundRoll     = 0.0f;
float groundPitch    = 0.0f;

// ── PID state [axis: 0=Roll  1=Pitch  2=Yaw] ─────────────────
float pidPrevErr[3]  = {0, 0, 0};  // previous loop error
float pidIntegral[3] = {0, 0, 0};  // accumulated integral

// ── Arm/disarm timing ─────────────────────────────────────────
unsigned long lastDisarmMs    = 0;

// ── IMU reset button timing ───────────────────────────────────
unsigned long bothBtnHeldMs   = 0;  // when both A-buttons were first pressed
bool          bothBtnHeld     = false;

// ── Loop timer ────────────────────────────────────────────────
unsigned long loopTimer = 0;

// ── Telemetry print throttle ──────────────────────────────────
int  telTick = 0;
#define TEL_EVERY   20    // print once every N loops

// ── Motor test overrides (when MOTOR_TEST_MODE = 1) ───────────
int  testM1 = ESC_MIN, testM2 = ESC_MIN;
int  testM3 = ESC_MIN, testM4 = ESC_MIN;
bool testOverride = false;   // true while a motor test is active


// ================================================================
//  SECTION 1 — ESC / MOTOR HELPERS
// ================================================================

// ── Raw write to all four ESCs ────────────────────────────────
// Clamps every value to ESC_MIN..ESC_MAX before writing.
// In TEST_MODE this function prints but does NOT drive ESCs.
void writeMotors(int m1, int m2, int m3, int m4)
{
  m1 = constrain(m1, ESC_MIN, ESC_MAX);
  m2 = constrain(m2, ESC_MIN, ESC_MAX);
  m3 = constrain(m3, ESC_MIN, ESC_MAX);
  m4 = constrain(m4, ESC_MIN, ESC_MAX);

#if TEST_MODE
  // In test mode just return — telemetry loop prints the values
  return;
#endif

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

// ── Cut all motors immediately ────────────────────────────────
// Sends ESC_MIN to every ESC. Used on disarm and failsafe.
// This bypasses TEST_MODE so ESCs always receive the stop signal.
void motorsOff()
{
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

// ── SimonK arm sequence ───────────────────────────────────────
// SimonK ESCs need a sustained low signal followed by a brief
// higher pulse to confirm they should start accepting commands.
// The craft MUST be disarmed (throttle low) before calling this.
void runArmSequence()
{
  Serial.println("[FC] Arm sequence — sending low throttle for 2 s ...");

  // Step 1: Hold minimum throttle for 2 seconds so ESC sees it
  for (int i = 0; i < 200; i++)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(10);
  }

  // Step 2: Brief pulse just above minimum — some SimonK variants
  //         need this to confirm the throttle range endpoint.
  for (int i = 0; i < 50; i++)
  {
    esc1.writeMicroseconds(ESC_MIN + 50);
    esc2.writeMicroseconds(ESC_MIN + 50);
    esc3.writeMicroseconds(ESC_MIN + 50);
    esc4.writeMicroseconds(ESC_MIN + 50);
    delay(10);
  }

  // Back to idle so motors don't spin on arm
  motorsOff();
  Serial.println("[FC] Arm sequence complete — craft ARMED");
}

// ── Attach ESC PWM outputs ────────────────────────────────────
// Must be called in setup() before anything else touches the ESCs.
void initESCs()
{
  // Allocate all four hardware PWM timers (ESP32 has 4 LEDC timer groups)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach each ESC at 50 Hz (standard servo/ESC frequency)
  esc1.setPeriodHertz(50);  esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50);  esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50);  esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50);  esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);

  // Immediately send minimum throttle so ESCs don't arm on their own
  motorsOff();
  Serial.println("[FC] ESCs attached — min throttle sent (1000 µs)");
}


// ================================================================
//  SECTION 2 — IMU HELPERS
// ================================================================

// ── Read current roll and pitch with ground offset removed ─────
// Applies PITCH_FLIP / ROLL_FLIP for orientation correction.
// Subtracts the ground reference captured at boot (or after reset).
float getAdjustedRoll()
{
  return ROLL_FLIP  * (mpu.getAngleX() - groundRoll);
}
float getAdjustedPitch()
{
  return PITCH_FLIP * (mpu.getAngleY() - groundPitch);
}
float getAdjustedYawRate()
{
  // Yaw uses gyro rate (deg/s), not absolute angle
  return YAW_FLIP * mpu.getGyroZ();
}

// ── Capture the current MPU reading as "ground level zero" ────
// Called at boot and whenever both A-buttons are held 5 s.
// After this call, getAdjustedRoll/Pitch return 0 when the craft
// is in the same orientation as when this was captured.
void captureGroundReference()
{
  // Take several readings and average to reduce noise
  float sumRoll = 0, sumPitch = 0;
  const int samples = 50;

  Serial.println("[FC][IMU] Capturing ground reference ...");
  for (int i = 0; i < samples; i++)
  {
    mpu.update();
    sumRoll  += mpu.getAngleX();
    sumPitch += mpu.getAngleY();
    delay(5);
  }
  groundRoll  = sumRoll  / samples;
  groundPitch = sumPitch / samples;

  Serial.printf("[FC][IMU] Ground ref set → Roll=%.2f°  Pitch=%.2f°\n",
                groundRoll, groundPitch);
  Serial.println("[FC][IMU] Adjusted readings will now be 0 at this angle.");
}

// ── Initialise MPU-6050 ───────────────────────────────────────
// Returns false if IMU is not found (check wiring).
bool initIMU()
{
  Wire.begin(MPU_SDA, MPU_SCL);
  byte status = mpu.begin();

  if (status != 0)
  {
    Serial.printf("[FC][IMU] INIT FAILED — error %d — check SDA/SCL wiring!\n", status);
    return false;
  }

  Serial.println("[FC][IMU] MPU-6050 found OK");

  // Apply stored calibration offsets (from CALIBRATE_IMU run)
  mpu.setAccOffsets(IMU_OFF_AX, IMU_OFF_AY, IMU_OFF_AZ);
  mpu.setGyroOffsets(IMU_OFF_GX, IMU_OFF_GY, IMU_OFF_GZ);

  // Let the IMU settle for a moment before taking ground reference
  delay(200);
  for (int i = 0; i < 20; i++) { mpu.update(); delay(10); }

  // Capture the boot orientation as the reference "flat" position
  captureGroundReference();
  return true;
}

// ── Print orientation check info ─────────────────────────────
// Shows which physical direction maps to + and - on each axis.
// Useful when you're unsure if your MPU is mounted correctly.
void printOrientationInfo()
{
  Serial.println("");
  Serial.println("[FC][ORIENT] ── IMU Orientation Guide ──────────────────────");
  Serial.println("[FC][ORIENT]  Tilt the craft NOSE DOWN (pitch forward):");
  Serial.println("[FC][ORIENT]    → Adjusted Pitch should go NEGATIVE");
  Serial.println("[FC][ORIENT]    → If it goes positive, set PITCH_FLIP -1.0");
  Serial.println("[FC][ORIENT]");
  Serial.println("[FC][ORIENT]  Tilt the craft RIGHT side DOWN (roll right):");
  Serial.println("[FC][ORIENT]    → Adjusted Roll should go POSITIVE");
  Serial.println("[FC][ORIENT]    → If it goes negative, set ROLL_FLIP -1.0");
  Serial.println("[FC][ORIENT]");
  Serial.printf("[FC][ORIENT]  Current PITCH_FLIP: %.1f   ROLL_FLIP: %.1f\n",
                PITCH_FLIP, ROLL_FLIP);
  Serial.println("[FC][ORIENT]  Live readings below — tilt the craft to verify:");
  Serial.println("[FC][ORIENT] ───────────────────────────────────────────────");

  // Stream 100 readings so the user can watch while tilting
  for (int i = 0; i < 100; i++)
  {
    mpu.update();
    Serial.printf("[ORIENT] Roll:%+7.2f°  Pitch:%+7.2f°  YawRate:%+7.2f°/s\n",
                  getAdjustedRoll(), getAdjustedPitch(), getAdjustedYawRate());
    delay(50);
  }
  Serial.println("[FC][ORIENT] ── Orientation check done ──");
}


// ================================================================
//  SECTION 3 — ARM / DISARM
// ================================================================

// ── Arm the craft ─────────────────────────────────────────────
// Only arms if:  (a) TX signal is present
//                (b) re-arm guard has expired
//                (c) craft is not already armed
void doArm()
{
  if (armed)
  {
    Serial.println("[FC][ARM] Already armed — ignoring");
    return;
  }

  if (!receiver.connected())
  {
    Serial.println("[FC][ARM] ✖ No TX signal — cannot arm!");
    return;
  }

  unsigned long guard = millis() - lastDisarmMs;
  if (guard < REARM_GUARD_MS)
  {
    Serial.printf("[FC][ARM] ✖ Re-arm guard active — wait %lu ms\n",
                  REARM_GUARD_MS - guard);
    return;
  }

  // Clear PID history so old error doesn't spike the output
  for (int i = 0; i < 3; i++) { pidPrevErr[i] = 0; pidIntegral[i] = 0; }

  armed = true;

#if TEST_MODE
  Serial.println("[FC][ARM] ── ARMED (TEST MODE — no ESC output) ──");
#else
  runArmSequence();
#endif
}

// ── Disarm the craft ──────────────────────────────────────────
// Cuts motors immediately, records disarm timestamp for re-arm guard.
void doDisarm()
{
  if (!armed)
  {
    Serial.println("[FC][ARM] Already disarmed — ignoring");
    return;
  }
  armed        = false;
  lastDisarmMs = millis();
  motorsOff();
  Serial.println("[FC][ARM] ── DISARMED — motors stopped ──");
}


// ================================================================
//  SECTION 4 — RC INPUT PROCESSING
// ================================================================

// ── Cubic expo curve ─────────────────────────────────────────
// Makes the stick feel gentle at centre and sharp at the edges.
// v is -1..1, expo is 0..1.  expo=0 → linear, expo=1 → pure cubic.
float applyExpo(float v, float expo)
{
  return expo * v * v * v + (1.0f - expo) * v;
}

// ── Deadband ─────────────────────────────────────────────────
// Returns 0 if |raw| ≤ DEADBAND, otherwise shifts inward so
// there is no jump at the deadband edge.
int applyDeadband(int raw)
{
  if (raw >  DEADBAND) return raw - DEADBAND;
  if (raw < -DEADBAND) return raw + DEADBAND;
  return 0;
}

// ── Map throttle stick to ESC µs ─────────────────────────────
// Center stick (raw = 0) → ESC_HOVER  (holds altitude)
// Full up      (raw = +99) → ESC_MAX  (climb)
// Full down    (raw = -99) → ESC_IDLE (slow descent / ground idle)
//
// The range is split at center:
//   lower half:  -99 .. 0  →  ESC_IDLE .. ESC_HOVER
//   upper half:   0 .. 99  →  ESC_HOVER .. ESC_MAX
int mapThrottle(int raw)
{
  if (raw >= 0)
  {
    // Above center: hover → max
    return (int)map(raw, 0, 99, ESC_HOVER, ESC_MAX);
  }
  else
  {
    // Below center: idle → hover
    return (int)map(raw, -99, 0, ESC_IDLE, ESC_HOVER);
  }
}

// ── Read and process all stick channels ───────────────────────
// Fills the passed references with processed values ready for PID.
// throttleUs: final µs value for base throttle
// targetRoll / targetPitch: desired angle in degrees
// targetYawRate: desired rotation rate in deg/s
void readSticks(int   &throttleUs,
                float &targetRoll,
                float &targetPitch,
                float &targetYawRate)
{
  // ── Raw stick values from receiver (-99 .. +99) ───────────
  // Mode 2: Left Y = Throttle, Left X = Yaw
  //         Right Y = Pitch,   Right X = Roll
  int rawThrottle = receiver.data.Ly;
  int rawYaw      = receiver.data.Lx;
  int rawPitch    = receiver.data.Ry;   // positive = nose up / forward
  int rawRoll     = receiver.data.Rx;   // positive = right

  // ── Apply deadband to all axes except throttle top range ──
  rawYaw   = applyDeadband(rawYaw);
  rawPitch = applyDeadband(rawPitch);
  rawRoll  = applyDeadband(rawRoll);
  // Throttle: only snap the center band to zero, preserve edges
  if (rawThrottle > -DEADBAND && rawThrottle < DEADBAND) rawThrottle = 0;

  // ── Normalise to -1.0 .. +1.0 for expo calculation ────────
  float nYaw   = rawYaw   / 99.0f;
  float nPitch = rawPitch / 99.0f;
  float nRoll  = rawRoll  / 99.0f;

  // ── Apply expo (throttle gets no expo for predictable climb) ─
  nYaw   = applyExpo(nYaw,   EXPO_YAW);
  nPitch = applyExpo(nPitch, EXPO_PITCH);
  nRoll  = applyExpo(nRoll,  EXPO_ROLL);

  // ── Map throttle stick to ESC µs using split-center method ──
  throttleUs = mapThrottle(rawThrottle);

  // ── Convert normalised roll/pitch to angle targets (degrees) ─
  targetRoll     = nRoll  * MAX_ANGLE_ROLL;
  targetPitch    = nPitch * MAX_ANGLE_PITCH;
  targetYawRate  = nYaw   * MAX_RATE_YAW;
}


// ================================================================
//  SECTION 5 — PID CONTROLLER
// ================================================================

// ── Compute PID output for one axis ──────────────────────────
// axis:       0 = Roll,  1 = Pitch,  2 = Yaw
// setpoint:   desired value (angle in deg for Roll/Pitch,
//                             rate in deg/s for Yaw)
// measured:   actual value from IMU
// dt:         loop period in seconds
// returns µs correction (positive or negative)
float computePID(int axis, float setpoint, float measured, float dt)
{
  float error     = setpoint - measured;

  // Integral — accumulates over time, corrects steady-state error
  pidIntegral[axis] += error * dt;
  // Clamp integral to prevent windup (e.g. craft flipped, props off)
  pidIntegral[axis]  = constrain(pidIntegral[axis], -ITERM_MAX, ITERM_MAX);

  // Derivative — reacts to rate of error change, dampens oscillation
  float derivative = (error - pidPrevErr[axis]) / dt;
  pidPrevErr[axis]  = error;

  return KP[axis] * error
       + KI[axis] * pidIntegral[axis]
       + KD[axis] * derivative;
}


// ================================================================
//  SECTION 6 — MOTOR MIXING  (X-Config)
// ================================================================

// ── Mix throttle + PID outputs into four motor commands ───────
//
//  X-config mixing table (props spin as shown):
//
//       M1(CW)   M2(CCW)
//         |  ╲ ╱  |
//         |  ╱ ╲  |
//       M4(CCW)  M3(CW)
//
//  Positive pitch  = nose up    → front motors faster, rear slower
//  Positive roll   = right down → left motors faster, right slower
//  Positive yaw    = CW         → CCW motors faster, CW motors slower
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
//  SECTION 7 — FAILSAFE (signal lost)
// ================================================================

// ── Handle signal loss — gradual descent ──────────────────────
// Called when linkOK is false while the craft is armed.
// Reduces failThrottle slowly so the craft descends gently.
// PID is still fed current IMU data so the craft stays level.
void handleFailsafe(float pidRoll, float pidPitch, float pidYaw)
{
  // Decrease throttle by DESCENT_RATE each loop tick
  failThrottle -= DESCENT_RATE;
  failThrottle  = max(failThrottle, (int)DESCENT_FLOOR);

  // Apply PID only on yaw-free axes during descent
  // (Roll/Pitch corrections keep the craft flat, yaw just holds)
  int fm1 = failThrottle + (int)pidPitch + (int)pidRoll  - (int)pidYaw;
  int fm2 = failThrottle + (int)pidPitch - (int)pidRoll  + (int)pidYaw;
  int fm3 = failThrottle - (int)pidPitch - (int)pidRoll  - (int)pidYaw;
  int fm4 = failThrottle - (int)pidPitch + (int)pidRoll  + (int)pidYaw;

  writeMotors(fm1, fm2, fm3, fm4);
}


// ================================================================
//  SECTION 8 — BUTTON HANDLERS
// ================================================================

// ── Process LBt: arm when ON, disarm when OFF ─────────────────
// This is a level-based switch, not toggle:
//   LBt = true  → arm
//   LBt = false → disarm
void handleArmSwitch(bool curLBt)
{
  if (curLBt && !armed)
  {
    doArm();
  }
  else if (!curLBt && armed)
  {
    doDisarm();
  }
  // Store for next loop — not strictly needed for level-based but
  // kept for consistency and future edge detection use.
  lastLBt = curLBt;
}

// ── Process RBt: toggle LED on rising edge ────────────────────
void handleLEDToggle(bool curRBt)
{
  if (curRBt && !lastRBt)   // rising edge only
  {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    Serial.printf("[FC][LED] LED %s\n", ledState ? "ON" : "OFF");
  }
  lastRBt = curRBt;
}

// ── Process LABt + RABt: hold 5 s to reset IMU reference ──────
void handleIMUReset(bool la, bool ra)
{
  if (la && ra)
  {
    if (!bothBtnHeld)
    {
      // Buttons just came ON — record the start time
      bothBtnHeld   = true;
      bothBtnHeldMs = millis();
      Serial.println("[FC][IMU] Both A-buttons held — keep holding 5 s to reset IMU ...");
    }
    else
    {
      // Buttons are still held — check elapsed time
      unsigned long elapsed = millis() - bothBtnHeldMs;
      if (elapsed >= IMU_RESET_HOLD_MS)
      {
        // 5 seconds reached — recalibrate
        Serial.println("[FC][IMU] ── 5 s reached — resetting IMU ground reference ──");
        captureGroundReference();

        // Clear PID state so old integrals don't cause a jerk after reset
        for (int i = 0; i < 3; i++) { pidPrevErr[i] = 0; pidIntegral[i] = 0; }

        // Reset the timer so holding longer doesn't trigger again immediately
        bothBtnHeldMs = millis();

        // Short LED flash to confirm the reset happened
        for (int i = 0; i < 4; i++)
        {
          digitalWrite(PIN_LED, HIGH); delay(80);
          digitalWrite(PIN_LED, LOW);  delay(80);
        }
        // Restore LED to its previous state
        digitalWrite(PIN_LED, ledState ? HIGH : LOW);
      }
    }
  }
  else
  {
    // Buttons released before 5 s
    if (bothBtnHeld)
    {
      Serial.println("[FC][IMU] Buttons released — IMU reset cancelled");
    }
    bothBtnHeld = false;
  }
}


// ================================================================
//  SECTION 9 — SERIAL MOTOR TEST  (MOTOR_TEST_MODE = 1)
// ================================================================

// ── Parse and execute a motor test command from Serial ────────
//
// Commands (type into Serial Monitor, 115200 baud):
//   m1 1300      → set M1 to 1300 µs  (1000–2000)
//   m2 1400      → set M2 to 1400 µs
//   m3 1200      → set M3 to 1200 µs
//   m4 1500      → set M4 to 1500 µs
//   all 1250     → all motors to 1250 µs
//   stop         → all motors to ESC_MIN
//   mpu          → print 50 IMU readings
//   orient       → run full orientation check (100 readings)
//   imu reset    → recapture ground reference now
//   arm          → manually arm (ignores re-arm guard in test mode)
//   disarm       → manually disarm
//
// !! ALWAYS REMOVE PROPS BEFORE USING MOTOR COMMANDS !!
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
    // Individual motor test: "m1 1300"
    int motorNum = cmd.charAt(1) - '0';   // 1, 2, 3, or 4
    int us = cmd.substring(3).toInt();
    us = constrain(us, ESC_MIN, ESC_MAX);

    Serial.printf("[TEST] Motor M%d → %d µs\n", motorNum, us);

    testOverride = true;
    switch (motorNum) {
      case 1: testM1 = us; break;
      case 2: testM2 = us; break;
      case 3: testM3 = us; break;
      case 4: testM4 = us; break;
    }

    // Write directly, bypassing TEST_MODE suppression
    esc1.writeMicroseconds(testM1);
    esc2.writeMicroseconds(testM2);
    esc3.writeMicroseconds(testM3);
    esc4.writeMicroseconds(testM4);
  }
  else if (cmd.startsWith("all "))
  {
    // All motors to same speed: "all 1300"
    int us = cmd.substring(4).toInt();
    us = constrain(us, ESC_MIN, ESC_MAX);
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
    // Print 50 raw IMU readings
    Serial.println("[TEST][MPU] 50 readings:");
    for (int i = 0; i < 50; i++)
    {
      mpu.update();
      Serial.printf("[MPU] AdjRoll:%+7.2f°  AdjPitch:%+7.2f°"
                    "  GyroZ:%+7.2f°/s  rawAngleX:%.2f  rawAngleY:%.2f\n",
                    getAdjustedRoll(), getAdjustedPitch(),
                    getAdjustedYawRate(),
                    mpu.getAngleX(), mpu.getAngleY());
      delay(50);
    }
    Serial.println("[TEST][MPU] Done.");
  }
  else if (cmd == "orient")
  {
    // Full orientation check with 100 readings
    printOrientationInfo();
  }
  else if (cmd == "imu reset")
  {
    Serial.println("[TEST][IMU] Manual ground reference reset ...");
    for (int i = 0; i < 20; i++) { mpu.update(); delay(10); }
    captureGroundReference();
    for (int i = 0; i < 3; i++) { pidPrevErr[i] = 0; pidIntegral[i] = 0; }
  }
  else if (cmd == "arm")
  {
    // Force arm in test mode (skips re-arm guard)
    armed = true;
    Serial.println("[TEST] Force ARM — no ESC output in TEST_MODE");
  }
  else if (cmd == "disarm")
  {
    armed = false; lastDisarmMs = millis();
    testOverride = false; testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff();
    Serial.println("[TEST] Force DISARM");
  }
  else
  {
    Serial.println("[TEST] Unknown command. Available:");
    Serial.println("  m1/m2/m3/m4 <µs>   e.g. m1 1300");
    Serial.println("  all <µs>           e.g. all 1200");
    Serial.println("  stop");
    Serial.println("  mpu                50 IMU readings");
    Serial.println("  orient             orientation check");
    Serial.println("  imu reset          recapture ground level");
    Serial.println("  arm  /  disarm");
  }
}
#endif  // MOTOR_TEST_MODE


// ================================================================
//  SECTION 10 — TELEMETRY
// ================================================================

// ── Print one line of flight telemetry to Serial ──────────────
void printTelemetry(bool linkOK,
                    int  throttleUs,
                    float tgtRoll,  float tgtPitch, float tgtYaw,
                    float actRoll,  float actPitch, float actYawRate,
                    float pRoll,    float pPitch,   float pYaw,
                    int m1, int m2, int m3, int m4)
{
  Serial.printf(
    "[FC] ARM:%s LNK:%s | THR:%4d | "
    "TGT R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "ACT R:%+5.1f P:%+5.1f YR:%+5.1f | "
    "PID R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "M1:%4d M2:%4d M3:%4d M4:%4d | "
    "BAT:%3d%%\n",
    armed   ? "Y" : "N",
    linkOK  ? "Y" : "N",
    throttleUs,
    tgtRoll, tgtPitch, tgtYaw,
    actRoll, actPitch, actYawRate,
    pRoll,   pPitch,   pYaw,
    constrain(m1, ESC_MIN, ESC_MAX),
    constrain(m2, ESC_MIN, ESC_MAX),
    constrain(m3, ESC_MIN, ESC_MAX),
    constrain(m4, ESC_MIN, ESC_MAX),
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
  Serial.println("[FC]  Q450 Flight Controller  v2.0  — BOOT");
  Serial.printf ("[FC]  TEST_MODE     : %s\n", TEST_MODE ? "ON  (no ESC output)" : "OFF (LIVE FLIGHT)");
  Serial.printf ("[FC]  MOTOR_TEST    : %s\n", MOTOR_TEST_MODE ? "enabled" : "disabled");
  Serial.printf ("[FC]  LOOP_HZ       : %d Hz\n", LOOP_HZ);
  Serial.printf ("[FC]  PITCH_FLIP    : %.1f   ROLL_FLIP: %.1f\n", PITCH_FLIP, ROLL_FLIP);
  Serial.println("[FC] ═══════════════════════════════════════════\n");

  // ── LED ───────────────────────────────────────────────────────
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // ── ESCs ──────────────────────────────────────────────────────
  initESCs();

  // ── IMU ───────────────────────────────────────────────────────
#if CALIBRATE_IMU
  // ── IMU CALIBRATION MODE — place craft flat, still ───────────
  // Flash until stable, then print offsets and halt.
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.begin();
  Serial.println("[FC][CAL] ── IMU CALIBRATION MODE ──");
  Serial.println("[FC][CAL] Lay craft FLAT and STILL. Starting in 3 s ...");
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.println("[FC][CAL] ── Copy these into the TUNING SECTION ──");
  Serial.printf("[FC][CAL] IMU_OFF_AX = %.4f;\n", mpu.getAccXoffset());
  Serial.printf("[FC][CAL] IMU_OFF_AY = %.4f;\n", mpu.getAccYoffset());
  Serial.printf("[FC][CAL] IMU_OFF_AZ = %.4f;\n", mpu.getAccZoffset());
  Serial.printf("[FC][CAL] IMU_OFF_GX = %.4f;\n", mpu.getGyroXoffset());
  Serial.printf("[FC][CAL] IMU_OFF_GY = %.4f;\n", mpu.getGyroYoffset());
  Serial.printf("[FC][CAL] IMU_OFF_GZ = %.4f;\n", mpu.getGyroZoffset());
  Serial.println("[FC][CAL] ── Set CALIBRATE_IMU 0 and re-flash ──");
  while (1);   // halt here
#endif

  if (!initIMU())
  {
    // IMU failed — blink LED rapidly forever, do not proceed
    Serial.println("[FC] FATAL: IMU not found — halting");
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  // ── ESP-NOW Receiver ──────────────────────────────────────────
  receiver.timeoutMs = 1000;    // signal loss declared after 1 s silence
  receiver.begin(TX_MAC, true); // true = send ACK back to TX

  // ── Print orientation guide ───────────────────────────────────
  Serial.println("[FC] ── Orientation reference ──────────────────");
  Serial.printf ("[FC]  Nose-down pitch → Adjusted Pitch should go %s\n",
                 PITCH_FLIP > 0 ? "NEGATIVE" : "POSITIVE");
  Serial.printf ("[FC]  Right-side-down roll → Adjusted Roll should go %s\n",
                 ROLL_FLIP  > 0 ? "POSITIVE" : "NEGATIVE");
  Serial.println("[FC]  Send 'orient' in Serial Monitor for a live check.");
  Serial.println("[FC] ───────────────────────────────────────────");

#if MOTOR_TEST_MODE
  Serial.println("[FC] ── Motor Test Mode ACTIVE ─────────────────");
  Serial.println("[FC]  Commands: m1/m2/m3/m4 <µs>  all <µs>  stop");
  Serial.println("[FC]            mpu   orient   imu reset   arm   disarm");
  Serial.println("[FC] !! REMOVE PROPS BEFORE SENDING MOTOR COMMANDS !!");
  Serial.println("[FC] ───────────────────────────────────────────");
#endif

  // ── Ready blink — 3 short flashes ─────────────────────────────
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_LED, HIGH); delay(150);
    digitalWrite(PIN_LED, LOW);  delay(150);
  }

  Serial.println("\n[FC] ── Ready. Set LBt ON to ARM ──\n");
  loopTimer = micros();
}


// ================================================================
//  MAIN LOOP
// ================================================================
void loop()
{
  // ── Fixed-rate gate ───────────────────────────────────────────
  // Spin here until LOOP_US µs have elapsed since last iteration.
  // This makes dt (and therefore PID I/D terms) rock-steady.
  while (micros() - loopTimer < LOOP_US);
  float dt      = (micros() - loopTimer) / 1e6f;  // actual dt in seconds
  loopTimer     = micros();

  // ── 1. Serial command check ───────────────────────────────────
#if MOTOR_TEST_MODE
  handleSerialCommands();
  // If a test override is active, skip normal flight logic
  if (testOverride) { delay(2); return; }
#endif

  // ── 2. Receiver update ───────────────────────────────────────
  // Must call every loop — updates connected_flag and failsafe timer.
  receiver.update();
  bool linkOK = receiver.connected();

  // ── 3. IMU update ────────────────────────────────────────────
  mpu.update();   // reads raw IMU, runs internal complementary filter

  float actualRoll    = getAdjustedRoll();      // degrees
  float actualPitch   = getAdjustedPitch();     // degrees
  float actualYawRate = getAdjustedYawRate();   // deg/s

  // ── 4. Signal-loss detection (fires ONCE on loss edge) ────────
  if (!linkOK && !signalLost)
  {
    signalLost   = true;
    // Start descending from hover throttle if armed, or min if not
    failThrottle = armed ? ESC_HOVER + 50 : ESC_MIN;
    Serial.println("[FC] ⚠  SIGNAL LOST — failsafe descent active");
    // Clear PID so stale error doesn't spike when signal returns
    for (int i = 0; i < 3; i++) { pidPrevErr[i] = 0; pidIntegral[i] = 0; }
  }
  else if (linkOK && signalLost)
  {
    signalLost = false;
    Serial.println("[FC] ✔  Signal restored");
  }

  // ── 5. Button handling ────────────────────────────────────────
  bool curLBt = receiver.data.LBt;
  bool curRBt = receiver.data.RBt;
  bool curLA  = receiver.data.LABt;
  bool curRA  = receiver.data.RABt;

  handleArmSwitch(curLBt);         // arm if LBt ON, disarm if LBt OFF
  handleLEDToggle(curRBt);         // LED toggle on RBt rising edge
  handleIMUReset(curLA, curRA);    // IMU reset if both A-btns held 5 s

  // ── 6. Stick processing (skip if signal lost — data is zeroed) ─
  int   throttleUs  = ESC_IDLE;
  float tgtRoll     = 0, tgtPitch = 0, tgtYawRate = 0;

  if (linkOK)
  {
    readSticks(throttleUs, tgtRoll, tgtPitch, tgtYawRate);
  }
  else
  {
    // Signal lost — commands default to 0; failsafe handles throttle
    throttleUs = failThrottle;
  }

  // ── 7. PID computation ────────────────────────────────────────
  // Angle mode: Roll and Pitch PID targets angle (degrees).
  // Yaw is rate-based (deg/s) — yaw has no absolute reference.
  float pidRoll  = computePID(0, tgtRoll,    actualRoll,    dt);
  float pidPitch = computePID(1, tgtPitch,   actualPitch,   dt);
  float pidYaw   = computePID(2, tgtYawRate, actualYawRate, dt);

  // ── 8. Motor output decision ──────────────────────────────────
  int m1, m2, m3, m4;

  if (!armed)
  {
    // DISARMED: always off regardless of sticks
    motorsOff();
    m1 = m2 = m3 = m4 = ESC_MIN;
  }
  else if (signalLost)
  {
    // FAILSAFE: PID-stabilised gradual descent
    handleFailsafe(pidRoll, pidPitch, pidYaw);
    m1 = m2 = m3 = m4 = failThrottle;   // for telemetry display
  }
  else
  {
    // NORMAL FLIGHT: mix throttle + PID into motor commands
    // Compute values for telemetry then write
    m1 = throttleUs + (int)pidPitch + (int)pidRoll  - (int)pidYaw;
    m2 = throttleUs + (int)pidPitch - (int)pidRoll  + (int)pidYaw;
    m3 = throttleUs - (int)pidPitch - (int)pidRoll  - (int)pidYaw;
    m4 = throttleUs - (int)pidPitch + (int)pidRoll  + (int)pidYaw;
    mixAndWrite(throttleUs, pidRoll, pidPitch, pidYaw);
  }

  // ── 9. Telemetry (printed every TEL_EVERY loops) ──────────────
  telTick++;
  if (telTick >= TEL_EVERY)
  {
    telTick = 0;
    printTelemetry(linkOK,
                   throttleUs,
                   tgtRoll, tgtPitch, tgtYawRate,
                   actualRoll, actualPitch, actualYawRate,
                   pidRoll, pidPitch, pidYaw,
                   m1, m2, m3, m4);
  }
}
