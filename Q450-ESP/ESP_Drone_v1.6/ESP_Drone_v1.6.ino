// ================================================================
//  Q450 ESP32 Flight Controller  —  v1.6
//  Frame   : Q450 Quadcopter (X-Config)
//  Motors  : A2212/13T 1000KV
//  ESCs    : SimonK 30A  (Standard PWM 1000–2000 µs)
//  IMU     : MPU-6050 (I2C)
//  Radio   : ESP-NOW via ReceiverModule.h
//
// ----------------------------------------------------------------
//  CHANGES v1.5 → v1.6
//  • PID tuned for stable hover — KP 0.70, KI 0.05, KD 0.25
//  • MAX_ANGLE reduced to 12° for safer first flights
//  • THROTTLE_RAMP_RATE lowered to 1 (100µs/s — very slow climb)
//  • THROTTLE_MAX capped at 1700µs for safety ceiling
//  • EXPO raised to 0.50 for gentler stick response at center
//  • IMU_ANGLE_DEADZONE raised to 1.0° to absorb residual offset
//  • MAX_MOTOR_STEP reduced to 20µs for smoother motor response
//  • ITERM_MAX tightened to 20µs to prevent integral windup
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
//  GPIO:  25→M1  26→M2  27→M3  14→M4
//         21→SDA  22→SCL  2→LED
// ================================================================

#include "ReceiverModule.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>   // Library Manager: "MPU6050_light" by rfetick 
#include <esp_task_wdt.h>      // ← ADD THIS — hardware watchdog

// ================================================================
//  ████████╗██╗   ██╗███╗   ██╗██╗███╗   ██╗ ██████╗
//     ██╔══╝██║   ██║████╗  ██║██║████╗  ██║██╔════╝
//     ██║   ██║   ██║██╔██╗ ██║██║██╔██╗ ██║██║  ███╗
//     ██║   ██║   ██║██║╚██╗██║██║██║╚██╗██║██║   ██║
//     ██║   ╚██████╔╝██║ ╚████║██║██║ ╚████║╚██████╔╝
//     ╚═╝    ╚═════╝ ╚═╝  ╚═══╝╚═╝╚═╝  ╚═══╝ ╚═════╝
//  ALL TWEAKABLE SETTINGS ARE HERE — ONE PLACE ONLY
// ================================================================

// ── Hardware pins ─────────────────────────────────────────────
#define PIN_M1        25    // Front-Left  (CW)
#define PIN_M2        26    // Front-Right (CCW)
#define PIN_M3        27    // Back-Right  (CW)
#define PIN_M4        14    // Back-Left   (CCW)
#define PIN_LED        2    // Built-in LED (HIGH = on)
#define MPU_SDA       21
#define MPU_SCL       22

// ── TX MAC — your transmitter's MAC, printed at TX boot ───────
#define TX_MAC  "A4:F0:0F:90:34:28"

// ── ESC PWM range ─────────────────────────────────────────────
#define ESC_MIN     1000    // µs — absolute off / signal floor
#define ESC_MAX     2000    // µs — absolute full throttle

// ── ESC_IDLE: lowest µs where ALL 4 motors spin reliably ──────
// If any motor doesn't spin on arm, raise by 5 until it does.
// This should be BELOW the point where the craft starts to lift.
// Typical range for A2212 1000KV on 3S: 1080–1150
#define ESC_IDLE    1100    // µs  ★ TUNE FIRST — raise if motors don't spin

// ── ESC_SAFE: µs the craft idles at while armed, waiting ──────
// Must be >= ESC_IDLE. Must NOT lift the craft off the ground.
// Start low. Use "all 11XX" in serial to find where motors spin
// without lifting, then set this 10µs above that point.
// Your 1200 was too high — starting at 1120 is safer.
#define ESC_SAFE    1100    // µs  ★ TUNE ME — lower if still lifting

// ── Throttle accumulator limits ───────────────────────────────
// accThrottle (the persistent throttle) is clamped to this range.
// THROTTLE_MIN = floor when armed (usually == ESC_SAFE)
// THROTTLE_MAX = safety ceiling — keep below ESC_MAX for safety
#define THROTTLE_MIN   ESC_SAFE
#define THROTTLE_MAX   1700    // µs  ★ TUNE ME — raise slowly in flight

// ── Throttle ramp rate ────────────────────────────────────────
// µs added/subtracted per loop tick when stick is fully pushed.
// At 100Hz:  1 µs/tick = 100 µs/s  (very slow)
//            2 µs/tick = 200 µs/s  (recommended for cheap sticks)
//            4 µs/tick = 400 µs/s  (if sticks feel sluggish)
// With cheap PS2 sticks start at 2 — very smooth and predictable.
#define THROTTLE_RAMP_RATE   1     // 100µs/s — very slow climb

// ── Per-motor trim offsets ────────────────────────────────────
// Use these to balance motors that spin at different speeds.
// POSITIVE = that motor gets MORE power than commanded.
// NEGATIVE = that motor gets LESS power than commanded.
//
// HOW TO TUNE:
//   1. Remove props. Run "all 1300" in serial monitor.
//   2. Listen — the loudest/fastest motor needs a NEGATIVE trim.
//   3. Adjust in steps of ±10 until all 4 sound equal.
//   4. Re-check at "all 1500" as well.
//
//              M1    M2    M3    M4
int MOTOR_TRIM[4] = {  0,    0,    0,    0  };  // µs  ★ TUNE ME

// ── Stick deadband ────────────────────────────────────────────
// Raw stick units inside this range are treated as zero.
// PS2-style cheap analog sticks: start at 8–10.
// If sticks feel twitchy at center, raise this value.
#define DEADBAND       8    // raw units (-99..99)  ★ TUNE ME

// ── IMU angle deadzone ────────────────────────────────────────
// Angles smaller than this are treated as exactly 0.
// Eliminates micro-corrections from sensor noise at rest.
// 0.5° is safe to start. Lower to 0.3° once flying well.
#define IMU_ANGLE_DEADZONE   1.0f   // degrees  ★ TUNE ME

// ── Gyro rate deadzone ────────────────────────────────────────
// Yaw gyro noise below this deg/s is treated as zero.
// Your bench showed ~1.4 deg/s noise — 2.0 clears it safely.
#define GYRO_DEADZONE_DPS    2.0f   // deg/s  ★ TUNE ME

// ── IMU angle IIR filter ──────────────────────────────────────
// Smooths the angle reading before feeding the PID.
// 1.0 = raw (no smoothing)   0.8 = moderate   0.5 = heavy
// Too low = sluggish levelling. Too high = noisy corrections.
#define ANGLE_FILTER_ALPHA   0.8f   // 0.0–1.0  ★ TUNE ME

// ── Ground reference capture ──────────────────────────────────
// Number of accelerometer samples averaged at boot.
// More = stabler reference, longer boot (~5ms each).
// 150 samples ≈ 0.75 seconds. Usually fine.
#define GROUND_REF_SAMPLES      150   // ★ TUNE ME (100–250)

// Reject a sample if it differs from the rough average by more
// than this. Protects against bumps during the capture window.
#define GROUND_REF_REJECT_DEG   2.0f  // degrees  ★ TUNE ME

// ── Motor output slew limit ───────────────────────────────────
// Max µs change per loop tick per motor.
// Prevents PID spikes from reaching ESCs as sudden jumps.
// At 100Hz: 30µs/tick = 3000µs/s max change rate.
#define MAX_MOTOR_STEP   20    // µs/tick  ★ TUNE ME

// ── Expo curves ───────────────────────────────────────────────
// 0.0 = linear (direct).  0.5 = strong curve (gentle at center).
// Higher expo = more "dead feeling" near center stick = stability.
#define EXPO_ROLL     0.50f   // raised from 0.40 — gentler center
#define EXPO_PITCH    0.50f
#define EXPO_YAW      0.40f

// ── Max angle and rate targets ────────────────────────────────
// Full stick deflection commands this many degrees of tilt.
// Start LOW (15–20°). Raise once you trust the tune.
// 12° max tilt means gentle forward/back, hard to flip
#define MAX_ANGLE_ROLL    12.0f
#define MAX_ANGLE_PITCH   12.0f
#define MAX_RATE_YAW      80.0f

// ── IMU axis orientation ──────────────────────────────────────
// If an axis reads backwards on your mount, flip it with -1.0f.
// Use serial command "orient" to check: tilt nose down should give NEGATIVE pitch, tilt right side down = POSITIVE roll.
#define PITCH_FLIP   -1.0f    // 1.0 = normal | -1.0 = inverted
#define ROLL_FLIP    1.0f    // 1.0 = normal | -1.0 = inverted
#define YAW_FLIP     1.0f    // 1.0 = normal | -1.0 = inverted

// ── Stick direction flips ─────────────────────────────────────
// Use these if pushing a stick causes movement in the wrong direction.
// These ONLY affect stick input — do NOT touch PITCH_FLIP/ROLL_FLIP.
// PITCH_FLIP/ROLL_FLIP = how the IMU is physically mounted (don't change)
// STICK_*_FLIP        = which way your controller sends the signal
//  1.0 = normal   -1.0 = reversed
#define STICK_PITCH_FLIP  -1.0f   // ★ your pitch was backwards — fixed here
#define STICK_ROLL_FLIP    1.0f   // flip if rolling right moves left
#define STICK_YAW_FLIP     1.0f   // flip if yaw spins wrong direction

// ── PID gains ─────────────────────────────────────────────────
//
//  HOW TO TUNE (always with props on, outside, low throttle):
//    Step 1 — Roll/Pitch P:
//      Set I=0, D=0. Raise P until craft oscillates quickly.
//      Back off P by 20%. That's your P.
//    Step 2 — Roll/Pitch D:
//      Add D slowly (0.1 steps) until oscillation damps out.
//      Too much D = high-pitch motor noise / hot ESCs.
//    Step 3 — Roll/Pitch I:
//      Add tiny I (0.01 steps) only to fix a slow consistent
//      drift in one direction that P+D can't correct.
//    Step 4 — Yaw:
//      Yaw is rate-based, not angle-based, so P is much smaller.
//      Tune the same way but expect smaller numbers.
//
//              Roll     Pitch    Yaw
float KP[3] = { 0.70f,   0.70f,   0.50f };  // ★ START HERE
float KI[3] = { 0.05f,   0.05f,   0.00f };  // add I only after P+D stable
float KD[3] = { 0.25f,   0.25f,   0.10f };  

// ── PID integral clamp ────────────────────────────────────────
// Prevents the integral term from winding up to large values
// when the craft is held still or sat on the ground.
#define ITERM_MAX   20.0f   // µs max I contribution per axis

// ── Failsafe descent ──────────────────────────────────────────
// When signal is lost, throttle drops by this many µs per tick
// until it reaches DESCENT_FLOOR, then holds there.
// At 100Hz: 3µs/tick = 300µs/s descent rate.
#define DESCENT_RATE    3       // µs/tick
#define DESCENT_FLOOR   ESC_IDLE

// ── Loop timing ───────────────────────────────────────────────
#define LOOP_HZ    100
#define LOOP_US    (1000000 / LOOP_HZ)

// ── Re-arm guard ──────────────────────────────────────────────
// Minimum ms between disarm and next arm. Prevents accidental
// instant re-arm if the arm switch bounces.
#define REARM_GUARD_MS   2000

// ── IMU manual reset hold time ────────────────────────────────
// Hold both analog buttons for this long to reset IMU reference.
#define IMU_RESET_HOLD_MS   5000

// ── Test / Debug mode ─────────────────────────────────────────
// TEST_MODE 1 → full serial telemetry, ESCs NOT driven (safe).
// TEST_MODE 0 → live flight. REMOVE PROPS before switching!
#define TEST_MODE   0    // ← SET TO 0 FOR REAL FLIGHT

// ── IMU calibration helper ────────────────────────────────────
// Set CALIBRATE_IMU 1, flash, lay flat, note printed offsets,
// paste into IMU_OFF_* below, set back to 0, reflash.
// Only needed once per build. Greatly improves angle accuracy.
#define CALIBRATE_IMU   0
float IMU_OFF_AX = 0.0155f;   // accelerometer X offset
float IMU_OFF_AY = -0.0046f;   // accelerometer Y offset
float IMU_OFF_AZ = 0.0514f;   // accelerometer Z offset
float IMU_OFF_GX = 3.2470f;   // gyro X offset
float IMU_OFF_GY = 1.5249f;   // gyro Y offset
float IMU_OFF_GZ = -1.7060f;   // gyro Z offset

// ── Serial motor test mode ────────────────────────────────────
// Enables serial commands to spin individual motors for testing.
// Safe to leave ON even in live flight (TEST_MODE=0).
// Commands: m1/m2/m3/m4 <µs>  all <µs>  stop
//           mpu  orient  imu reset  arm  disarm  escal  status
// !! REMOVE PROPS BEFORE USING MOTOR COMMANDS !!
#define MOTOR_TEST_MODE   1

// ── Telemetry rate ────────────────────────────────────────────
// Print one telemetry line every this many loop ticks.
// At 100Hz: 20 = prints 5 times/second (good for reading)
//           10 = 10 times/second (detailed but scrolls fast)
#define TEL_EVERY   20

// ── Watchdog timeout ──────────────────────────────────────────
// If the main loop freezes for longer than this, ESP32 resets.
// At 100Hz each loop tick is 10ms. 250ms = 25 missed ticks.
// This catches crashes, infinite loops, I2C hangs, stack overflow.
#define WDT_TIMEOUT_MS   250    // ms before forced reboot  ★ TUNE ME
// ================================================================
//  END OF TUNING SECTION
// ================================================================


// ── Library objects ───────────────────────────────────────────
Servo   esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// ── Flight state ──────────────────────────────────────────────
bool  armed        = false;    // true = motors active
bool  ledState     = false;    // current LED on/off state
bool  lastLBt      = false;    // previous arm button state
bool  lastRBt      = false;    // previous LED button state
bool  signalLost   = false;    // true = failsafe active
int   failThrottle = ESC_MIN;  // throttle used during failsafe descent

// ── Accumulated throttle ──────────────────────────────────────
// This persists between loop ticks. Stick nudges it up/down.
// Center stick = no change = craft holds altitude.
float accThrottle = ESC_SAFE;

// ── Slew limiting state — previous motor values ───────────────
int prevM1 = ESC_MIN, prevM2 = ESC_MIN;
int prevM3 = ESC_MIN, prevM4 = ESC_MIN;

// ── IMU ground reference ──────────────────────────────────────
// The angle the craft sits at on flat ground. Subtracted from
// all angle readings so "level" always reads as 0°.
float groundRoll  = 0.0f;
float groundPitch = 0.0f;

// ── IIR-filtered angle state ──────────────────────────────────
// Running filtered angles fed into the PID each tick.
float filtRoll  = 0.0f;
float filtPitch = 0.0f;

// ── PID state ─────────────────────────────────────────────────
// [0]=Roll  [1]=Pitch  [2]=Yaw
float pidPrevErr[3]  = {0, 0, 0};
float pidIntegral[3] = {0, 0, 0};

// ── Timing ────────────────────────────────────────────────────
unsigned long lastDisarmMs  = 0;   // for re-arm guard
unsigned long bothBtnHeldMs = 0;   // for IMU reset hold timer
bool          bothBtnHeld   = false;
unsigned long loopTimer     = 0;

// ── Telemetry ─────────────────────────────────────────────────
int telTick = 0;

// ── Last motor outputs (for telemetry display) ────────────────
int lastM1 = ESC_MIN, lastM2 = ESC_MIN;
int lastM3 = ESC_MIN, lastM4 = ESC_MIN;

// ── Serial motor test state ───────────────────────────────────
int  testM1 = ESC_MIN, testM2 = ESC_MIN;
int  testM3 = ESC_MIN, testM4 = ESC_MIN;
bool testOverride = false;   // true = serial test has control


// ================================================================
//  SECTION 1 — ESC / MOTOR HELPERS
// ================================================================

// ── Slew limit: cap how fast one motor value can change ───────
// Prevents sudden PID output spikes from reaching the ESCs.
// target   = where we want to go
// previous = where we were last tick
int slewMotor(int target, int previous)
{
  int delta = constrain(target - previous, -MAX_MOTOR_STEP, MAX_MOTOR_STEP);
  return previous + delta;
}

// ── Write to all four ESCs ────────────────────────────────────
// Applies trim, clamps to ESC range, applies slew limit, writes.
void writeMotors(int m1, int m2, int m3, int m4)
{
  // 1. Apply per-motor trim to compensate uneven motor speeds
  m1 += MOTOR_TRIM[0];
  m2 += MOTOR_TRIM[1];
  m3 += MOTOR_TRIM[2];
  m4 += MOTOR_TRIM[3];

  // 2. Hard clamp — never send outside valid ESC range
  m1 = constrain(m1, ESC_MIN, ESC_MAX);
  m2 = constrain(m2, ESC_MIN, ESC_MAX);
  m3 = constrain(m3, ESC_MIN, ESC_MAX);
  m4 = constrain(m4, ESC_MIN, ESC_MAX);

  // 3. Slew limit — smooths motor commands, reduces vibration
  m1 = slewMotor(m1, prevM1);
  m2 = slewMotor(m2, prevM2);
  m3 = slewMotor(m3, prevM3);
  m4 = slewMotor(m4, prevM4);

  // 4. Remember values for next tick and for telemetry
  prevM1 = m1; prevM2 = m2; prevM3 = m3; prevM4 = m4;
  lastM1 = m1; lastM2 = m2; lastM3 = m3; lastM4 = m4;

#if TEST_MODE
  return;   // TEST_MODE: record the values but don't drive ESCs
#endif

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

// ── Hard stop all motors — bypasses slew and TEST_MODE ────────
// Used on disarm and failsafe floor. Always fires the ESCs.
void motorsOff()
{
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_MIN;
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

// ── Arm sequence — safe, no 2000µs spike ──────────────────────
// Previous versions sent MAX (2000µs) to calibrate ESC range.
// That's dangerous. SimonK ESCs remember calibration from the
// first "escal" run. Normal arm now just:
//   1. Hold MIN for 1s so ESC sees signal is alive
//   2. Ramp gently to ESC_SAFE over ~1s
void runArmSequence()
{
  Serial.println("[FC][ARM] Arming — gentle ramp, no spike");

  // Step 1: hold minimum signal — ESC needs to see this first
  Serial.println("[FC][ARM] Step 1: holding MIN for 1s ...");
  unsigned long t = millis();
  while (millis() - t < 1000)
  {
    esc1.writeMicroseconds(ESC_MIN);
    esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN);
    esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }

  // Step 2: ramp gently from MIN to ESC_SAFE
  // Step size of 2µs with 20ms delay = very smooth spin-up
  Serial.printf("[FC][ARM] Step 2: ramping to safe idle (%dµs) ...\n", ESC_SAFE);
  for (int us = ESC_MIN; us <= ESC_SAFE; us += 2)
  {
    esc1.writeMicroseconds(us);
    esc2.writeMicroseconds(us);
    esc3.writeMicroseconds(us);
    esc4.writeMicroseconds(us);
    delay(20);
  }

  // Sync slew state — prevents first loop tick from jumping
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_SAFE;
  lastM1 = lastM2 = lastM3 = lastM4 = ESC_SAFE;

  Serial.printf("[FC][ARM] Armed — motors at %dµs. Push LEFT stick UP to climb.\n",
                ESC_SAFE);
}

// ── One-time ESC range calibration (serial "escal" command) ───
// Run this ONCE per ESC with props removed.
// After this, the normal safe arm sequence is all you need.
// SimonK saves the calibration to flash — it survives power off.
void runESCCalibration()
{
  Serial.println("[FC][ESCAL] !! REMOVE PROPS !! ESC range calibration starting");
  Serial.println("[FC][ESCAL] Sending MIN for 3s ...");
  unsigned long t = millis();
  while (millis() - t < 3000)
  {
    esc1.writeMicroseconds(ESC_MIN); esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN); esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Sending MAX for 0.5s — motors may spin briefly!");
  t = millis();
  while (millis() - t < 500)
  {
    esc1.writeMicroseconds(ESC_MAX); esc2.writeMicroseconds(ESC_MAX);
    esc3.writeMicroseconds(ESC_MAX); esc4.writeMicroseconds(ESC_MAX);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Back to MIN — listen for ESC confirmation beeps ...");
  t = millis();
  while (millis() - t < 2000)
  {
    esc1.writeMicroseconds(ESC_MIN); esc2.writeMicroseconds(ESC_MIN);
    esc3.writeMicroseconds(ESC_MIN); esc4.writeMicroseconds(ESC_MIN);
    delay(20);
  }
  Serial.println("[FC][ESCAL] Done. ESCs calibrated. Safe to arm normally now.");
}

// ── Attach ESC PWM outputs ────────────────────────────────────
void initESCs()
{
  // Each ESC needs its own hardware timer on ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // 50Hz PWM, constrained to 1000–2000µs range
  esc1.setPeriodHertz(50); esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.setPeriodHertz(50); esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.setPeriodHertz(50); esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.setPeriodHertz(50); esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);

  // Start with motors off
  motorsOff();
  Serial.println("[FC] ESCs attached — MIN throttle sent");
}


// ================================================================
//  SECTION 2 — IMU HELPERS
// ================================================================

// ── Angle deadzone ────────────────────────────────────────────
// Returns 0 if value is within ±dz, otherwise shifts inward.
// Prevents tiny oscillating corrections when craft is still.
float angleDeadzone(float val, float dz)
{
  if (val >  dz) return val - dz;
  if (val < -dz) return val + dz;
  return 0.0f;
}

// ── Gyro deadzone ─────────────────────────────────────────────
float gyroDeadzone(float val, float dz)
{
  if (val >  dz) return val - dz;
  if (val < -dz) return val + dz;
  return 0.0f;
}

// ── Get roll angle: adjusted, filtered, deadzone applied ──────
float getAdjustedRoll()
{
  // Subtract ground reference so flat ground = 0°
  float raw = ROLL_FLIP * (mpu.getAngleX() - groundRoll);

  // IIR low-pass filter — smooths out vibration noise
  filtRoll = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtRoll;

  // Apply deadzone to filtered result
  return angleDeadzone(filtRoll, IMU_ANGLE_DEADZONE);
}

// ── Get pitch angle: adjusted, filtered, deadzone applied ─────
float getAdjustedPitch()
{
  float raw  = PITCH_FLIP * (mpu.getAngleY() - groundPitch);
  filtPitch  = ANGLE_FILTER_ALPHA * raw + (1.0f - ANGLE_FILTER_ALPHA) * filtPitch;
  return angleDeadzone(filtPitch, IMU_ANGLE_DEADZONE);
}

// ── Get yaw rate with deadzone ────────────────────────────────
float getAdjustedYawRate()
{
  return gyroDeadzone(YAW_FLIP * mpu.getGyroZ(), GYRO_DEADZONE_DPS);
}

// ── Capture ground reference from accelerometer ───────────────
// Averages many samples directly from the accelerometer.
// This is fast (~1s) and accurate — does NOT wait for the
// complementary filter to converge (that takes 20–30s).
//
// The accelerometer always knows the correct gravity vector.
// We just need to average enough samples to wash out noise.
void captureGroundReference()
{
  Serial.printf("[FC][IMU] Capturing ground reference (%d samples) ...\n",
                GROUND_REF_SAMPLES);
  Serial.println("[FC][IMU] Keep craft flat and still.");

  // First pass: rough average over 20 samples to get a baseline
  // for outlier rejection in the main pass.
  float roughRoll = 0, roughPitch = 0;
  for (int i = 0; i < 20; i++)
  {
    mpu.update();
    roughRoll  += mpu.getAngleX();
    roughPitch += mpu.getAngleY();
    delay(5);
  }
  roughRoll  /= 20.0f;
  roughPitch /= 20.0f;

  // Second pass: collect GROUND_REF_SAMPLES good samples,
  // rejecting any that differ from the rough average by more
  // than GROUND_REF_REJECT_DEG (protects against bumps/taps).
  float sumRoll  = 0.0f;
  float sumPitch = 0.0f;
  int   accepted = 0;
  int   attempts = 0;

  while (accepted < GROUND_REF_SAMPLES &&
         attempts < GROUND_REF_SAMPLES * 3)  // give up after 3× attempts
  {
    mpu.update();
    float r = mpu.getAngleX();
    float p = mpu.getAngleY();
    attempts++;

    // Reject if too far from rough average (craft was bumped)
    if (fabsf(r - roughRoll)  > GROUND_REF_REJECT_DEG ||
        fabsf(p - roughPitch) > GROUND_REF_REJECT_DEG)
    {
      delay(5);
      continue;   // skip this sample, try again
    }

    sumRoll  += r;
    sumPitch += p;
    accepted++;
    delay(5);   // 5ms per sample → 150 samples ≈ 0.75s total
  }

  // If most samples were rejected, the craft was moving.
  // Fall back to rough average rather than crashing out.
  if (accepted < GROUND_REF_SAMPLES / 2)
  {
    Serial.println("[FC][IMU] Warning: many samples rejected — was craft moving?");
    Serial.println("[FC][IMU] Using rough average. Send 'imu reset' when still.");
    groundRoll  = roughRoll;
    groundPitch = roughPitch;
  }
  else
  {
    groundRoll  = sumRoll  / (float)accepted;
    groundPitch = sumPitch / (float)accepted;
    Serial.printf("[FC][IMU] Accepted %d/%d samples\n", accepted, attempts);
  }

  // Reset IIR filter state to zero (new reference = zero point).
  // Without this the filter would unwind slowly from its old state.
  filtRoll  = 0.0f;
  filtPitch = 0.0f;

  Serial.printf("[FC][IMU] Ground ref → Roll=%.3f°  Pitch=%.3f°\n",
                groundRoll, groundPitch);
  Serial.println("[FC][IMU] Ready.");
}

// ── Initialise MPU-6050 ───────────────────────────────────────
// HARDWARE NOTE:
//   Add 100nF ceramic cap from VCC to GND on the MPU module,
//   as close to the chip as possible (decoupling).
//   Add 10µF electrolytic in parallel for bulk filtering.
//   GY-521 modules already have 4.7kΩ pull-ups on SDA/SCL.
//   Keep I2C wires twisted together, away from ESC wires.
bool initIMU()
{
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);   // 400kHz fast mode — faster reads

  byte status = mpu.begin();
  if (status != 0)
  {
    Serial.printf("[FC][IMU] INIT FAILED — error %d\n", status);
    return false;
  }
  Serial.println("[FC][IMU] MPU-6050 OK");

  // Apply calibration offsets (set via CALIBRATE_IMU routine)
  mpu.setAccOffsets(IMU_OFF_AX, IMU_OFF_AY, IMU_OFF_AZ);
  mpu.setGyroOffsets(IMU_OFF_GX, IMU_OFF_GY, IMU_OFF_GZ);

  // Brief warm-up — give the library filter some initial data.
  // 500ms + 30 reads is enough; we don't wait for convergence
  // because captureGroundReference() reads the accel directly.
  delay(500);
  for (int i = 0; i < 30; i++) { mpu.update(); delay(10); }

  // Capture ground reference (~0.75s, fast accel method)
  captureGroundReference();

  return true;
}

// ── Orientation check (serial command "orient") ───────────────
void printOrientationInfo()
{
  Serial.println("[ORIENT] Tilt NOSE DOWN  → Adjusted Pitch should be NEGATIVE");
  Serial.println("[ORIENT] Tilt RIGHT DOWN → Adjusted Roll  should be POSITIVE");
  Serial.printf ("[ORIENT] PITCH_FLIP=%.1f  ROLL_FLIP=%.1f\n", PITCH_FLIP, ROLL_FLIP);
  Serial.println("[ORIENT] Live readings for 5s:");
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
  for (int i = 0; i < 3; i++)
  {
    pidPrevErr[i]  = 0;
    pidIntegral[i] = 0;
  }
}

void doArm()
{
  if (armed) { Serial.println("[FC][ARM] Already armed"); return; }

  if (!receiver.connected())
  {
    Serial.println("[FC][ARM] Cannot arm — no TX signal");
    return;
  }

  unsigned long guard = millis() - lastDisarmMs;
  if (guard < REARM_GUARD_MS)
  {
    Serial.printf("[FC][ARM] Re-arm guard — wait %lums\n",
                  REARM_GUARD_MS - guard);
    return;
  }

  resetPID();

  // Reset accumulated throttle to safe idle.
  // Pilot must deliberately push the stick up to climb.
  accThrottle = ESC_SAFE;

  // Reset slew state so arm ramp doesn't fight stale prev values
  prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;

  armed = true;

#if TEST_MODE
  Serial.println("[FC][ARM] ARMED (TEST MODE — no ESC output)");
  Serial.printf ("[FC][ARM] accThrottle reset to %dµs\n", ESC_SAFE);
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

// ── Expo curve ────────────────────────────────────────────────
// Blends linear and cubic response.
// At center stick: feels linear and gentle.
// At full stick: full authority.
// v = normalised stick (-1.0 to +1.0)
float applyExpo(float v, float expo)
{
  return expo * v * v * v + (1.0f - expo) * v;
}

// ── Stick deadband ────────────────────────────────────────────
// Returns 0 if inside deadband, otherwise shifts value inward
// so there is no jump at the deadband edge.
int applyDeadband(int raw)
{
  if (raw >  DEADBAND) return raw - DEADBAND;
  if (raw < -DEADBAND) return raw + DEADBAND;
  return 0;
}

// ── Accumulated throttle update ───────────────────────────────
// The throttle stick is treated as a RATE command, not a
// position command. This means:
//   Stick UP   → accThrottle increases slowly each tick
//   Stick CENTER → accThrottle stays exactly where it is
//   Stick DOWN → accThrottle decreases slowly each tick
//
// This is what gives you "hands-off hold" — just center the
// stick and the craft stays at its current throttle.
// It also makes cheap sticks much safer since snap-back to
// center doesn't cause a throttle change.
void updateAccumulatedThrottle(int rawThrottle)
{
  int db = applyDeadband(rawThrottle);

  if (db > 0)
  {
    // Stick above center — scale ramp by how far it's pushed
    // so half-stick climbs at half speed, full-stick at full speed
    float fraction = (float)db / (99.0f - DEADBAND);
    accThrottle += THROTTLE_RAMP_RATE * fraction;
  }
  else if (db < 0)
  {
    float fraction = (float)(-db) / (99.0f - DEADBAND);
    accThrottle -= THROTTLE_RAMP_RATE * fraction;
  }
  // db == 0: do nothing — throttle holds

  // Clamp to safe range — can't go below safe idle or above max
  accThrottle = constrain(accThrottle,
                          (float)THROTTLE_MIN,
                          (float)THROTTLE_MAX);
}

// ── Read and process all stick inputs ─────────────────────────
void readSticks(int   &throttleUs,
                float &targetRoll,
                float &targetPitch,
                float &targetYawRate)
{
  // Update accumulated throttle from left stick Y axis
  updateAccumulatedThrottle(receiver.data.Ly);
  throttleUs = (int)accThrottle;

  // Roll/Pitch/Yaw: deadband then expo then scale to degrees/deg/s
  int rawYaw   = applyDeadband(receiver.data.Lx);
  int rawPitch = applyDeadband(receiver.data.Ry);
  int rawRoll  = applyDeadband(receiver.data.Rx);

  // Normalise to -1.0..+1.0 then apply expo curve
  float nYaw   = applyExpo(rawYaw   / 99.0f, EXPO_YAW);
  float nPitch = applyExpo(rawPitch / 99.0f, EXPO_PITCH);
  float nRoll  = applyExpo(rawRoll  / 99.0f, EXPO_ROLL);

  // Scale to target degrees / deg/s
  // STICK_*_FLIP applied here — does not affect IMU orientation
  targetRoll    = nRoll  * MAX_ANGLE_ROLL  * STICK_ROLL_FLIP;
  targetPitch   = nPitch * MAX_ANGLE_PITCH * STICK_PITCH_FLIP;
  targetYawRate = nYaw   * MAX_RATE_YAW   * STICK_YAW_FLIP;
}


// ================================================================
//  SECTION 5 — PID CONTROLLER
// ================================================================

// ── Compute one PID axis ──────────────────────────────────────
// axis     : 0=Roll, 1=Pitch, 2=Yaw
// setpoint : desired angle or rate
// measured : actual angle or rate from IMU
// dt       : time since last call in seconds
// returns  : µs correction to add to motor mix
float computePID(int axis, float setpoint, float measured, float dt)
{
  float error = setpoint - measured;

  // Integral: accumulates over time, corrects persistent drift.
  // Clamped to ITERM_MAX to prevent windup on the ground.
  pidIntegral[axis] += error * dt;
  pidIntegral[axis]  = constrain(pidIntegral[axis],
                                 -ITERM_MAX, ITERM_MAX);

  // Derivative: rate of error change, damps oscillation.
  float derivative = (error - pidPrevErr[axis]) / dt;
  pidPrevErr[axis]  = error;

  return KP[axis] * error
       + KI[axis] * pidIntegral[axis]
       + KD[axis] * derivative;
}


// ================================================================
//  SECTION 6 — MOTOR MIXING  (X-Config)
// ================================================================
//
//  Motor positions (viewed from above):
//       M1(CW)   M2(CCW)
//         \  FRONT  /
//          \       /
//          /       \
//         /  BACK   \
//       M4(CCW)  M3(CW)
//
//  Each motor's contribution to each axis:
//  M1 Front-Left  CW  : +throttle +pitch +roll -yaw
//  M2 Front-Right CCW : +throttle +pitch -roll +yaw
//  M3 Back-Right  CW  : +throttle -pitch -roll -yaw
//  M4 Back-Left   CCW : +throttle -pitch +roll +yaw
//
//  Sign logic:
//    ROLL right  → M1,M4 (left side) speed up,  M2,M3 (right) slow down
//    PITCH forward → M3,M4 (back)   speed up,  M1,M2 (front) slow down
//    YAW clockwise → CW motors (M1,M3) slow down, CCW (M2,M4) speed up

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

// Called when signal is lost while armed.
// Slowly reduces throttle until craft lands at DESCENT_FLOOR.
// PID still active to keep craft level during descent.
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

// ── Arm switch (left button) ──────────────────────────────────
void handleArmSwitch(bool curLBt)
{
  if (curLBt && !armed)      doArm();
  else if (!curLBt && armed) doDisarm();
  lastLBt = curLBt;
}

// ── LED toggle (right button) ─────────────────────────────────
void handleLEDToggle(bool curRBt)
{
  if (curRBt && !lastRBt)
  {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }
  lastRBt = curRBt;
}

// ── IMU reset (hold both analog buttons 5s) ───────────────────
// Re-captures the ground reference without rebooting.
// Useful if craft was moved between power-on and flight.
void handleIMUReset(bool la, bool ra)
{
  if (la && ra)
  {
    if (!bothBtnHeld)
    {
      bothBtnHeld   = true;
      bothBtnHeldMs = millis();
      Serial.println("[FC][IMU] Hold 5s to reset IMU ground reference ...");
    }
    else if (millis() - bothBtnHeldMs >= IMU_RESET_HOLD_MS)
    {
      Serial.println("[FC][IMU] IMU ground reference RESET");
      // Extra reads so filter has fresh data before we sample
      for (int i = 0; i < 30; i++) { mpu.update(); delay(10); }
      captureGroundReference();
      resetPID();
      bothBtnHeldMs = millis();   // reset timer so it doesn't re-fire
      // 4 quick LED blinks to confirm
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
//  SECTION 9 — SERIAL MOTOR TEST COMMANDS
// ================================================================

#if MOTOR_TEST_MODE
void handleSerialCommands()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  // ── m1/m2/m3/m4 <µs> — spin one motor ──────────────────────
  if (cmd.startsWith("m1 ") || cmd.startsWith("m2 ") ||
      cmd.startsWith("m3 ") || cmd.startsWith("m4 "))
  {
    int motorNum = cmd.charAt(1) - '0';
    int us = constrain(cmd.substring(3).toInt(), ESC_MIN, ESC_MAX);
    testOverride = true;
    switch (motorNum)
    {
      case 1: testM1 = us; break;
      case 2: testM2 = us; break;
      case 3: testM3 = us; break;
      case 4: testM4 = us; break;
    }
    esc1.writeMicroseconds(testM1); esc2.writeMicroseconds(testM2);
    esc3.writeMicroseconds(testM3); esc4.writeMicroseconds(testM4);
    Serial.printf("[TEST] M%d → %dµs\n", motorNum, us);
  }
  // ── all <µs> — spin all motors to same speed ────────────────
  else if (cmd.startsWith("all "))
  {
    int us = constrain(cmd.substring(4).toInt(), ESC_MIN, ESC_MAX);
    testOverride = true;
    testM1 = testM2 = testM3 = testM4 = us;
    esc1.writeMicroseconds(us); esc2.writeMicroseconds(us);
    esc3.writeMicroseconds(us); esc4.writeMicroseconds(us);
    Serial.printf("[TEST] All motors → %dµs\n", us);
  }
  // ── stop — cut all motors ───────────────────────────────────
  else if (cmd == "stop")
  {
    testOverride = false;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff();
    Serial.println("[TEST] All motors STOPPED");
  }
  // ── escal — one-time ESC range calibration ──────────────────
  // Run once per ESC with props removed. Saves to ESC flash.
  else if (cmd == "escal")
  {
    runESCCalibration();
  }
  // ── mpu — 50 live IMU readings ──────────────────────────────
  else if (cmd == "mpu")
  {
    Serial.println("[TEST][MPU] 50 readings:");
    for (int i = 0; i < 50; i++)
    {
      mpu.update();
      Serial.printf("[MPU] Roll:%+7.2f°  Pitch:%+7.2f°  Yaw:%+7.2f°/s\n",
                    getAdjustedRoll(), getAdjustedPitch(), getAdjustedYawRate());
      delay(50);
    }
    Serial.println("[TEST][MPU] Done.");
  }
  // ── orient — orientation check ──────────────────────────────
  else if (cmd == "orient")
  {
    printOrientationInfo();
  }
  // ── imu reset — re-capture ground reference ─────────────────
  else if (cmd == "imu reset")
  {
    Serial.println("[TEST][IMU] Resetting ground reference ...");
    for (int i = 0; i < 30; i++) { mpu.update(); delay(10); }
    captureGroundReference();
    resetPID();
  }
  // ── arm / disarm — force state change ───────────────────────
  else if (cmd == "arm")
  {
    armed = true;
    resetPID();
    accThrottle = ESC_SAFE;
    prevM1 = prevM2 = prevM3 = prevM4 = ESC_MIN;
    Serial.printf("[TEST] Force ARM — accThrottle=%dµs\n", ESC_SAFE);
  }
  else if (cmd == "disarm")
  {
    armed = false;
    lastDisarmMs = millis();
    testOverride = false;
    accThrottle  = ESC_SAFE;
    testM1 = testM2 = testM3 = testM4 = ESC_MIN;
    motorsOff();
    resetPID();
    Serial.println("[TEST] Force DISARM");
  }
  // ── thr — show current accumulated throttle ─────────────────
  else if (cmd == "thr")
  {
    Serial.printf("[TEST] accThrottle=%.1fµs  min=%d  max=%d\n",
                  accThrottle, THROTTLE_MIN, THROTTLE_MAX);
  }
  // ── trim — show motor trims ─────────────────────────────────
  else if (cmd == "trim")
  {
    Serial.printf("[TEST] MOTOR_TRIM  M1=%d  M2=%d  M3=%d  M4=%d µs\n",
                  MOTOR_TRIM[0], MOTOR_TRIM[1], MOTOR_TRIM[2], MOTOR_TRIM[3]);
  }
  // ── status — print all settings ─────────────────────────────
  else if (cmd == "status")
  {
    Serial.printf("[STATUS] Armed:%s  Link:%s  accThr:%.0f\n",
                  armed ? "Y" : "N",
                  receiver.connected() ? "Y" : "N",
                  accThrottle);
    Serial.printf("[STATUS] ESC_IDLE:%d  ESC_SAFE:%d  THR_MAX:%d  RAMP:%d\n",
                  ESC_IDLE, ESC_SAFE, THROTTLE_MAX, THROTTLE_RAMP_RATE);
    Serial.printf("[STATUS] TRIM M1=%d M2=%d M3=%d M4=%d\n",
                  MOTOR_TRIM[0],MOTOR_TRIM[1],MOTOR_TRIM[2],MOTOR_TRIM[3]);
    Serial.printf("[STATUS] AngleDZ:%.1f°  GyroDZ:%.1f°/s  Slew:%dµs\n",
                  IMU_ANGLE_DEADZONE, GYRO_DEADZONE_DPS, MAX_MOTOR_STEP);
    Serial.printf("[STATUS] KP:%.2f/%.2f/%.2f  KI:%.3f/%.3f/%.3f  KD:%.2f/%.2f/%.2f\n",
                  KP[0],KP[1],KP[2], KI[0],KI[1],KI[2], KD[0],KD[1],KD[2]);
  }
  // ── help ────────────────────────────────────────────────────
  else
  {
    Serial.println("[TEST] Commands:");
    Serial.println("  m1/m2/m3/m4 <µs>   e.g.  m1 1250");
    Serial.println("  all <µs>           e.g.  all 1200");
    Serial.println("  stop               cut all motors");
    Serial.println("  escal              one-time ESC calibration (props off!)");
    Serial.println("  mpu                50 IMU readings");
    Serial.println("  orient             orientation axis check");
    Serial.println("  imu reset          re-capture ground reference");
    Serial.println("  arm  /  disarm     force arm state");
    Serial.println("  thr                show accumulated throttle");
    Serial.println("  trim               show motor trims");
    Serial.println("  status             all settings");
  }
}
#endif


// ================================================================
//  SECTION 10 — TELEMETRY
// ================================================================

void printTelemetry(bool  linkOK,
                    int   throttleUs,
                    float tgtRoll,    float tgtPitch,   float tgtYaw,
                    float actRoll,    float actPitch,   float actYawRate,
                    float pRoll,      float pPitch,     float pYaw)
{
  // Format: ARM LNK | THR acc | targets | actuals | PID | motors | BAT
  Serial.printf(
    "[FC] ARM:%s LNK:%s | THR:%4d acc:%.0f | "
    "TGT R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "ACT R:%+5.1f P:%+5.1f YR:%+5.1f | "
    "PID R:%+5.1f P:%+5.1f Y:%+5.1f | "
    "M:%4d %4d %4d %4d | BAT:%3d%%\n",
    armed  ? "Y" : "N",
    linkOK ? "Y" : "N",
    throttleUs, accThrottle,
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
  // ── Emergency motor kill — first thing on every boot ─────────
  // If we rebooted due to a watchdog reset, the ESCs may still
  // be spinning from the last PWM signal. Drive all ESC pins LOW
  // immediately before any library initialisation.
  // This sends an invalid signal that causes SimonK to cut motors.
  pinMode(PIN_M1, OUTPUT); digitalWrite(PIN_M1, LOW);
  pinMode(PIN_M2, OUTPUT); digitalWrite(PIN_M2, LOW);
  pinMode(PIN_M3, OUTPUT); digitalWrite(PIN_M3, LOW);
  pinMode(PIN_M4, OUTPUT); digitalWrite(PIN_M4, LOW);
  delay(100);   // hold LOW long enough for ESC to see it
  // initESCs() will re-attach the Servo library to these pins shortly

  Serial.begin(115200);
  delay(600);

  // ── Hardware watchdog (ESP32 Arduino core v3.x API) ──────────
  // If the loop freezes, WDT resets the ESP32 after WDT_TIMEOUT_MS. 
  // motorsOff() is called on reboot via initESCs() in setup().
  // This is the last line of defence against a frozen flight controller.
  esp_task_wdt_config_t wdt_config = {.timeout_ms     = WDT_TIMEOUT_MS,.idle_core_mask = 0,.trigger_panic  = true};// ms before reset, don't watch idle task, hard reset on timeout
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("[FC] Watchdog armed — timeout: " +
                 String(WDT_TIMEOUT_MS) + "ms");

  Serial.println("\n[FC] ═══════════════════════════════════════════");
  Serial.println("[FC]  Q450 Flight Controller  v2.4  — BOOT");
  Serial.printf ("[FC]  TEST_MODE    : %s\n",
                 TEST_MODE ? "ON  (no ESC output)" : "OFF (LIVE)");
  Serial.printf ("[FC]  ESC_IDLE     : %dµs\n", ESC_IDLE);
  Serial.printf ("[FC]  ESC_SAFE     : %dµs  (armed idle)\n", ESC_SAFE);
  Serial.printf ("[FC]  THROTTLE_MAX : %dµs\n", THROTTLE_MAX);
  Serial.printf ("[FC]  RAMP_RATE    : %dµs/tick at %dHz\n",
                 THROTTLE_RAMP_RATE, LOOP_HZ);
  Serial.printf ("[FC]  TRIM         : M1=%d M2=%d M3=%d M4=%d\n",
                 MOTOR_TRIM[0],MOTOR_TRIM[1],MOTOR_TRIM[2],MOTOR_TRIM[3]);
  Serial.printf ("[FC]  KP %.2f/%.2f/%.2f  KI %.3f/%.3f/%.3f  KD %.2f/%.2f/%.2f\n",
                 KP[0],KP[1],KP[2], KI[0],KI[1],KI[2], KD[0],KD[1],KD[2]);
  Serial.println("[FC] ═══════════════════════════════════════════\n");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Attach ESC PWM outputs and send MIN signal
  initESCs();

  // ── IMU calibration helper ──────────────────────────────────
  // Set CALIBRATE_IMU 1 at top of file, flash, lay craft flat,
  // copy the printed offsets into IMU_OFF_* values, set back to 0.
#if CALIBRATE_IMU
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.begin();
  Serial.println("[CAL] Lay craft FLAT — starting in 3s ...");
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.printf("[CAL] IMU_OFF_AX = %.4f;\n", mpu.getAccXoffset());
  Serial.printf("[CAL] IMU_OFF_AY = %.4f;\n", mpu.getAccYoffset());
  Serial.printf("[CAL] IMU_OFF_AZ = %.4f;\n", mpu.getAccZoffset());
  Serial.printf("[CAL] IMU_OFF_GX = %.4f;\n", mpu.getGyroXoffset());
  Serial.printf("[CAL] IMU_OFF_GY = %.4f;\n", mpu.getGyroYoffset());
  Serial.printf("[CAL] IMU_OFF_GZ = %.4f;\n", mpu.getGyroZoffset());
  Serial.println("[CAL] Paste values above into IMU_OFF_* and set CALIBRATE_IMU 0.");
  while (1);   // halt until reflashed
#endif

  // Initialise IMU — includes fast ground reference capture
  if (!initIMU())
  {
    Serial.println("[FC] FATAL: IMU not found — halting");
    // Fast blink forever to signal hardware fault
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  // Start receiver — listens for ESP-NOW packets from TX_MAC
  receiver.timeoutMs = 1000;
  receiver.begin(TX_MAC, true);

  // ── Orientation reminder ────────────────────────────────────
  Serial.println("[FC] ── Orientation check hint ─────────────────");
  Serial.printf ("[FC]  Nose-down  pitch → Adjusted Pitch should be %s\n",
                 PITCH_FLIP > 0 ? "NEGATIVE" : "POSITIVE");
  Serial.printf ("[FC]  Right-down roll  → Adjusted Roll  should be %s\n",
                 ROLL_FLIP  > 0 ? "POSITIVE" : "NEGATIVE");
  Serial.println("[FC]  Send 'orient' to verify.  Send 'status' for all settings.");
  Serial.println("[FC] ────────────────────────────────────────────");

#if MOTOR_TEST_MODE
  Serial.println("[FC] Motor Test ACTIVE — REMOVE PROPS!");
  Serial.println("[FC]  Type 'help' for commands, 'status' for settings.");
#endif

  // 3 ready blinks
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_LED, HIGH); delay(150); 
    digitalWrite(PIN_LED, LOW);  delay(150);
  }

  Serial.println("\n[FC] Ready — set LBt ON to ARM\n");
  loopTimer = micros();
}


// ================================================================
//  MAIN LOOP
// ================================================================

void loop()
{
  // ── Fixed-rate gate — runs at exactly LOOP_HZ ────────────────
  while (micros() - loopTimer < LOOP_US);
  float dt  = (micros() - loopTimer) / 1e6f;   // seconds since last tick
  loopTimer = micros();

  // ── 1. Serial motor test commands ────────────────────────────
  // Processed first so test override takes immediate effect
#if MOTOR_TEST_MODE
  handleSerialCommands();
  // If serial test has taken control, skip normal flight loop
  if (testOverride) { delay(2); return; }
#endif

  // ── 2. Update receiver and IMU ───────────────────────────────
  receiver.update();
  bool linkOK = receiver.connected();
  mpu.update();   // reads new gyro+accel data into library buffers

  // Compute adjusted, filtered, deadzone-applied angles
  float actualRoll    = getAdjustedRoll();
  float actualPitch   = getAdjustedPitch();
  float actualYawRate = getAdjustedYawRate();

  // ── 3. Signal-loss detection ──────────────────────────────────
  if (!linkOK && !signalLost)
  {
    // Signal just dropped — start failsafe
    signalLost   = true;
    // Failsafe starts from current throttle (not ESC_MIN)
    // so there's no sudden drop or climb when signal cuts
    failThrottle = armed ? (int)accThrottle : ESC_MIN;
    resetPID();
    Serial.println("[FC] SIGNAL LOST — failsafe descent");
  }
  else if (linkOK && signalLost)
  {
    // Signal restored — sync accThrottle to where failsafe landed
    // so there's no jump when normal control resumes
    signalLost  = false;
    accThrottle = (float)failThrottle;
    Serial.println("[FC] Signal restored");
  }

  // ── 4. Button handling ────────────────────────────────────────
  handleArmSwitch(receiver.data.LBt);
  handleLEDToggle(receiver.data.RBt);
  handleIMUReset(receiver.data.LABt, receiver.data.RABt);

  // ── 5. Stick processing ───────────────────────────────────────
  int   throttleUs   = (int)accThrottle;
  float tgtRoll      = 0.0f;
  float tgtPitch     = 0.0f;
  float tgtYawRate   = 0.0f;

  if (linkOK && armed)
  {
    // Normal flight: read sticks (also updates accThrottle)
    readSticks(throttleUs, tgtRoll, tgtPitch, tgtYawRate);
  }
  else if (!linkOK)
  {
    // No signal: use failsafe throttle, zero attitude targets
    throttleUs = failThrottle;
  }
  // Disarmed + link OK: don't update accThrottle, motors stay off

  // ── 6. PID integral reset when on the ground ─────────────────
  // If throttle is near safe idle, craft is probably on the ground.
  // Resetting integral prevents windup that would cause a sudden
  // jump when throttle is first increased.
  if (throttleUs <= ESC_SAFE + 20)
  {
    pidIntegral[0] = 0;
    pidIntegral[1] = 0;
    pidIntegral[2] = 0;
  }

  // ── 7. Run PID controllers ────────────────────────────────────
  float pidRoll  = computePID(0, tgtRoll,    actualRoll,    dt);
  float pidPitch = computePID(1, tgtPitch,   actualPitch,   dt);
  float pidYaw   = computePID(2, tgtYawRate, actualYawRate, dt);

  // ── 8. Motor output ───────────────────────────────────────────
  if (!armed)
  {
    // Disarmed: always cut motors regardless of anything else
    motorsOff();
  }
  else if (signalLost)
  {
    // Failsafe: descend slowly with levelling active
    handleFailsafe(pidRoll, pidPitch, pidYaw);
  }
  else
  {
    // Normal flight: mix PID output into motor commands
    mixAndWrite(throttleUs, pidRoll, pidPitch, pidYaw);
  }

  // ── 9. Telemetry — print every TEL_EVERY ticks ───────────────
  if (++telTick >= TEL_EVERY)
  {
    telTick = 0;
    printTelemetry(linkOK, throttleUs,
                   tgtRoll, tgtPitch, tgtYawRate,
                   actualRoll, actualPitch, actualYawRate,
                   pidRoll, pidPitch, pidYaw);
  }

  // ── Feed the watchdog ─────────────────────────────────────────
  // Must be called every loop tick. If this line is never reached
  // (freeze, infinite loop, I2C hang) the WDT resets the ESP32.
  esp_task_wdt_reset();

}