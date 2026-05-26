# Q450 ESP32 Flight Controller — Quick Reference
### Firmware v1.6 | May 2026

---

## Version History

| Version | Key Change |
|---|---|
| v1.6 | PID tuned for stable hover, angles and limits tightened |
| v1.5 | Hardware watchdog, emergency motor kill on boot, stick flip fixes |
| v1.4 | Fast IMU boot (~1s), PITCH_FLIP corrected, IMU calibration offsets |
| v1.3 | Accumulated throttle, per-motor trim, safe arm sequence |
| v1.2 | Angle mode, self-levelling PID, ground reference at boot |

---

## What Changed Each Version

### v1.5 → v1.6
- PID tuned for stable hover: KP 0.70, KI 0.05, KD 0.25
- MAX_ANGLE reduced to 12° — much harder to flip
- THROTTLE_RAMP_RATE lowered to 1 (100µs/s — very slow climb)
- THROTTLE_MAX capped at 1700µs for safety
- EXPO raised to 0.50 — gentler stick feel around center
- IMU_ANGLE_DEADZONE raised to 1.0° to absorb residual sensor offset
- MAX_MOTOR_STEP reduced to 20µs for smoother motor response
- ITERM_MAX tightened to 20µs to prevent integral windup

### v1.4 → v1.5
- Hardware watchdog added — 250ms timeout, force-reboots ESP32 if loop freezes
- Emergency motor kill at start of every boot — drives ESC pins LOW before
  Servo library attaches, prevents latched PWM from spinning motors after a crash/reset
- STICK_PITCH_FLIP / STICK_ROLL_FLIP / STICK_YAW_FLIP added — fixes reversed
  pitch/roll direction from controller without touching IMU orientation defines
- Motor spin directions corrected — all four motors had CW/CCW swapped,
  fixed by swapping two bullet connectors per motor (not in software)
- Pin comments corrected to match actual motor directions after swap

### v1.3 → v1.4
- PITCH_FLIP set to -1.0f — GY-521 mounted with Y axis inverted on this frame
- IMU calibration offsets filled in from CALIBRATE_IMU routine
- captureGroundReference() rewritten to use fast accelerometer averaging (~0.75s)
  instead of waiting 20-30s for complementary filter to converge
- Outlier rejection added — samples >2° from rough average discarded (bump protection)
- Capacitor guidance added: 100nF + 10µF on MPU, 100nF + 10µF on ESP32 3.3V,
  2200µF at PDB battery leads, 10µF on ESP32 EN pin

### v1.2 → v1.3
- Accumulated throttle — left stick Y is now a rate command not a position command
  Center stick = hold throttle, up = climb slowly, down = descend slowly
- THROTTLE_RAMP_RATE controls how fast throttle changes per loop tick
- Per-motor trim offsets (MOTOR_TRIM[4]) — compensates uneven motor speeds
- Safe arm sequence — removed dangerous MIN→MAX→MIN ESC calibration from normal arm
  Now just ramps gently from ESC_MIN to ESC_SAFE
- ESC calibration moved to serial command "escal" — only needed once, saves to ESC flash
- ESC_SAFE lowered from 1200 to 1120 (1200 was lifting the craft)
- accThrottle and prevM* reset on arm — fixes throttle jump and slew lag after re-arm

---

## Current Tuning Values (v1.6)

```cpp
// PID
float KP[3] = { 0.70f,  0.70f,  0.50f };   // Roll, Pitch, Yaw
float KI[3] = { 0.05f,  0.05f,  0.00f };
float KD[3] = { 0.25f,  0.25f,  0.10f };

// Limits
#define MAX_ANGLE_ROLL    12.0f
#define MAX_ANGLE_PITCH   12.0f
#define MAX_RATE_YAW      80.0f
#define THROTTLE_RAMP_RATE   1
#define THROTTLE_MAX      1700
#define ITERM_MAX         20.0f
#define MAX_MOTOR_STEP    20
#define EXPO_ROLL         0.50f
#define EXPO_PITCH        0.50f
#define IMU_ANGLE_DEADZONE  1.0f

// IMU orientation for this build
#define PITCH_FLIP   -1.0f
#define ROLL_FLIP     1.0f
#define YAW_FLIP      1.0f

// Stick direction (separate from IMU orientation)
#define STICK_PITCH_FLIP  -1.0f
#define STICK_ROLL_FLIP    1.0f
#define STICK_YAW_FLIP     1.0f
```

---

## First Boot Checklist

```
[ ] 1. CALIBRATE_IMU 1 → flash → copy 6 offsets → paste → set back to 0
[ ] 2. escal command   → one-time ESC range calibration (props off)
[ ] 3. TEST_MODE 1     → bench test: orient, mpu, arm/disarm, signal loss
[ ] 4. Motor check     → m1/m2/m3/m4 1300 → verify CW/CCW each motor
[ ] 5. TEST_MODE 0     → props off → arm → raise throttle → check spin
[ ] 6. imu reset       → on flat hard surface before every flight
[ ] 7. First hover     → 10-20cm only, very slow throttle ramp
```

---

## Motor Direction Check

Props off. Type in serial monitor one at a time:

```
m1 1300   →  Front-Left   must spin CLOCKWISE        (viewed from above)
m2 1300   →  Front-Right  must spin COUNTER-CLOCKWISE
m3 1300   →  Back-Right   must spin CLOCKWISE
m4 1300   →  Back-Left    must spin COUNTER-CLOCKWISE
stop
```

Wrong direction → swap any two bullet connectors on that motor. Never fix in software.

---

## IMU Orientation Check

Type `orient` in serial. Tilt craft physically:

| Action | Expected reading |
|---|---|
| Tilt nose DOWN | ACT Pitch goes NEGATIVE |
| Tilt nose UP | ACT Pitch goes POSITIVE |
| Tilt RIGHT side down | ACT Roll goes POSITIVE |
| Tilt LEFT side down | ACT Roll goes NEGATIVE |

If backwards → change `PITCH_FLIP` or `ROLL_FLIP` to -1.0f.

If sticks move craft wrong direction → change `STICK_PITCH_FLIP` or `STICK_ROLL_FLIP`.
These are separate. IMU flips fix sensor reading. Stick flips fix controller input.

---

## IMU Ground Reference Reset

**At boot:** automatic, takes ~0.75s. Keep craft flat and still during boot.

**In flight session:** hold LABt + RABt for 5 full seconds while craft sits flat.
LED blinks 4× to confirm. Use this if craft drifts even with sticks centered.

**Via serial:** type `imu reset` — recaptures immediately.

---

## PID Symptoms and Fixes

| Symptom | Cause | Fix |
|---|---|---|
| Fast shaking/oscillation | P too high | Lower KP by 0.1 steps |
| Slow rocking, doesn't correct | P too low | Raise KP by 0.1 steps |
| Drifts one direction slowly | I too low | Raise KI by 0.01 steps |
| Overcorrects then wobbles | D too high | Lower KD by 0.05 steps |
| Jerky on small stick inputs | Deadband too small | Raise DEADBAND or EXPO |
| Flips instantly on takeoff | Wrong motor direction | Check spin direction physically |
| Motors buzz loudly in air | D too high | Lower KD |
| Slow to respond to tilt | P too low | Raise KP |

Tune order: P → D → I. Never add I until P and D are stable.
Roll and Pitch usually need the same values on a symmetric frame. Tune Yaw last.

---

## Watchdog Safety (added v1.5)

The ESP32 hardware watchdog resets the board if the main loop freezes for >250ms.
On every reboot, ESC pins are driven LOW before the Servo library starts —
this cuts any latched PWM signal immediately.

SimonK ESCs also have a built-in ~1s signal timeout — if no PWM pulse arrives
for 1 second they cut the motor independently of the flight controller.

Three layers total:
1. Software failsafe — TX disconnect → controlled descent
2. Hardware watchdog — ESP32 freeze → forced reboot → motors cut
3. SimonK timeout — no signal for 1s → ESC cuts motor independently

---

## Telemetry Quick Read

```
ARM:Y  = armed
LNK:Y  = TX connected
THR    = throttle µs (from accumulated stick)
acc    = accumulated throttle value
TGT    = what sticks are commanding (degrees / deg/s)
ACT    = what IMU is actually measuring
PID    = correction output in µs per axis
M:     = final PWM sent to each ESC
BAT    = TX battery %
```

Healthy hover looks like:
- ACT R and P within ±2° of zero
- PID corrections under ±20µs
- All 4 M values within ±30µs of each other

---

## Stick Layout (Mode 2)

```
  LEFT STICK              RIGHT STICK
  ┌──────────┐            ┌──────────┐
  │    ↑     │            │    ↑     │
  │ Throttle │            │  Pitch   │
  │ ← Yaw → │            │ ← Roll → │
  │    ↓     │            │    ↓     │
  └──────────┘            └──────────┘

  LBt              = ARM / DISARM
  RBt              = LED toggle
  LABt + RABt (5s) = IMU ground reference reset
```

Throttle center = hold current altitude (accumulated throttle system).
Push up slowly to climb. Release to hold. Push down to descend.

