# Q450 ESP32 Flight Controller — Wiring & Setup Guide
### Firmware v1.6 | Last updated: May 2026

---

## Hardware Build

| Component | Spec |
|---|---|
| Frame | Q450 (X-Config, 450mm) |
| Motors | A2212/13T 1000KV × 4 |
| ESCs | SimonK 30A × 4 (Standard PWM 1000–2000µs) |
| Flight Controller | ESP32 Dev Board |
| IMU | MPU-6050 on GY-521 breakout |
| Battery | 3S LiPo (11.1V nominal, 12.6V max) |
| Radio | ESP-NOW (custom TX/RX via ReceiverModule.h) |

---

## Required Libraries

Install via Arduino Library Manager:

| Library | Author | Purpose |
|---|---|---|
| `ESP32Servo` | Kevin Harrington | PWM servo/ESC signals on ESP32 |
| `MPU6050_light` | rfetick | Lightweight MPU-6050 driver |
| `esp_task_wdt` | ESP32 Arduino core (built-in) | Hardware watchdog |

---

## Pin Reference

| ESP32 GPIO | Connected To | Notes |
|---|---|---|
| GPIO 25 | ESC M1 Signal | Front-Left motor — CW |
| GPIO 26 | ESC M2 Signal | Front-Right motor — CCW |
| GPIO 27 | ESC M3 Signal | Back-Right motor — CW |
| GPIO 14 | ESC M4 Signal | Back-Left motor — CCW |
| GPIO 21 | MPU-6050 SDA | I2C Data |
| GPIO 22 | MPU-6050 SCL | I2C Clock |
| GPIO 2 | Built-in LED | Toggled via RBt button |
| 3.3V | MPU-6050 VCC | Do NOT power MPU from 5V |
| GND | Common ground | All grounds must share |

---

## Motor Layout — X Config (viewed from above)

```
              FRONT
         M1 (CW)  M2 (CCW)
            ↻  \  /  ↺
                \/
                /\
            ↺  /  \  ↻
         M4 (CCW)  M3 (CW)
              BACK
```

| Motor | Position | Rotation | GPIO | Prop |
|---|---|---|---|---|
| M1 | Front-Left | Clockwise | 25 | R (reversed) |
| M2 | Front-Right | Counter-Clockwise | 26 | L (standard) |
| M3 | Back-Right | Clockwise | 27 | R (reversed) |
| M4 | Back-Left | Counter-Clockwise | 14 | L (standard) |

> To reverse a motor spin direction: swap any two of its three bullet connectors.
> Never fix motor direction in software — always fix it physically.

---

## Full Wiring Diagram

```
                    ┌──────────────────────────────┐
                    │         LIPO BATTERY          │
                    │      3S  11.1V / 12.6V max    │
                    └────┬─────┬────────────────────┘
                         │ +   │ –              
                         ▼     ▼           
                       ┌──────────┐        
                       │   PDB    │      
                       │ + XT60 - │ 
                       └──┬─┬─┬─┬─┘ 
                          │ │ │ │ 
                ┌─────────┘ │ │ └────────┐  
                │    ┌──────┘ └──────┐   │ 
                ▼    ▼               ▼   ▼  
             [ESC1][ESC2]         [ESC3][ESC4] 
               │      │              │    │  
GND ───────────+──────+──────────────+────+───────┐
               │      │              │    │       │
5V ────────────└──────└──────────────└────└─────┐ │                                         
                                                │ │
┌─────────────┐                                 │ │
│   ESP32     │                                 │ │
│             │                                 │ │
│ 3V3 ──────► MPU VCC                           │ │
│ GND ──────► MPU GND                           │ │
│ GPIO21 ───► MPU SDA                           │ │
│ GPIO22 ───► MPU SCL                           │ │
│             │                                 │ │
│ GPIO25 ───► ESC M1 Signal                     │ │
│ GPIO26 ───► ESC M2 Signal                     │ │
│ GPIO27 ───► ESC M3 Signal                     │ │
│ GPIO14 ───► ESC M4 Signal                     │ │
│             │                                 │ │
│ VIN ──────────────────────────────────────────┘ │
│ GND ────────────────────────────────────────────┘
└─────────────┘


ESC Signal Plug (3-wire JST/servo):
  RED    (5V BEC) → ESP32 Vin (one ESC only, rest leave unconnected)
  BLACK  (GND)    → ESP32 GND ← connect ALL four ESC GNDs
  YELLOW (Signal) → ESP32 GPIO (25 / 26 / 27 / 14)
```

---

## Capacitors — Required for Stability

These suppress ESC switching noise that causes IMU angle drift and ESP32 resets.

### At the PDB (battery leads)

| Cap | Value | Voltage | Placement |
|---|---|---|---|
| Electrolytic | 2200µF (or 4700µF) | 25V min | Across + and − pads at XT60, as close as possible |
| Ceramic (optional) | 100nF | 25V | Same pads, in parallel with above |

### On the ESP32

| Cap | Value | Placement |
|---|---|---|
| Ceramic | 100nF (0.1µF) | 3.3V pin → GND pin |
| Electrolytic | 10µF | 3.3V pin → GND pin (+ to 3.3V) |
| Electrolytic | 10µF | EN pin → GND pin (reset filter, + to EN) |

### On the MPU-6050 (GY-521)

| Cap | Value | Placement |
|---|---|---|
| Ceramic | 100nF | VCC → GND, as close to chip as possible |
| Ceramic or electrolytic | 1µF or 10µF | VCC → GND in parallel (+ to VCC if electrolytic) |

> All caps on the MPU must be physically as close to the black MPU chip as the board allows.
> Distance reduces effectiveness significantly.

---

## MPU-6050 Mounting

- Mount flat and centered on the frame.
- Use foam double-sided tape underneath to dampen motor vibration.
- Keep I2C wires (SDA/SCL) twisted together and routed away from ESC signal wires.
- The GY-521 module already has 4.7kΩ pull-up resistors on SDA and SCL.

---

## Step-by-Step First Boot

### Step 1 — IMU Calibration (once per build)
1. Set `#define CALIBRATE_IMU 1` in the tuning section.
2. Place craft completely flat and motionless on a hard level surface.
3. Flash and open Serial Monitor at 115200 baud.
4. Copy the 6 offset values that print.
5. Paste into `IMU_OFF_AX` through `IMU_OFF_GZ` in the tuning section.
6. Set `CALIBRATE_IMU` back to `0` and reflash.

### Step 2 — ESC Calibration (once per ESC set)
1. Remove all props.
2. Open Serial Monitor.
3. Type `escal` and press Enter.
4. Follow the printed instructions — ESCs will beep to confirm.
5. This saves to ESC flash and never needs repeating unless ESCs are replaced.

### Step 3 — Test Mode Bench Check
1. Set `#define TEST_MODE 1`.
2. Flash and open Serial Monitor.
3. Verify: `MPU-6050 OK`, LED blinks 3× at boot.
4. Type `orient` — tilt craft and confirm pitch/roll directions.
5. Type `mpu` — verify angles read near zero on flat surface.
6. Toggle arm switch — confirm `ARMED` and `DISARMED` print.
7. Kill TX — confirm `SIGNAL LOST` prints once.

### Step 4 — Motor Direction Check (props off, TEST_MODE 0)
1. Set `TEST_MODE 0` and flash.
2. Type each command and verify spin direction (viewed from above):
   ```
   m1 1300   → Front-Left  must spin CLOCKWISE
   m2 1300   → Front-Right must spin COUNTER-CLOCKWISE
   m3 1300   → Back-Right  must spin CLOCKWISE
   m4 1300   → Back-Left   must spin COUNTER-CLOCKWISE
   stop
   ```
3. Wrong direction → swap any two bullet connectors on that motor.

### Step 5 — First Hover (props on, outdoors)
1. Place craft on flat hard ground.
2. Power on — wait for 3 LED blinks (boot complete, ~2s).
3. Type `imu reset` in serial OR let boot capture run.
4. Arm via LBt.
5. Push left stick up very slowly — craft should lift smoothly.
6. Keep first flights under 20cm altitude until PID is confirmed stable.

---

## Serial Monitor Commands

Open at 115200 baud:

| Command | Action |
|---|---|
| `m1 1300` | Spin M1 to 1300µs |
| `m2 1300` | Spin M2 to 1300µs |
| `m3 1300` | Spin M3 to 1300µs |
| `m4 1300` | Spin M4 to 1300µs |
| `all 1200` | All motors to 1200µs |
| `stop` | Cut all motors immediately |
| `escal` | One-time ESC range calibration |
| `mpu` | 50 live IMU angle readings |
| `orient` | 5s orientation axis check |
| `imu reset` | Re-capture ground reference now |
| `arm` | Force arm (test mode) |
| `disarm` | Force disarm |
| `thr` | Show current accumulated throttle |
| `trim` | Show per-motor trim values |
| `status` | Print all current settings |

---

## Telemetry Legend

```
[FC] ARM:Y LNK:Y | THR:1380 acc:1380 | TGT R: +0.0 P: +0.0 Y: +0.0 | ACT R: +0.3 P:-0.5 YR:+0.1 | PID R:+0.7 P:-1.2 Y:+0.1 | M:1381 1379 1381 1379 | BAT: 88%
```

| Field | Meaning |
|---|---|
| `ARM:Y/N` | Armed state |
| `LNK:Y/N` | TX signal present |
| `THR` | Current throttle µs from accumulator |
| `acc` | Accumulated throttle value (same as THR in normal flight) |
| `TGT R/P/Y` | Target roll/pitch/yawrate commanded by sticks |
| `ACT R/P/YR` | Actual roll/pitch/yawrate from IMU |
| `PID R/P/Y` | PID correction per axis in µs |
| `M:` | Final PWM output to each motor (µs) |
| `BAT` | TX battery percentage |

---

## Failsafe Behaviour

When TX signal is lost for more than 1000ms:
1. `signalLost` goes true — single warning print, no spam.
2. Throttle descends at `DESCENT_RATE` µs per loop tick (default 3µs).
3. PID remains active — craft holds level while descending.
4. Floor is `ESC_IDLE` — motors keep spinning, craft lands softly.
5. Signal restored → `accThrottle` syncs to failsafe level, normal control resumes.

---

## Stick Layout (Mode 2)

```
  LEFT STICK                RIGHT STICK
  ┌──────────┐              ┌──────────┐
  │    ↑     │              │    ↑     │
  │ Throttle │              │  Pitch   │
  │ ← Yaw → │              │ ← Roll → │
  │    ↓     │              │    ↓     │
  └──────────┘              └──────────┘

  LBt  = ARM (ON) / DISARM (OFF)
  RBt  = LED toggle
  LABt + RABt held 5s = IMU ground reference reset
```

> Throttle uses accumulated system — center stick HOLDS current altitude.
> Push up to climb, push down to descend, release to hold.

