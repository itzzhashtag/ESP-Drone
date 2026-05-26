// ================================================================
//  URC v0.9.3 — ESP-NOW RC Controller  (16×2 LCD Edition)
//  Role    : Master / Transmitter
//  Board   : ESP32 38-pin DevKit
//  Remote 1 : A4:F0:0F:82:6c:a0
//  Hardware:
//    2× Analog Joystick     LX/LY (GPIO34/35), RX/RY (GPIO32/33)
//    2× Joystick Click      LABt (GPIO26), RABt (GPIO27)  INPUT_PULLUP active LOW
//    2× Push Button         LBt  (GPIO25), RBt  (GPIO14)  INPUT_PULLUP active LOW
//    1× Buzzer              GPIO18
//    16×2 I2C LCD           address 0x27
//    2× LED                 RED=GPIO13, GREEN=GPIO12
//    Battery divider        GPIO36  (100kΩ + 47kΩ, 2S LiPo)
//
//  LCD Layout (16 chars × 2 rows):
//  ┌────────────────┐
//  │L-99+99 R+99-99 │  row 0 — left & right stick axes (Lx,Ly,Rx,Ry)
//  │L:0 [d] 99% [d] R:0│  row 1 — LBt, Ldir-arrow, BAT%, Rdir-arrow, RBt
//  └────────────────┘
//  (d) = CGRAM direction arrow character
//
//  Startup caution screen (16×2):
//  ┌────────────────┐
//  │== Caution! ====│
//  │LB:OK RB:OK B:OK│   or FAIL for each
//  └────────────────┘
//
//  Changelog v0.9.2:
//    - Ported entirely to 16×2 LCD (was 20×4)
//    - lcd() init changed to LiquidCrystal_I2C(0x27, 16, 2)
//    - All snprintf buffers capped at 17 chars (16 + null)
//    - Row/col references updated throughout
//    - Startup screen fits 16 columns
//    - CGRAM custom chars retained (slots 0–7, same bitmaps)
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================================================================
//  LCD — 16 columns, 2 rows  ← THIS was the bug in v0.9.1
//  Original code used LiquidCrystal_I2C(0x27, 20, 4) for a 20×4.
//  A 16×2 display with wrong dimensions shows nothing.
// ================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define RED_LED 12
#define GREEN_LED 14
#define BLUE_LED 13
#define BUZZ_PIN 18

// Joystick analog axes — ADC1 pins only (safe when WiFi is active)
// ADC2 pins (GPIO0,2,4,12–15,25–27) are unusable when WiFi is on.
#define LX_PIN 35  // Left  stick X
#define LY_PIN 34  // Left  stick Y
#define RX_PIN 33  // Right stick X
#define RY_PIN 32  // Right stick Y

// Digital inputs — all INPUT_PULLUP, active LOW (pressed = LOW → reads false)
#define L_STICK_PIN 26  // Left  joystick click  (LABt)
#define R_STICK_PIN 27  // Right joystick click  (RABt)
#define L_BTN_PIN 17    // Left  toggle button     (LBt) RX2 pin on some boards
#define R_BTN_PIN 16    // Right toggle button     (RBt) TX2 pin on some boards

// Battery sense — 100kΩ : 47kΩ voltage divider, GPIO36 is input-only
#define BAT_PIN 36

// ================================================================
//  BATTERY CALIBRATION
//  Divider ratio: (100k + 47k) / 47k ≈ 3.128  →  multiply v_adc by that
//  BAT_CAL_FACTOR corrects any real-world component tolerance.
//
//  How to find your factor:
//    1. Measure real battery voltage with a multimeter  (e.g. 8.20 V)
//    2. Open Serial Monitor, check "[BAT]" line voltage (e.g. 7.80 V)
//    3. BAT_CAL_FACTOR = 8.20 / 7.80 ≈ 1.051
// ================================================================
#define FULL_BAT 8.4f           // 100% — 2S Li-Ion fully charged
#define EMPTY_BAT 6.0f          // 0%   — 2S Li-Ion lower cutoff
#define LOW_BAT_THRESHOLD 6.2f  // Safe-mode trigger voltage
#define LOW_BAT_PERCENT 10      // Startup blocks below this %
#define BAT_CAL_FACTOR 1.12f    // Voltage measured in esp / V​oltage in Multimeter

// ================================================================
//  JOYSTICK TUNING
//  DEADZONE         — raw units (0–99 scale) ignored around center.
//                     Prevents stick drift when you're not touching it.
//                     Increase (e.g. 20) if you see unwanted movement.
//  CHANGE_THRESHOLD — axis must move this much before the value updates.
//                     Filters ADC jitter, reduces unnecessary radio packets.
// ================================================================
#define DEADZONE 15
#define CHANGE_THRESHOLD 2

// ================================================================
//  BUZZER
// ================================================================
#define BEEP_FREQ 1200  // Hz
#define BEEP_DUR 40     // ms

// ================================================================
//  RECEIVER MAC
//  Replace with your robot ESP32C3 MAC address.
//  All 0xFF = broadcast (useful on the bench without the robot).
// ================================================================
uint8_t receiverMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//uint8_t receiverMAC[] = { 0x10, 0x00, 0x3B, 0xCC, 0x40, 0xAC };

// ================================================================
//  DATA STRUCT — byte-identical on TX (this file) and RX (robot)
//  If you add/remove/reorder fields, update BOTH sides.
// ================================================================
typedef struct {
  int Lx, Ly;       // Left  stick  -99 … +99
  int Rx, Ry;       // Right stick  -99 … +99
  bool LBt, RBt;    // Push buttons  true = held
  bool LABt, RABt;  // Stick clicks  true = pressed
  int BAT;          // Battery %     0–100
} ControllerData;

typedef struct {
  bool alive;  // Robot sends this as heartbeat ACK
} AckData;

ControllerData data;  // current packet being built
ControllerData prev;  // previous packet (unused beyond struct declaration)

// ================================================================
//  RUNTIME STATE
// ================================================================
int BaT = 99;                   // battery percent (0–100)
float batteryVolt = 8.4f;       // calculated voltage
bool Stat = false;              // true if robot ACK received recently
unsigned long lastAckTime = 0;  // millis() timestamp of last ACK

// ================================================================
//  LCD LINE CACHE
//  Stores what was last written to each row.
//  printIfChanged() skips the I2C write if the string hasn't changed,
//  preventing flicker and reducing bus traffic.
// ================================================================
String line0 = "";
String line1 = "";

// ================================================================
//  CUSTOM LCD CHARACTERS (CGRAM slots 0–7)
//  Each byte is a row of 5 pixels (bits 4→0).
//  Slots:  0=↗  1=↑  2=↓  3=←  4=→  5=↙  6=↘  7=↖
//  These fit in a single character cell (5×8 pixels).
// ================================================================
byte UR_c[8] = { B00000, B00000, B01111, B00011, B00101, B01001, B10000, B00000 };  // ↗
byte U_c[8] = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100 };   // ↑
byte D_c[8] = { B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100 };   // ↓
byte L_c[8] = { B00000, B00010, B00100, B01000, B11111, B01000, B00100, B00010 };   // ←
byte R_c[8] = { B00000, B01000, B00100, B00010, B11111, B00010, B00100, B01000 };   // →
byte DL_c[8] = { B00000, B00000, B00001, B10010, B10100, B11000, B11110, B00000 };  // ↙
byte DR_c[8] = { B00000, B00000, B10000, B01001, B00101, B00011, B01111, B00000 };  // ↘
byte UL_c[8] = { B00000, B00000, B11110, B11000, B10100, B10010, B00001, B00000 };  // ↖

// ================================================================
//  HELPERS
// ================================================================

// One short beep
void beep() {
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

// Map raw ADC (0–4095) to -99…+99 with deadzone applied.
// analogRead() returns 0 at one extreme and 4095 at the other;
// map() flips it so pushing forward gives a positive Ly.
int mapAxis(int val) {
  int mapped = map(val, 0, 4095, 99, -99);
  if (abs(mapped) < DEADZONE) return 0;  // center dead zone → zero
  return mapped;
}

// Format axis value for display:
//   0 → "  0"   +50 → "+50"   -99 → "-99"
// We want exactly 3 chars for alignment.
String fmt(int v) {
  char buf[5];
  if (v == 0) sprintf(buf, "  0");
  else sprintf(buf, "%+3d", v);  // %+3d always prints sign
  return String(buf);
}

// Write text to LCD only if it differs from the cached value.
// col/row = cursor position, text = new string, cache = last written value.
void printIfChanged(int col, int row, String text, String &cache) {
  if (text != cache) {
    lcd.setCursor(col, row);
    lcd.print(text);
    cache = text;
  }
}

// Return CGRAM slot (0–7) for the direction of a joystick.
// Returns 255 if the stick is within the deadzone (= centered).
// Diagonal is detected when BOTH axes exceed the deadzone.
byte getDirIcon(int x, int y) {
  bool xPos = (x > DEADZONE);
  bool xNeg = (x < -DEADZONE);
  bool yPos = (y > DEADZONE);
  bool yNeg = (y < -DEADZONE);

  if (yPos && xPos) return 0;  // ↗
  if (yPos && xNeg) return 7;  // ↖
  if (yNeg && xPos) return 6;  // ↘
  if (yNeg && xNeg) return 5;  // ↙
  if (yPos) return 1;          // ↑
  if (yNeg) return 2;          // ↓
  if (xNeg) return 3;          // ←
  if (xPos) return 4;          // →
  return 255;                  // centered
}

// ================================================================
//  BEEP POLICY
//  Push buttons (LBt, RBt)  → beep on press AND release  (toggle feel)
//  Stick clicks (LABt,RABt) → beep on press only          (click feel)
//  Edge detection via static booleans that remember last state.
// ================================================================
void checkBeeps() {
  static bool pLBt = false, pRBt = false;
  static bool pLABt = false, pRABt = false;

  if (data.LBt && !pLBt) beep();    // LBt just pressed
  if (!data.LBt && pLBt) beep();    // LBt just released
  if (data.RBt && !pRBt) beep();    // RBt just pressed
  if (!data.RBt && pRBt) beep();    // RBt just released
  if (data.LABt && !pLABt) beep();  // LABt just pressed
  if (data.RABt && !pRABt) beep();  // RABt just pressed

  // Save states for next call
  pLBt = data.LBt;
  pRBt = data.RBt;
  pLABt = data.LABt;
  pRABt = data.RABt;
}

// ================================================================
//  ESP-NOW CALLBACK — runs in WiFi task context when ACK arrives
//  We only accept packets whose size matches AckData exactly.
// ================================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckData)) {
    AckData ack;
    memcpy(&ack, incomingData, sizeof(ack));
    if (ack.alive) {
      lastAckTime = millis();
      Stat = true;
    }
  }
}

// ================================================================
//  DataWork — read inputs, build packet, transmit
//  Called every loop tick; no blocking code inside.
// ================================================================
void DataWork() {
  // --- Joystick axes ---
  int newLx = mapAxis(analogRead(LX_PIN));
  int newLy = mapAxis(analogRead(LY_PIN));
  int newRx = mapAxis(analogRead(RX_PIN));
  int newRy = mapAxis(analogRead(RY_PIN));

  // Apply CHANGE_THRESHOLD: only commit new value if the axis moved enough.
  // This prevents tiny ADC fluctuations from constantly updating the packet.
  if (abs(newLx - data.Lx) > CHANGE_THRESHOLD) data.Lx = newLx;
  if (abs(newLy - data.Ly) > CHANGE_THRESHOLD) data.Ly = newLy;
  if (abs(newRx - data.Rx) > CHANGE_THRESHOLD) data.Rx = newRx;
  if (abs(newRy - data.Ry) > CHANGE_THRESHOLD) data.Ry = newRy;

  // --- Buttons (INPUT_PULLUP: LOW when pressed → invert with !) ---
  data.LABt = !digitalRead(L_STICK_PIN);
  data.RABt = !digitalRead(R_STICK_PIN);
  data.LBt = !digitalRead(L_BTN_PIN);
  data.RBt = !digitalRead(R_BTN_PIN);
  data.BAT = BaT;

  checkBeeps();  // edge detection — must run after data is updated

  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  Serial.printf("L(%+3d,%+3d) R(%+3d,%+3d) | LBt:%d RBt:%d LABt:%d RABt:%d | %.2fV %d%% | %s\n",
                data.Lx, data.Ly, data.Rx, data.Ry,
                (int)data.LBt, (int)data.RBt, (int)data.LABt, (int)data.RABt,
                batteryVolt, BaT,
                Stat ? "CONNECTED" : "NO LINK");
}

// ================================================================
//  updateDisplay — refresh the 16×2 LCD
//
//  Row 0 layout (16 chars):
//    "L-99+99 R+99-99"
//     ^      ^
//     Lx+Ly  Rx+Ry  (each axis is 3 chars, sign included via fmt())
//
//  Row 1 layout (16 chars):
//    "L:0 [d] 99% [d] R:0"  — wait, that's 20 chars.
//    For 16 columns we need to trim.  Final layout:
//    "L:0[d]99%[d]R:0 "
//     0123456789012345
//     L:0  = cols 0-2   (LBt)
//     [d]  = col  3     (left arrow, 1 CGRAM char)
//     99%  = cols 4-6   (battery, 3 chars)
//     [d]  = col  7     (right arrow, 1 CGRAM char)
//     R:0  = cols 8-10  (RBt)
//     (space pad to 16) = cols 11-15
//
//  Direction arrow: if centered → prints '+', else prints CGRAM char.
// ================================================================
void updateDisplay() {
  // ── Row 0: axis values ──────────────────────────────────────────
  // fmt() returns exactly 3 chars each (e.g. "+99", "  0", "-12")
  // "L" + Lx(3) + Ly(3) + "  R" + Rx(3) + Ry(3) = 1+3+3+2+3+3 = 15 chars + 1 trailing space = 16
  char l0[17];
  snprintf(l0, 17, "L%s%s  R%s%s ",
           fmt(data.Lx).c_str(),
           fmt(data.Ly).c_str(),
           fmt(data.Rx).c_str(),
           fmt(data.Ry).c_str());

  printIfChanged(0, 0, String(l0), line0);

  // ── Row 1: L:<LBt> <iconL>  <bat%2d>% <iconR> R:<RBt> ──────────
  // Col layout (16 cols total):
  // 0  1  2   3   4      5  6   7  8   9   10  11     12  13 14  15
  // L  :  X   _   iconL  _  _   X  X   %   _   iconR  _   R  :   X
  //
  // Battery is capped 0-99, always 2 chars with leading space if <10.

  // Build comparison string (CGRAM glyphs replaced with printable placeholders)
  char l1cmp[17];
  byte iconL = getDirIcon(data.Lx, data.Ly);
  byte iconR = getDirIcon(data.Rx, data.Ry);
  snprintf(l1cmp, 17, "L:%d %c  %2d%% %c R:%d",
           (int)data.LBt,
           (iconL == 255) ? '+' : '?',  // placeholder for CGRAM glyph
           BaT,
           (iconR == 255) ? '+' : '?',
           (int)data.RBt);

  if (String(l1cmp) != line1) {
    line1 = String(l1cmp);

    // col 0-2: "L:X"
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print((int)data.LBt);

    // col 3: space
    lcd.setCursor(3, 1);
    lcd.print(' ');

    // col 4: left direction icon
    lcd.setCursor(4, 1);
    if (iconL == 255) lcd.print('+');
    else lcd.write(iconL);

    // cols 5-6: two spaces
    lcd.setCursor(5, 1);
    lcd.print("  ");

    // cols 7-8: battery percent, always 2 chars (" 4" or "85")
    lcd.setCursor(7, 1);
    char batStr[3];
    snprintf(batStr, 3, "%2d", BaT);
    lcd.print(batStr);

    // col 9: '%'
    // col 10: space
    lcd.setCursor(9, 1);
    lcd.print("% ");

    // col 11: right direction icon
    lcd.setCursor(11, 1);
    if (iconR == 255) lcd.print('+');
    else lcd.write(iconR);

    // col 12: space
    // cols 13-14: "R:"
    // col 15: RBt value
    lcd.setCursor(12, 1);
    lcd.print(" R:");
    lcd.print((int)data.RBt);
  }
}

// ================================================================
//  updateConnectionLED
//  GREEN = robot sent an ACK within the last 500 ms → link is live
//  RED   = no ACK recently → link lost or robot is off
// ================================================================
void updateConnectionLED() {
  if (millis() - lastAckTime < 500) {
    Stat = true;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  } else {
    Stat = false;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
  }
}

// ================================================================
//  BatteryManager — non-blocking battery read + low-bat safe mode
//
//  Normal operation:
//    Reads BAT_PIN every 5 seconds, calculates voltage and percent,
//    stores in global BaT and batteryVolt.
//
//  Low battery safe mode (batteryVolt ≤ LOW_BAT_THRESHOLD):
//    1. Sends a zeroed packet to kill the robot drive.
//    2. Blinks the red LED.
//    3. Overwrites the LCD with a LOW BAT warning.
//    4. Returns true so the caller knows we're in fault state.
//
//  Returns: true if safe mode is active, false otherwise.
// ================================================================
bool BatteryManager() {
  static unsigned long lastRead = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static bool killSent = false;

  // Read battery every 5 seconds (millis()-based, non-blocking)
  if (millis() - lastRead > 5000) {
    lastRead = millis();

    int raw = analogRead(BAT_PIN);
    float v_adc = raw * (3.3f / 4095.0f);

    // Reverse the voltage divider: V_bat = V_adc × (R1 + R2) / R2
    // R1 = 100kΩ, R2 = 47kΩ  →  factor = 147/47 ≈ 3.128
    batteryVolt = v_adc * ((100.0f + 47.0f) / 47.0f) * BAT_CAL_FACTOR;
    // Clamp to 0–100%
    if (batteryVolt >= FULL_BAT) BaT = 99;
    else if (batteryVolt <= EMPTY_BAT) BaT = 0;
    else BaT = (int)((batteryVolt - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));
    if (BaT > 99) BaT = 99;
    if (BaT < 0) BaT = 0;
    Serial.printf("[BAT] %.2fV  %d%%\n", batteryVolt, BaT);
  }

  // ── Low battery safe mode ───────────────────────────────────────
  if (batteryVolt <= LOW_BAT_THRESHOLD) {
    if (!killSent) {
      memset(&data, 0, sizeof(data));  // zero all axes and buttons
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      Serial.printf("[BAT] SAFE MODE %.2fV %d%% — radio KILLED\n", batteryVolt, BaT);
      killSent = true;
    }

    // Blink red LED at ~1.7 Hz (300 ms half-period)
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
    }
    digitalWrite(GREEN_LED, LOW);

    // 16×2 LOW BAT screen — each row is exactly 16 chars
    lcd.setCursor(0, 0);
    lcd.print(" |  LOW BATT  | ");
    lcd.setCursor(0, 1);
    lcd.print(" | Radio OFF! | ");

    return true;  // caller should not do normal work
  }

  killSent = false;  // reset so we re-send kill if battery dips again
  return false;
}

// ================================================================
//  SystemControl — wraps BatteryManager; handles recovery restart
//  If battery was low and has now recovered, re-run the startup
//  safety gate so the operator confirms before driving again.
// ================================================================
void SystemControl() {
  static bool wasLow = false;
  bool isLow = BatteryManager();

  if (isLow) {
    wasLow = true;
    // Spin here (still calling BatteryManager) until voltage recovers
    while (BatteryManager()) {
      delay(50);
    }
  }

  if (wasLow) {
    wasLow = false;
    startup();           // re-run safety gate after recovery
    line0 = line1 = "";  // invalidate LCD cache so it fully redraws
  }
}

// ================================================================
//  startup — blocking safety check before the main loop starts
//
//  Blocks until ALL of these are satisfied:
//    • LBt  = not pressed  (INPUT_PULLUP HIGH = safe)
//    • RBt  = not pressed
//    • Battery ≥ LOW_BAT_PERCENT %
//
//  16×2 screen while waiting:
//  ┌────────────────┐
//  │== Caution! ====│   row 0  (fixed)
//  │LB:OK RB:OK B:OK│   row 1  ("OK" or "FAIL" for each condition)
//  └────────────────┘
//
//  When all clear:
//  ┌────────────────┐
//  │== Caution! ====│
//  │   ALL CLEAR    │
//  └────────────────┘
//  Holds for 800 ms then returns.
// ================================================================
void startup() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("=== Caution! ===");  // exactly 16 chars
  Serial.println("[STARTUP] Safety check — release LBt, RBt and check battery");

  while (true) {
    // Read battery (no calibration factor here — just a rough startup check)
    int rawBat = analogRead(BAT_PIN);
    float v_adc = rawBat * (3.3f / 4095.0f);
    //float batV   = v_adc * ((100.0f + 47.0f) / 47.0f);   // no CAL_FACTOR intentional
    float batV = v_adc * ((100.0f + 47.0f) / 47.0f) * BAT_CAL_FACTOR;
    int batPct = 0;
    if (batV >= FULL_BAT) batPct = 100;
    else if (batV <= EMPTY_BAT) batPct = 0;
    else batPct = (int)((batV - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    // digitalRead HIGH (not pressed on pullup) = safe
    bool lbtSafe = digitalRead(L_BTN_PIN);  // HIGH → button released = OK
    bool rbtSafe = digitalRead(R_BTN_PIN);
    bool batSafe = (batPct >= LOW_BAT_PERCENT);

    if (lbtSafe && rbtSafe && batSafe) {
      // ── ALL CLEAR ──
      lcd.setCursor(0, 1);
      lcd.print("   ALL CLEAR    ");  // 16 chars
      Serial.println("[STARTUP] All clear — proceeding");
      delay(800);
      lcd.clear();
      return;
    }

    // ── Still waiting — show which checks are failing ──
    // Row 1: "LB:OK RB:FAIL   "  — or "LB:OK RB:OK B:LO" etc.
    // We have 16 columns.  Fit: "LB:" + 2 + " RB:" + 2 + " B:" + 2 = 15 chars + 1 space.
    // Use "OK" (2 chars) or "NO" (2 chars) to keep alignment.
    char row1[17];
    snprintf(row1, 17, "LB:%s RB:%s B:%s",
             lbtSafe ? "OK" : "NO",
             rbtSafe ? "OK" : "NO",
             batSafe ? "OK" : "LOW");  // "LO" = low battery
    lcd.setCursor(0, 1);
    lcd.print(row1);

    delay(150);  // poll at ~6 Hz while waiting
  }
}

// ================================================================
//  SETUP — runs once at power-on
// ================================================================
void setup() {
  Serial.begin(115200);

  Serial.println("\n==============================");
  Serial.println("  URC TX v0.9.2 — Booting...");
  Serial.println("==============================");
  WiFi.mode(WIFI_STA);
  delay(200);  // Print MAC address
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  // ── GPIO ──
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);  // red on during boot
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN, LOW);

  // Joystick click and push button inputs with internal pull-ups.
  // Active LOW: button pressed → pin reads LOW → !LOW = true.
  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN, INPUT_PULLUP);
  pinMode(R_BTN_PIN, INPUT_PULLUP);
  pinMode(BAT_PIN, INPUT);  // GPIO36 is input-only, no mode needed

  // Full 0–3.3 V range for all ADC channels.
  // ADC_11db allows reading up to 3.3 V (default is 0–1 V which would clip battery sense).
  analogSetAttenuation(ADC_11db);

  // ── LCD init ──
  lcd.init();       // initialise I2C and the HD44780 controller
  lcd.backlight();  // turn on backlight (off by default after init)

  // Load custom direction arrow characters into CGRAM slots 0–7.
  // CGRAM survives lcd.clear() but not power cycling.
  lcd.createChar(0, UR_c);  // ↗
  lcd.createChar(1, U_c);   // ↑
  lcd.createChar(2, D_c);   // ↓
  lcd.createChar(3, L_c);   // ←
  lcd.createChar(4, R_c);   // →
  lcd.createChar(5, DL_c);  // ↙
  lcd.createChar(6, DR_c);  // ↘
  lcd.createChar(7, UL_c);  // ↖
  lcd.home();

  // ── Startup safety gate (blocks until buttons released + battery OK) ──
  startup();

  // ── Boot splash while radio initialises ──
  lcd.setCursor(0, 0);
  lcd.print("  Radio Booting ");  // 16 chars
  lcd.setCursor(0, 1);
  lcd.print(" ESP-NOW v0.9.2 ");

  // Two-tone ascending beep — signals successful startup
  tone(BUZZ_PIN, 1000, 80);
  delay(130);
  tone(BUZZ_PIN, 1400, 80);
  delay(350);

  // ── WiFi + ESP-NOW init ──
  WiFi.mode(WIFI_STA);  // Station mode required for ESP-NOW; AP mode not used
  Serial.print("[INIT] TX MAC: ");
  Serial.println(WiFi.macAddress());
  delay(100);

  if (esp_now_init() != ESP_OK) {
    // Fatal: blink red forever so the operator knows
    Serial.println("[ERROR] ESP-NOW init FAILED");
    while (1) {
      digitalWrite(RED_LED, !digitalRead(RED_LED));
      delay(200);
    }
  }
  Serial.println("[INIT] ESP-NOW OK");

  esp_now_register_recv_cb(OnDataRecv);  // register ACK handler

  // Add robot as a peer — without this esp_now_send() will fail
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;      // 0 = same channel as WiFi
  peerInfo.encrypt = false;  // no encryption (faster, simpler)
  esp_now_add_peer(&peerInfo);

  Serial.println("[INIT] Peer added — entering main loop");
  lcd.clear();  // wipe boot splash before main loop starts
}

// ================================================================
//  MAIN LOOP — target ~50 Hz  (20 ms/tick)
//  No blocking code here — everything is non-blocking except
//  BatteryManager's internal while() in safe-mode (via SystemControl).
// ================================================================
void loop() {
  SystemControl();        // 1. battery watchdog — safe mode if low
  DataWork();             // 2. read sticks/buttons, beep edges, send packet
  updateDisplay();        // 3. refresh LCD (cached writes, minimal flicker)
  updateConnectionLED();  // 4. green/red LED based on ACK timeout
  delay(20);              // 5. ~50 Hz pacing — reduce to 10 for faster response
}