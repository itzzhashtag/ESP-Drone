// ================================================================
//  ReceiverModule.h — ESP-NOW Drop-in Receiver
//  URC v2.0 — Universal RC Receiver Module
//  Merged: Broadcast/Unicast filter + ACK reply + Failsafe
//
//  HOW TO USE:
//  ─────────────────────────────────────────────────────────────
//  1. Copy this file into your sketch folder (same folder as .ino)
//  2. At the very top of your .ino add:
//       #include "ReceiverModule.h"
//  3. Inside setup() add:
//       receiver.begin();
//  4. Inside loop() use:
//       receiver.update();           // ← MUST call every loop for
//                                    //   failsafe + connection check
//       if (receiver.available())
//       {
//         // use receiver.data.Lx, receiver.data.Ly, etc.
//       }
//
//  ─────────────────────────────────────────────────────────────
//  MAC FILTERING (optional):
//    receiver.begin("20:E7:C8:9F:47:F8");   // TX prints its MAC at boot
//    Leave empty → accept from ANY sender (open / broadcast mode)
//
//  ACK (optional — enabled by default):
//    The receiver automatically sends a small alive-packet back to
//    every accepted TX so the transmitter knows the link is up.
//    To disable: pass false as second argument:
//       receiver.begin("", false);           // no ACK
//       receiver.begin("AA:BB:CC:DD:EE:FF", false);
//
//  FAILSAFE TIMEOUT:
//    Default 1000 ms. Change before begin():
//       receiver.timeoutMs = 500;
//    When timeout fires: data is zeroed and connected() returns false.
//
//  ─────────────────────────────────────────────────────────────
//  COMPLETE EXAMPLE at bottom of this file ↓
// ================================================================

#pragma once
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

// ================================================================
//  PAYLOAD STRUCT — must be byte-for-byte identical to TX side
//
//  Field reference:
//    Lx, Ly          Left  joystick axes     -99 .. 99
//    Rx, Ry          Right joystick axes     -99 .. 99
//    LABt, RABt      A-buttons (left/right)  true/false
//    LBt,  RBt       B-buttons (left/right)  true/false
//    BAT             Battery percent         0 .. 100
// ================================================================
typedef struct
{
  int  Lx,  Ly;   // Left  joystick X and Y axes  (-99 to +99)
  int  Rx,  Ry;   // Right joystick X and Y axes  (-99 to +99)
  bool LBt, RBt;// A-button: left thumb / right thumb
  bool LABt,  RABt; // B-button: left side  / right side
  int  BAT;       // Transmitter battery level    (0 to 100 %)
} ControllerData;

// ================================================================
//  ACK STRUCT — tiny alive-packet sent back to the transmitter
//  TX can listen for this to confirm the link is up.
//  Keep it small; just one bool is enough.
// ================================================================
typedef struct
{
  bool alive; // always true — just signals the receiver is alive
} AckData;

// ================================================================
//  RECEIVER CLASS
// ================================================================
class ESPNowReceiver
{
public:

  // ── Public data members ───────────────────────────────────────

  ControllerData data = {};   // Latest received packet — read freely in loop()
  uint32_t lastRecvMs  = 0;   // millis() timestamp of the last valid packet
  uint32_t timeoutMs   = 1000;// Failsafe timeout in ms (default 1 second)
                               // Change BEFORE calling begin() if needed

  // connection status flag — true when packets arrive within timeoutMs
  // equivalent to the old "Stat" variable
  bool connected_flag = false;

  // ── begin(allowedMAC, sendAck) ────────────────────────────────
  // Call ONCE in setup().
  //
  //  allowedMAC : transmitter MAC "AA:BB:CC:DD:EE:FF"
  //               "" or omit → accept from ANY sender (broadcast mode)
  //
  //  sendAck    : true  → reply an AckData packet to every accepted TX
  //               false → no reply (saves airtime, slightly faster)
  // ─────────────────────────────────────────────────────────────
  void begin(const char *allowedMAC = "", bool sendAck = true)
  {
    _instance = this;           // store singleton pointer for static callback
    _ackEnabled = sendAck;      // remember whether to send ACK replies

    Serial.println("[RX→INIT] ───────────────────────────────────────");

    // ── Parse MAC filter ────────────────────────────────────────
    // A valid MAC string is exactly 17 chars: "AA:BB:CC:DD:EE:FF"
    _filterMAC = (allowedMAC && strlen(allowedMAC) == 17);

    if (_filterMAC)
    {
      // sscanf parses each hex byte separated by ':'
      sscanf(allowedMAC,
             "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
             &_allowedMAC[0], &_allowedMAC[1], &_allowedMAC[2],
             &_allowedMAC[3], &_allowedMAC[4], &_allowedMAC[5]);

      Serial.printf("[RX→INIT] MAC filter ON  — accepting ONLY from %s\n", allowedMAC);
    }
    else
    {
      Serial.println("[RX→INIT] MAC filter OFF — accepting from ANY transmitter");
    }

    // ── Log ACK mode ────────────────────────────────────────────
    Serial.printf("[RX→INIT] ACK reply: %s\n", _ackEnabled ? "ENABLED" : "disabled");

    // ── WiFi init ───────────────────────────────────────────────
    WiFi.mode(WIFI_STA);  // must be STA mode for ESP-NOW
    delay(400);
    Serial.printf("[RX→INIT] Receiver MAC : %s\n", WiFi.macAddress().c_str());
    Serial.println("[RX→INIT] ↑ Give this MAC to your TX if using UNICAST mode");

    // ── ESP-NOW init ─────────────────────────────────────────────
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("[RX→ERROR] ESP-NOW init FAILED — halting");
      while (1); // hard stop; check power / flash size
    }

    // ── Register receive callback ────────────────────────────────
    // Core 2.x signature: esp_now_register_recv_cb(_onRecv_v2);
    // Core 3.x signature (default below): cast needed to silence warning
    esp_now_register_recv_cb((esp_now_recv_cb_t)_onRecv_v3); // Core 3.x ← default
    // If you get a compile error here, comment the line above and
    // uncomment the Core 2.x line in the private section below.

    Serial.println("[RX→INIT] ESP-NOW ready — waiting for packets");
    Serial.printf("[RX→INIT] Failsafe timeout: %lu ms\n", timeoutMs);
    Serial.println("[RX→INIT] ───────────────────────────────────────");
  }

  // ── available() ──────────────────────────────────────────────
  // Returns true ONCE per new packet, then clears the flag.
  // Use inside loop():  if (receiver.available()) { ... }
  // ─────────────────────────────────────────────────────────────
  bool available()
  {
    if (_newData)
    {
      _newData = false;
      return true;
    }
    return false;
  }

  // ── connected(ms) ────────────────────────────────────────────
  // Returns true if a valid packet arrived within the last `ms` ms.
  // Default uses timeoutMs member (1000 ms).
  // ─────────────────────────────────────────────────────────────
  bool connected(uint32_t ms = 0)
  {
    uint32_t t = (ms > 0) ? ms : timeoutMs;
    return (millis() - lastRecvMs < t);
  }

  // ── update() ─────────────────────────────────────────────────
  // MUST be called every loop iteration.
  // Handles:
  //   • connection_flag update  (equivalent to old checkConnection())
  //   • failsafe data-zero      (equivalent to old checkSignal())
  //   • optional periodic ACK   (removed from here — ACK is per-packet)
  //
  // When signal is lost for > timeoutMs:
  //   • connected_flag → false
  //   • all data fields → 0 (safe/stopped state)
  //   • prints a warning once per loss event
  // ─────────────────────────────────────────────────────────────
  void update()
  {
    bool nowConn = connected();

    if (!nowConn && connected_flag)
    {
      // Transition: just lost signal → zero all data immediately
      Serial.println("[RX] ⚠  SIGNAL LOST — zeroing data (failsafe)");
      memset(&data, 0, sizeof(data)); // zero all joystick/button/pot values
    }

    connected_flag = nowConn; // update public status flag
  }

  private:

  bool     _filterMAC  = false;   // true when a specific TX MAC is set
  uint8_t  _allowedMAC[6] = {};   // parsed allowed TX MAC bytes
  bool     _ackEnabled = true;    // send ACK back to TX on each packet
  volatile bool _newData = false; // set by ISR-like callback; cleared by available()

  // Singleton pointer — lets the static callback reach instance members
  static ESPNowReceiver *_instance;

  // ── Internal: send ACK back to the sender ────────────────────
  // Automatically adds the sender as a peer if not already known,
  // then sends a tiny AckData{alive=true} struct back.
  // ─────────────────────────────────────────────────────────────
  static void _sendAck(const uint8_t *senderMAC)
  {
    // Add sender as peer if not yet registered
    // (required before esp_now_send to that address)
    if (!esp_now_is_peer_exist(senderMAC))
    {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, senderMAC, 6);
      peerInfo.channel = 0;       // 0 = same channel as receiver
      peerInfo.encrypt = false;   // no encryption (keep simple)
      esp_now_add_peer(&peerInfo);
    }

    // Build and send the ACK payload
    AckData ack;
    ack.alive = true; // just a heartbeat flag
    esp_now_send(senderMAC, (uint8_t *)&ack, sizeof(ack));
    // Optional debug: uncomment next line to see every ACK sent
    // Serial.println("[RX→ACK] ACK sent to TX");
  }

  // ── Core 2.x receive callback (comment in if needed) ─────────
  // static void _onRecv_v2(const uint8_t *mac,
  //                        const uint8_t *buf, int len)
  // {
  //   _handlePacket(mac, buf, len);
  // }

  // ── Core 3.x receive callback (default) ──────────────────────
  static void _onRecv_v3(const esp_now_recv_info_t *info,
                          const uint8_t *buf, int len)
  {
    _handlePacket(info->src_addr, buf, len);
  }

  // ── Shared packet handler ─────────────────────────────────────
  // Called by the callback above for every incoming ESP-NOW frame.
  // Performs size check → MAC filter → copy → ACK → debug print.
  // ─────────────────────────────────────────────────────────────
  static void _handlePacket(const uint8_t *mac, const uint8_t *buf, int len)
  {
    if (!_instance) return; // safety: no instance yet

    // ── 1. Size check ────────────────────────────────────────────
    // If the incoming payload doesn't match our struct size it's
    // either noise, an ACK from another device, or a struct mismatch.
    if (len != (int)sizeof(ControllerData))
    {
      Serial.printf("[RX→PKT] Ignored: size mismatch (got %d, expected %d)\n", len, (int)sizeof(ControllerData));
      return;
    }

    // ── 2. MAC filter ────────────────────────────────────────────
    // If MAC filtering is active, drop packets from unknown senders.
    // isAllowed() does a byte-by-byte comparison (same as old code).
    if (_instance->_filterMAC && !_isAllowed(mac))
    {
      Serial.printf("[RX→PKT] Ignored: unknown sender %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return;
    }

    // ── 3. Copy payload into public struct ──────────────────────
    memcpy(&_instance->data, buf, sizeof(ControllerData));

    // ── 4. Update timestamp + signal the main loop ───────────────
    _instance->lastRecvMs = millis(); // record when we last heard from TX
    _instance->_newData   = true;     // flag for available()

    // ── 5. Send ACK back to transmitter (if enabled) ─────────────
    // This lets the TX know the receiver is alive.
    // The old code always did this; here it's optional via _ackEnabled.
    if (_instance->_ackEnabled)
    {
      _sendAck(mac); // mac = src_addr of incoming packet
    }

    // ── 6. Serial debug — printed for every accepted packet ──────
    // Format mirrors the old code's Serial.printf output.
    /*
    Serial.printf("[RX←PKT]MAC : %02X:%02X:%02X:%02X:%02X:%02X || ",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf(" L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | BAT:%3d | Link : %s\n",
      _instance->data.Lx,   _instance->data.Ly, _instance->data.Rx,   _instance->data.Ry,
      _instance->data.LABt, _instance->data.RABt,_instance->data.LBt,  _instance->data.RBt,
      _instance->data.BAT, _instance->connected_flag ? "Connected" : "Dead"); // show status
    */
  }

  // ── isAllowed() ──────────────────────────────────────────────
  // Byte-by-byte MAC comparison — same logic as old code's isAllowed().
  // Returns true only if all 6 bytes match the stored allowed MAC.
  // ─────────────────────────────────────────────────────────────
  static bool _isAllowed(const uint8_t *mac)
  {
    for (int i = 0; i < 6; i++)
      if (mac[i] != _instance->_allowedMAC[i]) return false;
    return true;
  }
};

// ================================================================
//  SINGLETON — one global receiver object.
//  Use receiver.begin(), receiver.update(), receiver.available(),
//  and receiver.data.Lx / .Ly / etc. throughout your sketch.
// ================================================================
ESPNowReceiver  receiver;
ESPNowReceiver *ESPNowReceiver::_instance = nullptr;


// ================================================================
//  USAGE EXAMPLE
//  Copy this block into a new .ino file.
//  Do NOT modify ReceiverModule.h directly for your project logic.
// ================================================================
/*

#include "ReceiverModule.h"

// -- Include your robot / servo / motor libraries here ----------
// #include <Adafruit_PWMServoDriver.h>

void setup()
{
  Serial.begin(115200);

  // ── Option A: Accept packets from ANY transmitter (broadcast) ─
  receiver.begin();

  // ── Option B: Accept ONLY from one specific TX (unicast / safe)
  // Paste your TX MAC below — TX prints it on Serial at boot.
  // receiver.begin("20:E7:C8:9F:47:F8");

  // ── Option C: No ACK reply (lighter link, TX won't get feedback)
  // receiver.begin("", false);
  // receiver.begin("20:E7:C8:9F:47:F8", false);

  // ── Failsafe timeout (optional, default 1000 ms) ─────────────
  // receiver.timeoutMs = 500; // set before begin() if you want <1 s

  // -- Your hardware init goes here ----------------------------
  // pwm.begin(); etc.
}

void loop()
{
  // ── MUST call update() every loop for failsafe + status flag ──
  receiver.update();

  // ── Check for new data ────────────────────────────────────────
  if (receiver.available())
  {
    // ── All available fields from receiver.data: ──────────────

    int  leftX   = receiver.data.Lx;    // Left  joystick X : -99..99
    int  leftY   = receiver.data.Ly;    // Left  joystick Y : -99..99
    int  rightX  = receiver.data.Rx;    // Right joystick X : -99..99
    int  rightY  = receiver.data.Ry;    // Right joystick Y : -99..99
    bool btnLA   = receiver.data.LABt;  // Left  A-button
    bool btnRA   = receiver.data.RABt;  // Right A-button
    bool btnLB   = receiver.data.LBt;   // Left  B-button
    bool btnRB   = receiver.data.RBt;   // Right B-button
    int  battery = receiver.data.BAT;   // TX battery %     :  0..100

    // ── Example: two-motor differential drive ─────────────────
    // int spd = map(leftY, -99, 99, -255, 255);
    // analogWrite(MOTOR_A_PIN, abs(spd));
    // digitalWrite(MOTOR_DIR_PIN, spd > 0 ? HIGH : LOW);

    // ── Example: arm servo from right Y ───────────────────────
    // int angle = map(rightY, -99, 99, 0, 180);
    // myServo.write(angle);
  }

  // ── Handle signal loss (after receiver.update() zeroes data) ──
  if (!receiver.connected())
  {
    // Data is already zeroed by update() — just add your safe-state
    // logic here (e.g. hold position, stop motors, raise flag):
    // Serial.println("TX disconnected — motors stopped");
  }

  // ── Read connection flag at any time ─────────────────────────
  // bool linkOK = receiver.connected_flag;

  // Non-blocking logic here — avoid long delay() calls
  delay(5);
}

*/
// ================================================================