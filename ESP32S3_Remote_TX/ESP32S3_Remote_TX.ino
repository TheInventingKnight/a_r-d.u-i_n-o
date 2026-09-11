/*
 * ESP32-S3 N16R8 Remote Controller - COMPLETE PRODUCTION FIRMWARE (Transmitter)
 * ==============================================================================
 * Dedicated handheld remote with 1.3" OLED display for the PTZ Camera Mount.
 *
 * Hardware Architecture:
 * ------------------------------------------------------------------------------
 *   - MCU: ESP32-S3 N16R8 (16MB Flash, 8MB OPI PSRAM)
 *   - Display: 1.3" 128x64 OLED (Adafruit SH110X / SH1106G) via I2C at 400 kHz
 *       SDA: GPIO 8
 *       SCL: GPIO 9
 *   - Battery & Power Sensing:
 *       PIN_BATT_ADC  : GPIO 1 (100k/100k divider, calibration factor 2.23f)
 *       PIN_USB_SENSE : GPIO 2 (USB 5V presence detection ADC)
 *   - Left Joystick (Pan / Tilt):
 *       VRx (Pan)     : GPIO 4 (ADC1_CH3)
 *       VRy (Tilt)    : GPIO 5 (ADC1_CH4)
 *       Click (SW)    : GPIO 15 (INPUT_PULLUP)
 *   - Right Joystick (Zoom & Aux):
 *       VRx (Aux)     : GPIO 6 (ADC1_CH5)
 *       VRy (Zoom)    : GPIO 7 (ADC1_CH6: Forward=Tele/In, Back=Wide/Out)
 *       Click (SW)    : GPIO 16 (INPUT_PULLUP)
 *   - Rotary Encoder (HW-787AB Quadrature):
 *       Phase A (TRA) : GPIO 10 (Interrupt, any edge)
 *       Phase B (TRB) : GPIO 11 (Interrupt, any edge)
 *       Push (PSH)    : GPIO 12 (INPUT_PULLUP) -> Menu Select / Open
 *   - Menu & Navigation Pushbuttons:
 *       Confirm (CON) : GPIO 13 (INPUT_PULLUP) -> Toggle Main UI / Mount Diag
 *       Back (BAK)    : GPIO 17 (INPUT_PULLUP) -> Return to Main UI
 *   - 5 Color Preset Pushbuttons:
 *       Color 1..5    : GPIO 38, 39, 40, 41, 42 (INPUT_PULLUP)
 *                       Short Click: Recall Preset 1..5
 *                       Long Press (>1.5s): Save Preset 1..5
 *
 * Required Libraries (Arduino Library Manager):
 *   - Adafruit SH110X (by Adafruit)
 *   - Adafruit GFX Library (by Adafruit)
 *
 * Board Settings (Arduino IDE):
 *   - Board: ESP32S3 Dev Module
 *   - Flash Size: 16MB (128Mb)
 *   - PSRAM: "OPI PSRAM"
 *   - USB CDC On Boot: "Enabled"
 *   - Upload Speed: 921600
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// Forward declarations
struct Button;
struct PtzPacket;
struct MountTelemetryPacket;

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define PIN_BATT_ADC      1   // Battery voltage divider (100k/100k)
#define PIN_USB_SENSE     2   // USB 5V presence sense (ADC)

#define PIN_LJOY_X        4   // Left joystick VRx  -> PAN
#define PIN_LJOY_Y        5   // Left joystick VRy  -> TILT
#define PIN_LJOY_SW      15   // Left joystick click (active LOW)

#define PIN_RJOY_X        6   // Right joystick VRx -> AUX
#define PIN_RJOY_Y        7   // Right joystick VRy -> ZOOM (Forward=Tele, Back=Wide)
#define PIN_RJOY_SW      16   // Right joystick click (active LOW)

#define PIN_I2C_SDA       8   // OLED I2C SDA
#define PIN_I2C_SCL       9   // OLED I2C SCL

#define PIN_ENC_A        10   // HW-787AB encoder phase A (TRA)
#define PIN_ENC_B        11   // HW-787AB encoder phase B (TRB)
#define PIN_ENC_BTN      12   // Encoder push (PSH, active LOW) -> menu open/select
#define PIN_BTN_CON      13   // Confirm button (active LOW) -> toggle mount diag
#define PIN_BTN_BAK      17   // Back button    (active LOW) -> return to main

// 5 Color preset action buttons (active LOW)
static const uint8_t PIN_COLOR_BTNS[5] = {38, 39, 40, 41, 42};

// ---------------------------------------------------------------------------
// Display Configuration (Adafruit SH110X 1.3" OLED)
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDR      0x3C
#define I2C_CLOCK   400000UL        // Fast I2C: 400 kHz
#define STATUS_BAR_H     11

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayActive = false;

// ---------------------------------------------------------------------------
// Battery Configuration (5000 mAh Li-Po / 18650 Pack)
// ---------------------------------------------------------------------------
const int   BATTERY_CAPACITY_MAH    = 5000;
const float BATT_CALIBRATION_FACTOR = 2.23f;
const float BATT_V_MIN              = 3.2f;   // 0%
const float BATT_V_MAX              = 4.2f;   // 100%
const float TP4056_CHARGE_MA        = 1000.0f;// 1A charge rate
const int   USB_SENSE_THRESHOLD     = 1000;   // raw ADC threshold for 5V USB
#define ADC_SAMPLES                 8

// ---------------------------------------------------------------------------
// ESP-NOW Packet Protocol (MATCH WITH MOUNT RX)
// ---------------------------------------------------------------------------
#define PKT_PAIR_REQ      0x01
#define PKT_PAIR_ACK      0x02
#define PKT_CONTROL       0x03
#define PKT_SAVE_PRESET   0x04
#define PKT_GOTO_PRESET   0x05
#define PKT_START_HOMING  0x06
#define PKT_TELEMETRY     0x07

struct __attribute__((packed)) PtzPacket {
  uint8_t  type;                  // PKT_*
  uint8_t  mac[6];                // sender MAC (pairing handshake)
  int16_t  pan;                   // -1000..+1000 velocity command
  int16_t  tilt;                  // -1000..+1000 velocity command
  int16_t  zoom;                  // -1000..+1000 zoom command (Right Stick Y: + Tele, - Wide)
  int16_t  auxX;                  // right stick X (reserved)
  uint16_t buttons;               // BTNBIT_* bitfield
  uint8_t  presetId;              // Preset slot (1..8) for SAVE/GOTO
  uint8_t  seq;                   // rolling sequence number
};

struct __attribute__((packed)) MountTelemetryPacket {
  uint8_t  type;                  // PKT_TELEMETRY (0x07)
  uint8_t  mac[6];                // Mount MAC
  uint8_t  panUartOk;             // 1 if Pan TMC2209 version == 0x21, else 0
  uint8_t  panVersion;            // IC version register value (e.g. 0x21)
  uint8_t  tiltUartOk;            // 1 if Tilt TMC2209 version == 0x21, else 0
  uint8_t  tiltVersion;           // IC version register value (e.g. 0x21)
  uint8_t  panHallDetected;       // 1 if magnet detected (LOW on pin), 0 if clear
  uint8_t  tiltHallDetected;      // 1 if magnet detected (LOW on pin), 0 if clear
  int8_t   panSide;               // -1 = Left (CCW), 0 = Center, +1 = Right (CW)
  int8_t   tiltSide;              // -1 = Down, 0 = Center, +1 = Up
  int32_t  panSteps;              // Current pan position in steps
  int32_t  tiltSteps;             // Current tilt position in steps
  float    panDeg;                // Calculated pan angle in degrees (-180.0 .. +180.0)
  float    tiltDeg;               // Calculated tilt angle in degrees (-90.0 .. +90.0)
  uint16_t zoomMs;                // Current zoom dead-reckoning position (0..3500 ms)
  uint8_t  zoomPct;               // Zoom percentage (0..100%)
  uint8_t  isHomed;               // 1 if homing complete
  uint8_t  state;                 // 0=IDLE, 1=LIVE_MOVING, 2=HOMING, 3=GOTO_PRESET
  uint8_t  activePreset;          // Target preset ID if moving
  char     eventMsg[32];          // Null-terminated event string
};

enum ButtonBits : uint16_t {
  BTNBIT_LJOY      = 1 << 0,
  BTNBIT_RJOY      = 1 << 1,
  BTNBIT_PSH       = 1 << 2,
  BTNBIT_CON       = 1 << 3,
  BTNBIT_BAK       = 1 << 4,
  BTNBIT_C1        = 1 << 5,
  BTNBIT_C2        = 1 << 6,
  BTNBIT_C3        = 1 << 7,
  BTNBIT_C4        = 1 << 8,
  BTNBIT_C5        = 1 << 9,
  BTNBIT_ZOOM_SLOW = 1 << 10,
};

// ---------------------------------------------------------------------------
// UI & Menu Model
// ---------------------------------------------------------------------------
// UI & Menu Model (Professional Remote OS)
// ---------------------------------------------------------------------------
enum UiMode   : uint8_t { UI_LIVE = 0, UI_PRESETS = 1, UI_LOGS = 2, UI_DIAG = 3, UI_MENU = 4 };
enum MenuItem : uint8_t {
  MI_PRESETS = 0,
  MI_LOGS,
  MI_DIAG,
  MI_HOME,
  MI_BRIGHT,
  MI_SENS,
  MI_INV_PAN,
  MI_INV_TILT,
  MI_EXIT,
  MI_COUNT
};

const char* MENU_NAMES[MI_COUNT] = {
  "Preset Hub", "System Logs", "Mount Diag", "Home Mount",
  "Brightness", "Sensitivity", "Invert Pan", "Invert Tilt", "Exit Menu"
};

// ---------------------------------------------------------------------------
// Remote Preset Cache & OS Notification Banners
// ---------------------------------------------------------------------------
struct PresetSummary {
  bool    valid;
  float   panDeg;
  float   tiltDeg;
  uint8_t zoomPct;
};
PresetSummary remotePresets[8] = {};
uint8_t activePresetSlot  = 1; // Currently selected active slot (1..8)
uint8_t presetHubSelected = 1; // Highlighted slot inside Preset Hub (1..8)

char     bannerText[28]  = "";
uint32_t bannerExpireMs  = 0;

void showBanner(const char* txt, uint32_t ms = 2500) {
  strncpy(bannerText, txt, sizeof(bannerText) - 1);
  bannerText[sizeof(bannerText) - 1] = '\0';
  bannerExpireMs = millis() + ms;
}

// Dedicated System Log Ring Buffer
#define MAX_LOG_LINES 15
#define LOG_LINE_LEN  26
char    logLines[MAX_LOG_LINES][LOG_LINE_LEN];
uint8_t logHead   = 0;
uint8_t logTotal  = 0;
int8_t  logScroll = 0; // 0 = newest at bottom, positive = scrolled up

void addLog(const char* msg) {
  if (!msg || strlen(msg) == 0) return;
  uint8_t prev = (logHead == 0) ? (MAX_LOG_LINES - 1) : (logHead - 1);
  if (logTotal > 0 && strcmp(logLines[prev], msg) == 0) return; // ignore duplicates

  strncpy(logLines[logHead], msg, LOG_LINE_LEN - 1);
  logLines[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (logHead + 1) % MAX_LOG_LINES;
  if (logTotal < MAX_LOG_LINES) logTotal++;
  logScroll = 0;
}

// ---------------------------------------------------------------------------
// ESP-NOW Link State
// ---------------------------------------------------------------------------
#define TX_INTERVAL_MS    20      // 50 Hz control stream
#define LINK_FAIL_LIMIT   100     // consecutive TX failures -> re-scan (~2s)

enum LinkState : uint8_t { LINK_SCAN = 0, LINK_PAIRED };

LinkState             linkState    = LINK_SCAN;
uint8_t               mountMac[6]  = {0};
uint8_t               mountChannel = 0;
uint8_t               scanChannel  = 1;
uint8_t               activeChannel = 1;
uint32_t              nextScanMs   = 0;
uint32_t              lastTxMs     = 0;
uint8_t               txSeq        = 0;
volatile bool         pairAckFlag  = false;
volatile uint8_t      pairAckMac[6];
volatile uint32_t     txFailCount  = 0;

static const uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Telemetry State from Mount
MountTelemetryPacket latestMountTelem = {};
bool     hasMountTelem        = false;
uint32_t lastMountTelemMs     = 0;
char     lastLoggedEvent[32]  = "";

// ---------------------------------------------------------------------------
// Persistent Settings (NVS)
// ---------------------------------------------------------------------------
struct Settings {
  uint8_t brightness;             // SH110X contrast 0..255
  float   sensitivity;            // joystick output scaler 0.2..3.0
  bool    invertPan;
  bool    invertTilt;
  bool    zoomSlowMode;           // true = Slow Creep Zoom, false = Fast Standard Zoom
};
Settings settings = {128, 1.0f, false, false, false};

Preferences prefs;

void loadRemotePresets() {
  prefs.begin("rem_p", true);
  for (uint8_t i = 0; i < 8; i++) {
    char k[16];
    snprintf(k, sizeof(k), "p%u_v", i);
    remotePresets[i].valid = prefs.getBool(k, false);
    if (remotePresets[i].valid) {
      snprintf(k, sizeof(k), "p%u_p", i); remotePresets[i].panDeg = prefs.getFloat(k, 0.0f);
      snprintf(k, sizeof(k), "p%u_t", i); remotePresets[i].tiltDeg = prefs.getFloat(k, 0.0f);
      snprintf(k, sizeof(k), "p%u_z", i); remotePresets[i].zoomPct = prefs.getUChar(k, 0);
    }
  }
  prefs.end();
}

void loadSettings() {
  prefs.begin("ptz", true);
  settings.brightness   = prefs.getUChar("bright", 128);
  settings.sensitivity  = prefs.getFloat("sens", 1.0f);
  settings.invertPan    = prefs.getBool("invP", false);
  settings.invertTilt   = prefs.getBool("invT", false);
  settings.zoomSlowMode = prefs.getBool("zmSlow", false);
  prefs.end();
}

void saveSettings() {
  prefs.begin("ptz", false);
  prefs.putUChar("bright", settings.brightness);
  prefs.putFloat("sens", settings.sensitivity);
  prefs.putBool("invP", settings.invertPan);
  prefs.putBool("invT", settings.invertTilt);
  prefs.putBool("zmSlow", settings.zoomSlowMode);
  prefs.end();
}

void saveLink() {
  prefs.begin("ptzlink", false);
  prefs.putUChar("ch", mountChannel);
  prefs.putBytes("mac", mountMac, 6);
  prefs.end();
}

uint8_t loadLinkChannel() {
  prefs.begin("ptzlink", true);
  uint8_t ch = prefs.getUChar("ch", 1);
  prefs.end();
  return (ch >= 1 && ch <= 13) ? ch : 1;
}

// ---------------------------------------------------------------------------
// ESP-NOW Callbacks & Peer Handling
// ---------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* senderMac = info->src_addr;
#else
void onEspNowRecv(const uint8_t* senderMac, const uint8_t* data, int len) {
#endif
  if (len == (int)sizeof(PtzPacket)) {
    const PtzPacket* pkt = (const PtzPacket*)data;
    if (pkt->type == PKT_PAIR_ACK) {
      memcpy((void*)pairAckMac, senderMac, 6);
      pairAckFlag = true;
    }
  } else if (len == (int)sizeof(MountTelemetryPacket)) {
    const MountTelemetryPacket* telem = (const MountTelemetryPacket*)data;
    if (telem->type == PKT_TELEMETRY) {
      memcpy(&latestMountTelem, telem, sizeof(MountTelemetryPacket));
      hasMountTelem = true;
      lastMountTelemMs = millis();

      if (strlen(telem->eventMsg) > 0 && strcmp(telem->eventMsg, lastLoggedEvent) != 0) {
        strncpy(lastLoggedEvent, telem->eventMsg, sizeof(lastLoggedEvent) - 1);
        lastLoggedEvent[sizeof(lastLoggedEvent) - 1] = '\0';
        Serial.printf("\n>>> [MOUNT EVENT] %s <<<\n\n", telem->eventMsg);

        // Show banner notification and add to dedicated log console
        showBanner(telem->eventMsg, 3000);
        addLog(telem->eventMsg);
      }
    }
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowSent(const wifi_tx_info_t*, esp_now_send_status_t status) {
#else
void onEspNowSent(const uint8_t*, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_FAIL) {
    txFailCount++;
  } else {
    txFailCount = 0;
  }
}

bool espNowAddPeer(const uint8_t* mac, uint8_t channel) {
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = channel;         // 0 = follow current STA home channel!
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ---------------------------------------------------------------------------
// Rotary Encoder (Quadrature Interrupt Driven)
// ---------------------------------------------------------------------------
volatile int16_t encSteps  = 0;
volatile uint8_t encLastAB = 0;

static const int8_t ENC_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encoderISR() {
  uint8_t ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encSteps += ENC_TABLE[(encLastAB << 2) | ab];
  encLastAB = ab;
}

int encoderDetents() {
  static int16_t accum = 0;
  noInterrupts();
  int16_t steps = encSteps;
  encSteps = 0;
  interrupts();
  accum += steps;
  int detents = accum / 4;
  accum -= detents * 4;
  return detents;
}

// ---------------------------------------------------------------------------
// Button Debounce with Long-Press Detection
// ---------------------------------------------------------------------------
struct Button {
  uint8_t  pin;
  bool     stable;
  bool     lastRaw;
  uint32_t lastChangeMs;
  uint32_t pressStartMs;
  bool     longPressHandled;
};

void buttonInit(Button &b, uint8_t pin) {
  b.pin = pin; b.stable = false; b.lastRaw = false;
  b.lastChangeMs = 0; b.pressStartMs = 0; b.longPressHandled = false;
  pinMode(pin, INPUT_PULLUP);
}

bool buttonPressed(Button &b) {
  bool raw = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChangeMs = now; }
  if ((now - b.lastChangeMs) > 25 && raw != b.stable) {
    b.stable = raw;
    if (b.stable) {
      b.pressStartMs = now;
      b.longPressHandled = false;
      return true;
    }
  }
  return false;
}

bool buttonLongPressed(Button &b, uint32_t holdMs = 1500) {
  if (b.stable && !b.longPressHandled && (millis() - b.pressStartMs >= holdMs)) {
    b.longPressHandled = true;
    return true;
  }
  return false;
}

Button btnLJoy, btnRJoy, btnEnc, btnCon, btnBak, btnColor[5];

uint16_t buildButtonBits() {
  uint16_t b = 0;
  if (digitalRead(PIN_LJOY_SW)  == LOW) b |= BTNBIT_LJOY;
  if (digitalRead(PIN_RJOY_SW)  == LOW) b |= BTNBIT_RJOY;
  if (digitalRead(PIN_ENC_BTN)  == LOW) b |= BTNBIT_PSH;
  if (digitalRead(PIN_BTN_CON)  == LOW) b |= BTNBIT_CON;
  if (digitalRead(PIN_BTN_BAK)  == LOW) b |= BTNBIT_BAK;
  for (uint8_t i = 0; i < 5; i++)
    if (digitalRead(PIN_COLOR_BTNS[i]) == LOW) b |= (BTNBIT_C1 << i);
  if (settings.zoomSlowMode) b |= BTNBIT_ZOOM_SLOW;
  return b;
}

// ---------------------------------------------------------------------------
// Analog Inputs & Dynamic Calibration
// ---------------------------------------------------------------------------
uint16_t analogReadAvg(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) acc += analogRead(pin);
  return acc / ADC_SAMPLES;
}

int joyCenterLX = 2048, joyCenterLY = 2048;
int joyCenterRX = 2048, joyCenterRY = 2048;

int* joyCenterFor(uint8_t pin) {
  switch (pin) {
    case PIN_LJOY_X: return &joyCenterLX;
    case PIN_LJOY_Y: return &joyCenterLY;
    case PIN_RJOY_X: return &joyCenterRX;
    default:         return &joyCenterRY;
  }
}

void calibrateJoysticks() {
  const uint8_t pins[4] = {PIN_LJOY_X, PIN_LJOY_Y, PIN_RJOY_X, PIN_RJOY_Y};
  for (uint8_t pass = 0; pass < 16; pass++)
    for (uint8_t i = 0; i < 4; i++)
      *joyCenterFor(pins[i]) = analogReadAvg(pins[i]);
}

float joyAxis(uint8_t pin) {
  int   raw    = analogReadAvg(pin);
  int   center = *joyCenterFor(pin);
  float delta  = (float)(raw - center);
  float v = (delta >= 0) ? delta / (4095.0f - center) : delta / (float)center;
  if (fabsf(v) < 0.12f) return 0.0f;
  return constrain(v, -1.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Battery & Power Monitoring
// ---------------------------------------------------------------------------
float readBatteryVoltage() {
  return (analogReadMilliVolts(PIN_BATT_ADC) * BATT_CALIBRATION_FACTOR) / 1000.0f;
}

float filteredBatteryVoltage() {
  static float filtered = 0.0f;
  static bool  primed   = false;
  float raw = readBatteryVoltage();
  if (!primed) { filtered = raw; primed = true; }
  filtered += (raw - filtered) * 0.05f;
  return filtered;
}

int batteryPercent(float volts) {
  static int lastPct = -1;
  float pct = (volts - BATT_V_MIN) / (BATT_V_MAX - BATT_V_MIN) * 100.0f;
  int currentPct = constrain((int)(pct + 0.5f), 0, 100);
  if (lastPct == -1) lastPct = currentPct;
  if (abs(currentPct - lastPct) >= 2) lastPct = currentPct;
  return lastPct;
}

bool usbConnected() {
  return analogReadAvg(PIN_USB_SENSE) > USB_SENSE_THRESHOLD;
}

uint16_t chargeEtaMinutes(int pct) {
  float remainingMah = BATTERY_CAPACITY_MAH * (100 - pct) / 100.0f;
  return (uint16_t)((remainingMah / TP4056_CHARGE_MA) * 60.0f * 1.3f);
}

// ---------------------------------------------------------------------------
// ESP-NOW Link Management
// ---------------------------------------------------------------------------
void updateLink(uint32_t now) {
  if (linkState == LINK_PAIRED) {
    if (txFailCount > LINK_FAIL_LIMIT) {
      linkState = LINK_SCAN;
      txFailCount = 0;
      esp_now_del_peer(mountMac);
      Serial.println("[LINK] Link lost, re-scanning channels...");
    }
    return;
  }

  // Handle successful ACK from Mount
  if (pairAckFlag) {
    pairAckFlag = false;
    memcpy(mountMac, (const void*)pairAckMac, 6);
    mountChannel = activeChannel;
    if (espNowAddPeer(mountMac, 0)) { // 0 = follow current channel!
      linkState = LINK_PAIRED;
      txFailCount = 0;
      saveLink();
      Serial.printf("[PAIR SUCCESS] Connected to Mount MAC: %02X:%02X:%02X:%02X:%02X:%02X on Channel %u!\n",
                    mountMac[0], mountMac[1], mountMac[2], mountMac[3], mountMac[4], mountMac[5], mountChannel);
    }
    return;
  }

  // Scan channels broadcasting PAIR_REQ
  if (now >= nextScanMs) {
    nextScanMs = now + 150;
    activeChannel = scanChannel;
    esp_wifi_set_channel(scanChannel, WIFI_SECOND_CHAN_NONE);
    PtzPacket req = {};
    req.type = PKT_PAIR_REQ;
    WiFi.macAddress(req.mac);
    esp_now_send(BCAST_MAC, (uint8_t*)&req, sizeof(req));
    scanChannel = (scanChannel >= 13) ? 1 : scanChannel + 1;
  }
}

void sendControl(int16_t pan, int16_t tilt, int16_t zoom, int16_t auxX, uint16_t buttons) {
  PtzPacket pkt = {};
  pkt.type = PKT_CONTROL;
  WiFi.macAddress(pkt.mac);
  pkt.pan = pan;
  pkt.tilt = tilt;
  pkt.zoom = zoom;
  pkt.auxX = auxX;
  pkt.buttons = buttons;
  pkt.seq = txSeq++;
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));
}

void commandSavePreset(uint8_t id) {
  if (id < 1 || id > 8) return;
  PtzPacket pkt = {};
  pkt.type = PKT_SAVE_PRESET;
  pkt.presetId = id;
  WiFi.macAddress(pkt.mac);
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));

  // Save to local cache & NVS
  uint8_t idx = id - 1;
  remotePresets[idx].valid = true;
  remotePresets[idx].panDeg = latestMountTelem.panDeg;
  remotePresets[idx].tiltDeg = latestMountTelem.tiltDeg;
  remotePresets[idx].zoomPct = latestMountTelem.zoomPct;

  prefs.begin("rem_p", false);
  char k[16];
  snprintf(k, sizeof(k), "p%u_v", idx); prefs.putBool(k, true);
  snprintf(k, sizeof(k), "p%u_p", idx); prefs.putFloat(k, remotePresets[idx].panDeg);
  snprintf(k, sizeof(k), "p%u_t", idx); prefs.putFloat(k, remotePresets[idx].tiltDeg);
  snprintf(k, sizeof(k), "p%u_z", idx); prefs.putUChar(k, remotePresets[idx].zoomPct);
  prefs.end();

  char b[28];
  snprintf(b, sizeof(b), "P%u SAVED!", id);
  showBanner(b, 2500);
  addLog(b);
  activePresetSlot = id;
  Serial.printf("\n>>> [PRESET] Saved Preset %u to Mount & Remote! <<<\n\n", id);
}

void commandGotoPreset(uint8_t id) {
  if (id < 1 || id > 8) return;
  PtzPacket pkt = {};
  pkt.type = PKT_GOTO_PRESET;
  pkt.presetId = id;
  WiFi.macAddress(pkt.mac);
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));

  char b[28];
  snprintf(b, sizeof(b), "RECALLING P%u...", id);
  showBanner(b, 2500);
  addLog(b);
  activePresetSlot = id;
  Serial.printf("\n>>> [RECALL] Sent GOTO Preset %u to Mount! <<<\n\n", id);
}

void commandStartHoming() {
  PtzPacket pkt = {};
  pkt.type = PKT_START_HOMING;
  WiFi.macAddress(pkt.mac);
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));
  Serial.println("\n>>> [HOMING] Sent START HOMING command to Mount! <<<\n");

  showBanner("HOMING MOUNT...", 4000);
  addLog("Homing Mount...");
}

// ---------------------------------------------------------------------------
// Settings Menu & UI Navigation
// ---------------------------------------------------------------------------
UiMode   uiMode      = UI_LIVE;
MenuItem menuSel     = MI_PRESETS;
bool     menuEditing = false;

void handleMenuInput(int detents) {
  bool pshPress = buttonPressed(btnEnc);
  bool pshHold  = buttonLongPressed(btnEnc, 1200);
  bool bakPress = buttonPressed(btnBak);
  bool conPress = buttonPressed(btnCon);

  // CON button: Toggle Preset Hub from anywhere
  if (conPress) {
    if (uiMode == UI_PRESETS) {
      uiMode = UI_LIVE;
    } else {
      uiMode = UI_PRESETS;
      presetHubSelected = activePresetSlot;
    }
    return;
  }

  // Live Screen
  if (uiMode == UI_LIVE) {
    if (pshPress) {
      uiMode = UI_MENU;
      menuEditing = false;
    }
    return;
  }

  // Preset Hub Screen
  if (uiMode == UI_PRESETS) {
    if (bakPress) {
      uiMode = UI_LIVE;
      return;
    }
    // Long press encoder (>1.2s) to save current framing into selected slot
    if (pshHold) {
      commandSavePreset(presetHubSelected);
      return;
    }
    // Click encoder to recall selected slot
    if (pshPress) {
      commandGotoPreset(presetHubSelected);
      uiMode = UI_LIVE;
      return;
    }
    // Encoder scroll through slots 1..8
    if (detents != 0) {
      int s = (int)presetHubSelected + detents;
      while (s < 1) s += 8;
      while (s > 8) s -= 8;
      presetHubSelected = (uint8_t)s;
    }
    return;
  }

  // System Logs Screen (Scrollable Console)
  if (uiMode == UI_LOGS) {
    if (bakPress || pshPress) {
      uiMode = UI_MENU;
      return;
    }
    if (detents != 0) {
      int maxScroll = (logTotal > 5) ? (logTotal - 5) : 0;
      logScroll = constrain(logScroll - detents, 0, maxScroll);
    }
    return;
  }

  // Mount Diag Screen
  if (uiMode == UI_DIAG) {
    if (bakPress || pshPress) {
      uiMode = UI_MENU;
      return;
    }
    return;
  }

  // Settings Menu
  if (uiMode == UI_MENU) {
    if (bakPress) {
      if (menuEditing) {
        menuEditing = false;
        saveSettings();
      } else {
        uiMode = UI_LIVE;
      }
      return;
    }

    if (pshPress) {
      if (menuEditing) {
        menuEditing = false;
        saveSettings();
      } else {
        switch (menuSel) {
          case MI_PRESETS:
            uiMode = UI_PRESETS;
            presetHubSelected = activePresetSlot;
            break;
          case MI_LOGS:
            uiMode = UI_LOGS;
            logScroll = 0;
            break;
          case MI_DIAG:
            uiMode = UI_DIAG;
            break;
          case MI_HOME:
            commandStartHoming();
            uiMode = UI_LIVE;
            break;
          case MI_INV_PAN:
            settings.invertPan = !settings.invertPan;
            saveSettings();
            break;
          case MI_INV_TILT:
            settings.invertTilt = !settings.invertTilt;
            saveSettings();
            break;
          case MI_EXIT:
            uiMode = UI_LIVE;
            break;
          default:
            menuEditing = true;
            break;
        }
      }
      return;
    }

    if (detents == 0) return;
    if (!menuEditing) {
      int s = (int)menuSel + detents;
      while (s < 0) s += MI_COUNT;
      menuSel = (MenuItem)(s % MI_COUNT);
    } else if (menuSel == MI_BRIGHT) {
      int b = (int)settings.brightness + detents * 15;
      settings.brightness = (uint8_t)constrain(b, 0, 255);
      display.setContrast(settings.brightness);
    } else if (menuSel == MI_SENS) {
      settings.sensitivity = constrain(settings.sensitivity + detents * 0.1f, 0.2f, 3.0f);
    }
  }
}

void menuValueStr(MenuItem item, char* buf, size_t n) {
  switch (item) {
    case MI_BRIGHT:   snprintf(buf, n, "%u", settings.brightness); break;
    case MI_SENS:     snprintf(buf, n, "x%.1f", settings.sensitivity); break;
    case MI_INV_PAN:  snprintf(buf, n, "%s", settings.invertPan  ? "ON" : "OFF"); break;
    case MI_INV_TILT: snprintf(buf, n, "%s", settings.invertTilt ? "ON" : "OFF"); break;
    default:          buf[0] = '\0'; break;
  }
}

// ---------------------------------------------------------------------------
// OLED Display Rendering
// ---------------------------------------------------------------------------
// Fullscreen Animated Radar / Satellite Search Screen when unlinked
void drawSearchingScreen() {
  uint32_t now = millis();
  display.clearDisplay();

  int16_t cx = 64;
  int16_t cy = 22;

  // Expanding pulsing radar waves
  uint8_t phase = (now / 35) % 24; // 0..23
  for (uint8_t r = phase; r < 24; r += 8) {
    if (r > 2) display.drawCircle(cx, cy, r, SH110X_WHITE);
  }

  // Central transmitter dot
  display.fillCircle(cx, cy, 3, SH110X_WHITE);

  // Rotating scanner sweep beam
  float angle = (float)((now / 12) % 360) * 0.0174533f; // radians
  int16_t bx = cx + (int16_t)(cosf(angle) * 20);
  int16_t by = cy + (int16_t)(sinf(angle) * 20);
  display.drawLine(cx, cy, bx, by, SH110X_WHITE);

  // Status text
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Animated title
  const char* title = "SEARCHING MOUNT";
  int16_t tx = (SCREEN_WIDTH - (int16_t)(strlen(title) * 6)) / 2;
  display.setCursor(tx - 6, 44);
  display.print(title);

  // Animated dots ...
  uint8_t numDots = (now / 300) % 4;
  for (uint8_t d = 0; d < numDots; d++) display.print(".");

  // Subtitle
  char sub[32];
  snprintf(sub, sizeof(sub), "Scan Ch %u  |  21dBm Max", scanChannel);
  int16_t sx = (SCREEN_WIDTH - (int16_t)(strlen(sub) * 6)) / 2;
  if (sx < 0) sx = 0;
  display.setCursor(sx, 55);
  display.print(sub);
}

void drawStatusBar() {
  float volts = filteredBatteryVoltage();
  int   pct   = batteryPercent(volts);
  bool  usb   = usbConnected();

  display.setTextSize(1);

  if (usb) {
    // Lightning bolt icon + charge ETA
    display.fillTriangle(2, 0, 7, 0, 4, 5, SH110X_WHITE);
    display.fillTriangle(4, 4, 8, 4, 3, 10, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(11, 1);
    display.printf("ETA:%um", chargeEtaMinutes(pct));
  } else {
    // Battery gauge outline + filled proportional bars
    display.drawRect(0, 1, 15, 8, SH110X_WHITE);
    display.fillRect(15, 3, 2, 4, SH110X_WHITE);
    int fillW = (13 * pct) / 100;
    if (fillW > 0) display.fillRect(1, 2, fillW, 6, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(19, 1);
    display.printf("%d%%", pct);
  }

  // Active Preset badge in center of status bar
  display.setTextColor(SH110X_WHITE);
  display.setCursor(42, 1);
  display.printf("[PRESET %u]", activePresetSlot);

  // Link status
  display.setCursor(92, 1);
  if (linkState == LINK_PAIRED) display.printf("LK:C%u", mountChannel);
  else                          display.printf("SCN%u", scanChannel);

  display.drawFastHLine(0, STATUS_BAR_H, SCREEN_WIDTH, SH110X_WHITE);
}

void drawDial(int16_t cx, int16_t cy, int16_t r, float x, float y, const char* label) {
  display.drawCircle(cx, cy, r, SH110X_WHITE);
  display.drawFastHLine(cx - r + 2, cy, 2 * r - 3, SH110X_WHITE);
  display.drawFastVLine(cx, cy - r + 2, 2 * r - 3, SH110X_WHITE);
  int16_t dx = cx + (int16_t)(x * (r - 4));
  int16_t dy = cy + (int16_t)(y * (r - 4));
  display.fillCircle(dx, dy, 2, SH110X_WHITE);
  int16_t tx = cx - (int16_t)(strlen(label) * 3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(tx, cy + r + 2);
  display.print(label);
}

void drawMainUi(int16_t pan, int16_t tilt, int16_t zoom, int16_t auxX) {
  // Left Dial: Pan / Tilt
  drawDial(26, 31, 12, pan / 1000.0f, tilt / 1000.0f, "PAN/TILT");

  // Center live readouts
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(44, 18);
  display.printf("P:%+4.0f\xF7", latestMountTelem.panDeg);
  display.setCursor(44, 28);
  display.printf("T:%+4.0f\xF7", latestMountTelem.tiltDeg);
  display.setCursor(44, 38);
  display.printf("Z:%3u%%", latestMountTelem.zoomPct);

  // Right Dial: Zoom (Y-axis: Up=Tele, Down=Wide) / Aux
  drawDial(102, 31, 12, auxX / 1000.0f, -zoom / 1000.0f, "ZOOM/AUX");

  // Bottom Area: High-Contrast Popup Banner or Contextual Prompt
  if (millis() < bannerExpireMs && strlen(bannerText) > 0) {
    int16_t textLen = strlen(bannerText);
    int16_t boxW = min(SCREEN_WIDTH, (textLen * 6) + 12);
    int16_t boxX = (SCREEN_WIDTH - boxW) / 2;
    display.fillRect(boxX, 53, boxW, 11, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(boxX + 6, 55);
    display.print(bannerText);
  } else {
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 55);
    display.print("CON:PRESETS  ENC:MENU");
  }
}

// Preset Hub: 8 slots list view with page scrolling, instant recall, and hold-to-save
void drawPresetsHub() {
  display.setTextSize(1);

  // Title bar
  display.setTextColor(SH110X_WHITE);
  display.setCursor(2, 13);
  display.print("PRESETS (PSH:Go Hld:Save)");

  // 4 items visible per page
  uint8_t startIdx = ((presetHubSelected - 1) / 4) * 4;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t slotIdx = startIdx + i;
    if (slotIdx >= 8) break;

    uint8_t slotNum = slotIdx + 1;
    int16_t y = 24 + (i * 10);
    bool isSelected = (slotNum == presetHubSelected);

    if (isSelected) {
      display.fillRect(0, y - 1, SCREEN_WIDTH, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }

    display.setCursor(2, y);
    display.printf("P%u:", slotNum);

    if (remotePresets[slotIdx].valid) {
      display.setCursor(24, y);
      display.printf("%+4.0f\xF7 %+3.0f\xF7  Zm:%2u%%",
                     remotePresets[slotIdx].panDeg,
                     remotePresets[slotIdx].tiltDeg,
                     remotePresets[slotIdx].zoomPct);
    } else {
      display.setCursor(36, y);
      display.print("[ EMPTY SLOT ]");
    }
  }
}

// Dedicated Scrolling System Log Console
void drawLogs() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(2, 13);
  display.print("SYSTEM LOGS (Scroll)");

  if (logTotal == 0) {
    display.setCursor(14, 34);
    display.print("No logged events yet");
    return;
  }

  // Display up to 5 lines from circular log buffer with logScroll
  uint8_t visibleLines = min((uint8_t)5, logTotal);
  for (uint8_t i = 0; i < visibleLines; i++) {
    int lineIdx = (int)logHead - 1 - (int)logScroll - (int)(visibleLines - 1 - i);
    while (lineIdx < 0) lineIdx += MAX_LOG_LINES;
    lineIdx = lineIdx % MAX_LOG_LINES;

    int16_t y = 23 + (i * 8);
    display.setCursor(2, y);
    display.print(logLines[lineIdx]);
  }
}

void drawMountDiag() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Line 1: Header / Link
  display.setCursor(2, 13);
  if (linkState != LINK_PAIRED || !hasMountTelem) {
    display.print("SEARCHING MOUNT...");
    display.setCursor(2, 26);
    display.printf("Scan Wi-Fi Ch: %u", scanChannel);
    display.setCursor(2, 38);
    display.print("Waiting for Mount RX");
    display.setCursor(2, 53);
    display.print("BAK: Exit to Menu");
    return;
  }

  // Line 2: UART Health (authentic TMC2209 silicon 0x21 check)
  display.printf("UART: P:%s  T:%s",
                 latestMountTelem.panUartOk ? "OK" : "ERR",
                 latestMountTelem.tiltUartOk ? "OK" : "ERR");

  // Line 3: Hall Effect Sensors
  display.setCursor(2, 23);
  display.printf("Hall: P:%s T:%s",
                 latestMountTelem.panHallDetected ? "DETECT" : "CLEAR",
                 latestMountTelem.tiltHallDetected ? "DETECT" : "CLEAR");

  // Line 4: Pan Position & Direction away from Hall
  const char* pDir = (latestMountTelem.panHallDetected || abs(latestMountTelem.panSteps) < 50)
                     ? "CTR" : (latestMountTelem.panSteps > 0 ? "RIGHT" : "LEFT");
  display.setCursor(2, 33);
  display.printf("Pan : %+5.1f\xF7 (%s)", latestMountTelem.panDeg, pDir);

  // Line 5: Tilt Position & Direction away from Hall
  const char* tDir = (latestMountTelem.tiltHallDetected || abs(latestMountTelem.tiltSteps) < 25)
                     ? "CTR" : (latestMountTelem.tiltSteps > 0 ? "UP" : "DOWN");
  display.setCursor(2, 43);
  display.printf("Tilt: %+5.1f\xF7 (%s)", latestMountTelem.tiltDeg, tDir);

  // Line 6: Zoom Dead-Reckoning & Exit prompt
  display.setCursor(2, 53);
  display.printf("Zm:%u%% [BAK:Exit]", latestMountTelem.zoomPct);
}

void drawMenu() {
  display.drawRect(2, 13, 124, 50, SH110X_WHITE);
  char val[8];

  // Up to 5 visible items with scrolling window
  uint8_t windowStart = 0;
  if ((uint8_t)menuSel >= 4) {
    windowStart = min((uint8_t)(menuSel - 3), (uint8_t)(MI_COUNT - 5));
  }

  for (uint8_t row = 0; row < 5; row++) {
    uint8_t i = windowStart + row;
    if (i >= MI_COUNT) break;

    int16_t y = 16 + row * 9;
    bool selected = (i == (uint8_t)menuSel);
    bool editing  = selected && menuEditing;

    if (selected) {
      display.fillRect(3, y - 1, 122, 9, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }
    display.setCursor(6, y);
    display.print(MENU_NAMES[i]);
    menuValueStr((MenuItem)i, val, sizeof(val));
    if (strlen(val) > 0) {
      display.setCursor(120 - strlen(val) * 6 - (editing ? 6 : 0), y);
      display.print(val);
      if (editing) display.print("<");
    }
  }
}

// ---------------------------------------------------------------------------
// Serial Terminal Diagnostic Console
// ---------------------------------------------------------------------------
void printMountConsoleSerial(bool forcePrint = false) {
  uint32_t now = millis();
  static uint32_t lastConsolePrintMs = 0;
  if (!forcePrint && (now - lastConsolePrintMs < 3000)) return;
  lastConsolePrintMs = now;

  if (linkState != LINK_PAIRED || !hasMountTelem) {
    Serial.printf("[MOUNT CONSOLE] Searching for Mount telemetry... (Ch %u, Link: %s)\n",
                  scanChannel, (linkState == LINK_PAIRED) ? "PAIRED" : "SCANNING");
    return;
  }

  Serial.println("\n+---------------------- [MOUNT CONSOLE] ----------------------+");
  Serial.printf("| UART Drivers  : Pan: %s (0x%02X) | Tilt: %s (0x%02X)\n",
                latestMountTelem.panUartOk ? "OK [PASS]" : "FAIL [ERROR]",
                latestMountTelem.panVersion,
                latestMountTelem.tiltUartOk ? "OK [PASS]" : "FAIL [ERROR]",
                latestMountTelem.tiltVersion);
  Serial.printf("| Hall Sensors  : Pan: %s | Tilt: %s\n",
                latestMountTelem.panHallDetected ? "DETECTED (At Center Magnet!)" : "CLEAR (No Magnet)",
                latestMountTelem.tiltHallDetected ? "DETECTED (At Center Magnet!)" : "CLEAR (No Magnet)");

  const char* panDirStr = (latestMountTelem.panHallDetected || latestMountTelem.panSteps == 0)
                          ? "CENTER (Aligned with Hall)"
                          : (latestMountTelem.panSteps > 0 ? "RIGHT / CW of Hall" : "LEFT / CCW of Hall");
  const char* tiltDirStr = (latestMountTelem.tiltHallDetected || latestMountTelem.tiltSteps == 0)
                           ? "CENTER (Aligned with Hall)"
                           : (latestMountTelem.tiltSteps > 0 ? "UP of Hall" : "DOWN of Hall");

  Serial.printf("| Pan Position  : %+5ld stp (%+6.1f°) -> %s\n",
                latestMountTelem.panSteps, latestMountTelem.panDeg, panDirStr);
  Serial.printf("| Tilt Position : %+5ld stp (%+6.1f°) -> %s\n",
                latestMountTelem.tiltSteps, latestMountTelem.tiltDeg, tiltDirStr);
  Serial.printf("| Optical Zoom  : %4u ms / 3500 ms (%3u%% Telephoto)\n",
                latestMountTelem.zoomMs, latestMountTelem.zoomPct);

  const char* stateStr = "IDLE (Standstill)";
  if (latestMountTelem.state == 1)      stateStr = "LIVE MOTOR MOTION";
  else if (latestMountTelem.state == 2) stateStr = "HOMING IN PROGRESS";
  else if (latestMountTelem.state == 3) stateStr = "MOVING TO PRESET";

  Serial.printf("| Mount State   : %s [100%% Torque Locked]\n", stateStr);
  if (strlen(latestMountTelem.eventMsg) > 0) {
    Serial.printf("| Last Event    : %s\n", latestMountTelem.eventMsg);
  }
  Serial.println("+-------------------------------------------------------------+\n");
}

void processSerialCli() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  int arg = 0;
  int spaceIdx = line.indexOf(' ');
  if (spaceIdx > 0) {
    cmd = line.substring(0, spaceIdx);
    arg = line.substring(spaceIdx + 1).toInt();
  }
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") {
    Serial.println("\n--- PTZ Remote Controller CLI Commands ---");
    Serial.println("  mount         : Display live Mount Diagnostics Console (UART, Hall, Zoom, Angles)");
    Serial.println("  save <1-8>    : Save current framing to Preset slot");
    Serial.println("  goto <1-8>    : Smoothly move camera to Preset slot");
    Serial.println("  home          : Trigger full Pan, Tilt & Zoom homing");
    Serial.println("  diag          : Switch OLED display to Mount Diagnostics screen");
    Serial.println("  main          : Switch OLED display to Main Dials screen");
    Serial.println("  status        : Print current sticks, link status, and full mount telemetry");
    Serial.println("-------------------------------------------\n");
  } else if (cmd == "mount" || cmd == "telemetry") {
    printMountConsoleSerial(true);
  } else if (cmd == "diag") {
    uiMode = UI_DIAG;
    Serial.println("OLED switched to Mount Diagnostics screen.");
  } else if (cmd == "presets") {
    uiMode = UI_PRESETS;
    Serial.println("OLED switched to Preset Hub.");
    Serial.println("--- Remote Presets Cache ---");
    for (uint8_t i = 0; i < 8; i++) {
      if (remotePresets[i].valid) {
        Serial.printf("  P%u: Pan=%+5.1f deg, Tilt=%+5.1f deg, Zoom=%u%%\n",
                      i + 1, remotePresets[i].panDeg, remotePresets[i].tiltDeg, remotePresets[i].zoomPct);
      } else {
        Serial.printf("  P%u: [EMPTY]\n", i + 1);
      }
    }
  } else if (cmd == "logs") {
    uiMode = UI_LOGS;
    Serial.println("OLED switched to System Logs screen.");
    Serial.println("--- System Log History ---");
    for (uint8_t i = 0; i < logTotal; i++) {
      int idx = (int)logHead - 1 - (int)(logTotal - 1 - i);
      while (idx < 0) idx += MAX_LOG_LINES;
      Serial.printf("  [%u] %s\n", i + 1, logLines[idx % MAX_LOG_LINES]);
    }
  } else if (cmd == "main") {
    uiMode = UI_LIVE;
    Serial.println("OLED switched to Main Dials screen.");
  } else if (cmd == "save") {
    if (arg >= 1 && arg <= 8) commandSavePreset(arg);
    else Serial.println("Error: Slot must be 1 to 8 (e.g., 'save 1')");
  } else if (cmd == "goto") {
    if (arg >= 1 && arg <= 8) commandGotoPreset(arg);
    else Serial.println("Error: Slot must be 1 to 8 (e.g., 'goto 1')");
  } else if (cmd == "home") {
    commandStartHoming();
  } else if (cmd == "status") {
    Serial.printf("[Status] Link: %s (Ch %u) | UI: %u | Active Slot: P%u\n",
                  (linkState == LINK_PAIRED) ? "PAIRED" : "SCANNING",
                  mountChannel, (uint8_t)uiMode, activePresetSlot);
    printMountConsoleSerial(true);
  } else {
    Serial.printf("Unknown command: '%s'. Type 'help' for available commands.\n", cmd.c_str());
  }
}

// ---------------------------------------------------------------------------
// Arduino setup()
// ---------------------------------------------------------------------------
void setup() {
  delay(500);
  setCpuFrequencyMhz(240);  // 240 MHz maximum CPU speed for lightning-fast UI and ADC response
  Serial.begin(115200);

  loadSettings();
  loadRemotePresets();

  // Analog pin configuration
  const uint8_t adcPins[] = {PIN_BATT_ADC, PIN_USB_SENSE,
                             PIN_LJOY_X, PIN_LJOY_Y, PIN_RJOY_X, PIN_RJOY_Y};
  for (uint8_t i = 0; i < sizeof(adcPins); i++) {
    analogSetPinAttenuation(adcPins[i], ADC_11db);
    pinMode(adcPins[i], INPUT);
  }

  // Calibrate joystick resting centers
  calibrateJoysticks();

  // Pushbutton digital inputs
  buttonInit(btnLJoy, PIN_LJOY_SW);
  buttonInit(btnRJoy, PIN_RJOY_SW);
  buttonInit(btnEnc,  PIN_ENC_BTN);
  buttonInit(btnCon,  PIN_BTN_CON);
  buttonInit(btnBak,  PIN_BTN_BAK);
  for (uint8_t i = 0; i < 5; i++) buttonInit(btnColor[i], PIN_COLOR_BTNS[i]);

  // Rotary encoder interrupts
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encLastAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  // Initialize Adafruit SH110X (SH1106G 1.3" OLED)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK);

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("[OLED ERROR] Adafruit SH1106G init failed - check I2C wiring (SDA:8, SCL:9)!");
  } else {
    displayActive = true;
    display.setContrast(settings.brightness);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(14, 20);
    display.print("PTZ CAMERA REMOTE");
    display.setCursor(18, 36);
    display.print("SONY FDR-AX53");
    display.display();
    delay(300);
  }

  // Initialize Wi-Fi in Station Mode & ESP-NOW with Maximum RF Power (21 dBm)
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(84); // 21.0 dBm absolute hardware max limit
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, rebooting...");
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  // Register broadcast peer with channel 0 (follows active STA channel)
  espNowAddPeer(BCAST_MAC, 0);

  scanChannel = loadLinkChannel();
  Serial.printf("Remote MAC: %s | Max RF Power (21 dBm) @ 240 MHz | Scan Ch: %u...\n",
                WiFi.macAddress().c_str(), scanChannel);

  addLog("Remote OS (240MHz) Boot");
}

// ---------------------------------------------------------------------------
// Arduino loop()
// ---------------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  // 1. Process USB Serial CLI
  processSerialCli();

  // 2. Link Management & Channel Scanning
  updateLink(now);

  // 3. Read Joysticks
  float lx = joyAxis(PIN_LJOY_X);
  float ly = joyAxis(PIN_LJOY_Y);
  float rx = joyAxis(PIN_RJOY_X);
  float ry = joyAxis(PIN_RJOY_Y);

  if (settings.invertPan)  lx = -lx;
  if (settings.invertTilt) ly = -ly;

  int16_t pan  = (int16_t)constrain(lx * settings.sensitivity * 1000.0f, -1000, 1000);
  int16_t tilt = (int16_t)constrain(ly * settings.sensitivity * 1000.0f, -1000, 1000);
  int16_t auxX = (int16_t)(rx * 1000.0f);
  int16_t zoom = (int16_t)(ry * 1000.0f); // Right Stick Y: Positive=Tele/In, Negative=Wide/Out

  // 4. Read Rotary Encoder & Navigation Buttons
  int detents = encoderDetents();
  handleMenuInput(detents);

  // 5. Joystick Button Click Actions
  if (buttonPressed(btnLJoy)) {
    // Left Stick Click: Cycle active preset slot (P1 -> P2 -> ... -> P8 -> P1)
    activePresetSlot = (activePresetSlot % 8) + 1;
    char b[16];
    snprintf(b, sizeof(b), "ACTIVE: P%u", activePresetSlot);
    showBanner(b, 1800);
  }

  // Right Stick Click: Save current framing into active preset slot (P1..P8)
  if (buttonPressed(btnRJoy)) {
    commandSavePreset(activePresetSlot);
  }

  // 6. Read 5 Color Preset Pushbuttons (Short Press = Recall, Long Press = Save)
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t slot = i + 1;
    buttonPressed(btnColor[i]);

    if (buttonLongPressed(btnColor[i], 1200)) {
      // Long press (>1.2s) -> Save current framing to preset slot
      commandSavePreset(slot);
    }

    // On release after a short press (not long pressed):
    if (digitalRead(btnColor[i].pin) == HIGH && btnColor[i].stable) {
      if (!btnColor[i].longPressHandled && (now - btnColor[i].pressStartMs > 50)) {
        commandGotoPreset(slot);
      }
      btnColor[i].stable = false;
    }
  }

  // 7. Stream 50 Hz Control Packets while Paired
  if (linkState == LINK_PAIRED && now - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs = now;
    sendControl(pan, tilt, zoom, auxX, buildButtonBits());
  }

  // 8. Render OLED Display (1.3" Adafruit SH1106G)
  if (displayActive) {
    if (linkState != LINK_PAIRED) {
      drawSearchingScreen();
    } else {
      display.clearDisplay();
      drawStatusBar();

      if (uiMode == UI_MENU) {
        drawMenu();
      } else if (uiMode == UI_PRESETS) {
        drawPresetsHub();
      } else if (uiMode == UI_LOGS) {
        drawLogs();
      } else if (uiMode == UI_DIAG) {
        drawMountDiag();
      } else {
        drawMainUi(pan, tilt, zoom, auxX);
      }
    }
    display.display();
  }

  // 9. Periodic Serial Terminal Console (every 3 seconds)
  printMountConsoleSerial(false);

  delay(5);
}
