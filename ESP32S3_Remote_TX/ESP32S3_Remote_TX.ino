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
 *   - Joystick Controls:
 *       Left Stick VRx (Pan)  : GPIO 4 (ADC1_CH3) -> PAN (Left = CCW, Right = CW)
 *       Left Stick VRy (Aux)  : GPIO 5 (ADC1_CH4) -> AUX / Reserved
 *       Left Stick Click (SW) : GPIO 15 (INPUT_PULLUP)
 *       Right Stick VRx (Aux) : GPIO 6 (ADC1_CH5) -> AUX / Reserved
 *       Right Stick VRy (Tilt): GPIO 7 (ADC1_CH6) -> TILT (Up = Up, Down = Down)
 *       Right Stick Click (SW): GPIO 16 (INPUT_PULLUP)
 *   - Pushbutton Controls (Auto-Calibrated Resting Baseline):
 *       Color Button 1: GPIO 38 -> Zoom In / Tele (Held: +1000)
 *       Color Button 2: GPIO 39 -> Zoom Out / Wide (Held: -1000)
 *       Color Button 3: GPIO 40 -> Quick Recall Preset 1
 *       Color Button 4: GPIO 41 -> Quick Recall Preset 2
 *       Color Button 5: GPIO 42 -> Quick Recall Preset 3
 *   - Rotary Encoder (HW-787AB Quadrature) & Navigation:
 *       Phase A (TRA) : GPIO 10 (Interrupt, any edge)
 *       Phase B (TRB) : GPIO 11 (Interrupt, any edge)
 *       Push (PSH)    : GPIO 12 -> Menu Select / Confirm
 *       Confirm (CON) : GPIO 13 -> Menu Select / Confirm / Open Menu
 *       Back (BAK)    : GPIO 17 -> Back / Cancel / Delete Character
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
#define PIN_LJOY_Y        5   // Left joystick VRy  -> AUX / Reserved
#define PIN_LJOY_SW      15   // Left joystick click (active LOW)

#define PIN_RJOY_X        6   // Right joystick VRx -> AUX / Reserved
#define PIN_RJOY_Y        7   // Right joystick VRy -> TILT
#define PIN_RJOY_SW      16   // Right joystick click (active LOW)

#define PIN_I2C_SDA       8   // OLED I2C SDA
#define PIN_I2C_SCL       9   // OLED I2C SCL

#define PIN_ENC_A        10   // HW-787AB encoder phase A (TRA)
#define PIN_ENC_B        11   // HW-787AB encoder phase B (TRB)
#define PIN_ENC_BTN      12   // Encoder push (PSH, active LOW) -> Confirm / Select
#define PIN_BTN_CON      13   // Confirm button (active LOW) -> Confirm / Select
#define PIN_BTN_BAK      17   // Back button    (active LOW) -> Back / Delete / Exit

// 5 Color pushbuttons
// Btn 1: Zoom In, Btn 2: Zoom Out, Btn 3..5: Quick Recall Presets 1..3
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

// Keep in sync with the mount receiver's ZOOM_FULL_RANGE_MS: full optical zoom travel (0 ms = Wide)
#define ZOOM_FULL_RANGE_MS  13000

struct __attribute__((packed)) PtzPacket {
  uint8_t  type;                  // PKT_*
  uint8_t  mac[6];                // sender MAC (pairing handshake)
  int16_t  pan;                   // -1000..+1000 velocity command
  int16_t  tilt;                  // -1000..+1000 velocity command
  int16_t  zoom;                  // -1000..+1000 zoom command (+ Tele/In, - Wide/Out)
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
  uint16_t zoomMs;                // Current zoom dead-reckoning position (0..ZOOM_FULL_RANGE_MS)
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
// UI & Menu Model (Professional Remote OS)
// ---------------------------------------------------------------------------
enum UiMode : uint8_t {
  UI_LIVE = 0,
  UI_MENU,
  UI_PRESETS_LIST,
  UI_PRESET_ACTION,
  UI_NAME_EDITOR,
  UI_HOMING,
  UI_LOGS,
  UI_DIAG
};

enum MenuItem : uint8_t {
  MI_PRESETS = 0,
  MI_HOME,
  MI_DIAG,
  MI_LOGS,
  MI_BRIGHT,
  MI_SENS,
  MI_INV_PAN,
  MI_INV_TILT,
  MI_EXIT,
  MI_COUNT
};

const char* MENU_NAMES[MI_COUNT] = {
  "Preset Hub", "Home Mount", "Mount Diag", "System Logs",
  "Brightness", "Sensitivity", "Invert Pan", "Invert Tilt", "Exit Menu"
};

enum PresetActionItem : uint8_t {
  PACT_GOTO = 0,
  PACT_SAVE,
  PACT_RENAME,
  PACT_BACK,
  PACT_COUNT
};

const char* PRESET_ACTION_NAMES[PACT_COUNT] = {
  "Goto Position", "Save Current Framing", "Rename Preset", "Back"
};

// ---------------------------------------------------------------------------
// Remote Preset Cache & Custom Naming (Upper & Lower Case Support)
// ---------------------------------------------------------------------------
struct PresetSummary {
  bool    valid;
  char    name[12];
  float   panDeg;
  float   tiltDeg;
  uint8_t zoomPct;
};

static const char* DEFAULT_PRESET_NAMES[8] = {
  "Pulpit", "Piano", "Choir", "Wide Stage",
  "Preset 5", "Preset 6", "Preset 7", "Preset 8"
};

PresetSummary remotePresets[8] = {};
uint8_t activePresetSlot   = 1; // Currently selected active slot (1..8)
uint8_t presetListSelected = 0; // Highlighted slot index in list (0..7)
uint8_t presetActSelected  = 0; // Highlighted action in Preset Action (0..3)

// Name Editor State
char    editNameBuffer[12] = "";
uint8_t editNameSlotIdx    = 0;
int16_t editCharIdx        = 0;

// Character set ribbon with UPPERCASE, LOWERCASE, numbers, symbols, DEL, and OK
static const char CHAR_SET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 -_#&./";
#define CHAR_SET_LEN ((int16_t)(sizeof(CHAR_SET) - 1))
#define CHAR_CODE_DEL  (CHAR_SET_LEN)
#define CHAR_CODE_SAVE (CHAR_SET_LEN + 1)
#define TOTAL_EDITOR_SYMBOLS (CHAR_SET_LEN + 2)

// Banner Notifications
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
#define LINK_FAIL_LIMIT   250     // ~5s consecutive TX failures before re-scanning

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
uint8_t  lastMountState       = 0;

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

    snprintf(k, sizeof(k), "p%u_name", i);
    String sName = prefs.getString(k, DEFAULT_PRESET_NAMES[i]);
    if (sName.length() == 0) sName = DEFAULT_PRESET_NAMES[i];
    strncpy(remotePresets[i].name, sName.c_str(), sizeof(remotePresets[i].name) - 1);
    remotePresets[i].name[sizeof(remotePresets[i].name) - 1] = '\0';

    if (remotePresets[i].valid) {
      snprintf(k, sizeof(k), "p%u_p", i); remotePresets[i].panDeg = prefs.getFloat(k, 0.0f);
      snprintf(k, sizeof(k), "p%u_t", i); remotePresets[i].tiltDeg = prefs.getFloat(k, 0.0f);
      snprintf(k, sizeof(k), "p%u_z", i); remotePresets[i].zoomPct = prefs.getUChar(k, 0);
    }
  }
  prefs.end();
}

void savePresetToNVS(uint8_t idx) {
  if (idx >= 8) return;
  prefs.begin("rem_p", false);
  char k[16];
  snprintf(k, sizeof(k), "p%u_v", idx);    prefs.putBool(k, remotePresets[idx].valid);
  snprintf(k, sizeof(k), "p%u_name", idx); prefs.putString(k, String(remotePresets[idx].name));
  snprintf(k, sizeof(k), "p%u_p", idx);    prefs.putFloat(k, remotePresets[idx].panDeg);
  snprintf(k, sizeof(k), "p%u_t", idx);    prefs.putFloat(k, remotePresets[idx].tiltDeg);
  snprintf(k, sizeof(k), "p%u_z", idx);    prefs.putUChar(k, remotePresets[idx].zoomPct);
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
// Universal Auto-Calibrating Button Engine
// ---------------------------------------------------------------------------
// Auto-samples resting voltage levels at boot to support Active-LOW, Active-HIGH,
// Normally Closed switches, and pins with onboard pulldowns (e.g. WS2812 DIN).
struct Button {
  uint8_t  pin;
  bool     restingState;  // Sampled at boot (unpressed level)
  bool     stable;        // true = pressed, false = unpressed
  bool     lastRaw;
  bool     wasPressed;    // One-shot click event
  uint32_t lastChangeMs;
  uint32_t pressStartMs;
  bool     longPressHandled;
};

void buttonInit(Button &b, uint8_t pin) {
  b.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  b.restingState = digitalRead(pin); // Initial baseline sample
  b.stable = false;
  b.lastRaw = false;
  b.wasPressed = false;
  b.lastChangeMs = 0;
  b.pressStartMs = 0;
  b.longPressHandled = false;
}

Button btnLJoy, btnRJoy, btnEnc, btnCon, btnBak, btnColor[5];

void calibrateButtonRestingState() {
  delay(60); // Allow all external wiring and pull-ups to fully settle
  btnLJoy.restingState = digitalRead(btnLJoy.pin);
  btnRJoy.restingState = digitalRead(btnRJoy.pin);
  btnEnc.restingState  = digitalRead(btnEnc.pin);
  btnCon.restingState  = digitalRead(btnCon.pin);
  btnBak.restingState  = digitalRead(btnBak.pin);
  for (uint8_t i = 0; i < 5; i++) {
    btnColor[i].restingState = digitalRead(btnColor[i].pin);
  }
}

void buttonUpdate(Button &b) {
  // Raw is TRUE whenever the pin differs from its resting state (user is pressing the button)
  bool raw = (digitalRead(b.pin) != b.restingState);
  uint32_t now = millis();
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChangeMs = now;
  }
  if ((now - b.lastChangeMs) > 20 && raw != b.stable) {
    b.stable = raw;
    if (b.stable) {
      b.pressStartMs = now;
      b.longPressHandled = false;
      b.wasPressed = true;
    }
  }
}

bool buttonPressed(Button &b) {
  if (b.wasPressed) {
    b.wasPressed = false;
    return true;
  }
  return false;
}

bool buttonLongPressed(Button &b, uint32_t holdMs = 1200) {
  if (b.stable && !b.longPressHandled && (millis() - b.pressStartMs >= holdMs)) {
    b.longPressHandled = true;
    return true;
  }
  return false;
}

void updateAllButtons() {
  buttonUpdate(btnLJoy);
  buttonUpdate(btnRJoy);
  buttonUpdate(btnEnc);
  buttonUpdate(btnCon);
  buttonUpdate(btnBak);
  for (uint8_t i = 0; i < 5; i++) {
    buttonUpdate(btnColor[i]);
  }
}

uint16_t buildButtonBits() {
  uint16_t b = 0;
  if (btnLJoy.stable) b |= BTNBIT_LJOY;
  if (btnRJoy.stable) b |= BTNBIT_RJOY;
  if (btnEnc.stable)  b |= BTNBIT_PSH;
  if (btnCon.stable)  b |= BTNBIT_CON;
  if (btnBak.stable)  b |= BTNBIT_BAK;
  for (uint8_t i = 0; i < 5; i++)
    if (btnColor[i].stable) b |= (BTNBIT_C1 << i);
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

  savePresetToNVS(idx);

  char b[28];
  snprintf(b, sizeof(b), "SAVED [%s]!", remotePresets[idx].name);
  showBanner(b, 2500);
  addLog(b);
  activePresetSlot = id;
  Serial.printf("\n>>> [PRESET] Saved Preset %u [%s] to Mount & Remote! <<<\n\n", id, remotePresets[idx].name);
}

void commandGotoPreset(uint8_t id) {
  if (id < 1 || id > 8) return;
  PtzPacket pkt = {};
  pkt.type = PKT_GOTO_PRESET;
  pkt.presetId = id;
  WiFi.macAddress(pkt.mac);
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));

  uint8_t idx = id - 1;
  char b[28];
  snprintf(b, sizeof(b), "RECALL [%s]...", remotePresets[idx].name);
  showBanner(b, 2500);
  addLog(b);
  activePresetSlot = id;
  Serial.printf("\n>>> [RECALL] Sent GOTO Preset %u [%s] to Mount! <<<\n\n", id, remotePresets[idx].name);
}

// Fullscreen Homing Execution
UiMode   uiMode      = UI_LIVE;
MenuItem menuSel     = MI_PRESETS;
bool     menuEditing = false;

void commandStartHoming() {
  PtzPacket pkt = {};
  pkt.type = PKT_START_HOMING;
  WiFi.macAddress(pkt.mac);
  esp_now_send(mountMac, (uint8_t*)&pkt, sizeof(pkt));
  Serial.println("\n>>> [HOMING] Sent START HOMING command to Mount! <<<\n");

  uiMode = UI_HOMING;
  showBanner("HOMING MOUNT...", 4000);
  addLog("Homing Mount...");
}

// ---------------------------------------------------------------------------
// Settings Menu & UI Navigation
// ---------------------------------------------------------------------------
void handleMenuInput(int detents) {
  bool pshPress = buttonPressed(btnEnc);
  bool conPress = buttonPressed(btnCon);
  bool bakPress = buttonPressed(btnBak);
  bool actSelect = pshPress || conPress; // Both Confirm and Encoder Push act as Select / Confirm

  // Auto-manage Homing UI State: if Mount enters state 2 (HOMING), switch to UI_HOMING
  if (latestMountTelem.state == 2) {
    uiMode = UI_HOMING;
  } else if (uiMode == UI_HOMING && latestMountTelem.state != 2 && lastMountState == 2) {
    // Finished homing
    uiMode = UI_LIVE;
    showBanner("HOMING COMPLETE!", 3000);
    addLog("Homing Complete");
  }
  lastMountState = latestMountTelem.state;

  // 1. Fullscreen Homing Screen
  if (uiMode == UI_HOMING) {
    if (bakPress) {
      // Allow user to manually exit homing screen if desired
      uiMode = UI_LIVE;
    }
    return;
  }

  // 2. Live Screen
  if (uiMode == UI_LIVE) {
    if (actSelect) {
      uiMode = UI_MENU;
      menuEditing = false;
    }
    return;
  }

  // 3. Main Menu
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

    if (actSelect) {
      if (menuEditing) {
        menuEditing = false;
        saveSettings();
      } else {
        switch (menuSel) {
          case MI_PRESETS:
            uiMode = UI_PRESETS_LIST;
            break;
          case MI_HOME:
            commandStartHoming();
            break;
          case MI_DIAG:
            uiMode = UI_DIAG;
            break;
          case MI_LOGS:
            uiMode = UI_LOGS;
            logScroll = 0;
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
    return;
  }

  // 4. Presets List Screen (Select 1 of 8 Presets)
  if (uiMode == UI_PRESETS_LIST) {
    if (bakPress) {
      uiMode = UI_MENU;
      return;
    }
    if (actSelect) {
      uiMode = UI_PRESET_ACTION;
      presetActSelected = 0;
      return;
    }
    if (detents != 0) {
      int s = (int)presetListSelected + detents;
      while (s < 0) s += 8;
      presetListSelected = (uint8_t)(s % 8);
    }
    return;
  }

  // 5. Preset Action Screen (Goto, Save, Rename, Back)
  if (uiMode == UI_PRESET_ACTION) {
    if (bakPress) {
      uiMode = UI_PRESETS_LIST;
      return;
    }
    if (actSelect) {
      uint8_t slot = presetListSelected + 1;
      switch ((PresetActionItem)presetActSelected) {
        case PACT_GOTO:
          commandGotoPreset(slot);
          uiMode = UI_LIVE;
          break;
        case PACT_SAVE:
          commandSavePreset(slot);
          uiMode = UI_LIVE;
          break;
        case PACT_RENAME:
          editNameSlotIdx = presetListSelected;
          strncpy(editNameBuffer, remotePresets[editNameSlotIdx].name, sizeof(editNameBuffer) - 1);
          editNameBuffer[sizeof(editNameBuffer) - 1] = '\0';
          editCharIdx = 0;
          uiMode = UI_NAME_EDITOR;
          break;
        case PACT_BACK:
        default:
          uiMode = UI_PRESETS_LIST;
          break;
      }
      return;
    }
    if (detents != 0) {
      int s = (int)presetActSelected + detents;
      while (s < 0) s += PACT_COUNT;
      presetActSelected = (uint8_t)(s % PACT_COUNT);
    }
    return;
  }

  // 6. Character Wheel Name Editor
  if (uiMode == UI_NAME_EDITOR) {
    if (bakPress) {
      // Back button acts as backspace / delete last character
      size_t len = strlen(editNameBuffer);
      if (len > 0) {
        editNameBuffer[len - 1] = '\0';
      } else {
        // If empty, exit editor
        uiMode = UI_PRESET_ACTION;
      }
      return;
    }

    if (actSelect) {
      if (editCharIdx == CHAR_CODE_DEL) {
        // Backspace action
        size_t len = strlen(editNameBuffer);
        if (len > 0) editNameBuffer[len - 1] = '\0';
      } else if (editCharIdx == CHAR_CODE_SAVE) {
        // Save & Finish
        if (strlen(editNameBuffer) == 0) {
          strncpy(editNameBuffer, DEFAULT_PRESET_NAMES[editNameSlotIdx], sizeof(editNameBuffer) - 1);
        }
        strncpy(remotePresets[editNameSlotIdx].name, editNameBuffer, sizeof(remotePresets[editNameSlotIdx].name) - 1);
        remotePresets[editNameSlotIdx].name[sizeof(remotePresets[editNameSlotIdx].name) - 1] = '\0';
        savePresetToNVS(editNameSlotIdx);

        char b[28];
        snprintf(b, sizeof(b), "RENAMED TO [%s]", editNameBuffer);
        showBanner(b, 2500);
        addLog(b);
        uiMode = UI_PRESET_ACTION;
      } else if (editCharIdx < CHAR_SET_LEN) {
        // Append selected character
        size_t len = strlen(editNameBuffer);
        if (len < sizeof(editNameBuffer) - 1) {
          editNameBuffer[len] = CHAR_SET[editCharIdx];
          editNameBuffer[len + 1] = '\0';
        }
      }
      return;
    }

    if (detents != 0) {
      int s = (int)editCharIdx + detents;
      while (s < 0) s += TOTAL_EDITOR_SYMBOLS;
      editCharIdx = (int16_t)(s % TOTAL_EDITOR_SYMBOLS);
    }
    return;
  }

  // 7. System Logs Screen (Scrollable Console)
  if (uiMode == UI_LOGS) {
    if (bakPress || actSelect) {
      uiMode = UI_MENU;
      return;
    }
    if (detents != 0) {
      int maxScroll = (logTotal > 5) ? (logTotal - 5) : 0;
      logScroll = constrain(logScroll - detents, 0, maxScroll);
    }
    return;
  }

  // 8. Mount Diag Screen
  if (uiMode == UI_DIAG) {
    if (bakPress || actSelect) {
      uiMode = UI_MENU;
      return;
    }
    return;
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

// Clean Status Bar: Battery | Active Preset Name Badge | Link Status
void drawStatusBar() {
  float volts = filteredBatteryVoltage();
  int   pct   = batteryPercent(volts);
  bool  usb   = usbConnected();

  display.setTextSize(1);

  // 1. Left: Battery Gauge or Charge ETA (X: 1..32)
  if (usb) {
    display.fillTriangle(1, 0, 5, 0, 3, 4, SH110X_WHITE);
    display.fillTriangle(3, 4, 7, 4, 2, 9, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(9, 1);
    display.printf("%dm", chargeEtaMinutes(pct));
  } else {
    display.drawRect(0, 1, 13, 8, SH110X_WHITE);
    display.fillRect(13, 3, 2, 4, SH110X_WHITE);
    int fillW = (11 * pct) / 100;
    if (fillW > 0) display.fillRect(1, 2, fillW, 6, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(17, 1);
    display.printf("%d%%", pct);
  }

  // 2. Center: Active Preset Name Badge
  display.setTextColor(SH110X_WHITE);
  uint8_t activeIdx = activePresetSlot - 1;
  const char* activeName = (activeIdx < 8 && strlen(remotePresets[activeIdx].name) > 0)
                           ? remotePresets[activeIdx].name : DEFAULT_PRESET_NAMES[activeIdx];
  char badge[16];
  snprintf(badge, sizeof(badge), "[%.8s]", activeName);
  int16_t badgeW = strlen(badge) * 6;
  int16_t badgeX = (SCREEN_WIDTH - badgeW) / 2;
  display.setCursor(badgeX, 1);
  display.print(badge);

  // 3. Right: Clean Link Status Badge (X: 92..126)
  display.setCursor(92, 1);
  if (linkState == LINK_PAIRED) {
    display.print("[LINK]");
  } else {
    display.print("[SCAN]");
  }

  display.drawFastHLine(0, STATUS_BAR_H, SCREEN_WIDTH, SH110X_WHITE);
}

void drawDial(int16_t cx, int16_t cy, int16_t r, float val, bool isVertical, const char* label) {
  display.drawCircle(cx, cy, r, SH110X_WHITE);
  display.drawFastHLine(cx - r + 2, cy, 2 * r - 3, SH110X_WHITE);
  display.drawFastVLine(cx, cy - r + 2, 2 * r - 3, SH110X_WHITE);

  int16_t dx = cx;
  int16_t dy = cy;
  if (isVertical) {
    dy = cy - (int16_t)(val * (r - 4));
  } else {
    dx = cx + (int16_t)(val * (r - 4));
  }
  display.fillCircle(dx, dy, 2, SH110X_WHITE);

  int16_t tx = cx - (int16_t)(strlen(label) * 3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(tx, cy + r + 2);
  display.print(label);
}

void drawMainUi(int16_t pan, int16_t tilt, int16_t zoom) {
  // Left Dial: Pan (VRx on Left Joystick)
  drawDial(22, 31, 11, pan / 1000.0f, false, "PAN");

  // Center live telemetry readouts & Zoom state
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(41, 16);
  display.printf("P:%+4.0f\xF7", latestMountTelem.panDeg);
  display.setCursor(41, 26);
  display.printf("T:%+4.0f\xF7", latestMountTelem.tiltDeg);
  display.setCursor(41, 36);
  display.printf("Z:%3u%%", latestMountTelem.zoomPct);

  // Zoom In / Out Active Indicator Badge
  if (zoom > 100) {
    display.fillRect(41, 46, 44, 8, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(43, 46);
    display.print("TELE IN");
  } else if (zoom < -100) {
    display.fillRect(41, 46, 44, 8, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(43, 46);
    display.print("WIDE OUT");
  }

  // Right Dial: Tilt (VRy on Right Joystick)
  drawDial(105, 31, 11, tilt / 1000.0f, true, "TILT");

  // Bottom Area: Banner or Context Prompt
  if (millis() < bannerExpireMs && strlen(bannerText) > 0) {
    int16_t textLen = strlen(bannerText);
    int16_t boxW = min((int16_t)126, (int16_t)((textLen * 6) + 10));
    int16_t boxX = (SCREEN_WIDTH - boxW) / 2;
    display.fillRect(boxX, 54, boxW, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(boxX + 5, 55);
    display.print(bannerText);
  } else {
    display.setTextColor(SH110X_WHITE);
    display.setCursor(4, 55);
    display.print("CON:Menu  P1-3:Recall");
  }
}

// Fullscreen Dedicated Animated Homing Screen
void drawHomingScreen() {
  uint32_t now = millis();

  // Top Header (Y: 2)
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(6, 2);
  display.print("--- HOMING MOUNT ---");

  // Rotating Gyroscope / Radar in top right (X: 88..116, Y: 15..43)
  int16_t cx = 102;
  int16_t cy = 29;
  display.drawCircle(cx, cy, 12, SH110X_WHITE);
  display.drawFastHLine(cx - 12, cy, 25, SH110X_WHITE);
  display.drawFastVLine(cx, cy - 12, 25, SH110X_WHITE);

  float angle = (float)((now / 12) % 360) * 0.0174533f;
  int16_t rx = cx + (int16_t)(cosf(angle) * 10);
  int16_t ry = cy + (int16_t)(sinf(angle) * 10);
  display.fillCircle(rx, ry, 2, SH110X_WHITE);

  // Live Homing Axis Telemetry on Left
  display.setCursor(2, 16);
  display.printf("Pan : %+5.1f\xF7 %s",
                 latestMountTelem.panDeg,
                 latestMountTelem.panHallDetected ? "[ALIGN]" : "[FIND]");

  display.setCursor(2, 27);
  display.printf("Tilt: %+5.1f\xF7 %s",
                 latestMountTelem.tiltDeg,
                 latestMountTelem.tiltHallDetected ? "[ALIGN]" : "[FIND]");

  display.setCursor(2, 38);
  display.printf("Zoom: %3u%% Wide", latestMountTelem.zoomPct);

  // Bottom Animated Progress Scanning Bar (X: 4..124, Y: 52..60)
  display.drawRect(4, 52, 120, 9, SH110X_WHITE);
  uint8_t barPhase = (now / 25) % 110;
  display.fillRect(6 + barPhase, 54, 6, 5, SH110X_WHITE);
}

// Presets List: 8 Slots with custom string names
void drawPresetsList() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(2, 13);
  display.print("SELECT PRESET (1-8):");

  // Show 4 visible presets per page
  uint8_t startIdx = (presetListSelected / 4) * 4;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t slotIdx = startIdx + i;
    if (slotIdx >= 8) break;

    int16_t y = 24 + (i * 10);
    bool isSelected = (slotIdx == presetListSelected);

    if (isSelected) {
      display.fillRect(0, y - 1, SCREEN_WIDTH, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }

    display.setCursor(2, y);
    display.printf("P%u: %-9.9s", slotIdx + 1, remotePresets[slotIdx].name);

    if (remotePresets[slotIdx].valid) {
      display.setCursor(76, y);
      display.printf("%+3.0f\xF7 Z:%2u%%", remotePresets[slotIdx].panDeg, remotePresets[slotIdx].zoomPct);
    } else {
      display.setCursor(80, y);
      display.print("[EMPTY]");
    }
  }
}

// Preset Action Sub-Menu (Goto, Save, Rename, Back)
void drawPresetAction() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  uint8_t slot = presetListSelected + 1;
  display.setCursor(2, 13);
  display.printf("PRESET %u: [%s]", slot, remotePresets[presetListSelected].name);

  for (uint8_t i = 0; i < PACT_COUNT; i++) {
    int16_t y = 25 + (i * 10);
    bool isSelected = (i == presetActSelected);

    if (isSelected) {
      display.fillRect(0, y - 1, SCREEN_WIDTH, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }

    display.setCursor(4, y);
    display.printf("%u. %s", i + 1, PRESET_ACTION_NAMES[i]);
  }
}

// Interactive Character Wheel Name Editor
void drawNameEditor() {
  uint8_t slot = editNameSlotIdx + 1;
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Title
  display.setCursor(2, 13);
  display.printf("RENAME P%u (ROT+CONFIRM)", slot);

  // Name Input Box
  display.drawRect(2, 23, 124, 12, SH110X_WHITE);
  display.setCursor(6, 25);
  display.print(editNameBuffer);
  // Blinking text cursor
  if ((millis() / 350) % 2 == 0) {
    display.print("_");
  }

  // Character Wheel Ribbon (5 symbols centered)
  display.setCursor(2, 39);
  display.print("Sym: ");

  for (int16_t off = -2; off <= 2; off++) {
    int16_t symbolIdx = editCharIdx + off;
    while (symbolIdx < 0) symbolIdx += TOTAL_EDITOR_SYMBOLS;
    symbolIdx = symbolIdx % TOTAL_EDITOR_SYMBOLS;

    int16_t xPos = 34 + (off + 2) * 18;
    int16_t yPos = 38;

    if (off == 0) {
      // Selected highlighted character
      display.fillRect(xPos - 2, yPos - 1, 16, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }

    display.setCursor(xPos, yPos);
    if (symbolIdx == CHAR_CODE_DEL) {
      display.print("DEL");
    } else if (symbolIdx == CHAR_CODE_SAVE) {
      display.print("OK");
    } else {
      display.print(CHAR_SET[symbolIdx]);
    }
  }

  // Footer Help
  display.setTextColor(SH110X_WHITE);
  display.setCursor(2, 53);
  display.print("CON:Add  BAK:Del  OK:Save");
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
  Serial.printf("| Optical Zoom  : %4u ms / %u ms (%3u%% Telephoto)\n",
                latestMountTelem.zoomMs, ZOOM_FULL_RANGE_MS, latestMountTelem.zoomPct);

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

void printButtonsSerial() {
  Serial.println("\n--- Remote Pushbutton & Pin Status ---");
  Serial.printf("  Left Stick SW  (GPIO 15): Pin=%d, Rest=%d -> %s\n", digitalRead(PIN_LJOY_SW), btnLJoy.restingState, btnLJoy.stable ? "PRESSED" : "RELEASED");
  Serial.printf("  Right Stick SW (GPIO 16): Pin=%d, Rest=%d -> %s\n", digitalRead(PIN_RJOY_SW), btnRJoy.restingState, btnRJoy.stable ? "PRESSED" : "RELEASED");
  Serial.printf("  Encoder Push   (GPIO 12): Pin=%d, Rest=%d -> %s\n", digitalRead(PIN_ENC_BTN), btnEnc.restingState, btnEnc.stable ? "PRESSED" : "RELEASED");
  Serial.printf("  Confirm Button (GPIO 13): Pin=%d, Rest=%d -> %s\n", digitalRead(PIN_BTN_CON), btnCon.restingState, btnCon.stable ? "PRESSED" : "RELEASED");
  Serial.printf("  Back Button    (GPIO 17): Pin=%d, Rest=%d -> %s\n", digitalRead(PIN_BTN_BAK), btnBak.restingState, btnBak.stable ? "PRESSED" : "RELEASED");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.printf("  Color Button %u (GPIO %d): Pin=%d, Rest=%d -> %s\n",
                  i + 1, PIN_COLOR_BTNS[i], digitalRead(PIN_COLOR_BTNS[i]), btnColor[i].restingState, btnColor[i].stable ? "PRESSED" : "RELEASED");
  }
  Serial.println("--------------------------------------\n");
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
    Serial.println("  buttons       : Print live voltage & state of all remote pushbuttons");
    Serial.println("  save <1-8>    : Save current framing to Preset slot");
    Serial.println("  goto <1-8>    : Smoothly move camera to Preset slot");
    Serial.println("  home          : Trigger full Pan, Tilt & Zoom homing");
    Serial.println("  diag          : Switch OLED display to Mount Diagnostics screen");
    Serial.println("  main          : Switch OLED display to Main Dials screen");
    Serial.println("  presets       : List all configured presets & names");
    Serial.println("  status        : Print current sticks, link status, and full mount telemetry");
    Serial.println("-------------------------------------------\n");
  } else if (cmd == "buttons" || cmd == "pins") {
    printButtonsSerial();
  } else if (cmd == "mount" || cmd == "telemetry") {
    printMountConsoleSerial(true);
  } else if (cmd == "diag") {
    uiMode = UI_DIAG;
    Serial.println("OLED switched to Mount Diagnostics screen.");
  } else if (cmd == "presets") {
    uiMode = UI_PRESETS_LIST;
    Serial.println("OLED switched to Presets List.");
    Serial.println("--- Remote Presets Cache ---");
    for (uint8_t i = 0; i < 8; i++) {
      if (remotePresets[i].valid) {
        Serial.printf("  P%u [%s]: Pan=%+5.1f deg, Tilt=%+5.1f deg, Zoom=%u%%\n",
                      i + 1, remotePresets[i].name, remotePresets[i].panDeg, remotePresets[i].tiltDeg, remotePresets[i].zoomPct);
      } else {
        Serial.printf("  P%u [%s]: [EMPTY]\n", i + 1, remotePresets[i].name);
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
    Serial.printf("[Status] Link: %s (Ch %u) | UI: %u | Active Slot: P%u [%s]\n",
                  (linkState == LINK_PAIRED) ? "PAIRED" : "SCANNING",
                  mountChannel, (uint8_t)uiMode, activePresetSlot,
                  remotePresets[activePresetSlot - 1].name);
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

  // Pushbutton digital inputs with internal pull-up resistors
  buttonInit(btnLJoy, PIN_LJOY_SW);
  buttonInit(btnRJoy, PIN_RJOY_SW);
  buttonInit(btnEnc,  PIN_ENC_BTN);
  buttonInit(btnCon,  PIN_BTN_CON);
  buttonInit(btnBak,  PIN_BTN_BAK);
  for (uint8_t i = 0; i < 5; i++) {
    buttonInit(btnColor[i], PIN_COLOR_BTNS[i]);
  }

  // Calibrate baseline resting level for all pushbuttons
  calibrateButtonRestingState();

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

  // 3. Update all button debouncers with baseline calibration
  updateAllButtons();

  // 4. Read Joysticks: Left Stick X -> Pan, Right Stick Y -> Tilt
  float lx = joyAxis(PIN_LJOY_X);
  float rx = joyAxis(PIN_RJOY_X);
  float ry = joyAxis(PIN_RJOY_Y);

  if (settings.invertPan)  lx = -lx;
  if (settings.invertTilt) ry = -ry;

  int16_t pan  = (int16_t)constrain(lx * settings.sensitivity * 1000.0f, -1000, 1000);
  int16_t tilt = (int16_t)constrain(ry * settings.sensitivity * 1000.0f, -1000, 1000);
  int16_t auxX = (int16_t)(rx * 1000.0f);

  // 5. Zoom Buttons: Color Btn 1 (GPIO 38) -> Zoom In, Color Btn 2 (GPIO 39) -> Zoom Out
  int16_t zoom = 0;
  if (uiMode == UI_LIVE) {
    if (btnColor[0].stable) {
      zoom = 1000;  // Zoom In / Telephoto
    } else if (btnColor[1].stable) {
      zoom = -1000; // Zoom Out / Wide
    }
  } else {
    // When in Homing, Menus, or Dialogs, suppress stick and zoom motion
    pan = 0;
    tilt = 0;
    zoom = 0;
  }

  // 6. Read Rotary Encoder & Menu Navigation
  int detents = encoderDetents();
  handleMenuInput(detents);

  // 7. Joystick Button Click Actions (Optional Shortcuts)
  if (uiMode == UI_LIVE && buttonPressed(btnLJoy)) {
    // Left Stick Click: Cycle active preset slot (P1 -> P2 -> ... -> P8 -> P1)
    activePresetSlot = (activePresetSlot % 8) + 1;
    char b[24];
    snprintf(b, sizeof(b), "ACTIVE: [%s]", remotePresets[activePresetSlot - 1].name);
    showBanner(b, 1800);
  }

  // 8. Quick Recall Buttons: Color Buttons 3..5 (GPIO 40..42) -> Instant Recall Preset 1, 2, 3
  if (uiMode == UI_LIVE) {
    for (uint8_t i = 2; i < 5; i++) {
      if (buttonPressed(btnColor[i])) {
        uint8_t slot = (i - 2) + 1; // Btn3->P1, Btn4->P2, Btn5->P3
        commandGotoPreset(slot);
      }
    }
  }

  // 9. Stream 50 Hz Control Packets while Paired
  if (linkState == LINK_PAIRED && now - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs = now;
    sendControl(pan, tilt, zoom, auxX, buildButtonBits());
  }

  // 10. Render OLED Display (1.3" Adafruit SH1106G)
  if (displayActive) {
    display.clearDisplay(); // Always clear entire buffer before rendering frame

    if (linkState != LINK_PAIRED) {
      drawSearchingScreen();
    } else if (uiMode == UI_HOMING) {
      drawHomingScreen();
    } else {
      drawStatusBar();

      if (uiMode == UI_MENU) {
        drawMenu();
      } else if (uiMode == UI_PRESETS_LIST) {
        drawPresetsList();
      } else if (uiMode == UI_PRESET_ACTION) {
        drawPresetAction();
      } else if (uiMode == UI_NAME_EDITOR) {
        drawNameEditor();
      } else if (uiMode == UI_LOGS) {
        drawLogs();
      } else if (uiMode == UI_DIAG) {
        drawMountDiag();
      } else {
        drawMainUi(pan, tilt, zoom);
      }
    }
    display.display();
  }

  // 11. Periodic Serial Terminal Console (every 3 seconds)
  printMountConsoleSerial(false);

  delay(5);
}
