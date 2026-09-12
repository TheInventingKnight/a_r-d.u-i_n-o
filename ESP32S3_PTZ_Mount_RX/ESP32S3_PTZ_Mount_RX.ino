/*
 * ESP32-S3 N16R8 PTZ Camera Mount - PRODUCTION FIRMWARE (Receiver)
 * ================================================================
 * Translates incoming ESP-NOW commands from ESP32S3_Remote_TX into:
 *   1. Dual BIGTREETECH TMC2209 V1.3 stepper motion via HardwareSerial UART & FastAccelStepper
 *   2. Sony FDR-AX53 Zoom Control via 40 kHz 15-bit SIRC IR Transmitter on GPIO 8
 *   3. Dead-Reckoning Zoom Tracking & Automatic Wide Homing
 *   4. Center-Aware Stepper Homing with NVS direction memory & +/-180 deg cable-wrap safety
 *   5. Non-Volatile (NVS) Presets (Pan, Tilt, Zoom) recallable by Remote and future Laptop Wi-Fi
 *
 * Master Wiring & Hardware Architecture:
 * -------------------------------------------------------------------------
 *   - Pan Stepper (TMC2209 #1 - Serial1):
 *       EN: GPIO 4 | STEP: GPIO 5 | DIR: GPIO 6
 *       TX: GPIO 10 -> 1k Ohm resistor -> joins RX line
 *       RX: GPIO 11 -> connects to 1k resistor & goes to Pan Driver RX (PDN_UART)
 *
 *   - Tilt Stepper (TMC2209 #2 - Serial2):
 *       EN: GPIO 7 | STEP: GPIO 15 | DIR: GPIO 16
 *       TX: GPIO 38 -> 1k Ohm resistor -> joins RX line
 *       RX: GPIO 39 -> connects to 1k resistor & goes to Tilt Driver RX (PDN_UART)
 *
 *   - 3W IR Transmitter Module (Sony FDR-AX53 Zoom):
 *       Signal: ESP32-S3 GPIO 8 -> Logic Level Shifter LV1 -> HV1 (5V) -> IR IN pin
 *       Power:  5V Rail -> IR Module VCC | GND Rail -> IR Module GND
 *       Carrier: 40 kHz (33% duty cycle) hardware LEDC PWM
 *       Codes:  Sony 15-bit SIRC (Address 0xD9, Tele 0x1A, Wide 0x1B)
 *
 *   - Hall-Effect Sensors (A3144 - Active LOW):
 *       Pan Hall:  Pin 1 to 3V3, Pin 2 to GND, Pin 3 to GPIO 17
 *       Tilt Hall: Pin 1 to 3V3, Pin 2 to GND, Pin 3 to GPIO 18
 *
 *   - Relay Module (Aux):
 *       IN: GPIO 9 | VCC: 5V Rail | GND: 5V Rail
 *
 * Required Libraries (Install via Arduino Library Manager):
 *   1. FastAccelStepper (v0.30.0+) by Gin66
 *   2. TMCStepper       (v0.7.3+)  by teemuatlut
 *
 * Board Settings (Arduino IDE):
 *   - Board: ESP32S3 Dev Module
 *   - Flash Size: 16MB (128Mb)
 *   - PSRAM: "OPI PSRAM"
 *   - USB CDC On Boot: "Enabled"
 *   - Upload Speed: 921600
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <Preferences.h>
#include <freertos/queue.h>

// Forward declaration
struct MotorState;

// ---------------------------------------------------------------------------
// Wi-Fi credentials (2.4 GHz network the mount joins)
// ---------------------------------------------------------------------------
const char* WIFI_SSID = "Netgear";
const char* WIFI_PASS = "windows12";

// ---------------------------------------------------------------------------
// Pin definitions & Master Wiring Map
// ---------------------------------------------------------------------------
// Pan stepper (TMC2209 #1 - Serial1)
#define PIN_PAN_EN        4    // Active LOW enable
#define PIN_PAN_STEP      5    // Step pulse
#define PIN_PAN_DIR       6    // Direction
#define PIN_PAN_TX       10    // ESP32-S3 TX -> 1k resistor -> joins RX
#define PIN_PAN_RX       11    // ESP32-S3 RX -> connects to 1k resistor & TMC2209 RX (PDN_UART)

// Tilt stepper (TMC2209 #2 - Serial2)
#define PIN_TILT_EN       7    // Active LOW enable
#define PIN_TILT_STEP    15    // Step pulse
#define PIN_TILT_DIR     16    // Direction
#define PIN_TILT_TX      38    // ESP32-S3 TX -> 1k resistor -> joins RX
#define PIN_TILT_RX      39    // ESP32-S3 RX -> connects to 1k resistor & TMC2209 RX (PDN_UART)

// Hall-Effect Sensors (A3144 Active LOW with internal PULLUP)
#define PIN_PAN_HALL     17    // Pan zero-reference Hall sensor
#define PIN_TILT_HALL    18    // Tilt zero-reference Hall sensor

// Sony Camcorder IR Transmitter & Relay Aux
#define PIN_IR_IN         8    // 3W IR transmitter via 5V level shifter
#define PIN_RELAY_IN      9    // Relay trigger module

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// TMC2209 Driver Configuration Parameters
// ---------------------------------------------------------------------------
#define R_SENSE           0.11f // Sense resistor on BIGTREETECH TMC2209 V1.3 (0.11 Ohm)
#define DRIVER_ADDRESS    0b00  // TMC2209 UART address (MS1=GND, MS2=GND -> 0b00)
#define RMS_CURRENT_MA    900   // RMS current in mA for Usongshine 17HS4023 (0.90A reduced from 0.95A to keep motors cool)
#define MICROSTEPS_VAL    16    // 1/16 microstepping via UART register

// ---------------------------------------------------------------------------
// Herringbone Gear Ratios & Microstepping Calculations:
//   Motor: 200 full steps/rev (1.8 deg) * 16 microsteps = 3200 steps/motor rev
//   Pan Axis : 17T Motor -> 144T Driven (Ratio = 144/17 ~ 8.470588)
//              Output rev = 3200 * 144 / 17 = 460800 / 17 ~ 27105.88 steps / 360 deg
//              Steps/deg  = 1280 / 17 ~ 75.294118 steps/deg
//              Travel     = +/-180 deg = +/-13553 steps
//   Tilt Axis: 21T Motor -> 64T Driven (Ratio = 64/21 ~ 3.047619)
//              Output rev = 3200 * 64 / 21 = 204800 / 21 ~ 9752.38 steps / 360 deg
//              Steps/deg  = 5120 / 189 ~ 27.089947 steps/deg
//              Travel     = +/-90 deg = +/-2438 steps
// ---------------------------------------------------------------------------
#define PAN_STEPS_PER_DEG   75.2941f
#define TILT_STEPS_PER_DEG  27.0899f

#define MAX_PAN_STEPS       13553   // +/-180 deg max travel from center (0)
#define MAX_TILT_STEPS      2438    // +/-90 deg max travel from center (0)

#define TILT_LEVEL_OFFSET_STEPS  (+160) // ~ +6.0 deg upward adjustment to perfectly level horizon

#define MAX_SPEED_HZ        4500   // Full-stick max speed (steps/s) ~60 deg/s on Pan
#define MIN_SPEED_HZ         150   // Slowest creep speed at deadzone threshold
#define ACCEL_HZ_S         25000   // Operational acceleration ramp (steps/s^2)
#define JOY_DEAD              60   // Command deadzone (out of +/-1000)
#define RX_TIMEOUT_MS        2500  // Failsafe link-loss timeout (ms) - generous margin for dual-stick motion
#define PWR_SETTLE_DELAY_MS 2000   // Wait for 12V rail & driver logic to stabilize

// ---------------------------------------------------------------------------
// Cinematic Preset Smooth Transition Speeds
// ---------------------------------------------------------------------------
#define PRESET_MAX_SPEED_HZ 1600   // Smooth, elegant cinematic pan/tilt speed (~20 deg/s)
#define PRESET_MIN_SPEED_HZ  150   // Smooth arrival speed floor
#define PRESET_ACCEL_HZ_S   2500   // Gentle cinematic ease-in / ease-out acceleration ramp

// ---------------------------------------------------------------------------
// Homing Parameters (High-Precision Calibration)
// ---------------------------------------------------------------------------
#define HOMING_SEARCH_SPEED_HZ  1800 // Speed for initial sensor search (steps/s)
#define HOMING_CREEP_SPEED_HZ    250 // Precision latch speed (steps/s)
#define HOMING_BACKOFF_STEPS     350 // Optimal backoff to clear magnetic hysteresis without excessive travel
#define HOMING_ACCEL_HZ_S       8000 // Controlled acceleration during homing

// ---------------------------------------------------------------------------
// Sony FDR-AX53 IR Zoom Configuration (40 kHz, 15-bit SIRC)
// ---------------------------------------------------------------------------
#define SONY_IR_CARRIER_HZ      40000 // Sony standard carrier frequency (40 kHz, 33% duty)
#define SONY_IR_ADDR_CAM        0xD9  // Sony Camcorder Address (0xD9)
#define SONY_IR_CMD_ZOOM_TELE   0x1A  // Zoom Telephoto / In  (0x1A)
#define SONY_IR_CMD_ZOOM_WIDE   0x1B  // Zoom Wide / Out      (0x1B)

// ---------------------------------------------------------------------------
// Dead-Reckoning Zoom Timing Model (Sony FDR-AX53 Optical Travel)
// ---------------------------------------------------------------------------
// CALIBRATION: Standard Sony FDR-AX53 optical travel time from 0% (Wide) to 100% (Tele)
// is ~3500 ms (3.5s) at the 45ms Sony repeat rate.
// To fine-tune for your specific setup: adjust ZOOM_FULL_RANGE_MS to your measured travel time!
#define ZOOM_FULL_RANGE_MS     13000  // Full optical zoom travel in ms (0 = Wide, 13000 = Tele ~ 13.0s)
#define ZOOM_FRAME_MS             45  // Sony SIRC repeat period (25.5ms packet + 19.5ms gap)
#define ZOOM_TICK_MS   ZOOM_FRAME_MS  // Cadence tick step for dead-reckoning (45 ms)
#define ZOOM_HOMING_TIME_MS    14500  // Continuous Wide drive duration during startup homing (14.5s)
#define ZOOM_ACTIVE_WINDOW_MS    150  // Active window for fast telemetry updates during zoom
#define ZOOM_FAST_TELEMETRY_MS   100  // Interval for fast telemetry while zooming

// Zoom-Adaptive Dynamic Speed Scaling (Focal Length Adaptive)
#define ZOOM_SPEED_SCALE_MIN    0.20f // At 100% Tele, Pan/Tilt max speed scales down to 20% of normal
#define ZOOM_ACCEL_SCALE_MIN    0.35f // At 100% Tele, Pan/Tilt acceleration scales down to 35% of normal

// LEDC PWM Channel for IR Carrier
const uint8_t IR_LEDC_CH = 0;
const uint8_t IR_DUTY_33 = 85; // 33% duty cycle of 255

// ---------------------------------------------------------------------------
// ESP-NOW Packet Protocol (KEEP IN SYNC with ESP32S3_Remote_TX)
// ---------------------------------------------------------------------------
#define PKT_PAIR_REQ      0x01
#define PKT_PAIR_ACK      0x02
#define PKT_CONTROL       0x03
#define PKT_SAVE_PRESET   0x04
#define PKT_GOTO_PRESET   0x05
#define PKT_START_HOMING  0x06
#define PKT_TELEMETRY     0x07

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
  char     eventMsg[32];          // Null-terminated event string (e.g. "Pan Hall Tripped")
};

// ---------------------------------------------------------------------------
// Preset Structure & Storage
// ---------------------------------------------------------------------------
#define MAX_PRESETS 8

struct Preset {
  bool     valid;
  int32_t  pan;    // Step position
  int32_t  tilt;   // Step position
  uint32_t zoom;   // Milliseconds from Wide (0..3500)
};

Preset presets[MAX_PRESETS] = {};
Preferences prefs;

// ---------------------------------------------------------------------------
// Global Objects & State
// ---------------------------------------------------------------------------
// TMC2209 UART Drivers (HardwareSerial via TMCStepper)
TMC2209Stepper driverPan(&Serial1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverTilt(&Serial2, R_SENSE, DRIVER_ADDRESS);

// FastAccelStepper Motion Engine
FastAccelStepperEngine engine;
FastAccelStepper* panStepper  = nullptr;
FastAccelStepper* tiltStepper = nullptr;

struct MotorState {
  int8_t   dir;                 // -1 / 0 / +1 currently commanded direction
  uint32_t spd;                 // currently commanded speed (steps/s)
};
MotorState panState  = {0, 0};
MotorState tiltState = {0, 0};

// Virtual Zoom State (Dead Reckoning)
int32_t  currentZoomMs       = 0;  // 0 = Full Wide, ZOOM_FULL_RANGE_MS = Full Tele
int32_t  targetZoomMs        = 0;
bool     isZoomHomed         = false;
bool     isPanHomed          = false;
bool     isTiltHomed         = false;
bool     isHomingInProgress  = false;
bool     isMovingToPreset    = false;
uint8_t  currentTargetPreset = 0;

// Center Direction Memory: -1 = Left/Down of center, +1 = Right/Up of center, 0 = Center
int8_t   lastPanSide  = 0;
int8_t   lastTiltSide = 0;

// Hardware & Driver Health Status
bool     panUartOk          = false;
bool     tiltUartOk         = false;
uint8_t  panDriverVersion   = 0;
uint8_t  tiltDriverVersion  = 0;
bool     wifiConnected      = false;

// ESP-NOW and Remote Link State
uint8_t          remoteMac[6] = {0};
bool             hasRemote    = false;
PtzPacket        lastPkt      = {};
volatile bool    rxFresh      = false;
uint32_t         lastRxMs     = 0;
bool             linkAlive    = false;

// ---------------------------------------------------------------------------
// Hardware LEDC 40 kHz IR Carrier Generator
// ---------------------------------------------------------------------------
void initIrLedc() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttachChannel(PIN_IR_IN, SONY_IR_CARRIER_HZ, 8, IR_LEDC_CH);
  ledcWrite(PIN_IR_IN, 0);
#else
  ledcSetup(IR_LEDC_CH, SONY_IR_CARRIER_HZ, 8);
  ledcAttachPin(PIN_IR_IN, IR_LEDC_CH);
  ledcWrite(IR_LEDC_CH, 0);
#endif
}

inline void irCarrierOn() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_IR_IN, IR_DUTY_33);
#else
  ledcWrite(IR_LEDC_CH, IR_DUTY_33);
#endif
}

inline void irCarrierOff() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_IR_IN, 0);
#else
  ledcWrite(IR_LEDC_CH, 0);
#endif
}

// Transmit single 15-bit Sony SIRC packet (40 kHz modulated)
// Timings matched from genuine Sony FDR-AX53 remote captures:
// Header: 2450us ON, 550us OFF
// Bit 1 : 1250us ON, 550us OFF
// Bit 0 :  650us ON, 550us OFF
void sendSony15Packet(uint8_t address, uint8_t command) {
  noInterrupts(); // Ensure zero pulse jitter during 25ms packet

  // Header: 2450us mark, 550us space
  irCarrierOn();
  delayMicroseconds(2450);
  irCarrierOff();
  delayMicroseconds(550);

  // 7-bit Command (LSB first)
  for (uint8_t i = 0; i < 7; i++) {
    irCarrierOn();
    if ((command >> i) & 1) {
      delayMicroseconds(1250);
    } else {
      delayMicroseconds(650);
    }
    irCarrierOff();
    delayMicroseconds(550);
  }

  // 8-bit Address (LSB first)
  for (uint8_t i = 0; i < 8; i++) {
    irCarrierOn();
    if ((address >> i) & 1) {
      delayMicroseconds(1250);
    } else {
      delayMicroseconds(650);
    }
    irCarrierOff();
    delayMicroseconds(550);
  }

  interrupts();
}

// ---------------------------------------------------------------------------
// Non-Blocking Sony SIRC IR Transmitter (Dedicated Task + Precision Cadence)
// ---------------------------------------------------------------------------
// Pinned to Core 0 at priority 1 to keep Core 1 100% dedicated to FastAccelStepper.
// Uses canonical FreeRTOS vTaskDelayUntil for strictly deterministic 45.0 ms start-to-start timing.
volatile uint8_t  g_irActiveCmd     = 0;
volatile uint32_t g_irActiveUntilMs = 0;
volatile uint32_t irLastCmdMs       = 0;

void irTransmitTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(45); // Exact 45 ms period

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t nowMs = millis();
    if (g_irActiveCmd != 0 && nowMs < g_irActiveUntilMs) {
      sendSony15Packet(SONY_IR_ADDR_CAM, g_irActiveCmd);
      irLastCmdMs = millis();
    }
  }
}

// Backward-compatible non-blocking queue/command trigger
bool queueIrCommand(uint8_t cmd) {
  g_irActiveCmd = cmd;
  g_irActiveUntilMs = millis() + 180;
  return true;
}

// ---------------------------------------------------------------------------
// Dead-Reckoning Zoom Engine (Direct Wall-Clock Integration & Soft Limits)
// ---------------------------------------------------------------------------
void homeZoom() {
  Serial.printf("\n[Zoom Homing] Starting: Driving Wide to physical lens stop (%u ms)...\n", ZOOM_HOMING_TIME_MS);
  g_irActiveCmd = SONY_IR_CMD_ZOOM_WIDE;
  g_irActiveUntilMs = millis() + ZOOM_HOMING_TIME_MS;
  delay(ZOOM_HOMING_TIME_MS);
  g_irActiveCmd = 0;
  g_irActiveUntilMs = 0;
  currentZoomMs = 0;
  isZoomHomed = true;
  Serial.println("[Zoom Homing] Completed! Calibrated at 0.0% Wide (0 ms).\n");
}

// Handle real-time manual zoom commands from joystick (Sony 45 ms framing via IR TX task)
void applyZoom(int16_t cmd) {
  static uint32_t lastZoomActiveMs = 0;
  uint32_t now = millis();

  if (abs(cmd) < JOY_DEAD) {
    lastZoomActiveMs = 0;
    return; // Center deadzone -> idle (let in-flight 180ms window cleanly expire)
  }

  bool zoomIn = (cmd > 0);

  // Soft Limit Guard: Prevent driving past mechanical stops (0..ZOOM_FULL_RANGE_MS)
  if (zoomIn && currentZoomMs >= ZOOM_FULL_RANGE_MS) {
    lastZoomActiveMs = 0;
    g_irActiveCmd = 0;
    g_irActiveUntilMs = 0;
    return;
  }
  if (!zoomIn && currentZoomMs <= 0) {
    lastZoomActiveMs = 0;
    g_irActiveCmd = 0;
    g_irActiveUntilMs = 0;
    return;
  }

  // Calculate elapsed time while actively driving zoom
  uint32_t elapsedMs = 0;
  if (lastZoomActiveMs != 0) {
    elapsedMs = now - lastZoomActiveMs;
    if (elapsedMs > 100) elapsedMs = 100; // Guard against packet delay spikes
  }
  lastZoomActiveMs = now;

  // Signal continuous IR transmission: keep alive for 180 ms (auto-refreshed by 50 Hz control stream)
  uint8_t code = zoomIn ? SONY_IR_CMD_ZOOM_TELE : SONY_IR_CMD_ZOOM_WIDE;
  g_irActiveCmd = code;
  g_irActiveUntilMs = now + 180;

  // Dead-reckoning tracks exact elapsed wall-clock travel time (0..ZOOM_FULL_RANGE_MS)
  if (zoomIn) {
    currentZoomMs += elapsedMs;
    if (currentZoomMs > ZOOM_FULL_RANGE_MS) currentZoomMs = ZOOM_FULL_RANGE_MS;
  } else {
    currentZoomMs -= elapsedMs;
    if (currentZoomMs < 0) currentZoomMs = 0;
  }
}

// Coordinated Preset Zoom Smooth Transition (enqueues IR frames non-blocking)
void updatePresetZoomMove() {
  if (!isMovingToPreset) return;

  uint32_t now = millis();
  static uint32_t lastPresetStepMs = 0;

  uint32_t elapsedMs = 0;
  if (lastPresetStepMs != 0) {
    elapsedMs = now - lastPresetStepMs;
    if (elapsedMs > 100) elapsedMs = 100;
  }
  lastPresetStepMs = now;

  int32_t diff = (int32_t)targetZoomMs - (int32_t)currentZoomMs;

  if (abs(diff) <= 25) {
    currentZoomMs = targetZoomMs;
    lastPresetStepMs = 0;
    g_irActiveCmd = 0;
    g_irActiveUntilMs = 0;
    return;
  }

  bool zoomIn = (diff > 0);
  uint8_t code = zoomIn ? SONY_IR_CMD_ZOOM_TELE : SONY_IR_CMD_ZOOM_WIDE;
  g_irActiveCmd = code;
  g_irActiveUntilMs = now + 100;

  if (zoomIn) {
    currentZoomMs += elapsedMs;
    if (currentZoomMs > (int32_t)targetZoomMs) currentZoomMs = targetZoomMs;
  } else {
    currentZoomMs -= elapsedMs;
    if (currentZoomMs < (int32_t)targetZoomMs) currentZoomMs = targetZoomMs;
  }
}

// ---------------------------------------------------------------------------
// NVS Preset Management
// ---------------------------------------------------------------------------
void loadPresetsFromNVS() {
  prefs.begin("ptz_presets", true); // read-only
  for (uint8_t i = 0; i < MAX_PRESETS; i++) {
    char key[16];
    snprintf(key, sizeof(key), "p%u_v", i);
    presets[i].valid = prefs.getBool(key, false);
    if (presets[i].valid) {
      snprintf(key, sizeof(key), "p%u_pan", i);
      presets[i].pan = prefs.getInt(key, 0);
      snprintf(key, sizeof(key), "p%u_tilt", i);
      presets[i].tilt = prefs.getInt(key, 0);
      snprintf(key, sizeof(key), "p%u_zm", i);
      presets[i].zoom = prefs.getUInt(key, 0);
      Serial.printf("[NVS] Loaded Preset %u: Pan=%ld, Tilt=%ld, Zoom=%lu ms\n",
                    i + 1, presets[i].pan, presets[i].tilt, presets[i].zoom);
    }
  }
  prefs.end();
}

// ---------------------------------------------------------------------------
// Telemetry Transmission to Remote Controller
// ---------------------------------------------------------------------------
void sendMountTelemetry(const char* eventDesc = nullptr) {
  if (!hasRemote) return;

  MountTelemetryPacket telem = {};
  telem.type = PKT_TELEMETRY;
  WiFi.macAddress(telem.mac);

  telem.panUartOk = panUartOk ? 1 : 0;
  telem.panVersion = panDriverVersion;
  telem.tiltUartOk = tiltUartOk ? 1 : 0;
  telem.tiltVersion = tiltDriverVersion;

  telem.panHallDetected = (digitalRead(PIN_PAN_HALL) == LOW) ? 1 : 0;
  telem.tiltHallDetected = (digitalRead(PIN_TILT_HALL) == LOW) ? 1 : 0;

  int32_t curPan = panStepper ? panStepper->getCurrentPosition() : 0;
  int32_t curTilt = tiltStepper ? tiltStepper->getCurrentPosition() : 0;
  telem.panSteps = curPan;
  telem.tiltSteps = curTilt;

  telem.panDeg = (float)curPan / PAN_STEPS_PER_DEG;
  telem.tiltDeg = (float)curTilt / TILT_STEPS_PER_DEG;

  if (telem.panHallDetected || abs(curPan) < 50) {
    telem.panSide = 0; // At Center Hall
  } else if (curPan > 0) {
    telem.panSide = 1; // RIGHT / CW of Center Hall
  } else {
    telem.panSide = -1; // LEFT / CCW of Center Hall
  }

  if (telem.tiltHallDetected || abs(curTilt) < 25) {
    telem.tiltSide = 0; // At Center Hall
  } else if (curTilt > 0) {
    telem.tiltSide = 1; // UP of Center Hall
  } else {
    telem.tiltSide = -1; // DOWN of Center Hall
  }

  telem.zoomMs = (uint16_t)currentZoomMs;
  telem.zoomPct = (uint8_t)((currentZoomMs * 100) / ZOOM_FULL_RANGE_MS);
  telem.isHomed = (isPanHomed && isTiltHomed && isZoomHomed) ? 1 : 0;

  if (isHomingInProgress) telem.state = 2; // HOMING
  else if (isMovingToPreset) telem.state = 3; // GOTO_PRESET
  else if (panStepper && (panStepper->isRunning() || (tiltStepper && tiltStepper->isRunning()))) telem.state = 1; // LIVE_MOVING
  else telem.state = 0; // IDLE

  telem.activePreset = currentTargetPreset;

  if (eventDesc != nullptr) {
    strncpy(telem.eventMsg, eventDesc, sizeof(telem.eventMsg) - 1);
    telem.eventMsg[sizeof(telem.eventMsg) - 1] = '\0';
  } else {
    telem.eventMsg[0] = '\0';
  }

  esp_now_send(remoteMac, (uint8_t*)&telem, sizeof(telem));
}

void savePreset(uint8_t id) {
  if (id < 1 || id > MAX_PRESETS) return;
  uint8_t idx = id - 1;

  presets[idx].valid = true;
  presets[idx].pan   = panStepper  ? panStepper->getCurrentPosition()  : 0;
  presets[idx].tilt  = tiltStepper ? tiltStepper->getCurrentPosition() : 0;
  presets[idx].zoom  = currentZoomMs;

  prefs.begin("ptz_presets", false); // read-write
  char key[16];
  snprintf(key, sizeof(key), "p%u_v", idx);
  prefs.putBool(key, true);
  snprintf(key, sizeof(key), "p%u_pan", idx);
  prefs.putInt(key, presets[idx].pan);
  snprintf(key, sizeof(key), "p%u_tilt", idx);
  prefs.putInt(key, presets[idx].tilt);
  snprintf(key, sizeof(key), "p%u_zm", idx);
  prefs.putUInt(key, presets[idx].zoom);
  prefs.end();

  Serial.printf("\n>>> [PRESET SAVED] Preset %u: Pan=%ld, Tilt=%ld, Zoom=%lu ms (%0.1f%%) <<<\n\n",
                id, presets[idx].pan, presets[idx].tilt, presets[idx].zoom,
                (float)presets[idx].zoom / ZOOM_FULL_RANGE_MS * 100.0f);

  char evt[32];
  snprintf(evt, sizeof(evt), "Preset %u Saved to NVS", id);
  sendMountTelemetry(evt);
}

void gotoPreset(uint8_t id) {
  if (id < 1 || id > MAX_PRESETS) return;
  uint8_t idx = id - 1;

  if (!presets[idx].valid) {
    Serial.printf("[PRESET ERROR] Preset %u is unprogrammed!\n", id);
    char evt[32];
    snprintf(evt, sizeof(evt), "Preset %u Unprogrammed!", id);
    sendMountTelemetry(evt);
    return;
  }

  currentTargetPreset = id;
  int32_t targetPan = presets[idx].pan;
  int32_t targetTilt = presets[idx].tilt;

  int32_t curPan = panStepper ? panStepper->getCurrentPosition() : 0;
  int32_t curTilt = tiltStepper ? tiltStepper->getCurrentPosition() : 0;

  int32_t deltaPan = abs(targetPan - curPan);
  int32_t deltaTilt = abs(targetTilt - curTilt);
  int32_t maxDelta = max(deltaPan, deltaTilt);

  Serial.printf("\n>>> [COORDINATED GOTO P%u] Pan: %ld->%ld (d=%ld) | Tilt: %ld->%ld (d=%ld) | Zoom: %lu ms <<<\n",
                id, curPan, targetPan, deltaPan, curTilt, targetTilt, deltaTilt, presets[idx].zoom);

  if (maxDelta > 0) {
    // Proportional speed and acceleration scaling: Both steppers start, cruise, and arrive simultaneously with smooth cinematic ramps!
    uint32_t panSpeed = map(deltaPan, 0, maxDelta, PRESET_MIN_SPEED_HZ, PRESET_MAX_SPEED_HZ);
    uint32_t tiltSpeed = map(deltaTilt, 0, maxDelta, PRESET_MIN_SPEED_HZ, PRESET_MAX_SPEED_HZ);

    uint32_t panAccel = map(deltaPan, 0, maxDelta, 1000, PRESET_ACCEL_HZ_S);
    uint32_t tiltAccel = map(deltaTilt, 0, maxDelta, 1000, PRESET_ACCEL_HZ_S);

    if (panStepper) {
      panStepper->setSpeedInHz(max((uint32_t)150, panSpeed));
      panStepper->setAcceleration(max((uint32_t)1000, panAccel));
      panStepper->applySpeedAcceleration();
      panStepper->moveTo(targetPan);
    }

    if (tiltStepper) {
      tiltStepper->setSpeedInHz(max((uint32_t)150, tiltSpeed));
      tiltStepper->setAcceleration(max((uint32_t)1000, tiltAccel));
      tiltStepper->applySpeedAcceleration();
      tiltStepper->moveTo(targetTilt);
    }
  }

  // Zoom move (clamp to calibrated optical range, so stale presets can never over-drive)
  targetZoomMs = min((uint32_t)presets[idx].zoom, (uint32_t)ZOOM_FULL_RANGE_MS);
  isMovingToPreset = true;

  char evt[32];
  snprintf(evt, sizeof(evt), "Moving to Preset %u...", id);
  sendMountTelemetry(evt);
}

// ---------------------------------------------------------------------------
// Center-Memory NVS Tracker
// ---------------------------------------------------------------------------
void loadPanSideNVS() {
  prefs.begin("ptz_state", true);
  lastPanSide = prefs.getChar("pan_side", 0);
  prefs.end();
  Serial.printf("[NVS] Loaded last known Pan side from center: %d (%s)\n",
                lastPanSide, (lastPanSide > 0) ? "RIGHT" : ((lastPanSide < 0) ? "LEFT" : "CENTER"));
}

void savePanSideNVS(int8_t side) {
  if (side == lastPanSide) return;
  lastPanSide = side;
  prefs.begin("ptz_state", false);
  prefs.putChar("pan_side", side);
  prefs.end();
}

void loadTiltSideNVS() {
  prefs.begin("ptz_state", true);
  lastTiltSide = prefs.getChar("tilt_side", 0);
  prefs.end();
  Serial.printf("[NVS] Loaded last known Tilt side from center: %d (%s)\n",
                lastTiltSide, (lastTiltSide > 0) ? "UP" : ((lastTiltSide < 0) ? "DOWN" : "CENTER"));
}

void saveTiltSideNVS(int8_t side) {
  if (side == lastTiltSide) return;
  lastTiltSide = side;
  prefs.begin("ptz_state", false);
  prefs.putChar("tilt_side", side);
  prefs.end();
}

// ---------------------------------------------------------------------------
// TMC2209 Initialization via UART with Diagnostic Logging
// ---------------------------------------------------------------------------
bool setupTMC2209(TMC2209Stepper& driver, const char* name, uint8_t txPin, uint8_t rxPin, uint16_t current_mA) {
  driver.begin();
  delay(50);

  uint8_t version = driver.version();
  if (version == 0x00 || version == 0xFF) {
    delay(100);
    version = driver.version();
  }

  bool isOk = (version == 0x21);
  if (strcmp(name, "Pan") == 0) {
    panDriverVersion = version;
    panUartOk = isOk;
  } else {
    tiltDriverVersion = version;
    tiltUartOk = isOk;
  }

  Serial.printf("\n[TMC2209 DIAG] === %s Stepper Driver (TX: GPIO %d, RX: GPIO %d) ===\n", name, txPin, rxPin);
  Serial.printf("  IC Version Register      : 0x%02X\n", version);
  Serial.flush();

  if (version == 0x21) {
    Serial.printf("  [PASS] %s driver version 0x21 verified (TMC2209 authentic silicon OK)!\n", name);
  } else {
    Serial.printf("  [WARNING] %s driver returned version 0x%02X (Expected 0x21).\n", name, version);
  }

  uint8_t ifcnt_before = driver.IFCNT();

  driver.toff(5);                        // Enable driver power stage (TOFF = 5)
  driver.rms_current(current_mA, 0.90f); // 900 mA RMS with 90% standstill hold ratio
  driver.ihold(28);                      // 28/31 (~90%) holding current to prevent motor heating
  driver.irun(31);                       // 31/31 full running current
  driver.iholddelay(2);                  // Settle smoothly to standstill current
  driver.pwm_ofs(160);                   // Optimized standstill voltage offset for cool & quiet holding
  driver.pwm_autoscale(true);            // Automatic current scaling
  driver.pwm_autograd(true);             // Automatic gradient adaptation
  driver.en_spreadCycle(false);          // STRICTLY StealthChop2 enabled for silent operation
  driver.mstep_reg_select(true);         // Microstepping configured via UART register
  driver.microsteps(MICROSTEPS_VAL);     // 1/16 microstepping
  driver.blank_time(24);                 // Comparator blank time
  driver.GSTAT(7);                       // Clear reset, drv_err, and uv_cp error flags (write-1-to-clear)

  uint8_t ifcnt_after = driver.IFCNT();
  uint8_t cs = driver.cs_actual();

  Serial.printf("  IFCNT Transmission Writes: %u -> %u (%s)\n", 
                ifcnt_before, ifcnt_after, (ifcnt_after != ifcnt_before) ? "ACCEPTED" : "NO ACK");
  Serial.printf("  Holding / Run Current    : %u mA (CS_ACTUAL: %u/31, 0.90A / 90%% Hold Ratio Active)\n", current_mA, cs);
  Serial.printf("  StealthChop2 Silent Mode : %s (pwm_ofs=180 high-torque standstill active)\n", driver.stealth() ? "ACTIVE" : "SpreadCycle");
  Serial.printf("[TMC2209] %s initialization completed.\n\n", name);

  return (version == 0x21);
}

// ---------------------------------------------------------------------------
// FastAccelStepper Setup
// ---------------------------------------------------------------------------
FastAccelStepper* setupStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin) {
  FastAccelStepper* s = engine.stepperConnectToPin(stepPin);
  if (!s) {
    Serial.printf("ERROR: no stepper resource for STEP pin %d\n", stepPin);
    return nullptr;
  }
  s->setDirectionPin(dirPin);
  s->setEnablePin(enPin);        // TMC2209 EN is active LOW
  s->setAcceleration(ACCEL_HZ_S);
  s->enableOutputs();            // Maintain energized state for payload holding torque
  return s;
}

// ---------------------------------------------------------------------------
// Homing Abort Check Helper (Detects Deliberate Joystick Override)
// ---------------------------------------------------------------------------
bool checkHomingAbort() {
  if (abs(lastPkt.pan) > 600 || abs(lastPkt.tilt) > 600) {
    Serial.println("\n[HOMING ABORT] Manual joystick deflection detected! Aborting homing immediately.");
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Bulletproof Precision Simultaneous 2-Axis Homing Engine
// ---------------------------------------------------------------------------
bool homeAxesSimultaneous(int32_t tiltLevelOffset) {
  if (!panStepper || !tiltStepper) return false;

  Serial.println("\n=======================================================");
  Serial.println("[HOMING START] Initializing Simultaneous Precision Calibration");
  Serial.println("=======================================================");

  // Ensure outputs are energized
  panStepper->enableOutputs();
  tiltStepper->enableOutputs();
  panStepper->setAcceleration(HOMING_ACCEL_HZ_S);
  tiltStepper->setAcceleration(HOMING_ACCEL_HZ_S);

  // -------------------------------------------------------------------------
  // Stage 0: Pre-Clear (Back off if already sitting on Hall magnet)
  // -------------------------------------------------------------------------
  bool panOnSensor = (digitalRead(PIN_PAN_HALL) == LOW);
  bool tiltOnSensor = (digitalRead(PIN_TILT_HALL) == LOW);

  if (panOnSensor || tiltOnSensor) {
    Serial.printf("[Stage 0: Pre-Clear] Starting: PanPos=%ld (Hall=%s), TiltPos=%ld (Hall=%s)\n",
                  panStepper->getCurrentPosition(), panOnSensor ? "ON (LOW)" : "CLEAR (HIGH)",
                  tiltStepper->getCurrentPosition(), tiltOnSensor ? "ON (LOW)" : "CLEAR (HIGH)");

    if (panOnSensor) {
      panStepper->setSpeedInHz(600);
      panStepper->applySpeedAcceleration();
      panStepper->runBackward();
    }
    if (tiltOnSensor) {
      tiltStepper->setSpeedInHz(600);
      tiltStepper->applySpeedAcceleration();
      tiltStepper->runBackward();
    }

    uint32_t backoffStart = millis();
    bool panDone = !panOnSensor;
    bool tiltDone = !tiltOnSensor;
    int32_t panClearPos = panStepper->getCurrentPosition();
    int32_t tiltClearPos = tiltStepper->getCurrentPosition();
    bool panSawHigh = !panOnSensor;
    bool tiltSawHigh = !tiltOnSensor;

    while ((!panDone || !tiltDone) && (millis() - backoffStart < 8000)) {
      if (checkHomingAbort()) {
        panStepper->forceStop();
        tiltStepper->forceStop();
        return false;
      }

      if (!panDone) {
        if (!panSawHigh && digitalRead(PIN_PAN_HALL) == HIGH) {
          panSawHigh = true;
          panClearPos = panStepper->getCurrentPosition();
        }
        if (panSawHigh && abs(panStepper->getCurrentPosition() - panClearPos) >= HOMING_BACKOFF_STEPS) {
          panStepper->forceStop();
          panDone = true;
        }
      }

      if (!tiltDone) {
        if (!tiltSawHigh && digitalRead(PIN_TILT_HALL) == HIGH) {
          tiltSawHigh = true;
          tiltClearPos = tiltStepper->getCurrentPosition();
        }
        if (tiltSawHigh && abs(tiltStepper->getCurrentPosition() - tiltClearPos) >= HOMING_BACKOFF_STEPS) {
          tiltStepper->forceStop();
          tiltDone = true;
        }
      }
      delay(1);
    }
    panStepper->forceStop();
    tiltStepper->forceStop();
    delay(150);
    Serial.printf("[Stage 0 Done] PanPos=%ld (Hall=%s), TiltPos=%ld (Hall=%s)\n",
                  panStepper->getCurrentPosition(), (digitalRead(PIN_PAN_HALL) == LOW) ? "LOW" : "HIGH",
                  tiltStepper->getCurrentPosition(), (digitalRead(PIN_TILT_HALL) == LOW) ? "LOW" : "HIGH");
  }

  // -------------------------------------------------------------------------
  // Stage 1: Simultaneous Fast Search using NVS Initial Guess
  // -------------------------------------------------------------------------
  int8_t panDir = (lastPanSide > 0) ? -1 : 1;
  int8_t tiltDir = (lastTiltSide > 0) ? -1 : 1;

  Serial.printf("\n[Stage 1: Fast Search] Starting @ %u Hz (PanDir=%d, TiltDir=%d) | PanPos=%ld, TiltPos=%ld\n",
                HOMING_SEARCH_SPEED_HZ, panDir, tiltDir,
                panStepper->getCurrentPosition(), tiltStepper->getCurrentPosition());

  panStepper->setSpeedInHz(HOMING_SEARCH_SPEED_HZ);
  panStepper->applySpeedAcceleration();
  if (panDir > 0) panStepper->runForward();
  else            panStepper->runBackward();

  tiltStepper->setSpeedInHz(HOMING_SEARCH_SPEED_HZ);
  tiltStepper->applySpeedAcceleration();
  if (tiltDir > 0) tiltStepper->runForward();
  else             tiltStepper->runBackward();

  // Verification move check: ensure motors actually start moving
  delay(100);
  int32_t panVerifyPos = panStepper->getCurrentPosition();
  int32_t tiltVerifyPos = tiltStepper->getCurrentPosition();
  delay(100);
  if (abs(panStepper->getCurrentPosition() - panVerifyPos) == 0 && digitalRead(PIN_PAN_HALL) == HIGH) {
    Serial.println("[Stage 1 WARN] Pan stepper motion re-triggering...");
    panStepper->applySpeedAcceleration();
    if (panDir > 0) panStepper->runForward();
    else            panStepper->runBackward();
  }
  if (abs(tiltStepper->getCurrentPosition() - tiltVerifyPos) == 0 && digitalRead(PIN_TILT_HALL) == HIGH) {
    Serial.println("[Stage 1 WARN] Tilt stepper motion re-triggering...");
    tiltStepper->applySpeedAcceleration();
    if (tiltDir > 0) tiltStepper->runForward();
    else             tiltStepper->runBackward();
  }

  int32_t panStartPos = panStepper->getCurrentPosition();
  int32_t tiltStartPos = tiltStepper->getCurrentPosition();

  bool panFound = (digitalRead(PIN_PAN_HALL) == LOW);
  bool tiltFound = (digitalRead(PIN_TILT_HALL) == LOW);
  if (panFound) panStepper->stopMove();
  if (tiltFound) tiltStepper->stopMove();

  bool panReversed = false;
  bool tiltReversed = false;
  uint32_t searchStartMs = millis();
  uint32_t lastStage1LogMs = 0;

  while ((!panFound || !tiltFound) && (millis() - searchStartMs < 15000)) {
    uint32_t now = millis();
    if (checkHomingAbort()) {
      panStepper->forceStop();
      tiltStepper->forceStop();
      return false;
    }

    // Periodic continuous diagnostic logging (every 300ms)
    if (now - lastStage1LogMs >= 300) {
      lastStage1LogMs = now;
      Serial.printf("[Stage 1 Active] %lums: PanPos=%ld (Hall=%s), TiltPos=%ld (Hall=%s)\n",
                    now - searchStartMs,
                    panStepper->getCurrentPosition(), (digitalRead(PIN_PAN_HALL) == LOW) ? "TRIP" : "OK",
                    tiltStepper->getCurrentPosition(), (digitalRead(PIN_TILT_HALL) == LOW) ? "TRIP" : "OK");
    }

    // Check Pan Hall
    if (!panFound) {
      if (digitalRead(PIN_PAN_HALL) == LOW) {
        panStepper->stopMove();
        panFound = true;
        Serial.printf("[Stage 1] Pan tripped Hall at Pos=%ld!\n", panStepper->getCurrentPosition());
      } else {
        int32_t panDist = abs(panStepper->getCurrentPosition() - panStartPos);
        uint32_t panLimit = panReversed ? (MAX_PAN_STEPS * 2) : MAX_PAN_STEPS;
        if (panDist >= (int32_t)panLimit) {
          if (!panReversed) {
            panReversed = true;
            panDir = -panDir;
            panStartPos = panStepper->getCurrentPosition();
            Serial.printf("[Stage 1] Pan reversing search sweep at Pos=%ld...\n", panStartPos);
            panStepper->setSpeedInHz(HOMING_SEARCH_SPEED_HZ);
            panStepper->applySpeedAcceleration();
            if (panDir > 0) panStepper->runForward();
            else            panStepper->runBackward();
          } else {
            panStepper->forceStop();
            Serial.println("[Stage 1 ERROR] Pan sweep exceeded limits!");
            break;
          }
        }
      }
    }

    // Check Tilt Hall
    if (!tiltFound) {
      if (digitalRead(PIN_TILT_HALL) == LOW) {
        tiltStepper->stopMove();
        tiltFound = true;
        Serial.printf("[Stage 1] Tilt tripped Hall at Pos=%ld!\n", tiltStepper->getCurrentPosition());
      } else {
        int32_t tiltDist = abs(tiltStepper->getCurrentPosition() - tiltStartPos);
        uint32_t tiltLimit = tiltReversed ? (MAX_TILT_STEPS * 2) : MAX_TILT_STEPS;
        if (tiltDist >= (int32_t)tiltLimit) {
          if (!tiltReversed) {
            tiltReversed = true;
            tiltDir = -tiltDir;
            tiltStartPos = tiltStepper->getCurrentPosition();
            Serial.printf("[Stage 1] Tilt reversing search sweep at Pos=%ld...\n", tiltStartPos);
            tiltStepper->setSpeedInHz(HOMING_SEARCH_SPEED_HZ);
            tiltStepper->applySpeedAcceleration();
            if (tiltDir > 0) tiltStepper->runForward();
            else             tiltStepper->runBackward();
          } else {
            tiltStepper->forceStop();
            Serial.println("[Stage 1 ERROR] Tilt sweep exceeded limits!");
            break;
          }
        }
      }
    }

    delay(1);
  }

  while (panStepper->isRunning() || tiltStepper->isRunning()) delay(1);

  if (!panFound || !tiltFound) {
    panStepper->forceStop();
    tiltStepper->forceStop();
    Serial.printf("[Stage 1 FAILED] PanFound=%s (Pos=%ld), TiltFound=%s (Pos=%ld)\n",
                  panFound ? "YES" : "NO", panStepper->getCurrentPosition(),
                  tiltFound ? "YES" : "NO", tiltStepper->getCurrentPosition());
    return false;
  }

  delay(150);
  Serial.printf("[Stage 1 Complete] PanPos=%ld, TiltPos=%ld\n",
                panStepper->getCurrentPosition(), tiltStepper->getCurrentPosition());

  // -------------------------------------------------------------------------
  // Stage 2: Negative-Side Positioning (Guarantee Identical Final Approach)
  // -------------------------------------------------------------------------
  // Move both axes backward (-1 direction) until the Hall sensor is HIGH
  // AND continue for HOMING_BACKOFF_STEPS (350 steps) past the release point.
  Serial.printf("\n[Stage 2: Negative Positioning] Starting from PanPos=%ld, TiltPos=%ld\n",
                panStepper->getCurrentPosition(), tiltStepper->getCurrentPosition());

  panStepper->setSpeedInHz(600);
  panStepper->applySpeedAcceleration();
  panStepper->runBackward();

  tiltStepper->setSpeedInHz(600);
  tiltStepper->applySpeedAcceleration();
  tiltStepper->runBackward();

  bool panReleased = (digitalRead(PIN_PAN_HALL) == HIGH);
  int32_t panReleasePos = panStepper->getCurrentPosition();
  bool tiltReleased = (digitalRead(PIN_TILT_HALL) == HIGH);
  int32_t tiltReleasePos = tiltStepper->getCurrentPosition();

  bool panRelDone = false;
  bool tiltRelDone = false;
  uint32_t relStart = millis();
  uint32_t lastStage2LogMs = 0;

  while ((!panRelDone || !tiltRelDone) && (millis() - relStart < 8000)) {
    uint32_t now = millis();
    if (checkHomingAbort()) {
      panStepper->forceStop();
      tiltStepper->forceStop();
      return false;
    }

    if (now - lastStage2LogMs >= 300) {
      lastStage2LogMs = now;
      Serial.printf("[Stage 2 Active] %lums: PanPos=%ld (Hall=%s, Rel=%s), TiltPos=%ld (Hall=%s, Rel=%s)\n",
                    now - relStart,
                    panStepper->getCurrentPosition(), (digitalRead(PIN_PAN_HALL) == LOW) ? "LOW" : "HIGH", panReleased ? "YES" : "NO",
                    tiltStepper->getCurrentPosition(), (digitalRead(PIN_TILT_HALL) == LOW) ? "LOW" : "HIGH", tiltReleased ? "YES" : "NO");
    }

    if (!panRelDone) {
      if (!panReleased && digitalRead(PIN_PAN_HALL) == HIGH) {
        panReleased = true;
        panReleasePos = panStepper->getCurrentPosition();
        Serial.printf("[Stage 2] Pan exited magnet (HIGH) at Pos=%ld\n", panReleasePos);
      }
      if (panReleased && (abs(panStepper->getCurrentPosition() - panReleasePos) >= HOMING_BACKOFF_STEPS)) {
        panStepper->forceStop();
        panRelDone = true;
        Serial.printf("[Stage 2] Pan reached negative clearance at Pos=%ld (Backoff=%ld stp)\n",
                      panStepper->getCurrentPosition(), abs(panStepper->getCurrentPosition() - panReleasePos));
      }
    }

    if (!tiltRelDone) {
      if (!tiltReleased && digitalRead(PIN_TILT_HALL) == HIGH) {
        tiltReleased = true;
        tiltReleasePos = tiltStepper->getCurrentPosition();
        Serial.printf("[Stage 2] Tilt exited magnet (HIGH) at Pos=%ld\n", tiltReleasePos);
      }
      if (tiltReleased && (abs(tiltStepper->getCurrentPosition() - tiltReleasePos) >= HOMING_BACKOFF_STEPS)) {
        tiltStepper->forceStop();
        tiltRelDone = true;
        Serial.printf("[Stage 2] Tilt reached negative clearance at Pos=%ld (Backoff=%ld stp)\n",
                      tiltStepper->getCurrentPosition(), abs(tiltStepper->getCurrentPosition() - tiltReleasePos));
      }
    }
    delay(1);
  }
  panStepper->forceStop();
  tiltStepper->forceStop();
  delay(150);

  bool panCleared = (digitalRead(PIN_PAN_HALL) == HIGH);
  bool tiltCleared = (digitalRead(PIN_TILT_HALL) == HIGH);

  Serial.printf("[Stage 2 Complete] PanPos=%ld (Hall=%s), TiltPos=%ld (Hall=%s)\n",
                panStepper->getCurrentPosition(), panCleared ? "CLEAR (HIGH)" : "TRIP (LOW)",
                tiltStepper->getCurrentPosition(), tiltCleared ? "CLEAR (HIGH)" : "TRIP (LOW)");

  if (!panCleared || !tiltCleared) {
    Serial.println("[Stage 2 FAILED] Sensors not clear on negative side!");
    return false;
  }

  // -------------------------------------------------------------------------
  // Stage 3: Simultaneous Precision Creep (+1 Positive Direction @ 250 Hz)
  // -------------------------------------------------------------------------
  // Always approaches from the negative side to positive side.
  // 15-second timeout allows up to 3750 steps of travel at 250 Hz.
  Serial.printf("\n[Stage 3: Precision Creep] Creep Starting (+1 Forward @ %u Hz) from PanPos=%ld, TiltPos=%ld\n",
                HOMING_CREEP_SPEED_HZ, panStepper->getCurrentPosition(), tiltStepper->getCurrentPosition());

  panStepper->setSpeedInHz(HOMING_CREEP_SPEED_HZ);
  panStepper->applySpeedAcceleration();
  panStepper->runForward();

  tiltStepper->setSpeedInHz(HOMING_CREEP_SPEED_HZ);
  tiltStepper->applySpeedAcceleration();
  tiltStepper->runForward();

  // Verification move check: ensure motors actually start moving in Stage 3
  delay(120);
  int32_t panCreepVerifyPos = panStepper->getCurrentPosition();
  int32_t tiltCreepVerifyPos = tiltStepper->getCurrentPosition();
  delay(120);
  if (abs(panStepper->getCurrentPosition() - panCreepVerifyPos) == 0 && digitalRead(PIN_PAN_HALL) == HIGH) {
    Serial.println("[Stage 3 WARN] Pan creep re-triggering forward command...");
    panStepper->applySpeedAcceleration();
    panStepper->runForward();
  }
  if (abs(tiltStepper->getCurrentPosition() - tiltCreepVerifyPos) == 0 && digitalRead(PIN_TILT_HALL) == HIGH) {
    Serial.println("[Stage 3 WARN] Tilt creep re-triggering forward command...");
    tiltStepper->applySpeedAcceleration();
    tiltStepper->runForward();
  }

  bool panLatched = false;
  bool tiltLatched = false;
  uint32_t latchStart = millis();
  uint32_t lastStage3LogMs = 0;
  int32_t lastPanCreepPos = panStepper->getCurrentPosition();
  int32_t lastTiltCreepPos = tiltStepper->getCurrentPosition();
  uint32_t lastPanMoveMs = millis();
  uint32_t lastTiltMoveMs = millis();

  while ((!panLatched || !tiltLatched) && (millis() - latchStart < 15000)) {
    uint32_t now = millis();
    if (checkHomingAbort()) {
      panStepper->forceStop();
      tiltStepper->forceStop();
      return false;
    }

    // Continuous real-time diagnostic logging every 250ms
    if (now - lastStage3LogMs >= 250) {
      lastStage3LogMs = now;
      Serial.printf("[Stage 3 Creep Active] %lums: PanPos=%ld (Hall=%s, Latch=%s), TiltPos=%ld (Hall=%s, Latch=%s)\n",
                    now - latchStart,
                    panStepper->getCurrentPosition(), (digitalRead(PIN_PAN_HALL) == LOW) ? "TRIP" : "OK", panLatched ? "YES" : "NO",
                    tiltStepper->getCurrentPosition(), (digitalRead(PIN_TILT_HALL) == LOW) ? "TRIP" : "OK", tiltLatched ? "YES" : "NO");
    }

    // Monitor movement progress to prevent silent stalls
    if (!panLatched) {
      int32_t curP = panStepper->getCurrentPosition();
      if (curP != lastPanCreepPos) {
        lastPanCreepPos = curP;
        lastPanMoveMs = now;
      } else if (now - lastPanMoveMs > 1000) {
        // Position hasn't changed in 1 second while still unlatched: re-assert runForward
        Serial.println("[Stage 3 Stall Guard] Re-issuing Pan runForward...");
        panStepper->applySpeedAcceleration();
        panStepper->runForward();
        lastPanMoveMs = now;
      }

      if (digitalRead(PIN_PAN_HALL) == LOW) {
        panStepper->forceStop();
        panLatched = true;
        Serial.printf("[Stage 3] >>> Pan Precision Latch ACQUIRED at Pos=%ld <<<\n", panStepper->getCurrentPosition());
      }
    }

    if (!tiltLatched) {
      int32_t curT = tiltStepper->getCurrentPosition();
      if (curT != lastTiltCreepPos) {
        lastTiltCreepPos = curT;
        lastTiltMoveMs = now;
      } else if (now - lastTiltMoveMs > 1000) {
        // Position hasn't changed in 1 second while still unlatched: re-assert runForward
        Serial.println("[Stage 3 Stall Guard] Re-issuing Tilt runForward...");
        tiltStepper->applySpeedAcceleration();
        tiltStepper->runForward();
        lastTiltMoveMs = now;
      }

      if (digitalRead(PIN_TILT_HALL) == LOW) {
        tiltStepper->forceStop();
        tiltLatched = true;
        Serial.printf("[Stage 3] >>> Tilt Precision Latch ACQUIRED at Pos=%ld <<<\n", tiltStepper->getCurrentPosition());
      }
    }
    delay(1);
  }

  panStepper->forceStop();
  tiltStepper->forceStop();

  if (!panLatched || !tiltLatched) {
    Serial.printf("[Stage 3 FAILED] Precision creep timeout! PanLatched=%s (Pos=%ld, Hall=%s), TiltLatched=%s (Pos=%ld, Hall=%s)\n",
                  panLatched ? "YES" : "NO", panStepper->getCurrentPosition(), (digitalRead(PIN_PAN_HALL) == LOW) ? "LOW" : "HIGH",
                  tiltLatched ? "YES" : "NO", tiltStepper->getCurrentPosition(), (digitalRead(PIN_TILT_HALL) == LOW) ? "LOW" : "HIGH");
    return false;
  }

  delay(150);

  // -------------------------------------------------------------------------
  // Stage 4: Apply Horizon Leveling Offset to Tilt
  // -------------------------------------------------------------------------
  if (tiltLevelOffset != 0) {
    Serial.printf("\n[Stage 4: Level Offset] Applying Tilt offset: %+ld steps from Pos=%ld...\n",
                  tiltLevelOffset, tiltStepper->getCurrentPosition());
    tiltStepper->setCurrentPosition(0);
    tiltStepper->setSpeedInHz(350);
    tiltStepper->applySpeedAcceleration();
    tiltStepper->moveTo(tiltLevelOffset);
    while (tiltStepper->isRunning()) {
      if (checkHomingAbort()) {
        tiltStepper->forceStop();
        return false;
      }
      delay(1);
    }
    delay(50);
    Serial.printf("[Stage 4 Done] Tilt Level Offset Position: %ld steps\n", tiltStepper->getCurrentPosition());
  }

  // -------------------------------------------------------------------------
  // Stage 5: Zero Coordinates & Complete Calibration
  // -------------------------------------------------------------------------
  panStepper->setCurrentPosition(0);
  tiltStepper->setCurrentPosition(0);

  panStepper->setSpeedInHz(MAX_SPEED_HZ);
  tiltStepper->setSpeedInHz(MAX_SPEED_HZ);
  panStepper->setAcceleration(ACCEL_HZ_S);
  tiltStepper->setAcceleration(ACCEL_HZ_S);
  panStepper->applySpeedAcceleration();
  tiltStepper->applySpeedAcceleration();

  savePanSideNVS(0);
  saveTiltSideNVS(0);

  Serial.println("\n=======================================================");
  Serial.println("[HOMING SUCCESS] Simultaneous 3-Axis Calibration Complete!");
  Serial.println("  Pan Axis  : 0.0 deg (Zero Reference Latched)");
  Serial.printf("  Tilt Axis : 0.0 deg (+%ld stp Level Offset Applied)\n", tiltLevelOffset);
  Serial.println("  Zoom Axis : 0.0% Wide (Physical Lens Stop Latched)");
  Serial.println("=======================================================\n");

  return true;
}

// FreeRTOS task for concurrent Zoom Homing (Runs in parallel on Core 0)
void zoomHomingTask(void* pvParameters) {
  Serial.printf("[Zoom Homing Task] Started: Driving Wide to physical lens stop (%u ms)...\n", ZOOM_HOMING_TIME_MS);
  g_irActiveCmd = SONY_IR_CMD_ZOOM_WIDE;
  g_irActiveUntilMs = millis() + ZOOM_HOMING_TIME_MS;

  uint32_t startMs = millis();
  while (millis() - startMs < ZOOM_HOMING_TIME_MS) {
    if (checkHomingAbort()) {
      g_irActiveCmd = 0;
      g_irActiveUntilMs = 0;
      vTaskDelete(NULL);
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  g_irActiveCmd = 0;
  g_irActiveUntilMs = 0;
  currentZoomMs = 0;
  isZoomHomed = true;
  Serial.println("[Zoom Homing Task] Completed! Calibrated at 0.0% Wide (0 ms).");
  vTaskDelete(NULL);
}

void homeAll() {
  Serial.println("\n=================================================");
  Serial.println(">>> INITIATING SIMULTANEOUS 3-AXIS HOMING ROUTINE <<<");
  Serial.println("=================================================");

  isHomingInProgress = true;
  isZoomHomed = false;
  sendMountTelemetry("Homing In Progress...");

  // 1. Launch Zoom Wide homing concurrently in background task on Core 0
  xTaskCreatePinnedToCore(
    zoomHomingTask,
    "ZoomHomeTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  // 2. Concurrently Home Both Stepper Axes in Parallel on Core 1
  bool steppersOk = homeAxesSimultaneous(TILT_LEVEL_OFFSET_STEPS);
  isPanHomed  = steppersOk;
  isTiltHomed = steppersOk;

  panState.dir = 0;
  panState.spd = 0;
  tiltState.dir = 0;
  tiltState.spd = 0;

  // 3. Wait for concurrent Zoom homing to conclude
  uint32_t waitStart = millis();
  while (!isZoomHomed && (millis() - waitStart < (ZOOM_HOMING_TIME_MS + 2000))) {
    delay(50);
  }

  isHomingInProgress = false;

  if (isPanHomed && isTiltHomed && isZoomHomed) {
    Serial.println(">>> SIMULTANEOUS HOMING COMPLETE: Pan, Tilt & Zoom Calibrated at Zero! <<<\n");
    savePanSideNVS(0);
    saveTiltSideNVS(0);
    sendMountTelemetry("Homing Complete (Centered)");
  } else {
    Serial.println(">>> [WARNING] Homing encountered issues or was aborted by user. <<<\n");
    sendMountTelemetry("Homing Aborted / Incomplete");
  }
}

// ---------------------------------------------------------------------------
// Velocity Command Mapping with Soft Travel Limits (+/-180 deg)
// ---------------------------------------------------------------------------
void applyVelocity(FastAccelStepper* st, int16_t cmd, MotorState& ms, int32_t maxLimitSteps, bool isPan) {
  if (!st) return;

  int32_t currentPos = st->getCurrentPosition();

  // Update center-direction side for Pan and Tilt axes in NVS
  if (isPan) {
    if (currentPos > 200)       savePanSideNVS(1);
    else if (currentPos < -200) savePanSideNVS(-1);
    else                        savePanSideNVS(0);
  } else {
    if (currentPos > 100)       saveTiltSideNVS(1);
    else if (currentPos < -100) saveTiltSideNVS(-1);
    else                        saveTiltSideNVS(0);
  }

  // Deadzone check
  if (abs(cmd) < JOY_DEAD) {
    if (ms.dir != 0) {
      st->stopMove();
      ms.dir = 0;
    }
    return;
  }

  int8_t dir = (cmd > 0) ? 1 : -1;

  // Dynamic Zoom-Dependent Speed & Acceleration Scaling:
  // When at full Wide (0% zoom), scale = 1.0 (full speed: MAX_SPEED_HZ, full accel: ACCEL_HZ_S).
  // As zoom increases toward Tele (100% zoom), pan & tilt speeds scale down smoothly
  // so telephoto framing is smooth, stable, and precise rather than twitchy.
  float zoomFraction = (float)currentZoomMs / (float)ZOOM_FULL_RANGE_MS;
  if (zoomFraction < 0.0f) zoomFraction = 0.0f;
  if (zoomFraction > 1.0f) zoomFraction = 1.0f;

  float speedScale = 1.0f - zoomFraction * (1.0f - ZOOM_SPEED_SCALE_MIN);
  float accelScale = 1.0f - zoomFraction * (1.0f - ZOOM_ACCEL_SCALE_MIN);

  uint32_t effectiveMaxSpeed = (uint32_t)(MAX_SPEED_HZ * speedScale);
  if (effectiveMaxSpeed < MIN_SPEED_HZ) effectiveMaxSpeed = MIN_SPEED_HZ;

  uint32_t effectiveAccel = (uint32_t)(ACCEL_HZ_S * accelScale);
  if (effectiveAccel < 3000) effectiveAccel = 3000;

  uint32_t spd = map(abs(cmd), JOY_DEAD, 1000, MIN_SPEED_HZ, effectiveMaxSpeed);

  // Soft Limit Check (+/-180 deg from center)
  if (dir > 0 && currentPos >= maxLimitSteps) {
    if (ms.dir != 0) {
      st->stopMove();
      ms.dir = 0;
    }
    return; // Block positive movement beyond limit
  }
  if (dir < 0 && currentPos <= -maxLimitSteps) {
    if (ms.dir != 0) {
      st->stopMove();
      ms.dir = 0;
    }
    return; // Block negative movement beyond limit
  }

  if (dir != ms.dir) {
    if (ms.dir != 0 && st->isRunning()) {
      st->forceStop();
    }
    st->setSpeedInHz(spd);
    st->setAcceleration(effectiveAccel);
    st->applySpeedAcceleration();
    if (dir > 0) st->runForward();
    else         st->runBackward();
    ms.dir = dir;
    ms.spd = spd;
  } else if (spd != ms.spd) {
    st->setSpeedInHz(spd);
    st->setAcceleration(effectiveAccel);
    st->applySpeedAcceleration();
    ms.spd = spd;
  }
}

void stopAllMotors() {
  if (panStepper) {
    panStepper->stopMove();
    panStepper->setSpeedInHz(MAX_SPEED_HZ);
    panStepper->setAcceleration(ACCEL_HZ_S);
    panStepper->applySpeedAcceleration();
  }
  if (tiltStepper) {
    tiltStepper->stopMove();
    tiltStepper->setSpeedInHz(MAX_SPEED_HZ);
    tiltStepper->setAcceleration(ACCEL_HZ_S);
    tiltStepper->applySpeedAcceleration();
  }
  panState.dir = 0;
  tiltState.dir = 0;
  isMovingToPreset = false;
  currentTargetPreset = 0;
  targetZoomMs = 0;   // P3: fully clear preset state so next command starts clean
  g_irActiveCmd = 0;  // Immediately halt any active IR zoom transmission
  g_irActiveUntilMs = 0;
}

// Queued asynchronous command flags (processed safely in loop() rather than Wi-Fi ISR)
volatile bool    reqHome       = false;
volatile uint8_t reqSavePreset = 0;
volatile uint8_t reqGotoPreset = 0;

// ---------------------------------------------------------------------------
// ESP-NOW Receive Callback (Fast & Non-Blocking)
// ---------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* senderMac = info->src_addr;
#else
void onEspNowRecv(const uint8_t* senderMac, const uint8_t* data, int len) {
#endif
  if (len != (int)sizeof(PtzPacket)) return;
  const PtzPacket* pkt = (const PtzPacket*)data;

  // Track remote MAC
  if (!hasRemote || memcmp(remoteMac, senderMac, 6) != 0) {
    memcpy(remoteMac, senderMac, 6);
    hasRemote = true;
    if (!esp_now_is_peer_exist(remoteMac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, remoteMac, 6);
      peer.channel = 0; // IMPORTANT: 0 uses active home channel
      peer.encrypt = false;
      peer.ifidx = WIFI_IF_STA;
      esp_now_add_peer(&peer);
    }
  }

  // Refresh link watchdog on ANY valid received packet from remote
  lastRxMs = millis();

  if (pkt->type == PKT_PAIR_REQ) {
    PtzPacket ack = {};
    ack.type = PKT_PAIR_ACK;
    WiFi.macAddress(ack.mac);
    esp_now_send(senderMac, (uint8_t*)&ack, sizeof(ack));
    sendMountTelemetry("Remote Handshake Established");
  } else if (pkt->type == PKT_CONTROL) {
    memcpy(&lastPkt, data, sizeof(lastPkt));
    rxFresh = true;
  } else if (pkt->type == PKT_SAVE_PRESET) {
    reqSavePreset = pkt->presetId;
  } else if (pkt->type == PKT_GOTO_PRESET) {
    reqGotoPreset = pkt->presetId;
  } else if (pkt->type == PKT_START_HOMING) {
    reqHome = true;
  }
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n====================================================================");
  Serial.println("   ESP32-S3 N16R8 PTZ CAMERA MOUNT FIRMWARE (UART RECEIVER & IR)    ");
  Serial.println("====================================================================");
  Serial.println(" Pin Configuration:");
  Serial.printf("   Pan Driver  : EN=%d, STEP=%d, DIR=%d | Serial1 (TX=%d, RX=%d)\n",
                PIN_PAN_EN, PIN_PAN_STEP, PIN_PAN_DIR, PIN_PAN_TX, PIN_PAN_RX);
  Serial.printf("   Tilt Driver : EN=%d, STEP=%d, DIR=%d | Serial2 (TX=%d, RX=%d)\n",
                PIN_TILT_EN, PIN_TILT_STEP, PIN_TILT_DIR, PIN_TILT_TX, PIN_TILT_RX);
  Serial.printf("   IR Transmit : GPIO %d (40 kHz Sony 15-bit SIRC)\n", PIN_IR_IN);
  Serial.printf("   Sensors     : Pan Hall=GPIO %d, Tilt Hall=GPIO %d\n", PIN_PAN_HALL, PIN_TILT_HALL);
  Serial.printf("   Travel Limit: Pan +/-180 deg (%u steps), Tilt +/-90 deg (%u steps)\n",
                MAX_PAN_STEPS, MAX_TILT_STEPS);
  Serial.println("====================================================================");

  // Power sequencing: Park driver enables inactive (HIGH)
  pinMode(PIN_PAN_EN, OUTPUT);
  pinMode(PIN_TILT_EN, OUTPUT);
  digitalWrite(PIN_PAN_EN, HIGH);
  digitalWrite(PIN_TILT_EN, HIGH);

  Serial.printf("Waiting %u ms for 12V rail & TMC2209 logic to stabilize...\n", PWR_SETTLE_DELAY_MS);
  delay(PWR_SETTLE_DELAY_MS);
  Serial.println("12V rail settled - configuring hardware interfaces");

  // Configure Hall sensors with internal pull-ups
  pinMode(PIN_PAN_HALL, INPUT_PULLUP);
  pinMode(PIN_TILT_HALL, INPUT_PULLUP);

  // Configure auxiliary pins
  pinMode(PIN_RELAY_IN, OUTPUT);
  digitalWrite(PIN_RELAY_IN, LOW);

  // Initialize Sony 40 kHz IR LEDC hardware peripheral
  initIrLedc();
  Serial.printf("IR Transmitter initialized on GPIO %d (40 kHz carrier)\n", PIN_IR_IN);

  // Start the non-blocking IR transmit task (Core 0, priority 1; deterministic 45ms vTaskDelayUntil)
  xTaskCreatePinnedToCore(irTransmitTask, "IrTxTask", 4096, NULL, 1, NULL, 0);
  delay(10);

  // Load NVS stored presets and center memory
  loadPresetsFromNVS();
  loadPanSideNVS();
  loadTiltSideNVS();

  // Initialize FastAccelStepper pulse generator
  engine.init();
  panStepper  = setupStepper(PIN_PAN_STEP, PIN_PAN_DIR, PIN_PAN_EN);
  tiltStepper = setupStepper(PIN_TILT_STEP, PIN_TILT_DIR, PIN_TILT_EN);

  // Initialize HardwareSerial ports for dual-channel 1k-resistor UART
  Serial1.begin(115200, SERIAL_8N1, PIN_PAN_RX, PIN_PAN_TX);
  Serial2.begin(115200, SERIAL_8N1, PIN_TILT_RX, PIN_TILT_TX);
  delay(100);

  // Initialize TMC2209 drivers via UART (StealthChop2, 950mA RMS, 100% full holding torque)
  setupTMC2209(driverPan,  "Pan",  PIN_PAN_TX,  PIN_PAN_RX,  RMS_CURRENT_MA);
  setupTMC2209(driverTilt, "Tilt", PIN_TILT_TX, PIN_TILT_RX, RMS_CURRENT_MA);

  // Perform full startup homing (stepper center search & zoom calibration)
  homeAll();

  // Wi-Fi: Attempt to join the home AP (5 second timeout)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Attempting Wi-Fi connection to '%s' (5s timeout)...", WIFI_SSID);
  uint32_t t0 = millis();
  wifiConnected = false;
  while (millis() - t0 < 5000) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      break;
    }
    Serial.print('.');
    delay(250);
  }

  if (wifiConnected) {
    Serial.printf("\n[Wi-Fi] Connected to AP! IP: %s, Channel: %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println("\n[Wi-Fi] No router connection found. Operating in Standalone ESP-NOW mode on Channel 1.");
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  }
  Serial.printf("Mount Receiver MAC Address: %s | Active Channel: %d\n",
                WiFi.macAddress().c_str(), WiFi.channel());

  // ESP-NOW initialization
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, rebooting ESP32-S3...");
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);

  // Broadcast peer for handshake & discovery
  esp_now_peer_info_t bcast = {};
  memset(bcast.peer_addr, 0xFF, 6);
  bcast.channel = 0; // IMPORTANT: 0 uses active home channel!
  bcast.encrypt = false;
  bcast.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&bcast);

  lastRxMs = millis();
  Serial.println("ESP-NOW initialized. Listening for remote transmitter...\n");
}

// ---------------------------------------------------------------------------
// Arduino loop (Real-time Motion, Zoom, Preset Execution, and Watchdogs)
// ---------------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  // 0. Process queued commands from ESP-NOW (safe main thread execution)
  if (reqHome) {
    reqHome = false;
    homeAll();
  }
  if (reqSavePreset > 0) {
    uint8_t p = reqSavePreset;
    reqSavePreset = 0;
    savePreset(p);
  }
  if (reqGotoPreset > 0) {
    uint8_t p = reqGotoPreset;
    reqGotoPreset = 0;
    gotoPreset(p);
  }

  // 1. Process incoming fresh ESP-NOW packet
  if (rxFresh) {
    rxFresh = false;
    if (!linkAlive) {
      linkAlive = true;
      Serial.println("[LINK] Control link established with Remote TX");
      sendMountTelemetry("Remote Control Link Active");
    }

    // Manual joystick overrides preset transition (requires deliberate stick movement > 600)
    if (isMovingToPreset && (abs(lastPkt.pan) > 600 || abs(lastPkt.tilt) > 600)) {
      stopAllMotors();
      Serial.println("[MANUAL OVERRIDE] Preset motion aborted by joystick input.");
      sendMountTelemetry("Preset Move Aborted by Joystick");
    }

    if (!isMovingToPreset) {
      applyVelocity(panStepper,  lastPkt.pan,  panState,  MAX_PAN_STEPS,  true);
      applyVelocity(tiltStepper, lastPkt.tilt, tiltState, MAX_TILT_STEPS, false);
      applyZoom(lastPkt.zoom);
    }
  }

  // 2. Coordinated Preset Execution (Steppers + Zoom)
  if (isMovingToPreset) {
    updatePresetZoomMove();

    bool panBusy = (panStepper && panStepper->isRunning());
    bool tiltBusy = (tiltStepper && tiltStepper->isRunning());
    bool zoomBusy = (abs((int32_t)targetZoomMs - (int32_t)currentZoomMs) >= ZOOM_TICK_MS);

    if (!panBusy && !tiltBusy && !zoomBusy) {
      uint8_t pReached = currentTargetPreset;
      stopAllMotors(); // Cleanly resets preset state and restores manual speeds/accels

      char evt[32];
      snprintf(evt, sizeof(evt), "Preset %u Reached", pReached);
      sendMountTelemetry(evt);
    }
  }

  // 2b. Fast zoom telemetry: while the zoom lens is actively moving, report every 100 ms
  // so the remote's percentage tracks smoothly(no stale 2.5s gaps that caused big jumps..
  static uint32_t lastFastZoomTelemMs = 0;
  bool zoomActive = (now - irLastCmdMs < ZOOM_ACTIVE_WINDOW_MS);

  if (zoomActive && (now - lastFastZoomTelemMs >= ZOOM_FAST_TELEMETRY_MS)) {



    lastFastZoomTelemMs = now;

    sendMountTelemetry();
  }

  // 3. Failsafe: Link loss timeout watchdog -> halt motion & clear all preset state
  if (linkAlive && now - lastRxMs > RX_TIMEOUT_MS) {
    linkAlive = false;
    stopAllMotors();
    Serial.println("[LINK WARNING] Link timeout - motors and zoom safely stopped");
  }

  // 4. Hall Sensor Edge Detection (Instant Telemetry Event)
  static bool lastPanHallEdge = false;
  static bool lastTiltHallEdge = false;
  bool panHallNow = (digitalRead(PIN_PAN_HALL) == LOW);
  bool tiltHallNow = (digitalRead(PIN_TILT_HALL) == LOW);

  if (panHallNow != lastPanHallEdge) {
    lastPanHallEdge = panHallNow;
    if (panHallNow) sendMountTelemetry("Pan Hall: DETECTED (At Center)");
    else            sendMountTelemetry("Pan Hall: CLEARED (Away)");
  }

  if (tiltHallNow != lastTiltHallEdge) {
    lastTiltHallEdge = tiltHallNow;
    if (tiltHallNow) sendMountTelemetry("Tilt Hall: DETECTED (At Center)");
    else             sendMountTelemetry("Tilt Hall: CLEARED (Away)");
  }

  // 5. Periodic UART & System Telemetry Heartbeat (every 2.5 seconds to remote)
  static uint32_t lastTelemetryMs = 0;
  if (now - lastTelemetryMs >= 2500) {
    lastTelemetryMs = now;

    // Verify driver IC versions over UART periodically
    panDriverVersion = driverPan.version();
    panUartOk = (panDriverVersion == 0x21);
    tiltDriverVersion = driverTilt.version();
    tiltUartOk = (tiltDriverVersion == 0x21);

    // Send telemetry to remote
    sendMountTelemetry();

    int32_t pPos = panStepper  ? panStepper->getCurrentPosition()  : 0;
    int32_t tPos = tiltStepper ? tiltStepper->getCurrentPosition() : 0;
    float zoomPct = (float)currentZoomMs / ZOOM_FULL_RANGE_MS * 100.0f;

    Serial.printf("[TELEMETRY %lus] Link: %s | Pan: %ld stp (%s, Hall: %s) | Tilt: %ld stp (Hall: %s) | Zoom: %ld ms (%0.1f%%) | UART: P=%s T=%s\n",
                  now / 1000,
                  linkAlive ? "ALIVE" : "WAITING",
                  pPos, (lastPanSide > 0) ? "RIGHT" : ((lastPanSide < 0) ? "LEFT" : "CENTER"),
                  panHallNow ? "TRIP" : "OK",
                  tPos, tiltHallNow ? "TRIP" : "OK",
                  currentZoomMs, zoomPct,
                  panUartOk ? "OK" : "ERR", tiltUartOk ? "OK" : "ERR");
  }

  // 6. Wi-Fi watchdog: Auto-reconnect ONLY if router Wi-Fi was established on boot
  static uint32_t lastWifiCheck = 0;
  if (wifiConnected && (now - lastWifiCheck > 10000)) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi WARNING] Wi-Fi lost, reconnecting...");
      WiFi.reconnect();
    }
  }
}


