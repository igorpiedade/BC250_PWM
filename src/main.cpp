#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
// ESP32 Power Controller for ASRock BC250 + FlexATX PSU
// Behavior implemented:
// 1) Press button on GPIO18 -> enable transistor driver on GPIO25.
// 2) While motherboard status signal on GPIO34 is present, keep GPIO25 ON.
// 3) If GPIO34 signal disappears, wait 10 seconds and then turn GPIO25 OFF.
// 4) If button on GPIO18 is held for 5 seconds while ON, turn GPIO25 OFF.
// 5) After GPIO25 turns OFF, ignore power-on requests for 3 seconds.
// 6) ARGB on GPIO23:
//    - BOOTING: amber breathing between 40% and 80% for 15s after power-on.
//    - NORMAL: static white at 80%.
//    - SHUTTING_DOWN: blue breathing between 40% and 80% while GPIO34 signal is missing.
//      If GPIO34 signal is restored, return to NORMAL.

// ------------------------------
// Pin mapping
// ------------------------------
static const int PIN_TRANSISTOR_DRIVE = 25; // Output to 2N2222 base (through proper resistor)
static const int PIN_BUTTON_START = 18;     // Start button input
static const int PIN_MB_STATUS = 34;        // Input from BC250: signal present = board ON
static const int PIN_ARGB_DATA = 23;        // ARGB data output
static const uint16_t ARGB_LED_COUNT = 1;

// ------------------------------
// Input behavior configuration
// ------------------------------
// Set to LOW when using INPUT_PULLUP + button to GND.
// Set to HIGH if your touch module outputs HIGH when pressed.
static const int BUTTON_ACTIVE_LEVEL = LOW;

// Set to HIGH if GPIO34 is HIGH when motherboard is ON.
// Set to LOW if GPIO34 is LOW when motherboard is ON.
static const int MB_SIGNAL_PRESENT_LEVEL = HIGH;

static const unsigned long BUTTON_DEBOUNCE_MS = 40;
static const unsigned long BUTTON_ON_ARM_DELAY_AFTER_OFF_MS = 3000;
static const unsigned long BUTTON_HOLD_ARM_DELAY_MS = 3000;
static const unsigned long BUTTON_HOLD_TO_OFF_MS = 5000;
static const unsigned long SIGNAL_LOSS_TIMEOUT_MS = 10000;
static const unsigned long ARGB_BOOTING_DURATION_MS = 15000;
static const unsigned long ARGB_BREATH_PERIOD_MS = 2500;
static const uint8_t ARGB_BREATH_MIN_PCT = 40;
static const uint8_t ARGB_BREATH_MAX_PCT = 80;

bool powerEnabled = false;
bool signalLossTimerRunning = false;
unsigned long signalLossStartedAt = 0;

// Simple debounced edge detection for GPIO18
int lastRawButton = HIGH;
int stableButton = HIGH;
unsigned long lastButtonChangeAt = 0;
bool buttonPressedEdge = false;

bool offHoldTimerRunning = false;
unsigned long offHoldStartedAt = 0;
unsigned long offHoldLastProgressSecond = 0;
unsigned long powerEnabledAt = 0;
bool powerOnLockoutActive = false;
unsigned long powerOnLockoutStartedAt = 0;

bool isMainboardSignalPresent() {
  return digitalRead(PIN_MB_STATUS) == MB_SIGNAL_PRESENT_LEVEL;
}

Adafruit_NeoPixel argb(ARGB_LED_COUNT, PIN_ARGB_DATA, NEO_GRB + NEO_KHZ800);

enum class ArgbState {
  Off,
  Booting,
  Normal,
  ShuttingDown,
};

ArgbState argbState = ArgbState::Off;
ArgbState lastReportedArgbState = ArgbState::Off;
unsigned long argbBootStartedAt = 0;

void setAllArgb(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color = argb.Color(r, g, b);
  for (uint16_t i = 0; i < ARGB_LED_COUNT; ++i) {
    argb.setPixelColor(i, color);
  }
  argb.show();
}

uint8_t scaleChannelByPercent(uint8_t channel, uint8_t percent) {
  return static_cast<uint8_t>((static_cast<uint16_t>(channel) * percent) / 100);
}

uint8_t breathingPercent(unsigned long nowMs) {
  unsigned long phase = nowMs % ARGB_BREATH_PERIOD_MS;
  unsigned long half = ARGB_BREATH_PERIOD_MS / 2;
  unsigned long ramp = (phase <= half) ? phase : (ARGB_BREATH_PERIOD_MS - phase);
  unsigned long span = ARGB_BREATH_MAX_PCT - ARGB_BREATH_MIN_PCT;
  return static_cast<uint8_t>(ARGB_BREATH_MIN_PCT + ((span * ramp) / half));
}

void reportArgbStateIfChanged() {
  if (argbState == lastReportedArgbState) {
    return;
  }

  if (argbState == ArgbState::Off) {
    Serial.println("[ARGB] OFF");
  } else if (argbState == ArgbState::Booting) {
    Serial.println("[ARGB] BOOTING (amber breathing 40-80%)");
  } else if (argbState == ArgbState::Normal) {
    Serial.println("[ARGB] NORMAL (white 80%)");
  } else if (argbState == ArgbState::ShuttingDown) {
    Serial.println("[ARGB] SHUTTING_DOWN (blue breathing 40-80%)");
  }

  lastReportedArgbState = argbState;
}

void updateArgbStateMachine() {
  if (!powerEnabled) {
    argbState = ArgbState::Off;
    return;
  }

  bool signalPresent = (digitalRead(PIN_MB_STATUS) == MB_SIGNAL_PRESENT_LEVEL);

  if (argbState == ArgbState::Off) {
    argbState = ArgbState::Booting;
    argbBootStartedAt = millis();
  }

  if (argbState == ArgbState::Booting) {
    if (!signalPresent) {
      argbState = ArgbState::ShuttingDown;
    } else if ((millis() - argbBootStartedAt) >= ARGB_BOOTING_DURATION_MS) {
      argbState = ArgbState::Normal;
    }
    return;
  }

  if (argbState == ArgbState::Normal) {
    if (!signalPresent) {
      argbState = ArgbState::ShuttingDown;
    }
    return;
  }

  if (argbState == ArgbState::ShuttingDown && signalPresent) {
    argbState = ArgbState::Normal;
  }
}

void renderArgb() {
  if (argbState == ArgbState::Off) {
    setAllArgb(0, 0, 0);
    return;
  }

  if (argbState == ArgbState::Normal) {
    setAllArgb(
      scaleChannelByPercent(255, 80),
      scaleChannelByPercent(255, 80),
      scaleChannelByPercent(255, 80)
    );
    return;
  }

  uint8_t breathPct = breathingPercent(millis());
  if (argbState == ArgbState::Booting) {
    // Amber base color.
    setAllArgb(
      scaleChannelByPercent(255, breathPct),
      scaleChannelByPercent(140, breathPct),
      scaleChannelByPercent(0, breathPct)
    );
    return;
  }

  // Shutting down: blue base color.
  setAllArgb(
    scaleChannelByPercent(0, breathPct),
    scaleChannelByPercent(110, breathPct),
    scaleChannelByPercent(255, breathPct)
  );
}

void updateButtonState() {
  int raw = digitalRead(PIN_BUTTON_START);

  if (raw != lastRawButton) {
    lastRawButton = raw;
    lastButtonChangeAt = millis();
  }

  if ((millis() - lastButtonChangeAt) >= BUTTON_DEBOUNCE_MS && stableButton != raw) {
    stableButton = raw;
    if (stableButton == BUTTON_ACTIVE_LEVEL) {
      buttonPressedEdge = true;
    }
  }
}

bool consumeButtonPressedEdge() {
  if (buttonPressedEdge) {
    buttonPressedEdge = false;
    return true;
  }

  return false;
}

bool isButtonHeldPressed() {
  return stableButton == BUTTON_ACTIVE_LEVEL;
}

void enablePowerDrive() {
  powerEnabled = true;
  powerEnabledAt = millis();
  argbBootStartedAt = powerEnabledAt;
  argbState = ArgbState::Booting;
  signalLossTimerRunning = false;
  offHoldTimerRunning = false;
  offHoldLastProgressSecond = 0;
  digitalWrite(PIN_TRANSISTOR_DRIVE, HIGH);
  Serial.println("[POWER] GPIO25 ON");
}

void disablePowerDrive() {
  powerEnabled = false;
  powerEnabledAt = 0;
  powerOnLockoutActive = true;
  powerOnLockoutStartedAt = millis();
  buttonPressedEdge = false; // Discard stale press captured while power was ON.
  signalLossTimerRunning = false;
  offHoldTimerRunning = false;
  offHoldLastProgressSecond = 0;
  argbState = ArgbState::Off;
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);
  Serial.println("[POWER] GPIO25 OFF");
  Serial.println("[POWER] Power-on locked for 3s after OFF");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRANSISTOR_DRIVE, OUTPUT);
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);

  pinMode(PIN_BUTTON_START, INPUT_PULLUP);

  // GPIO34 is input-only and has no internal pull-up/pull-down.
  pinMode(PIN_MB_STATUS, INPUT);

  argb.begin();
  argb.clear();
  argb.show();

  // Initialize button state trackers
  lastRawButton = digitalRead(PIN_BUTTON_START);
  stableButton = lastRawButton;
  lastButtonChangeAt = millis();

  Serial.println("ESP32 Power Controller started");
}

void loop() {
  updateButtonState();

  // 1) Start request: press GPIO18 while power is OFF
  if (!powerEnabled) {
    if (powerOnLockoutActive) {
      if ((millis() - powerOnLockoutStartedAt) >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
        powerOnLockoutActive = false;
        Serial.println("[POWER] Power-on re-enabled");
      } else {
        // Drain any button press edge during lockout to avoid instant start after unlock.
        consumeButtonPressedEdge();
      }
    }

    if (!powerOnLockoutActive && consumeButtonPressedEdge()) {
      enablePowerDrive();
    }
  }

  // 2) If power drive is active, monitor motherboard status signal on GPIO34
  if (powerEnabled) {
    // 2.a) Manual power-off request via long press
    bool holdToOffArmed = (millis() - powerEnabledAt) >= BUTTON_HOLD_ARM_DELAY_MS;
    if (holdToOffArmed) {
      if (isButtonHeldPressed()) {
        if (!offHoldTimerRunning) {
          offHoldTimerRunning = true;
          offHoldStartedAt = millis();
          offHoldLastProgressSecond = 0;
          Serial.println("[BUTTON] Hold detected, waiting 5s for manual OFF");
        } else {
          unsigned long heldMs = millis() - offHoldStartedAt;
          unsigned long elapsedSec = heldMs / 1000;
          if (elapsedSec > 0 && elapsedSec <= 4 && elapsedSec != offHoldLastProgressSecond) {
            offHoldLastProgressSecond = elapsedSec;
            unsigned long remaining = 5 - elapsedSec;
            Serial.print("[BUTTON] Hold progress: ");
            Serial.print(elapsedSec);
            Serial.print("/5s (");
            Serial.print(remaining);
            Serial.println("s remaining)");
          }

          if (heldMs >= BUTTON_HOLD_TO_OFF_MS) {
            Serial.println("[BUTTON] Held for 5s, manual power OFF");
            disablePowerDrive();
          }
        }
      } else if (offHoldTimerRunning) {
        offHoldTimerRunning = false;
        offHoldLastProgressSecond = 0;
        Serial.println("[BUTTON] Hold canceled before 5s");
      }
    } else if (offHoldTimerRunning) {
      offHoldTimerRunning = false;
      offHoldLastProgressSecond = 0;
    }

    if (powerEnabled) {
      if (isMainboardSignalPresent()) {
        // Signal is valid, keep power on and cancel any pending shutdown timer.
        if (signalLossTimerRunning) {
          signalLossTimerRunning = false;
          Serial.println("[SIGNAL] Restored before timeout");
        }
      } else {
        // Signal missing: start (or continue) delayed shutdown timer.
        if (!signalLossTimerRunning) {
          signalLossTimerRunning = true;
          signalLossStartedAt = millis();
          Serial.println("[SIGNAL] Lost, starting 10s shutdown timer");
        } else if (millis() - signalLossStartedAt >= SIGNAL_LOSS_TIMEOUT_MS) {
          Serial.println("[SIGNAL] Missing for 10s, shutting down drive");
          disablePowerDrive();
        }
      }
    }
  }

  updateArgbStateMachine();
  reportArgbStateIfChanged();
  renderArgb();

  delay(5);
}
