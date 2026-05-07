#include <Arduino.h>
#include "driver/gpio.h"

// =====================================================
// USER SETTINGS
// =====================================================
// FlyPT frame: '!' '$' + 12 bytes
static const uint32_t BAUD = 115200;
static const uint8_t SYNC1 = 0x21;   // '!'
static const uint8_t SYNC2 = 0x24;   // '$'
static const uint8_t AXES_TOTAL = 6;
static const uint8_t PAYLOAD_BYTES = AXES_TOTAL * 2;

// =====================================================
// ACTUATOR SETTINGS & STATE
// =====================================================
static const int32_t PULSES_PER_REV = 500;
static const int32_t TOTAL_REV_RANGE = 10;

static const int32_t TRAVEL_MIN = 0;
static const int32_t TRAVEL_MAX = TOTAL_REV_RANGE * PULSES_PER_REV;   // 5000
static const int32_t TRAVEL_CENTER = (TRAVEL_MIN + TRAVEL_MAX) / 2;   // 2500

static const float HOMING_RPM = 30.0f;
static const float HOMING_PPS = (HOMING_RPM * PULSES_PER_REV) / 60.0f;

static const int32_t HOMING_BACKOFF_REVS = 9;
static const int32_t HOMING_BACKOFF_PULSES = HOMING_BACKOFF_REVS * PULSES_PER_REV;

static const int8_t HOMING_SEEK_SIGN = -1;
static const int HOME_SWITCH_ACTIVE_STATE = LOW;
static const uint32_t SWITCH_DEBOUNCE_MS = 20;

static const float MAX_RPM = 2300.0f;
static const float MAX_PPS = (MAX_RPM * PULSES_PER_REV) / 60.0f;
static const float ACCEL_PPS2 = 250000.0f;

static const uint32_t CONTROL_PERIOD_MS = 5;
static const uint32_t PACKET_TIMEOUT_MS = 200;
static const bool RETURN_TO_CENTER_ON_TIMEOUT = false;

static const bool CW_DIR_LEVEL = HIGH;
static const bool CCW_DIR_LEVEL = LOW;

// We use a high-frequency master timer (40 kHz) to run a software
// DDA (Digital Differential Analyzer) for all 6 axes simultaneously.
// 40 kHz is safe for the ESP32 CPU while still allowing up to 20,000 pulses per sec!
static const float TIMER_FREQ = 40000.0f; 

enum RunMode {
  MODE_HOMING_SEEK,
  MODE_HOMING_BACKOFF,
  MODE_ACTIVE
};

struct Actuator {
  uint8_t pulPin;
  uint8_t dirPin;
  uint8_t homeSwitchPin;
  
  volatile int32_t currentPos;
  volatile int32_t targetPos;
  volatile int8_t currentDirSign;
  
  volatile uint32_t currentPps;
  volatile uint32_t ppsAccumulator;
  
  RunMode runMode;
  bool switchStableSeen;
  uint32_t switchSeenMs;
  
  volatile bool pulseIsLow;
};

// Define pins for all 6 actuators here. Ensure these pins are valid for ESP32.
// Default typical ESP32 GPIOs used here as placeholders. Adjust to your actual wiring!
Actuator axes[AXES_TOTAL] = {
  {2,   4,  13, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}, // Axis 1
  {16, 17,  14, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}, // Axis 2
  {18, 19,  21, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}, // Axis 3
  {22, 23,  25, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}, // Axis 4
  {26, 27,  34, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}, // Axis 5
  {32, 33,  35, TRAVEL_CENTER, TRAVEL_CENTER, 1, 0, 0, MODE_HOMING_SEEK, false, 0, false}  // Axis 6
};

hw_timer_t *pulseTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// FLYPT PARSER STATE
// =====================================================
enum ParseState { FIND_SYNC1, FIND_SYNC2, READ_PAYLOAD };
ParseState parseState = FIND_SYNC1;

uint8_t payload[PAYLOAD_BYTES];
uint8_t payloadIndex = 0;
uint32_t lastPacketMs = 0;

// =====================================================
// HELPERS
// =====================================================
static inline uint16_t readU16_BE(const uint8_t* p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static inline bool isHomeSwitchPressed(int i) {
  return digitalRead(axes[i].homeSwitchPin) == HOME_SWITCH_ACTIVE_STATE;
}

static inline int32_t clampToTravel(int32_t x) {
  if (x < TRAVEL_MIN) return TRAVEL_MIN;
  if (x > TRAVEL_MAX) return TRAVEL_MAX;
  return x;
}

static inline int32_t mapU16ToPulsesNeutralCenter(uint16_t v) {
  const uint16_t MID = 32767;
  if (v <= MID) {
    int32_t s = (int32_t)(((uint32_t)v * (uint32_t)TRAVEL_CENTER) / MID);
    if (s < TRAVEL_MIN) s = TRAVEL_MIN;
    if (s > TRAVEL_CENTER) s = TRAVEL_CENTER;
    return s;
  } else {
    uint32_t upperSpanIn  = (uint32_t)(65535 - (MID + 1));
    uint32_t x            = (uint32_t)(v - (MID + 1));
    uint32_t upperSpanOut = (uint32_t)(TRAVEL_MAX - TRAVEL_CENTER);
    int32_t s = (int32_t)TRAVEL_CENTER + (int32_t)(((uint64_t)x * upperSpanOut) / upperSpanIn);
    if (s < TRAVEL_CENTER) s = TRAVEL_CENTER;
    if (s > TRAVEL_MAX) s = TRAVEL_MAX;
    return s;
  }
}

static inline void setDirectionFromSign(int i, int8_t sign) {
  axes[i].currentDirSign = (sign >= 0) ? +1 : -1;
  digitalWrite(axes[i].dirPin, (axes[i].currentDirSign > 0) ? CW_DIR_LEVEL : CCW_DIR_LEVEL);
}

static inline void commandTarget(int i, int32_t newTarget) {
  newTarget = clampToTravel(newTarget);
  portENTER_CRITICAL(&timerMux);
  axes[i].targetPos = newTarget;
  portEXIT_CRITICAL(&timerMux);
}

static inline void setCurrentAndTarget(int i, int32_t pos) {
  pos = clampToTravel(pos);
  portENTER_CRITICAL(&timerMux);
  axes[i].currentPos = pos;
  axes[i].targetPos = pos;
  portEXIT_CRITICAL(&timerMux);
}

// =====================================================
// TIMER ISR
// =====================================================
void ARDUINO_ISR_ATTR onPulseTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  for (int i = 0; i < AXES_TOTAL; i++) {
    Actuator &ax = axes[i];

    if (ax.currentPps > 0) {
      ax.ppsAccumulator += (ax.currentPps * 2);
      if (ax.ppsAccumulator >= (uint32_t)TIMER_FREQ) {
        ax.ppsAccumulator -= (uint32_t)TIMER_FREQ;
        
        ax.pulseIsLow = !ax.pulseIsLow;
        
        if (ax.pulseIsLow) {
          digitalWrite(ax.pulPin, LOW);
        } else {
          digitalWrite(ax.pulPin, HIGH);
          ax.currentPos += ax.currentDirSign;

          if (ax.runMode != MODE_HOMING_SEEK) {
            if (ax.currentPos <= TRAVEL_MIN) {
              ax.currentPos = TRAVEL_MIN;
              ax.currentPps = 0;
            } else if (ax.currentPos >= TRAVEL_MAX) {
              ax.currentPos = TRAVEL_MAX;
              ax.currentPps = 0;
            } else if (ax.currentPos == ax.targetPos) {
              ax.currentPps = 0;
            }
          }
        }
      }
    } else if (ax.pulseIsLow) {
      digitalWrite(ax.pulPin, HIGH);
      ax.pulseIsLow = false;
      ax.currentPos += ax.currentDirSign;
      
      if (ax.runMode != MODE_HOMING_SEEK) {
        if (ax.currentPos <= TRAVEL_MIN) {
          ax.currentPos = TRAVEL_MIN;
        } else if (ax.currentPos >= TRAVEL_MAX) {
          ax.currentPos = TRAVEL_MAX;
        }
      }
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// =====================================================
// APPLY FLYPT FRAME
// =====================================================
static inline void applyFlyptFrame() {
  for (int i = 0; i < AXES_TOTAL; i++) {
    if (axes[i].runMode != MODE_ACTIVE) continue;
    uint16_t raw = readU16_BE(&payload[i * 2]);
    int32_t mapped = mapU16ToPulsesNeutralCenter(raw);
    commandTarget(i, mapped);
  }
}

// =====================================================
// PARSE SERIAL
// =====================================================
void parseFlyptStream() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    switch (parseState) {
      case FIND_SYNC1:
        if (b == SYNC1) parseState = FIND_SYNC2;
        break;
      case FIND_SYNC2:
        if (b == SYNC2) {
          parseState = READ_PAYLOAD;
          payloadIndex = 0;
        } else if (b == SYNC1) {
          parseState = FIND_SYNC2;
        } else {
          parseState = FIND_SYNC1;
        }
        break;
      case READ_PAYLOAD:
        if (b == SYNC1) {
          parseState = FIND_SYNC2;
          payloadIndex = 0;
          break;
        }
        payload[payloadIndex++] = b;
        if (payloadIndex >= PAYLOAD_BYTES) {
          applyFlyptFrame();
          lastPacketMs = millis();
          parseState = FIND_SYNC1;
          payloadIndex = 0;
        }
        break;
    }
  }
}

// =====================================================
// HOMING UPDATE
// =====================================================
void updateHoming() {
  for (int i = 0; i < AXES_TOTAL; i++) {
    Actuator &ax = axes[i];

    if (ax.runMode == MODE_HOMING_SEEK) {
      setDirectionFromSign(i, HOMING_SEEK_SIGN);
      ax.currentPps = HOMING_PPS;

      if (isHomeSwitchPressed(i)) {
        if (!ax.switchStableSeen) {
          ax.switchStableSeen = true;
          ax.switchSeenMs = millis();
        } else if ((millis() - ax.switchSeenMs) >= SWITCH_DEBOUNCE_MS) {
          ax.currentPps = 0;
          setCurrentAndTarget(i, TRAVEL_MIN);
          commandTarget(i, TRAVEL_MIN + HOMING_BACKOFF_PULSES);
          ax.runMode = MODE_HOMING_BACKOFF;
        }
      } else {
        ax.switchStableSeen = false;
      }
    } 
    else if (ax.runMode == MODE_HOMING_BACKOFF) {
      int32_t c, t;
      portENTER_CRITICAL(&timerMux);
      c = ax.currentPos;
      t = ax.targetPos;
      portEXIT_CRITICAL(&timerMux);

      if (c == t) {
        ax.currentPps = 0;
        setCurrentAndTarget(i, TRAVEL_CENTER);
        ax.runMode = MODE_ACTIVE;
        lastPacketMs = millis(); // Refresh packet timeout
      }
    }
  }
}

// =====================================================
// CONTROL UPDATE
// =====================================================
void controlUpdate(float dt) {
  for (int i = 0; i < AXES_TOTAL; i++) {
    Actuator &ax = axes[i];
    int32_t c, t;

    portENTER_CRITICAL(&timerMux);
    c = ax.currentPos;
    t = ax.targetPos;
    portEXIT_CRITICAL(&timerMux);

    int32_t err = t - c;

    if (ax.runMode == MODE_HOMING_SEEK) {
      continue; // Handled rigidly by updateHoming()
    }

    if (ax.runMode == MODE_HOMING_BACKOFF) {
      if (err == 0) {
        ax.currentPps = 0;
        continue;
      }
      int8_t desiredSign = (err > 0) ? +1 : -1;
      setDirectionFromSign(i, desiredSign);
      ax.currentPps = (uint32_t)HOMING_PPS;
      continue;
    }

    // ACTIVE MODE
    if ((millis() - lastPacketMs) > PACKET_TIMEOUT_MS) {
      if (RETURN_TO_CENTER_ON_TIMEOUT) {
        commandTarget(i, TRAVEL_CENTER);
        portENTER_CRITICAL(&timerMux);
        t = ax.targetPos;
        portEXIT_CRITICAL(&timerMux);
        err = t - c;
      }
    }

    if (err == 0) {
      ax.currentPps = 0;
      continue;
    }

    int8_t desiredSign = (err > 0) ? +1 : -1;

    // Handle deceleration / reversing
    if (desiredSign != ax.currentDirSign && ax.currentPps > 0) {
      float maxDelta = ACCEL_PPS2 * dt;
      float nextPps = (float)ax.currentPps - maxDelta;
      if (nextPps < 0.0f) nextPps = 0.0f;
      ax.currentPps = (uint32_t)nextPps;

      if (ax.currentPps == 0) {
        if (!ax.pulseIsLow) {
          setDirectionFromSign(i, desiredSign);
        }
      }
      continue;
    }

    // If fully stopped but wanting to go opposite, flip direction immediately
    if (desiredSign != ax.currentDirSign && ax.currentPps == 0) {
      if (!ax.pulseIsLow) {
        setDirectionFromSign(i, desiredSign);
      } else {
        continue; // Wait for the current pulse to finish before flipping direction!
      }
    }

    // Accel/Decel Kinematics
    float distance = (float)abs(err);
    float curPpsF = (float)ax.currentPps;
    float stoppingDistance = (curPpsF * curPpsF) / (2.0f * ACCEL_PPS2);
    float desiredPps = (distance <= stoppingDistance) ? 0.0f : MAX_PPS;

    float maxDelta = ACCEL_PPS2 * dt;
    if (curPpsF < desiredPps) {
      float nextPps = curPpsF + maxDelta;
      if (nextPps > desiredPps) nextPps = desiredPps;
      ax.currentPps = (uint32_t)nextPps;
    } else if (curPpsF > desiredPps) {
      float nextPps = curPpsF - maxDelta;
      if (nextPps < desiredPps) nextPps = desiredPps;
      ax.currentPps = (uint32_t)nextPps;
    }
  }
}

// =====================================================
// DEBUG
// =====================================================
void printStatus() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint < 250) return;
  lastPrint = now;

  Serial.print("Modes:");
  for(int i=0; i<AXES_TOTAL; i++) {
    if (axes[i].runMode == MODE_HOMING_SEEK) Serial.print(" S");
    else if (axes[i].runMode == MODE_HOMING_BACKOFF) Serial.print(" B");
    else Serial.print(" A");
  }

  Serial.print(" | Pos:");
  for(int i=0; i<AXES_TOTAL; i++) {
    int32_t c;
    portENTER_CRITICAL(&timerMux);
    c = axes[i].currentPos;
    portEXIT_CRITICAL(&timerMux);
    Serial.print(" ");
    Serial.print(c);
  }

  Serial.print(" | PPS:");
  for(int i=0; i<AXES_TOTAL; i++) {
    Serial.print(" ");
    Serial.print(axes[i].currentPps);
  }

  Serial.println();
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(BAUD);

  for (int i = 0; i < AXES_TOTAL; i++) {
    pinMode(axes[i].pulPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    pinMode(axes[i].homeSwitchPin, INPUT_PULLUP);
    
    gpio_set_level((gpio_num_t)axes[i].pulPin, 1);
    gpio_set_level((gpio_num_t)axes[i].dirPin, CW_DIR_LEVEL);
    setDirectionFromSign(i, +1);
  }

  // Set up 100 kHz DDA Hardware Master Timer
  pulseTimer = timerBegin(1000000);   // 1 MHz base timer clock
  timerAttachInterrupt(pulseTimer, &onPulseTimer);
  timerAlarm(pulseTimer, (uint32_t)(1000000.0f / TIMER_FREQ), true, 0); // Fire every 10us (100kHz)
  timerRestart(pulseTimer);
  timerStart(pulseTimer);

  lastPacketMs = millis();

  Serial.println("6-axis FlyPT servo controller with boot homing");
}

void loop() {
  parseFlyptStream();
  updateHoming();

  static uint32_t lastCtrlMs = 0;
  uint32_t now = millis();
  if (now - lastCtrlMs >= CONTROL_PERIOD_MS) {
    lastCtrlMs = now;
    controlUpdate(CONTROL_PERIOD_MS / 1000.0f);
  }

  printStatus();
}