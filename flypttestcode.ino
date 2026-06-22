#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

// =====================================================
// USER SETTINGS
// =====================================================
static const uint32_t BAUD = 115200;
static const uint8_t SYNC1 = 0x21;   // '!'
static const uint8_t SYNC2 = 0x24;   // '$'
static const uint8_t AXES_TOTAL = 6;
static const uint8_t PAYLOAD_BYTES = AXES_TOTAL * 2;

// =====================================================
// HARDWARE PIN SETTINGS
// =====================================================
struct AxisConfig {
  uint8_t pulPin;
  uint8_t dirPin;
  uint8_t homePin;
};

// Default pin assignments for ESP32-S3
// ESP32-S3 pins: 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38 are all safe and have pull-ups
const AxisConfig config[AXES_TOTAL] = {
  { 2,  4, 13 }, // Axis 0 (FlyPT Axis 1)
  { 5,  6,  7 }, // Axis 1 (FlyPT Axis 2)
  { 8,  9, 10 }, // Axis 2 (FlyPT Axis 3)
  {11, 12, 14 }, // Axis 3 (FlyPT Axis 4)
  {15, 16, 17 }, // Axis 4 (FlyPT Axis 5)
  {18, 21, 38 }  // Axis 5 (FlyPT Axis 6)
};

// =====================================================
// ACTUATOR / TRAVEL SETTINGS
// =====================================================
static const int32_t PULSES_PER_REV = 500;
static const int32_t TOTAL_REV_RANGE = 10;
static const int32_t TRAVEL_MIN = 0;
static const int32_t TRAVEL_MAX = TOTAL_REV_RANGE * PULSES_PER_REV;   // 5000
static const int32_t TRAVEL_CENTER = (TRAVEL_MIN + TRAVEL_MAX) / 2;   // 2500

// =====================================================
// HOMING SETTINGS
// =====================================================
static const float HOMING_RPM = 30.0f;
static const float HOMING_PPS = (HOMING_RPM * PULSES_PER_REV) / 60.0f;
static const int32_t HOMING_BACKOFF_REVS = 7;
static const int32_t HOMING_BACKOFF_PULSES = HOMING_BACKOFF_REVS * PULSES_PER_REV;
static const int8_t HOMING_SEEK_SIGN = -1;
static const int HOME_SWITCH_ACTIVE_STATE = LOW;
static const uint32_t SWITCH_DEBOUNCE_MS = 20;

// =====================================================
// ACTIVE RUN SPEED SETTINGS
// =====================================================
static const float MAX_RPM = 2300.0f;
static const float MAX_PPS = (MAX_RPM * PULSES_PER_REV) / 60.0f;
static const float ACCEL_PPS2 = 250000.0f;

// =====================================================
// TIMING
// =====================================================
static const uint32_t CONTROL_PERIOD_MS = 5;
static const uint32_t PACKET_TIMEOUT_MS = 200;
static const bool RETURN_TO_CENTER_ON_TIMEOUT = false;

// =====================================================
// DIRECTION POLARITY
// =====================================================
static const bool CW_DIR_LEVEL = HIGH;
static const bool CCW_DIR_LEVEL = LOW;

// =====================================================
// TIMER / MOTION STATE
// =====================================================
// DDA Timer frequency: 100 kHz (10us period)
static const uint32_t TIMER_FREQ = 100000; 

enum RunMode {
  MODE_HOMING_SEEK,
  MODE_HOMING_BACKOFF,
  MODE_ACTIVE
};

struct AxisState {
  volatile bool timerActive;
  volatile bool pulseIsLow;
  volatile int32_t currentPos;
  volatile int32_t targetPos;
  volatile int8_t currentDirSign;
  
  float currentPps;
  volatile uint32_t isrStepRate;
  volatile uint32_t accumulator;
  
  RunMode runMode;
  bool switchStableSeen;
  uint32_t switchSeenMs;
};

AxisState axes[AXES_TOTAL];
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
// FAST PORT MANIPULATION
// =====================================================
// These macros avoid the heavy spinlocks and parameter checks of gpio_set_level()
#define FAST_SET_HIGH(pin) \
  if ((pin) < 32) REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << (pin)); \
  else REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << ((pin) - 32))

#define FAST_SET_LOW(pin) \
  if ((pin) < 32) REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << (pin)); \
  else REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << ((pin) - 32))

// =====================================================
// HELPERS
// =====================================================
static inline uint16_t readU16_BE(const uint8_t* p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static inline bool isHomeSwitchPressed(int axisIndex) {
  return digitalRead(config[axisIndex].homePin) == HOME_SWITCH_ACTIVE_STATE;
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

static inline void setDirectionFromSign(int axisIndex, int8_t sign) {
  axes[axisIndex].currentDirSign = (sign >= 0) ? +1 : -1;
  digitalWrite(config[axisIndex].dirPin, (axes[axisIndex].currentDirSign > 0) ? CW_DIR_LEVEL : CCW_DIR_LEVEL);
}

static inline void commandTarget(int axisIndex, int32_t newTarget) {
  newTarget = clampToTravel(newTarget);
  portENTER_CRITICAL(&timerMux);
  axes[axisIndex].targetPos = newTarget;
  portEXIT_CRITICAL(&timerMux);
}

static inline void setCurrentAndTarget(int axisIndex, int32_t pos) {
  pos = clampToTravel(pos);
  portENTER_CRITICAL(&timerMux);
  axes[axisIndex].currentPos = pos;
  axes[axisIndex].targetPos = pos;
  portEXIT_CRITICAL(&timerMux);
}

// =====================================================
// TIMER CONTROL HELPERS
// =====================================================
static inline void stopAxisPulseTimer(int i) {
  portENTER_CRITICAL(&timerMux);
  axes[i].timerActive = false;
  if (axes[i].pulseIsLow) {
    FAST_SET_HIGH(config[i].pulPin);
    axes[i].pulseIsLow = false;
    // We explicitly do NOT increment currentPos here. 
    // This truncated pulse is too short to be registered by the driver.
  }
  axes[i].isrStepRate = 0;
  portEXIT_CRITICAL(&timerMux);
}

// =====================================================
// TIMER ISR (DDA algorithm for 6 axes)
// =====================================================
void ARDUINO_ISR_ATTR onPulseTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  for (int i = 0; i < AXES_TOTAL; i++) {
    if (!axes[i].timerActive) continue;
    
    // Pure integer addition - extremely fast!
    axes[i].accumulator += axes[i].isrStepRate;
    
    while (axes[i].accumulator >= TIMER_FREQ) {
      axes[i].accumulator -= TIMER_FREQ;
      
      if (!axes[i].pulseIsLow) {
        // Start the pulse (Set LOW)
        FAST_SET_LOW(config[i].pulPin);
        axes[i].pulseIsLow = true;
      } else {
        // End the pulse (Set HIGH)
        FAST_SET_HIGH(config[i].pulPin);
        axes[i].pulseIsLow = false;
        
        axes[i].currentPos += axes[i].currentDirSign;
        
        // Stop exactly at target or soft limits
        if (axes[i].currentPos <= TRAVEL_MIN || 
            axes[i].currentPos >= TRAVEL_MAX ||
            axes[i].currentPos == axes[i].targetPos) {
          
          if (axes[i].currentPos < TRAVEL_MIN) axes[i].currentPos = TRAVEL_MIN;
          if (axes[i].currentPos > TRAVEL_MAX) axes[i].currentPos = TRAVEL_MAX;
            
          axes[i].timerActive = false;
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
    if (axes[i].runMode == MODE_HOMING_SEEK) {
      setDirectionFromSign(i, HOMING_SEEK_SIGN);
      
      axes[i].currentPps = HOMING_PPS;
      axes[i].isrStepRate = (uint32_t)(HOMING_PPS * 2.0f);
      portENTER_CRITICAL(&timerMux);
      axes[i].timerActive = true;
      portEXIT_CRITICAL(&timerMux);

      if (isHomeSwitchPressed(i)) {
        if (!axes[i].switchStableSeen) {
          axes[i].switchStableSeen = true;
          axes[i].switchSeenMs = millis();
        } else if ((millis() - axes[i].switchSeenMs) >= SWITCH_DEBOUNCE_MS) {
          stopAxisPulseTimer(i);
          
          axes[i].currentPps = 0.0f;
          setCurrentAndTarget(i, TRAVEL_MIN);
          commandTarget(i, TRAVEL_MIN + HOMING_BACKOFF_PULSES);
          axes[i].runMode = MODE_HOMING_BACKOFF;
        }
      } else {
        axes[i].switchStableSeen = false;
      }
    } 
    else if (axes[i].runMode == MODE_HOMING_BACKOFF) {
      int32_t c, t;
      portENTER_CRITICAL(&timerMux);
      c = axes[i].currentPos;
      t = axes[i].targetPos;
      portEXIT_CRITICAL(&timerMux);

      if (c == t) {
        stopAxisPulseTimer(i);
        
        axes[i].currentPps = 0.0f;
        setCurrentAndTarget(i, TRAVEL_CENTER);
        axes[i].runMode = MODE_ACTIVE;
        
        // Ensure last packet MS stays updated so we don't immediately timeout
        lastPacketMs = millis(); 
      }
    }
  }
}

// =====================================================
// CONTROL UPDATE
// =====================================================
void controlUpdate(float dt) {
  for (int i = 0; i < AXES_TOTAL; i++) {
    int32_t c, t;
    bool activeNow;

    portENTER_CRITICAL(&timerMux);
    c = axes[i].currentPos;
    t = axes[i].targetPos;
    activeNow = axes[i].timerActive;
    portEXIT_CRITICAL(&timerMux);

    int32_t err = t - c;

    if (axes[i].runMode == MODE_HOMING_SEEK) {
      continue;
    }

    if (axes[i].runMode == MODE_HOMING_BACKOFF) {
      if (err == 0) {
        axes[i].currentPps = 0.0f;
        if (activeNow) stopAxisPulseTimer(i);
        continue;
      }

      int8_t desiredSign = (err > 0) ? +1 : -1;
      setDirectionFromSign(i, desiredSign);
      axes[i].currentPps = HOMING_PPS;
      axes[i].isrStepRate = (uint32_t)(HOMING_PPS * 2.0f);
      
      if (!activeNow) {
        portENTER_CRITICAL(&timerMux);
        axes[i].timerActive = true;
        portEXIT_CRITICAL(&timerMux);
      }
      continue;
    }

    // ACTIVE MODE
    if ((millis() - lastPacketMs) > PACKET_TIMEOUT_MS) {
      if (RETURN_TO_CENTER_ON_TIMEOUT) {
        commandTarget(i, TRAVEL_CENTER);
        portENTER_CRITICAL(&timerMux);
        t = axes[i].targetPos;
        portEXIT_CRITICAL(&timerMux);
        err = t - c;
      }
    }

    if (err == 0) {
      axes[i].currentPps = 0.0f;
      if (activeNow) stopAxisPulseTimer(i);
      continue;
    }

    int8_t desiredSign = (err > 0) ? +1 : -1;

    if (desiredSign != axes[i].currentDirSign && axes[i].currentPps > 0.0f) {
      float maxDelta = ACCEL_PPS2 * dt;
      axes[i].currentPps -= maxDelta;
      if (axes[i].currentPps < 0.0f) axes[i].currentPps = 0.0f;

      if (axes[i].currentPps == 0.0f) {
        if (activeNow) stopAxisPulseTimer(i);
        setDirectionFromSign(i, desiredSign);
      } else {
        axes[i].isrStepRate = (uint32_t)(axes[i].currentPps * 2.0f);
      }
      continue;
    }

    if (desiredSign != axes[i].currentDirSign && axes[i].currentPps == 0.0f) {
      setDirectionFromSign(i, desiredSign);
    }

    float distance = (float)abs(err);
    float stoppingDistance = (axes[i].currentPps * axes[i].currentPps) / (2.0f * ACCEL_PPS2);
    float desiredPps = (distance <= stoppingDistance) ? 0.0f : MAX_PPS;

    float maxDelta = ACCEL_PPS2 * dt;
    if (axes[i].currentPps < desiredPps) {
      axes[i].currentPps += maxDelta;
      if (axes[i].currentPps > desiredPps) axes[i].currentPps = desiredPps;
    } else if (axes[i].currentPps > desiredPps) {
      axes[i].currentPps -= maxDelta;
      if (axes[i].currentPps < desiredPps) axes[i].currentPps = desiredPps;
    }

    if (axes[i].currentPps <= 0.0f) {
      axes[i].currentPps = 0.0f;
      if (activeNow) stopAxisPulseTimer(i);
      continue;
    }

    axes[i].isrStepRate = (uint32_t)(axes[i].currentPps * 2.0f);

    if (!activeNow) {
      portENTER_CRITICAL(&timerMux);
      axes[i].timerActive = true;
      portEXIT_CRITICAL(&timerMux);
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

  for (int i=0; i<AXES_TOTAL; i++) {
    int32_t c, t;
    bool activeNow;
    portENTER_CRITICAL(&timerMux);
    c = axes[i].currentPos;
    t = axes[i].targetPos;
    activeNow = axes[i].timerActive;
    portEXIT_CRITICAL(&timerMux);
    
    Serial.print("A"); Serial.print(i); Serial.print(":");
    if (axes[i].runMode == MODE_HOMING_SEEK) Serial.print("HSK");
    else if (axes[i].runMode == MODE_HOMING_BACKOFF) Serial.print("HBO");
    else Serial.print("ACT");
    
    Serial.print(" P:"); Serial.print(c);
    Serial.print(" T:"); Serial.print(t);
    Serial.print(" ");
  }
  Serial.println();
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(BAUD);

  for (int i = 0; i < AXES_TOTAL; i++) {
    pinMode(config[i].pulPin, OUTPUT);
    pinMode(config[i].dirPin, OUTPUT);
    pinMode(config[i].homePin, INPUT_PULLUP);

    digitalWrite(config[i].pulPin, HIGH);
    
    axes[i].runMode = MODE_HOMING_SEEK;
    axes[i].switchStableSeen = false;
    axes[i].switchSeenMs = 0;
    axes[i].timerActive = false;
    axes[i].pulseIsLow = false;
    axes[i].currentPos = TRAVEL_CENTER;
    axes[i].targetPos = TRAVEL_CENTER;
    axes[i].accumulator = 0;
    axes[i].currentPps = 0.0f;
    axes[i].isrStepRate = 0;
    
    setDirectionFromSign(i, +1);
  }

  // 1 MHz hardware timer base
  pulseTimer = timerBegin(1000000);
  timerAttachInterrupt(pulseTimer, &onPulseTimer);
  // Set alarm to trigger every 10 microseconds (100 kHz)
  timerAlarm(pulseTimer, 10, true, 0);
  timerRestart(pulseTimer);
  timerStart(pulseTimer);

  lastPacketMs = millis();

  Serial.println("6-axis FlyPT servo controller with boot homing (ESP32-S3 DDA)");
  Serial.print("Travel min/max/center: ");
  Serial.print(TRAVEL_MIN); Serial.print(" / ");
  Serial.print(TRAVEL_MAX); Serial.print(" / ");
  Serial.println(TRAVEL_CENTER);
  Serial.print("Max RPM: "); Serial.println(MAX_RPM);
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
