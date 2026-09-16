#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// USER SETTINGS
// =====================================================
static const uint32_t BAUD = 115200;
static const uint8_t SYNC1 = 0x21;   // '!'
static const uint8_t SYNC2 = 0x24;   // '$'
static const uint8_t AXES_TOTAL = 6;
static const uint8_t PAYLOAD_BYTES = AXES_TOTAL * 2;

// =====================================================
// WIFI TELEMETRY SETTINGS
// =====================================================
const char* WIFI_SSID = "Bare Metal Sim Systems";
const char* WIFI_PASS = "bmss@1690"; 
WebServer server(80);

IPAddress staticIP(192, 168, 29, 200);
IPAddress gateway(192, 168, 29, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// =====================================================
// RS485 SETTINGS (For Absolute Encoders)
// =====================================================
// We reuse the old limit switch pins since we don't need them anymore!
#define RS485_RX_PIN 13
#define RS485_TX_PIN 5
#define RS485_DE_RE_PIN 10
#define RS485_BAUD 57600
#define MODBUS_TIMEOUT_MS 100

// =====================================================
// HARDWARE PIN SETTINGS
// =====================================================
struct AxisConfig {
  uint8_t pulPin;
  uint8_t dirPin;
  uint8_t modbusId; // The ID set in parameter H0C_00 on each drive
};

// DO NOT CHANGE THESE PINS! We removed the home pins.
const AxisConfig config[AXES_TOTAL] = {
  { 2,  4,  6 }, // ACTUATOR 5
  { 7,  14, 4 }, // ACTUATOR 6
  { 8,  9,  5 }, // ACTUATOR 4
  {40, 41,  3 }, // ACTUATOR 3
  {15, 16,  1 }, // ACTUATOR 1
  {18, 21,  2 }  // ACTUATOR 2
};

// =====================================================
// ACTUATOR / TRAVEL SETTINGS
// =====================================================
static const int32_t PULSES_PER_REV = 500;
static const int32_t TOTAL_REV_RANGE = 10;
static const int32_t TRAVEL_MIN = 0;
static const int32_t TRAVEL_MAX = TOTAL_REV_RANGE * PULSES_PER_REV;   // 5000
static const int32_t TRAVEL_CENTER = (TRAVEL_MIN + TRAVEL_MAX) / 2;   // 2500

// --- CUSTOMER-READY SAFETY ENDSTOPS ---
// This creates a 5% physical deadzone at the absolute top and bottom.
// If bad telemetry commands 0% or 100%, the rig stops before hitting the metal hard-stops.
static const int32_t TRAVEL_PADDING = (TRAVEL_MAX - TRAVEL_MIN) * 0.05f; // 250 pulses
static const int32_t SAFE_MIN = TRAVEL_MIN + TRAVEL_PADDING; // 250
static const int32_t SAFE_MAX = TRAVEL_MAX - TRAVEL_PADDING; // 4750

// =====================================================
// ACTIVE RUN SPEED SETTINGS
// =====================================================
static const float MAX_RPM = 2300.0f;
static const float MAX_PPS = (MAX_RPM * PULSES_PER_REV) / 60.0f;
static const float STARTUP_RPM = 40.0f; // 40 RPM slow homing speed
static const float STARTUP_PPS = (STARTUP_RPM * PULSES_PER_REV) / 60.0f;
static const float ACCEL_PPS2 = 50000.0f; // Lowered from 250k for smooth start

float currentMaxPps = STARTUP_PPS; // Start locked at 40 RPM
// =====================================================
// TIMING
// =====================================================
static const uint32_t CONTROL_PERIOD_MS = 5;
static const uint32_t PACKET_TIMEOUT_MS = 200;
static const bool RETURN_TO_CENTER_ON_TIMEOUT = true;

// =====================================================
// EMERGENCY STOP SETTINGS
// =====================================================
static const uint8_t ESTOP_PIN = 39; // Restored to 39 (Pin 32 crashes PSRAM on ESP32-S3!)
static const int ESTOP_ACTIVE_STATE = LOW;

// =====================================================
// DIRECTION POLARITY
// =====================================================
static const bool CW_DIR_LEVEL = LOW;
static const bool CCW_DIR_LEVEL = HIGH;

// =====================================================
// TIMER / MOTION STATE
// =====================================================
static const uint32_t TIMER_FREQ = 100000; 

enum RunMode {
  MODE_ACTIVE,
  MODE_ESTOP,
  MODE_MODBUS_ERROR // If we can't read the encoders at startup
};

struct AxisState {
  volatile int32_t currentPos; 
  int32_t targetPos;
  RunMode runMode;

  float currentPps;
  int8_t currentDirSign;
  
  volatile uint32_t isrStepRate;
  volatile uint32_t accumulator;
  volatile bool timerActive;
  volatile bool pulseIsLow;
  
  volatile int32_t realAbsolutePos; // True Modbus encoder position tracking
};

AxisState axes[AXES_TOTAL];
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t rxBuffer[PAYLOAD_BYTES];
uint8_t rxIndex = 0;
uint32_t lastPacketTime = 0;
uint32_t lastControlTime = 0;
uint32_t lastPrintTime = 0;
bool packetTimeoutActive = false;

// =====================================================
// FAST PORT MANIPULATION
// =====================================================
static inline void fastSetPulseLow(uint8_t axis) {
  uint32_t pin = config[axis].pulPin;
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << pin));
  else REG_WRITE(GPIO_OUT1_W1TC_REG, (1UL << (pin - 32)));
}

static inline void fastSetPulseHigh(uint8_t axis) {
  uint32_t pin = config[axis].pulPin;
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << pin));
  else REG_WRITE(GPIO_OUT1_W1TS_REG, (1UL << (pin - 32)));
}

// =====================================================
// MODBUS RTU FUNCTIONS
// =====================================================
uint16_t calculateCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool readAbsoluteEncoder(uint8_t modbusId, int32_t &position) {
  uint8_t req[8];
  req[0] = modbusId;
  req[1] = 0x03; // Read Holding Registers
  req[2] = 0x0B; // Register 0x0B07 (H0B_07)
  req[3] = 0x07;
  req[4] = 0x00;
  req[5] = 0x02; // Read 2 registers (32 bits)
  
  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  // Clear serial buffer
  while(Serial1.available()) Serial1.read();

  // Send request
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  Serial1.write(req, 8);
  Serial1.flush();
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // Wait for response (9 bytes expected)
  uint32_t startMs = millis();
  uint8_t resp[9];
  int idx = 0;
  
  while (millis() - startMs < MODBUS_TIMEOUT_MS) {
    if (Serial1.available()) {
      resp[idx++] = Serial1.read();
      if (idx == 9) break;
    } else {
      yield(); // Safely yield without forcing a strict 1ms sleep schedule
    }
  }

  if (idx < 9) {
    Serial.printf("Modbus Error: Axis %d Timeout! Expected 9 bytes, got %d\n", modbusId, idx);
    return false;
  }

  // Check CRC
  uint16_t respCrc = calculateCRC(resp, 7);
  if (resp[7] != (respCrc & 0xFF) || resp[8] != ((respCrc >> 8) & 0xFF)) {
    Serial.printf("Modbus Error: Axis %d CRC Failed!\n", modbusId);
    return false;
  }

  // Assuming H0C_26 = 1 (Low 16 bits first, then High 16 bits)
  // resp[3] = Low Word High Byte
  // resp[4] = Low Word Low Byte
  // resp[5] = High Word High Byte
  // resp[6] = High Word Low Byte
  uint16_t lowWord = (resp[3] << 8) | resp[4];
  uint16_t highWord = (resp[5] << 8) | resp[6];
  position = (int32_t)((highWord << 16) | lowWord);
  
  return true;
}

// =====================================================
// TIMER INTERRUPT
// =====================================================
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  for (uint8_t i = 0; i < AXES_TOTAL; i++) {
    if (!axes[i].timerActive) continue;

    axes[i].accumulator += axes[i].isrStepRate;
    
    while (axes[i].accumulator >= TIMER_FREQ) {
      axes[i].accumulator -= TIMER_FREQ;
      
      if (!axes[i].pulseIsLow) {
        fastSetPulseLow(i);
        axes[i].pulseIsLow = true;
      } else {
        fastSetPulseHigh(i);
        axes[i].pulseIsLow = false;
        axes[i].currentPos += axes[i].currentDirSign;
      }
    }
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

// =====================================================
// HELPER FUNCTIONS
// =====================================================
void startAxisPulseTimer(int i) {
  portENTER_CRITICAL(&timerMux);
  axes[i].timerActive = true;
  portEXIT_CRITICAL(&timerMux);
}

void stopAxisPulseTimer(int i) {
  portENTER_CRITICAL(&timerMux);
  axes[i].timerActive = false;
  axes[i].accumulator = 0;
  portEXIT_CRITICAL(&timerMux);
  
  fastSetPulseHigh(i);
  axes[i].pulseIsLow = false;
}

void setDirectionFromSign(int i, int8_t sign) {
  axes[i].currentDirSign = sign;
  bool level = (sign == +1) ? CW_DIR_LEVEL : CCW_DIR_LEVEL;
  digitalWrite(config[i].dirPin, level);
}

// =====================================================
// FLYPT & SIMHUB DUAL PARSER
// =====================================================
bool isSimHubMode = false;
char simHubBuffer[128];
uint8_t simHubIndex = 0;

void parseSerialStream() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (rxIndex == 0) {
      if (b == SYNC1) {
        isSimHubMode = false;
        rxIndex = 1; 
        continue;
      }
      // SimHub sync byte '['
      if (b == '[') {
        isSimHubMode = true;
        simHubIndex = 0;
        rxIndex = 1;
        continue;
      }
    }

    if (!isSimHubMode) {
      // --- FLYPT BINARY MODE ---
      if (rxIndex == 1) {
        if (b == SYNC2) {
          rxIndex = 2;
        } else if (b == SYNC1) {
          rxIndex = 1;
        } else {
          rxIndex = 0;
        }
        continue;
      }
      
      if (rxIndex >= 2) {
        rxBuffer[rxIndex - 2] = b;
        rxIndex++;
  
        if (rxIndex - 2 == PAYLOAD_BYTES) {
          rxIndex = 0;
          bool allActive = true;
          for (int i = 0; i < AXES_TOTAL; i++) {
            if (axes[i].runMode != MODE_ACTIVE) {
              allActive = false;
              break;
            }
          }
          if (allActive) {
            portENTER_CRITICAL(&timerMux);
            for (int i = 0; i < AXES_TOTAL; i++) {
              int byteOffset = i * 2;
              // FlyPT <AxisXa> sends Big Endian (High Byte first, Low Byte second)
              uint16_t val16 = (rxBuffer[byteOffset] << 8) | rxBuffer[byteOffset + 1];
  
              float fraction = (float)val16 / 65535.0f;
              int32_t span = TRAVEL_MAX - TRAVEL_MIN;
              int32_t t = TRAVEL_MIN + (int32_t)(fraction * span);
  
              // Customer-Ready Hardware Protection
              if (t < SAFE_MIN) t = SAFE_MIN;
              if (t > SAFE_MAX) t = SAFE_MAX;
              
              axes[i].targetPos = t;
            }
            portEXIT_CRITICAL(&timerMux);
            lastPacketTime = millis();
            packetTimeoutActive = false;
          }
        }
      }
    } else {
      // --- SIMHUB STRING MODE ---
      if (b == '\n' || b == 10 || b == ']') {
        simHubBuffer[simHubIndex] = '\0';
        rxIndex = 0; // reset
        
        bool allActive = true;
        for (int i = 0; i < AXES_TOTAL; i++) {
          if (axes[i].runMode != MODE_ACTIVE) {
            allActive = false;
            break;
          }
        }
        
        if (allActive) {
          portENTER_CRITICAL(&timerMux);
          char* ptr = simHubBuffer;
          for (int i = 0; i < AXES_TOTAL; i++) {
            if (ptr == NULL || *ptr == '\0') break;
            
            uint16_t val16 = (uint16_t)atol(ptr);
            float fraction = (float)val16 / 65535.0f;
            int32_t span = TRAVEL_MAX - TRAVEL_MIN;
            int32_t t = TRAVEL_MIN + (int32_t)(fraction * span);
            
            // Customer-Ready Hardware Protection
            if (t < SAFE_MIN) t = SAFE_MIN;
            if (t > SAFE_MAX) t = SAFE_MAX;
            
            axes[i].targetPos = t;
            
            ptr = strchr(ptr, ',');
            if (ptr) ptr++; // skip comma
          }
          portEXIT_CRITICAL(&timerMux);
          lastPacketTime = millis();
          packetTimeoutActive = false;
        }
      } else if (b == '\r') {
        // ignore carriage returns
      } else {
        if (simHubIndex < 127) {
          simHubBuffer[simHubIndex++] = b;
        } else {
          rxIndex = 0; // abort on overflow
        }
      }
    }
  }
}

// =====================================================
// CONTROL LOOP
// =====================================================
void controlUpdate() {
  uint32_t nowMs = millis();
  float dt = (nowMs - lastControlTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f; 
  lastControlTime = nowMs;

  bool estopTriggered = (digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_STATE);

  for (int i = 0; i < AXES_TOTAL; i++) {
    int32_t c, t, absPos;
    bool activeNow;
    
    portENTER_CRITICAL(&timerMux);
    c = axes[i].currentPos;
    t = axes[i].targetPos;
    absPos = axes[i].realAbsolutePos;
    activeNow = axes[i].timerActive;
    portEXIT_CRITICAL(&timerMux);

    if (axes[i].runMode == MODE_MODBUS_ERROR) {
       // Do nothing until restarted
       continue;
    }

    if (estopTriggered && axes[i].runMode != MODE_ESTOP) {
      axes[i].runMode = MODE_ESTOP;
      axes[i].currentPps = 0.0f;
      stopAxisPulseTimer(i);
      continue; 
    }
    if (axes[i].runMode == MODE_ESTOP) {
      if (!estopTriggered) {
        axes[i].runMode = MODE_ACTIVE; // Recover from Estop
      } else {
        continue;
      }
    }

    if (packetTimeoutActive) {
      if (RETURN_TO_CENTER_ON_TIMEOUT) {
        t = TRAVEL_CENTER; // Override target to safely park at 2500!
      }
      
      if (!RETURN_TO_CENTER_ON_TIMEOUT) {
        axes[i].currentPps = 0.0f;
        if (activeNow) stopAxisPulseTimer(i);
        continue;
      }
    }
    
    // -------------------------------------------------------------------
    // IN-GAME IDLE DRIFT AUTO-SYNC
    // -------------------------------------------------------------------
    // If the motor is currently stopped (0 PPS) AND it thinks it has successfully 
    // reached its target (e.g. game is paused or resting), we instantly erase any drift!
    if (axes[i].currentPps == 0.0f && abs(t - c) <= 10) {
      if (abs(c - absPos) > 50) {
        portENTER_CRITICAL(&timerMux);
        axes[i].currentPos = absPos;
        portEXIT_CRITICAL(&timerMux);
        c = absPos; // Update our local variable so 'err' is calculated correctly below!
      }
    }

    int32_t err = t - c;

    if (!packetTimeoutActive) {
      if (c <= TRAVEL_MIN && err < 0) {
        err = 0;
      } else if (c >= TRAVEL_MAX && err > 0) {
        err = 0;
      } else {
        int32_t distToMin = c - TRAVEL_MIN;
        if (err < 0 && abs(err) > distToMin) err = -distToMin;

        int32_t distToMax = TRAVEL_MAX - c;
        if (err > 0 && err > distToMax) err = distToMax;
      }
    }

    // Removed the err == 0 instant stop block to prevent inertial skidding!
    // The normal acceleration/deceleration logic below will now smoothly brake the actuator.

    // -------------------------------------------------------------------
    // EXTREME DRIFT / HARD-CRASH AUTO-SYNC
    // -------------------------------------------------------------------
    // If the motor physically falls below the floor or above the ceiling 
    // due to extreme hardware pulse-loss, we MUST instantly synchronize 
    // the internal math with reality so the high-speed loop can pull it back!
    if (absPos < TRAVEL_MIN - 50 || absPos > TRAVEL_MAX + 50) {
      // The motor is severely out of bounds! 
      // Update the internal math to reflect this broken reality.
      if (abs(c - absPos) > 50) {
        portENTER_CRITICAL(&timerMux);
        axes[i].currentPos = absPos;
        portEXIT_CRITICAL(&timerMux);
        c = absPos;
        err = t - c; // Recalculate error! Now the loop will naturally pull it back to target!
      }
    }

    // -------------------------------------------------------------------
    // REAL-TIME MODBUS SAFETY BOUNDS (Brick Wall)
    // -------------------------------------------------------------------
    if (absPos <= TRAVEL_MIN && err < 0) {
      // If we are physically at or below the floor, REFUSE to go further down!
      err = 0; 
    } else if (absPos >= TRAVEL_MAX && err > 0) {
      // If we are physically at or above the ceiling, REFUSE to go further up!
      err = 0;
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
    float desiredPps = (distance <= stoppingDistance) ? 0.0f : currentMaxPps;

    float maxDelta = ACCEL_PPS2 * dt;
    if (axes[i].currentPps < desiredPps) {
      axes[i].currentPps += maxDelta;
      if (axes[i].currentPps > desiredPps) axes[i].currentPps = desiredPps;
    } else if (axes[i].currentPps > desiredPps) {
      axes[i].currentPps -= maxDelta;
      if (axes[i].currentPps < desiredPps) axes[i].currentPps = desiredPps;
    }

    axes[i].isrStepRate = (uint32_t)(axes[i].currentPps * 2.0f);

    if (axes[i].currentPps > 0.0f && !activeNow) {
      startAxisPulseTimer(i);
    } else if (axes[i].currentPps == 0.0f && activeNow) {
      stopAxisPulseTimer(i);
    }
  }

  // Unlock full speed once all actuators reach their initial target AFTER FlyPT connects
  if (currentMaxPps < MAX_PPS && lastPacketTime > 0) {
    bool allReached = true;
    for (int i = 0; i < AXES_TOTAL; i++) {
      if (abs(axes[i].targetPos - axes[i].currentPos) > 10) {
        allReached = false;
        break;
      }
    }
    if (allReached) {
      currentMaxPps = MAX_PPS; // Unlock 2300 RPM!
      Serial.println("Rig centered! Unlocking full 2300 RPM speed!");
    }
  }
}

// =====================================================
// BACKGROUND DRIFT CORRECTION TASK (CORE 0)
// =====================================================
TaskHandle_t modbusTaskHandle;

void modbusDriftCorrectionTask(void * pvParameters) {
  for(;;) {
    for (int i = 0; i < AXES_TOTAL; i++) {
       int32_t absolutePos = 0;
       
       if (readAbsoluteEncoder(config[i].modbusId, absolutePos)) {
          portENTER_CRITICAL(&timerMux);
          
          // Constantly update the Real-Time Absolute Position for the Safety Bounds
          axes[i].realAbsolutePos = absolutePos;
          
          portEXIT_CRITICAL(&timerMux);
       }
       
       // Wait 5ms before pinging the next drive to not overload the RS485 bus
       vTaskDelay(pdMS_TO_TICKS(5)); 
    }
  }
}

// =====================================================
// EMBEDDED WEB DASHBOARD
// =====================================================
const char* DASHBOARD_HTML = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Motion Rig Telemetry</title>
    <style>
        :root {
            --bg: #0f172a;
            --surface: rgba(30, 41, 59, 0.7);
            --border: rgba(255, 255, 255, 0.1);
            --primary: #3b82f6;
            --success: #10b981;
            --danger: #ef4444;
            --warning: #f59e0b;
            --text: #f8fafc;
            --text-muted: #94a3b8;
        }

        body {
            background-color: var(--bg);
            color: var(--text);
            font-family: 'Inter', -apple-system, sans-serif;
            margin: 0;
            padding: 2rem;
            min-height: 100vh;
            background-image: 
                radial-gradient(at 0% 0%, rgba(59, 130, 246, 0.15) 0px, transparent 50%),
                radial-gradient(at 100% 100%, rgba(16, 185, 129, 0.1) 0px, transparent 50%);
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 3rem;
        }

        .title h1 {
            margin: 0;
            font-size: 2.5rem;
            font-weight: 800;
            letter-spacing: -1px;
            background: linear-gradient(to right, #60a5fa, #34d399);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .title p {
            margin: 0.5rem 0 0;
            color: var(--text-muted);
        }

        .connection {
            display: flex;
            align-items: center;
            gap: 1rem;
            background: var(--surface);
            padding: 0.75rem 1.5rem;
            border-radius: 999px;
            border: 1px solid var(--border);
            backdrop-filter: blur(10px);
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background-color: var(--danger);
            box-shadow: 0 0 10px var(--danger);
            transition: all 0.3s;
        }

        .status-dot.connected {
            background-color: var(--success);
            box-shadow: 0 0 10px var(--success);
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 1.5rem;
        }

        .card {
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: 1.5rem;
            padding: 1.5rem;
            backdrop-filter: blur(12px);
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
            transition: transform 0.2s, box-shadow 0.2s;
        }

        .card:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05);
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
            border-bottom: 1px solid var(--border);
            padding-bottom: 1rem;
        }

        .axis-name {
            font-size: 1.25rem;
            font-weight: 600;
            margin: 0;
        }

        .badge {
            padding: 0.25rem 0.75rem;
            border-radius: 999px;
            font-size: 0.75rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        .badge.moving {
            background: rgba(59, 130, 246, 0.2);
            color: #60a5fa;
            border: 1px solid rgba(59, 130, 246, 0.3);
        }

        .badge.parked {
            background: rgba(148, 163, 184, 0.2);
            color: #cbd5e1;
            border: 1px solid rgba(148, 163, 184, 0.3);
        }

        .stats-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            margin-bottom: 1.5rem;
        }

        .stat {
            background: rgba(0, 0, 0, 0.2);
            padding: 1rem;
            border-radius: 1rem;
        }

        .stat-label {
            font-size: 0.75rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 0.5rem;
        }

        .stat-value {
            font-size: 1.5rem;
            font-weight: 700;
            font-variant-numeric: tabular-nums;
        }

        .stat.drift .stat-value {
            color: var(--success);
        }

        .stat.drift.danger .stat-value {
            color: var(--danger);
        }

        .bar-container {
            position: relative;
            height: 40px;
            background: rgba(0, 0, 0, 0.3);
            border-radius: 999px;
            overflow: hidden;
            margin-top: 1rem;
            border: 1px solid rgba(255, 255, 255, 0.05);
        }

        .bar-marker {
            position: absolute;
            top: 0;
            height: 100%;
            width: 4px;
            margin-left: -2px;
            border-radius: 2px;
            transition: left 0.1s linear;
        }

        .marker-target {
            background: var(--primary);
            box-shadow: 0 0 10px var(--primary);
            opacity: 0.5;
            z-index: 1;
        }

        .marker-current {
            background: var(--success);
            box-shadow: 0 0 10px var(--success);
            z-index: 2;
        }

        .marker-absolute {
            background: var(--warning);
            box-shadow: 0 0 10px var(--warning);
            height: 50%;
            top: 25%;
            z-index: 3;
        }

        .legend {
            display: flex;
            gap: 1rem;
            margin-top: 0.5rem;
            font-size: 0.75rem;
            color: var(--text-muted);
            justify-content: center;
        }

        .legend-item {
            display: flex;
            align-items: center;
            gap: 0.25rem;
        }

        .legend-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
        }
    </style>
</head>
<body>

    <div class="header">
        <div class="title">
            <h1>Motion Rig Telemetry</h1>
            <p>Live Real-Time Feedback directly from ESP32</p>
        </div>
        <div class="connection">
            <div id="statusDot" class="status-dot"></div>
            <span id="statusText">Disconnected</span>
        </div>
    </div>

    <div class="grid" id="grid">
        <!-- Cards injected by JS -->
    </div>

    <script>
        const MAX_PULSES = 5000;
        const FETCH_URL = '/telemetry'; // Dynamic relative URL since it's hosted on ESP32 directly!

        const actuatorNames = ["Actuator 5", "Actuator 6", "Actuator 4", "Actuator 3", "Actuator 1", "Actuator 2"];

        function createCards() {
            const grid = document.getElementById('grid');
            let html = '';
            for(let i=0; i<6; i++) {
                html += `
                <div class="card">
                    <div class="card-header">
                        <h2 class="axis-name">${actuatorNames[i]}</h2>
                        <div id="badge-${i}" class="badge parked">Parked</div>
                    </div>
                    <div class="stats-grid">
                        <div class="stat">
                            <div class="stat-label">Internal Pos</div>
                            <div id="val-c-${i}" class="stat-value">0</div>
                        </div>
                        <div class="stat">
                            <div class="stat-label">Modbus Pos</div>
                            <div id="val-a-${i}" class="stat-value">0</div>
                        </div>
                        <div class="stat">
                            <div class="stat-label">FlyPT Target</div>
                            <div id="val-t-${i}" class="stat-value">0</div>
                        </div>
                        <div id="drift-container-${i}" class="stat drift">
                            <div class="stat-label">Drift (Error)</div>
                            <div id="val-d-${i}" class="stat-value">0</div>
                        </div>
                    </div>
                    
                    <div class="bar-container">
                        <div id="bar-t-${i}" class="bar-marker marker-target" style="left: 50%"></div>
                        <div id="bar-a-${i}" class="bar-marker marker-absolute" style="left: 50%"></div>
                        <div id="bar-c-${i}" class="bar-marker marker-current" style="left: 50%"></div>
                    </div>
                    <div class="legend">
                        <div class="legend-item"><div class="legend-dot" style="background: var(--primary)"></div>Target</div>
                        <div class="legend-item"><div class="legend-dot" style="background: var(--success)"></div>Internal</div>
                        <div class="legend-item"><div class="legend-dot" style="background: var(--warning)"></div>Modbus</div>
                    </div>
                </div>
                `;
            }
            grid.innerHTML = html;
        }

        function updateUI(data) {
            document.getElementById('statusDot').classList.add('connected');
            document.getElementById('statusText').innerText = 'Live';

            data.forEach((axis, i) => {
                const c = axis.c;
                const t = axis.t;
                const a = axis.a;
                const drift = Math.abs(c - a);

                document.getElementById(`val-c-${i}`).innerText = c;
                document.getElementById(`val-t-${i}`).innerText = t;
                document.getElementById(`val-a-${i}`).innerText = a;
                document.getElementById(`val-d-${i}`).innerText = drift;

                const driftContainer = document.getElementById(`drift-container-${i}`);
                if (drift > 50) driftContainer.classList.add('danger');
                else driftContainer.classList.remove('danger');

                const badge = document.getElementById(`badge-${i}`);
                if (axis.s === 1) {
                    badge.className = 'badge moving';
                    badge.innerText = 'Moving';
                } else {
                    badge.className = 'badge parked';
                    badge.innerText = 'Parked';
                }

                document.getElementById(`bar-c-${i}`).style.left = Math.min(100, Math.max(0, (c / MAX_PULSES) * 100)) + '%';
                document.getElementById(`bar-t-${i}`).style.left = Math.min(100, Math.max(0, (t / MAX_PULSES) * 100)) + '%';
                document.getElementById(`bar-a-${i}`).style.left = Math.min(100, Math.max(0, (a / MAX_PULSES) * 100)) + '%';
            });
        }

        async function fetchTelemetry() {
            try {
                const controller = new AbortController();
                const timeoutId = setTimeout(() => controller.abort(), 2000);
                
                const response = await fetch(FETCH_URL, { signal: controller.signal });
                clearTimeout(timeoutId);
                
                if (response.ok) {
                    const data = await response.json();
                    updateUI(data);
                }
            } catch (err) {
                document.getElementById('statusDot').classList.remove('connected');
                document.getElementById('statusText').innerText = 'Disconnected';
            }
            
            setTimeout(fetchTelemetry, 100);
        }

        createCards();
        fetchTelemetry();

    </script>
</body>
</html>
)rawliteral";

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(BAUD);
  
  // --- START WIFI TELEMETRY SERVER ---
  Serial.println("Connecting to Home WiFi...");
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(staticIP, gateway, subnet, dns)) {
    Serial.println("Static IP Failed to configure");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 10) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi Connected! Dashboard IP: ");
    Serial.println(WiFi.localIP()); 
  } else {
    Serial.println("\nWiFi Failed to connect. Running offline.");
  }
  
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html", DASHBOARD_HTML);
  });

  server.on("/telemetry", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    char json[512];
    int len = snprintf(json, sizeof(json), "[");
    for (int i = 0; i < AXES_TOTAL; i++) {
      len += snprintf(json + len, sizeof(json) - len, 
        "{\"t\":%ld,\"c\":%ld,\"a\":%ld,\"s\":%d}%s", 
        (long)axes[i].targetPos, 
        (long)axes[i].currentPos, 
        (long)axes[i].realAbsolutePos,
        (axes[i].currentPps > 0.0f) ? 1 : 0,
        (i < AXES_TOTAL - 1) ? "," : "");
    }
    snprintf(json + len, sizeof(json) - len, "]");
    server.send(200, "application/json", json);
  });
  server.begin();
  // -----------------------------------
  
  // Initialize RS485 Serial Port
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW); // Set to receive mode
  Serial1.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("Starting Absolute Encoder Modbus Initialization...");

  for (int i = 0; i < AXES_TOTAL; i++) {
    pinMode(config[i].pulPin, OUTPUT);
    pinMode(config[i].dirPin, OUTPUT);
    digitalWrite(config[i].pulPin, HIGH);
    
    axes[i].timerActive = false;
    axes[i].pulseIsLow = false;
    axes[i].accumulator = 0;
    axes[i].isrStepRate = 0;
    axes[i].currentPps = 0.0f;
    setDirectionFromSign(i, 1);
    
    axes[i].runMode = MODE_MODBUS_ERROR; // Start locked until successfully read
  }

  Serial.println("Waiting for all 6 Servo Drives to power on...");
  bool allDrivesReady = false;
  
  while (!allDrivesReady) {
    allDrivesReady = true;
    for (int i = 0; i < AXES_TOTAL; i++) {
      if (axes[i].runMode == MODE_ACTIVE) continue; // Already successfully read this one

      int32_t absolutePos = 0;
      if (readAbsoluteEncoder(config[i].modbusId, absolutePos)) {
        Serial.printf("Axis %d (ID %d) Position successfully read: %d\n", i, config[i].modbusId, absolutePos);
        axes[i].currentPos = absolutePos;
        axes[i].targetPos = absolutePos;
        axes[i].runMode = MODE_ACTIVE;
      } else {
        allDrivesReady = false; // At least one drive is still not responding
      }
    }
    
    if (!allDrivesReady) {
      delay(500); // Wait half a second before trying to ping the missing drives again
    }
  }
  
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  
  // ESP32 Arduino Core v3.x Timer API
  timer = timerBegin(1000000); // 1 MHz timer (1 tick = 1us)
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10, true, 0); // Trigger alarm every 10 ticks (10us), auto-reload
  
  Serial.println("Initialization Complete. Ready for FlyPT.");
  
  // Launch the Background Drift Correction task on Core 0 (PRO_CPU)
  // This leaves Core 1 (APP_CPU) 100% free for high-speed FlyPT serial parsing and control
  xTaskCreatePinnedToCore(
      modbusDriftCorrectionTask, 
      "ModbusTask", 
      4096, 
      NULL, 
      1, 
      &modbusTaskHandle, 
      0 // Core 0
  );
}

void loop() {
  server.handleClient(); // Instantly serves the tiny JSON file without blocking the motion loop!
  
  parseSerialStream();

  uint32_t nowMs = millis();
  
  if (nowMs - lastPacketTime > PACKET_TIMEOUT_MS) {
    packetTimeoutActive = true;
  }

  if (nowMs - lastControlTime >= CONTROL_PERIOD_MS) {
    controlUpdate();
  }
}
