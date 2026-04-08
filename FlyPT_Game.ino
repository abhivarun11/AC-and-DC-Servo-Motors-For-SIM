/*
 Motion Simulator Controller

*/

#include <Arduino.h>

// ===== PINS =====
#define STEP_PIN 2   // Geen Colour Pin
#define DIR_PIN  4   // black colour pin // yellow and blue colour pins connect to 3.3v pin in esp32
#define EN_PIN   21  // optional pin need not required

// ===== TRAVEL SETTINGS =====
#define MAX_STROKE 40000L
#define CENTER_POS (MAX_STROKE / 2)

// ===== MOTION LIMITS =====
#define MAX_SPEED 15000.0f     // steps/sec
#define ACCEL     40000.0f     // steps/sec²
#define DEAD_BAND 5            // stop jitter

// ===== SIGNAL TIMEOUT =====
#define SIGNAL_TIMEOUT 500     // ms

// ===== TIMER =====
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ===== STATE =====
volatile long currentPos = CENTER_POS;
volatile long targetPos  = CENTER_POS;
volatile float speedSPS  = 0;
volatile bool stepState  = false;

unsigned long lastDataTime = 0;


// ======================================================
// STEP GENERATION ISR (NO DELAYS)
// ======================================================
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  long error = targetPos - currentPos;

  if (abs(error) > DEAD_BAND)
  {
    bool dir = (error > 0);
    digitalWrite(DIR_PIN, dir);

    stepState = !stepState;
    digitalWrite(STEP_PIN, stepState);

    if (stepState)  // count only on rising edge
      currentPos += dir ? 1 : -1;
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}


// ======================================================
// MOTION UPDATE — ACCEL + DECEL CONTROL
// ======================================================
void updateMotion(float dt)
{
  long error = targetPos - currentPos;

  if (abs(error) <= DEAD_BAND)
  {
    speedSPS = 0;
    timerAlarm(timer, 0, true, 0);
    return;
  }

  float distance = abs(error);

  // braking distance
  float brakeDist = (speedSPS * speedSPS) / (2.0f * ACCEL);

  // DECELERATION
  if (brakeDist >= distance)
  {
    speedSPS -= ACCEL * dt;
    if (speedSPS < 0) speedSPS = 0;
  }
  else
  {
    // ACCELERATION
    speedSPS += ACCEL * dt;
    if (speedSPS > MAX_SPEED) speedSPS = MAX_SPEED;
  }

  if (speedSPS < 1) speedSPS = 1;

  uint32_t interval = (uint32_t)(1000000.0f / speedSPS);

  timerAlarm(timer, interval, true, 0);
}


// ======================================================
// READ FlyPT BINARY STREAM
// ======================================================
void readFlyPT()
{
  while (Serial.available() >= 2)
  {
    uint8_t low  = Serial.read();
    uint8_t high = Serial.read();

    uint16_t value = (high << 8) | low;

    long newTarget = map(value, 0, 65535, 0, MAX_STROKE);

    portENTER_CRITICAL(&timerMux);
    targetPos = constrain(newTarget, 0, MAX_STROKE);
    portEXIT_CRITICAL(&timerMux);

    lastDataTime = millis();
  }
}


// ======================================================
// FAILSAFE — SIGNAL LOSS
// ======================================================
void checkSignalTimeout()
{
  if (millis() - lastDataTime > SIGNAL_TIMEOUT)
  {
    targetPos = CENTER_POS;  // return to safe position
  }
}


// ======================================================
// SETUP
// ======================================================
void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);

  // 1 MHz timer
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000, true, 0);

  lastDataTime = millis();
}


// ======================================================
// MAIN LOOP
// ======================================================
void loop()
{
  readFlyPT();
  checkSignalTimeout();

  static uint32_t lastTime = millis();
  uint32_t now = millis();

  if (now - lastTime >= 5)   // 200 Hz control loop
  {
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    updateMotion(dt);
  }
}