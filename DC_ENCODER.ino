#include <ModbusMaster.h>

#define PUL_PIN 2
#define DIR_PIN 4
#define RX_PIN 16
#define TX_PIN 17

ModbusMaster node;

void setup() {
  Serial.begin(115200);
  
  // Initialize RS485 for the Siheng driver
  Serial2.begin(57600, SERIAL_8N2, RX_PIN, TX_PIN);
  node.begin(1, Serial2);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.println("System Ready. Telemetry Dashboard Starting...");
  delay(2000);
}

void loop() {
  // ==========================================
  // 1. ANNOUNCE DIRECTION & MOVE FORWARD
  // ==========================================
  Serial.println("\n>>> Moving Forward at 1500 RPM... >>>");
  digitalWrite(DIR_PIN, LOW);

  // 40,000 pulses at 19us delay = ~1.5 seconds of movement at 1500 RPM
  for (long i = 0; i < 40000; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(9); 
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(9); 
  }

  // ==========================================
  // 2. STOP & PRINT DASHBOARD
  // ==========================================
  Serial.println("||| STOPPED |||");
  readFullTelemetry();
  delay(3000); // Wait 2 seconds so you can read it

  // ==========================================
  // 3. ANNOUNCE DIRECTION & MOVE REVERSE
  // ==========================================
  Serial.println("\n<<< Moving Reverse at 1500 RPM... <<<");
  digitalWrite(DIR_PIN, HIGH);

  for (long i = 0; i < 40000; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(9); 
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(9); 
  }

  // ==========================================
  // 4. STOP & PRINT DASHBOARD
  // ==========================================
  Serial.println("||| STOPPED |||");
  readFullTelemetry();
  delay(3000); // Wait 2 seconds so you can read it
}


// ==========================================
//      FULL TELEMETRY DASHBOARD FUNCTION
// ==========================================
void readFullTelemetry() {
  int16_t rpm = 0;
  float torque = 0;
  int32_t actualPos = 0;
  int32_t cmdPos = 0;
  int32_t errorPos = 0;
  float currentAmps = 0;

  Serial.println("=====================================");

  // --- CHUNK 1: Read Speed and Torque ---
  uint8_t result1 = node.readHoldingRegisters(0x0B00, 3);
  if (result1 == node.ku8MBSuccess) {
    rpm = (int16_t)node.getResponseBuffer(0);
    torque = (int16_t)node.getResponseBuffer(2) / 10.0; 
  } else {
    Serial.print("Chunk 1 Error: 0x"); Serial.println(result1, HEX);
  }

  // --- CHUNK 2: Read Positions ---
  uint8_t result2 = node.readHoldingRegisters(0x0B07, 8);
  if (result2 == node.ku8MBSuccess) {
    uint16_t actLow  = node.getResponseBuffer(0);
    uint16_t actHigh = node.getResponseBuffer(1);
    actualPos = (int32_t)((actHigh << 16) | actLow);

    uint16_t cmdLow  = node.getResponseBuffer(6);
    uint16_t cmdHigh = node.getResponseBuffer(7);
    cmdPos = (int32_t)((cmdHigh << 16) | cmdLow);

    // Calculate the exact error perfectly
    errorPos = cmdPos - actualPos;
  } 
  else {
    Serial.print("Chunk 2 Error: 0x"); Serial.println(result2, HEX);
  }

  // --- CHUNK 3: Read Phase Current ---
  uint8_t result3 = node.readHoldingRegisters(0x0B18, 1);
  if (result3 == node.ku8MBSuccess) {
    // Keep the (int16_t) fix so reverse current shows a correct negative number!
    currentAmps = (int16_t)node.getResponseBuffer(0) / 100.0;
  } else {
    Serial.print("Chunk 3 Error: 0x"); Serial.println(result3, HEX);
  }

  // --- PRINT THE DASHBOARD ---
  Serial.print("1. Speed (RPM):           "); Serial.println(rpm); // Will be 0 since motor is stopped
  Serial.print("2. Actual Position:       "); Serial.println(actualPos);
  Serial.print("3. Commanded Position:    "); Serial.println(cmdPos);
  Serial.print("4. Following Error:       "); Serial.println(errorPos);
  Serial.print("5. Motor Load/Torque (%): "); Serial.println(torque);
  
  Serial.print("6. Motor Current (Amps):  "); Serial.println(currentAmps);
  Serial.println("=====================================\n");
}