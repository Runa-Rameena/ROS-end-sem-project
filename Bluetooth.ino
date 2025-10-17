/*
 * ESP32 Snake Robot - Bluetooth RC Control with Debug Prints
 * Hardware: ESP32 DevKit V1 + 8x SG90 Servos
 * Control: Bluetooth RC Car App
 * Movements: Forward, Backward, Left, Right
 * 
 * Bluetooth RC Car App Commands:
 * 'F' - Forward
 * 'B' - Backward
 * 'L' - Left (Turn Left)
 * 'R' - Right (Turn Right)
 * 'S' - Stop
 */

#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Create Bluetooth Serial object
BluetoothSerial SerialBT;

// Create servo objects for 8 joints
Servo servo[8];

// GPIO pins for servos (as per circuit diagram)
const int servoPins[8] = {2, 4, 5, 18, 19, 21, 22, 23};

// Servo parameters
const int SERVO_MIN = 500;   // Minimum pulse width in microseconds
const int SERVO_MAX = 2500;  // Maximum pulse width in microseconds
const int SERVO_CENTER = 1500; // Center position (90 degrees)

// Gait parameters for snake locomotion
float amplitude = 0.6;        // Wave amplitude in radians (adjustable 0.2-0.8)
float frequency = 0.5;        // Temporal frequency in Hz
float spatialFreq = 0.8;      // Spatial frequency parameter

// Movement state
char currentCommand = 'S';    // Current movement command
bool isMoving = false;
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL = 50; // Update servo positions every 50ms

// Movement modes
enum MovementMode {
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  STOPPED
};

MovementMode currentMode = STOPPED;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("ESP32 Snake Robot - Bluetooth RC Control");
  Serial.println("========================================");
  
  Serial.println("\n[SETUP] Starting initialization...");
  
  // Initialize Bluetooth with device name
  Serial.print("[BLUETOOTH] Initializing Bluetooth... ");
  SerialBT.begin("ESP32_SnakeBot"); // Bluetooth device name
  Serial.println("SUCCESS");
  Serial.println("[BLUETOOTH] Device name: ESP32_SnakeBot");
  Serial.println("[BLUETOOTH] Waiting for connection...");
  
  // Initialize servos
  Serial.println("\n[SERVO] Initializing 8 servos...");
  for (int i = 0; i < 8; i++) {
    Serial.print("[SERVO] Attaching Servo ");
    Serial.print(i);
    Serial.print(" to GPIO ");
    Serial.print(servoPins[i]);
    Serial.print("... ");
    
    servo[i].attach(servoPins[i], SERVO_MIN, SERVO_MAX);
    servo[i].writeMicroseconds(SERVO_CENTER);
    
    Serial.print("SUCCESS - Position set to ");
    Serial.print(SERVO_CENTER);
    Serial.println(" µs (neutral)");
  }
  
  delay(1000); // Allow servos to reach neutral position
  
  Serial.println("\n[SETUP] Initialization complete!");
  Serial.println("[SYSTEM] Ready to receive commands!");
  Serial.println("========================================");
  Serial.println("Waiting for Bluetooth connection...\n");
}

void loop() {
  // Check for incoming Bluetooth commands
  if (SerialBT.available()) {
    char received = SerialBT.read();
    Serial.print("\n[INPUT] Bluetooth data received: '");
    Serial.print(received);
    Serial.print("' (ASCII: ");
    Serial.print((int)received);
    Serial.println(")");
    
    processCommand(received);
  }
  
  // Update servo positions based on current movement mode
  if (isMoving && (millis() - lastUpdateTime >= UPDATE_INTERVAL)) {
    updateServoPositions();
    lastUpdateTime = millis();
  }
}

// Process incoming Bluetooth commands
void processCommand(char cmd) {
  currentCommand = cmd;
  
  Serial.print("[COMMAND] Processing command: '");
  Serial.print(cmd);
  Serial.print("' -> ");
  
  switch (cmd) {
    case 'F': // Forward
      Serial.println("FORWARD");
      currentMode = FORWARD;
      isMoving = true;
      Serial.println("[MOVEMENT] Setting mode: FORWARD");
      Serial.println("[MOVEMENT] Motion started: Serpenoid forward wave");
      Serial.print("[PARAMETERS] Amplitude: ");
      Serial.print(amplitude);
      Serial.print(" rad, Frequency: ");
      Serial.print(frequency);
      Serial.print(" Hz, Spatial Freq: ");
      Serial.println(spatialFreq);
      break;
      
    case 'B': // Backward
      Serial.println("BACKWARD");
      currentMode = BACKWARD;
      isMoving = true;
      Serial.println("[MOVEMENT] Setting mode: BACKWARD");
      Serial.println("[MOVEMENT] Motion started: Reversed serpenoid wave");
      Serial.print("[PARAMETERS] Amplitude: ");
      Serial.print(amplitude);
      Serial.print(" rad, Frequency: ");
      Serial.print(frequency);
      Serial.print(" Hz, Spatial Freq: ");
      Serial.println(spatialFreq);
      break;
      
    case 'L': // Turn Left
      Serial.println("TURN LEFT");
      currentMode = TURN_LEFT;
      isMoving = true;
      Serial.println("[MOVEMENT] Setting mode: TURN_LEFT");
      Serial.println("[MOVEMENT] Motion started: Left-biased wave pattern");
      Serial.println("[PARAMETERS] Bias: -0.3 rad (left curve)");
      break;
      
    case 'R': // Turn Right
      Serial.println("TURN RIGHT");
      currentMode = TURN_RIGHT;
      isMoving = true;
      Serial.println("[MOVEMENT] Setting mode: TURN_RIGHT");
      Serial.println("[MOVEMENT] Motion started: Right-biased wave pattern");
      Serial.println("[PARAMETERS] Bias: +0.3 rad (right curve)");
      break;
      
    case 'S': // Stop
      Serial.println("STOP");
      currentMode = STOPPED;
      isMoving = false;
      stopRobot();
      Serial.println("[MOVEMENT] Setting mode: STOPPED");
      Serial.println("[MOVEMENT] All servos returned to neutral position");
      break;
      
    default:
      Serial.print("UNKNOWN (ignored)");
      Serial.println("\n[WARNING] Unrecognized command, ignoring");
      break;
  }
  
  Serial.println("--------------------");
}

// Update servo positions based on serpenoid gait
void updateServoPositions() {
  float t = millis() / 1000.0; // Current time in seconds
  
  Serial.print("[UPDATE] Time: ");
  Serial.print(t, 3);
  Serial.print("s | Mode: ");
  
  switch (currentMode) {
    case FORWARD:
      Serial.print("FORWARD");
      break;
    case BACKWARD:
      Serial.print("BACKWARD");
      break;
    case TURN_LEFT:
      Serial.print("TURN_LEFT");
      break;
    case TURN_RIGHT:
      Serial.print("TURN_RIGHT");
      break;
    case STOPPED:
      Serial.print("STOPPED");
      break;
  }
  
  Serial.println(" | Servo positions:");
  
  for (int i = 0; i < 8; i++) {
    float angle = 0.0;
    
    switch (currentMode) {
      case FORWARD:
        // Forward serpenoid wave: lateral undulation
        angle = amplitude * sin(2 * PI * frequency * t - spatialFreq * i);
        break;
        
      case BACKWARD:
        // Backward: reverse the wave direction
        angle = amplitude * sin(2 * PI * frequency * t + spatialFreq * i);
        break;
        
      case TURN_LEFT:
        // Left turn: asymmetric wave with bias to left
        angle = amplitude * sin(2 * PI * frequency * t - spatialFreq * i) - 0.3;
        break;
        
      case TURN_RIGHT:
        // Right turn: asymmetric wave with bias to right
        angle = amplitude * sin(2 * PI * frequency * t - spatialFreq * i) + 0.3;
        break;
        
      case STOPPED:
        angle = 0.0; // All joints to neutral
        break;
    }
    
    // Convert radians to microseconds for servo
    int pulseWidth = radiansToMicroseconds(angle);
    
    // Output to servo
    servo[i].writeMicroseconds(pulseWidth);
    
    // Print servo state
    Serial.print("  [SERVO ");
    Serial.print(i);
    Serial.print(" | GPIO ");
    Serial.print(servoPins[i]);
    Serial.print("] Angle: ");
    if (angle >= 0) Serial.print("+");
    Serial.print(angle, 3);
    Serial.print(" rad (");
    Serial.print(angle * 57.2958, 1); // Convert to degrees
    Serial.print("°) -> Pulse: ");
    Serial.print(pulseWidth);
    Serial.println(" µs");
  }
  
  Serial.println("--------------------\n");
}

// Stop all servos (return to neutral position)
void stopRobot() {
  Serial.println("[STOP] Stopping all servos...");
  
  for (int i = 0; i < 8; i++) {
    servo[i].writeMicroseconds(SERVO_CENTER);
    
    Serial.print("  [SERVO ");
    Serial.print(i);
    Serial.print(" | GPIO ");
    Serial.print(servoPins[i]);
    Serial.print("] Set to NEUTRAL (");
    Serial.print(SERVO_CENTER);
    Serial.println(" µs)");
  }
  
  Serial.println("[STOP] All servos stopped");
}

// Convert angle in radians to servo pulse width in microseconds
int radiansToMicroseconds(float radians) {
  // Map radians to servo pulse width
  // -PI/2 to PI/2 maps to SERVO_MIN to SERVO_MAX
  // Clamp to ±90 degrees (±1.57 radians)
  float originalRadians = radians;
  radians = constrain(radians, -1.57, 1.57);
  
  // Check if clamping occurred
  if (originalRadians != radians) {
    Serial.print("    [LIMIT] Angle clamped from ");
    Serial.print(originalRadians, 3);
    Serial.print(" to ");
    Serial.print(radians, 3);
    Serial.println(" rad");
  }
  
  // Map to pulse width
  int pulseWidth = map(radians * 1000, -1570, 1570, SERVO_MIN, SERVO_MAX);
  return pulseWidth;
}