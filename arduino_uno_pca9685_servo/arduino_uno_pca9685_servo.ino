/*
 * Arduino Uno R3 + PCA9685 Servo Control
 * Controls pan/tilt servos via PCA9685
 * 
 * Hardware:
 * - Arduino Uno R3
 * - PCA9685 16-channel servo driver
 * - External 5V power supply
 * - 2x 9g servo motors
 * 
 * Wiring:
 * - PCA9685 SDA → Arduino A4
 * - PCA9685 SCL → Arduino A5
 * - PCA9685 VCC → External 5V power supply +
 * - PCA9685 GND → External 5V power supply - AND Arduino GND (shared!)
 * - Pan servo → PCA9685 Channel 0
 * - Tilt servo → PCA9685 Channel 1
 * 
 * Commands via Serial:
 * - P90  -> Set pan servo to 90 degrees
 * - T45  -> Set tilt servo to 45 degrees
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object (default I2C address is 0x40)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo channels
const int PAN_CHANNEL = 0;   // Pan servo on channel 0
const int TILT_CHANNEL = 1;  // Tilt servo on channel 1

// Servo pulse width limits (PCA9685 ticks, not microseconds!)
// These values work for standard 9g servos
#define SERVOMIN 170   // pulse length for 0° (in PCA9685 ticks)
#define SERVOMAX 650   // pulse length for 180° (in PCA9685 ticks)

// Current angles
int panAngle = 90;
int tiltAngle = 90;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println();
  Serial.println("=========================================");
  Serial.println("Arduino Uno + PCA9685 Servo Controller");
  Serial.println("=========================================");
  Serial.println();
  
  // Initialize I2C (Arduino Uno uses A4/A5 for I2C)
  Wire.begin();
  
  // Initialize PCA9685
  Serial.println("Initializing PCA9685...");
  if (!pca.begin()) {
    Serial.println("❌ ERROR: PCA9685 not found!");
    Serial.println();
    Serial.println("Check connections:");
    Serial.println("  - SDA → A4");
    Serial.println("  - SCL → A5");
    Serial.println("  - VCC → 5V power");
    Serial.println("  - GND → Shared ground");
    Serial.println();
    while (1) delay(10);  // Stop if PCA9685 not found
  }
  
  // Set PWM frequency to 50Hz for servos
  pca.setPWMFreq(50);
  
  Serial.println("✅ PCA9685 initialized successfully!");
  Serial.println();
  
  // Disable servos on startup - they will NOT move until Python sends commands
  // This prevents servos from moving to any position when Arduino starts
  pca.setPWM(PAN_CHANNEL, 0, 0);
  pca.setPWM(TILT_CHANNEL, 0, 0);
  Serial.println("⚠️  Servos DISABLED on startup (will not move until commanded)");
  Serial.println();
  
  Serial.println("✅ Ready!");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  P<angle> - Set pan servo (0-180) absolute");
  Serial.println("  P+<n> or P-<n> - Move pan servo relative (+1, -1, etc.)");
  Serial.println("  T<angle> - Set tilt servo (0-180) absolute");
  Serial.println("  T+<n> or T-<n> - Move tilt servo relative (+1, -1, etc.)");
  Serial.println("  C - Center both servos (90°)");
  Serial.println("  S - Sweep test");
  Serial.println("  STOP - Stop/disable both servos");
  Serial.println();
  Serial.println("Example: P90, T45, P+1, T-1, C, S, STOP");
  Serial.println();
  Serial.println("Waiting for commands... (servos will not move until commanded)");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command.length() == 0) {
      return;
    }
    
    if (command[0] == 'P') {
      // Pan command: P90 (absolute) or P+1, P-1 (relative)
      String arg = command.substring(1);
      int angle;
      
      if (arg.startsWith("+") || arg.startsWith("-")) {
        // Relative movement: P+1, P-1
        int delta = arg.toInt();
        angle = panAngle + delta;
      } else {
        // Absolute position: P90
        angle = arg.toInt();
      }
      
      if (angle >= 0 && angle <= 180) {
        panAngle = angle;
        setServoAngle(PAN_CHANNEL, panAngle);  // This enables and moves the servo
        Serial.print("Pan set to: ");
        Serial.print(panAngle);
        Serial.println("°");
      } else {
        Serial.println("Invalid angle. Use 0-180");
      }
    }
    else if (command[0] == 'T') {
      // Tilt command: T45 (absolute) or T+1, T-1 (relative)
      String arg = command.substring(1);
      int angle;
      
      if (arg.startsWith("+") || arg.startsWith("-")) {
        // Relative movement: T+1, T-1
        int delta = arg.toInt();
        angle = tiltAngle + delta;
      } else {
        // Absolute position: T45
        angle = arg.toInt();
      }
      
      if (angle >= 0 && angle <= 180) {
        tiltAngle = angle;
        setServoAngle(TILT_CHANNEL, tiltAngle);  // This enables and moves the servo
        Serial.print("Tilt set to: ");
        Serial.print(tiltAngle);
        Serial.println("°");
      } else {
        Serial.println("Invalid angle. Use 0-180");
      }
    }
    else if (command[0] == 'C') {
      // Center command
      panAngle = 90;
      tiltAngle = 90;
      setServoAngle(PAN_CHANNEL, panAngle);
      setServoAngle(TILT_CHANNEL, tiltAngle);
      Serial.println("Both servos centered at 90°");
    }
    else if (command == "STOP") {
      // Stop/disable servos
      pca.setPWM(PAN_CHANNEL, 0, 0);
      pca.setPWM(TILT_CHANNEL, 0, 0);
      Serial.println("Servos stopped (PWM disabled)");
    }
    else if (command[0] == 'S' && command != "STOP") {
      // Sweep test
      Serial.println("Starting sweep test...");
      sweepTest();
      Serial.println("Sweep test complete!");
    }
    else {
      Serial.println("Unknown command. Use P<angle>/P+<n>/P-<n>, T<angle>/T+<n>/T-<n>, C, S, or STOP");
    }
  }
  
  delay(10);
}

void setServoAngle(int channel, int angle) {
  // Clamp angle to valid range
  angle = constrain(angle, 0, 180);
  
  // Convert angle directly to PCA9685 ticks
  // Using working values: 170 ticks (0°) to 650 ticks (180°)
  int pwmValue = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Set the PWM value
  pca.setPWM(channel, 0, pwmValue);
}

void testServos() {
  // Test pan servo
  Serial.println("  Testing pan servo...");
  for (int angle = 45; angle <= 135; angle += 15) {
    setServoAngle(PAN_CHANNEL, angle);
    Serial.print("    Pan: ");
    Serial.print(angle);
    Serial.println("°");
    delay(500);
  }
  setServoAngle(PAN_CHANNEL, 90);
  delay(500);
  
  // Test tilt servo
  Serial.println("  Testing tilt servo...");
  for (int angle = 45; angle <= 135; angle += 15) {
    setServoAngle(TILT_CHANNEL, angle);
    Serial.print("    Tilt: ");
    Serial.print(angle);
    Serial.println("°");
    delay(500);
  }
  setServoAngle(TILT_CHANNEL, 90);
}

void sweepTest() {
  // Sweep pan servo
  Serial.println("  Sweeping pan servo...");
  for (int angle = 0; angle <= 180; angle += 5) {
    setServoAngle(PAN_CHANNEL, angle);
    delay(50);
  }
  for (int angle = 180; angle >= 0; angle -= 5) {
    setServoAngle(PAN_CHANNEL, angle);
    delay(50);
  }
  setServoAngle(PAN_CHANNEL, 90);
  delay(500);
  
  // Sweep tilt servo
  Serial.println("  Sweeping tilt servo...");
  for (int angle = 0; angle <= 180; angle += 5) {
    setServoAngle(TILT_CHANNEL, angle);
    delay(50);
  }
  for (int angle = 180; angle >= 0; angle -= 5) {
    setServoAngle(TILT_CHANNEL, angle);
    delay(50);
  }
  setServoAngle(TILT_CHANNEL, 90);
}





