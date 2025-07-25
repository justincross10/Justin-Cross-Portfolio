#include <Servo.h>

// Create servo objects for pan and tilt
Servo panServo;
Servo tiltServo;

// Pin assignments
const int panPin = 9;    // PWM pin for pan servo
const int tiltPin = 10;  // PWM pin for tilt servo

// Current servo angles
int currentPan = 90;
int currentTilt = 90;

// Dummy position tracking
bool movedToDummy = false;
unsigned long dummyMoveTime = 0;

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging

  panServo.attach(panPin);  // Attach servos to PWM pins
  tiltServo.attach(tiltPin);

  // Move to neutral positions
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

void loop() {
  if (!movedToDummy) {

    int dummyPanAngle = 10;
    int dummyDistance = 60;          // horizontal distance in cm
    const float servoHeight = 20.0;  // height above ground in cm

    // Limit pan angle
    dummyPanAngle = constrain(dummyPanAngle, 0, 180);

    // Calculate angle of elevation in radians
    float tiltRadians = atan2(dummyDistance, servoHeight);

    // Convert to degrees
    int tiltAngle = 180 - degrees(tiltRadians);

    // Constrain tilt angle to valid servo range
    tiltAngle = constrain(tiltAngle, 0, 180);

    // Print result
    Serial.print("Distance: ");
    Serial.print(dummyDistance);
    Serial.print(" cm | Tilt angle: ");
    Serial.print(tiltAngle);
    Serial.println("°");

    // Move servos to dummy position
    if (abs(dummyPanAngle - currentPan) >= 2) {
      currentPan = dummyPanAngle;
      panServo.write(currentPan);
      Serial.print("Pan to: ");
      Serial.print(currentPan);
      Serial.println("°");
    }

    if (abs(tiltAngle - currentTilt) >= 2) {
      currentTilt = tiltAngle;
      tiltServo.write(currentTilt);
      Serial.print("Tilt to: ");
      Serial.print(currentTilt);
      Serial.println("°");
    }

    // Start 3-second timer
    dummyMoveTime = millis();
    movedToDummy = true;
  }

  // Return to neutral after 3 seconds
  if (movedToDummy && millis() - dummyMoveTime >= 3000) {
    currentPan = 90;
    currentTilt = 90;
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    Serial.println("Returning to neutral position (90°, 90°).");

    movedToDummy = false;  // Reset for another test cycle
    delay(2000);           // Optional: pause before next cycle starts
  }

  delay(100);
}
