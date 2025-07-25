#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>  // Servo library

// WiFi credentials
const char* ssid = "Esp32";
const char* password = "onetwothree";
const char* udpAddress = "172.20.10.5";
const int udpPort = 4210;

// Pins
const int SERVO_SCAN_PIN = 12; // Scanning servo
const int TRIG_PIN = 14;
const int ECHO_PIN = 27;
const int PAN_PIN = 18;        // Pan servo
const int TILT_PIN = 19;       // Tilt servo
const int RELAY_PIN = 26;      // Relay controlling the pump

// Hardware objects
Servo scanServo;
Servo panServo;
Servo tiltServo;
WiFiUDP udp;

// Detection state
long detectedDistance = -1;
int detectedAngle = -1;
bool pumpHasSprayed = false;
bool readyForNewDetection = true;

// Current servo positions
int currentPan = 90;
int currentTilt = 90;

// Servo mounting height (cm)
const float servoHeight = 20.0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Servo setup
  scanServo.attach(SERVO_SCAN_PIN);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // Sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Pump relay setup
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Relay OFF (active LOW)
}

void loop() {
  // Sweep from 0 to 180
  for (int angle = 0; angle <= 180; angle++) {
    scanServo.write(angle);
    delay(100);

    long distance = measureDistance();
    checkAndStoreDetection(angle, distance);
    sendUDP(angle, distance);
    respondWithServos();
  }

  // Sweep from 180 to 0
  for (int angle = 180; angle >= 0; angle--) {
    scanServo.write(angle);
    delay(100);

    long distance = measureDistance();
    checkAndStoreDetection(angle, distance);
    sendUDP(angle, distance);
    respondWithServos();
  }
}

// Measure distance using ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  long distanceCm = duration * 0.0343 / 2;
  if (distanceCm == 0 || distanceCm > 100) {
    distanceCm = 100;
  }
  return distanceCm;
}

// Send angle and distance over UDP
void sendUDP(int angle, long distance) {
  String message = String(angle) + "," + String(distance) + ".";
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
  Serial.println(message);
}

// Check if new object is detected
void checkAndStoreDetection(int angle, long distance) {
  if (readyForNewDetection && distance < 100) {
    detectedAngle = angle;
    detectedDistance = distance;
    pumpHasSprayed = false;
    readyForNewDetection = false;
  }
}

// Respond to detection by moving servos and spraying
void respondWithServos() {
  if (!readyForNewDetection && !pumpHasSprayed && detectedDistance < 50 && detectedDistance > 0) {
    // Move pan servo
    currentPan = constrain(detectedAngle, 0, 180);
    panServo.write(currentPan);
    Serial.printf("Pan to: %d°\n", currentPan);

    // Calculate tilt angle using servo height and distance
    float tiltRadians = atan2(detectedDistance, servoHeight);
    float tiltDegrees = degrees(tiltRadians);

    // Map tilt degrees to servo angle (inverted logic)
    currentTilt = 180 - tiltDegrees
    currentTilt = constrain(currentTilt, 0, 180);
    tiltServo.write(currentTilt);
    Serial.printf("Tilt to: %d° (from %.1f° downward angle)\n", currentTilt, tiltDegrees);

    // Spray water
    sprayPump(5000);
    pumpHasSprayed = true;

    // Return to neutral
    currentPan = 90;
    currentTilt = 90;
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    Serial.println("Servos returned to neutral");

    delay(500); // Optional pause
    readyForNewDetection = true;
  }
}

// Turn on pump for given duration (ms)
void sprayPump(unsigned long durationMs) {
  Serial.println("Pump ON");
  digitalWrite(RELAY_PIN, LOW);  // Pump ON (relay active LOW)
  delay(durationMs);             // Spray duration
  digitalWrite(RELAY_PIN, HIGH); // Pump OFF
  Serial.println("Pump OFF");
}
