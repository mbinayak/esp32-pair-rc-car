#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp32-hal-ledc.h>

#define leftMotorPWMPin 14  // Left motor speed (PWM) pin
#define rightMotorPWMPin 32 // right motor speed (PWM) pin
#define leftMotorClockwiseDirPin 27  // Motor A direction pin 1
#define leftMotorAntiClockwiseDirPin 26  // Motor A direction pin 2
#define rightMotorClockwiseDirPin 25  // Motor B direction pin 1
#define rightMotorAntiClockwiseDirPin 33  // Motor B direction pin 2

// Board_1 (usb type micro b) mac: 1C:69:20:A4:AB:80
// Board_2 (usb type c) mac: 80:F3:DA:41:7F:24

struct JoystickPayload {
  int16_t x;
  int16_t y;
  bool pressed;
};

JoystickPayload incomingJSPayloadData;

void setup() {
  Serial.begin(115200);

  // motor contoller pins configuration initialize
  pinMode(leftMotorClockwiseDirPin, OUTPUT);
  pinMode(leftMotorAntiClockwiseDirPin, OUTPUT);
  pinMode(rightMotorClockwiseDirPin, OUTPUT);
  pinMode(rightMotorAntiClockwiseDirPin, OUTPUT);

  // PWM pin setup
  ledcAttach(leftMotorPWMPin, 1000, 8); // Attach PWM @1kHz, 8-bit res
  ledcAttach(rightMotorPWMPin, 1000, 8); // Attach PWM @1kHz, 8-bit res

  // esp now receiver setup
  espNowWifiReceiverSetup();

  Serial.println("Ready to receive joystick data");
}

void loop() {
  // Nothing to do here, all handled in callback
}

void espNowWifiReceiverSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1);
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW active, receiver callback ready!");
}

void onReceive(const esp_now_recv_info_t * info, const uint8_t * incomingDataRaw, int len) {
  memcpy(&incomingJSPayloadData, incomingDataRaw, sizeof(incomingJSPayloadData));

  Serial.printf("Received - X: %d, Y: %d, Pressed: %s\n", incomingJSPayloadData.x, incomingJSPayloadData.y, incomingJSPayloadData.pressed ? "YES" : "NO");
  
  driveMotors(incomingJSPayloadData.x, incomingJSPayloadData.y);
}

void driveMotors(int16_t x, int16_t y) {
  // Calculate motor speeds [-255..255]
  // Simple tank drive style mixing
  int16_t leftMotorSpeed = constrain(y + x, -255, 255);
  int16_t rightMotorSpeed = constrain(y - x, -255, 255);

  setMotor(leftMotorPWMPin, leftMotorClockwiseDirPin, leftMotorAntiClockwiseDirPin, leftMotorSpeed);
  setMotor(rightMotorPWMPin, rightMotorClockwiseDirPin, rightMotorAntiClockwiseDirPin, rightMotorSpeed);
}

void setMotor(int pwmPin, int clockwiseDirPin, int antiClockwiseDirPin, int speed) {
  if (speed > 0) {
    // move forward
    digitalWrite(clockwiseDirPin, HIGH);
    digitalWrite(antiClockwiseDirPin, LOW);
    ledcWrite(pwmPin, speed); // instead of analogWrite
  } else if (speed < 0) {
    // move backwards
    digitalWrite(clockwiseDirPin, LOW);
    digitalWrite(antiClockwiseDirPin, HIGH);
    ledcWrite(pwmPin, -speed); // instead of analogWrite
  } else {
    // Stop movement
    digitalWrite(clockwiseDirPin, LOW);
    digitalWrite(antiClockwiseDirPin, LOW);
    ledcWrite(pwmPin, 0); // instead of analogWrite
  }

  // testing stuff
  Serial.printf("PWM %d | CW %d | CCW %d | Speed %d\n", pwmPin,
              digitalRead(clockwiseDirPin), digitalRead(antiClockwiseDirPin), speed);
}