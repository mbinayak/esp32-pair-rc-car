#include <WiFi.h>
#include <esp_now.h>

const int jsPinX = 34;
const int jsPinY = 35;
const int jsPinSW = 13;
constexpr int JS_READ_INTERVAL = 100;
const int DEADZONE = 20;
const int xCenter = 2916;
const int yCenter = 2989;

const int inputChangeIndicatorPin = 12;

struct JoystickState {
  int16_t x;
  int16_t y;
  bool pressed;
  unsigned long lastUpdated;
};

struct JoystickPayload {
  int16_t x;
  int16_t y;
  bool pressed;
};

JoystickState jsState;

bool inputJustChanged = false;
unsigned long inputChangeExpiryTime = 0;
const unsigned long INPUT_CHANGE_EFFECT_DURATION = 300;

bool shouldSendNeutral = false;
unsigned long lastInputChange = 0;
const unsigned long NEUTRAL_SEND_DELAY = 150;  // ms

// Board_1 (usb type micro b) mac: 1C:69:20:A4:AB:80 <-- current joystick controller (receiver)
// Board_2 (usb type c) mac: 80:F3:DA:41:7F:24 <--- current car controller (sender)

uint8_t receiverMAC[] = {0x1C, 0x69, 0x20, 0xA4, 0xAB, 0x80}; // board 1
// uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x41, 0x7F, 0x24}; // board 2

bool receiverOnline = false;
unsigned long lastSendAttempt = 0;
unsigned long lastReceiverOnline = 0;

const unsigned long RECEIVER_TIMEOUT = 2000; // ms to consider it offline
const unsigned long SEND_RETRY_INTERVAL = 500; // ms between retries
const int receiverOnlineIndicatorPin = 14; // example pin for online indicator

void setup() {
  // init serial output
  Serial.begin(115200);
  Serial.println("Joystick Controller: Booting up joystick controller...");

  // intialise joystick state.
  jsState.x = 0;
  jsState.y = 0;
  jsState.pressed = false;
  jsState.lastUpdated = 0;

  // Configure joystick input pins
  pinMode(jsPinX, INPUT); // redundant for analogread pins, just keeping it here to be consistent on pin use
  pinMode(jsPinY, INPUT); // redundant for analogread pins, just keeping it here to be consistent on pin use
  pinMode(jsPinSW, INPUT_PULLUP);  // Internal pull-up resistor for button, its digital input, so it need pinMode declaration

  // Setup ESP Now connection with the receiver board.
  setupESPNow();

  // configure indicator led output pins
  pinMode(receiverOnlineIndicatorPin, OUTPUT);
  pinMode(inputChangeIndicatorPin, OUTPUT);
  
  // initial indicator states
  digitalWrite(receiverOnlineIndicatorPin, LOW);
  digitalWrite(inputChangeIndicatorPin, HIGH);
  delay(1000);
  digitalWrite(inputChangeIndicatorPin, LOW);

  Serial.println("Joystick Controller: All setup for normal operation.");
}
 
void loop() {
  unsigned long now = millis(); // Get time once
  
  // read the joystick and update the state.
  readJoystick(now);

  // update any and all indicators here.
  updateIndicators(now);

  // transmit data over to receiver board via ESP Now protocol if latest input available.
  transmitJoystickDataOverESPNow(now);

  // clean up and flag updates
  houseKeeping(now);
}

void readJoystick(unsigned long now) {
  static unsigned long lastReadTime = 0; // Initialized only once

  if (now - lastReadTime < JS_READ_INTERVAL) {
    return;
  }
  
  lastReadTime = now;

  int xRaw = analogRead(jsPinX);
  int yRaw = analogRead(jsPinY);

  int16_t x = mapCentered(xRaw, xCenter);
  int16_t y = mapCentered(yRaw, yCenter);

  x = constrain(x, -255, 255);
  y = constrain(y, -255, 255);
  
  if (abs(x) < DEADZONE) x = 0;
  if (abs(y) < DEADZONE) y = 0;

  bool pressed = (digitalRead(jsPinSW) == LOW);

  bool xChanged = abs(x - jsState.x) > DEADZONE;
  bool yChanged = abs(y - jsState.y) > DEADZONE;
  bool changed = xChanged || yChanged || (pressed != jsState.pressed);
  if (changed) {
    inputJustChanged = true;
    jsState.x = x;
    jsState.y = y;
    jsState.pressed = pressed;
    jsState.lastUpdated = now;
    inputChangeExpiryTime = now + INPUT_CHANGE_EFFECT_DURATION;

    Serial.printf("Raw X: %d, Raw Y: %d, X: %d, Y: %d, Pressed: %s\n", xRaw, yRaw, x, y, pressed ? "YES" : "NO");
  }
}

void updateIndicators(unsigned long now) {
  bool indicateChange = inputJustChanged && (now <= inputChangeExpiryTime);
  digitalWrite(inputChangeIndicatorPin, indicateChange ? HIGH: LOW);
  digitalWrite(receiverOnlineIndicatorPin, receiverOnline ? HIGH : LOW);
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Just in case

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true);  // Halt
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receiverMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      while (true);
    }
  }

  // Optional: receive callback (if you expect a response)
  // esp_now_register_recv_cb(onDataRecv);

  Serial.println("ESP-NOW setup complete");
}

void transmitJoystickDataOverESPNow(unsigned long now) {
  static unsigned long lastSentTime = 0;
  static JoystickPayload jsStatePayload;
  if (lastSentTime >= jsState.lastUpdated) {
    return;
  }

  if (!receiverOnline && (now - lastSendAttempt < SEND_RETRY_INTERVAL)) {
    return; // 🪫 Respect backoff while receiver is considered offline
  }

  lastSendAttempt = now;
  jsStatePayload.x = jsState.x;
  jsStatePayload.y = jsState.y;
  jsStatePayload.pressed = jsState.pressed;
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&jsStatePayload, sizeof(jsStatePayload));
  if (result == ESP_OK) {
    receiverOnline = true;
    lastReceiverOnline = now;
    Serial.println("Joystick data sent");
    lastSentTime = jsState.lastUpdated;
  } else {
    Serial.printf("Error sending ESP-NOW data: %d\n", result);
  }
}

void houseKeeping(unsigned long now) {
  // Turn off the change flag after duration
  if (inputJustChanged && now > inputChangeExpiryTime) {
    inputJustChanged = false;
  }

  // Update receiver online indicator
  if (receiverOnline && now - lastReceiverOnline > RECEIVER_TIMEOUT) {
    receiverOnline = false;
    Serial.println("Receiver timed out — offline.");
  }
}

int16_t mapCentered(int raw, int center) {
  // Range from -center..(4095 - center)
  int span = max(center, 4095 - center);  // to handle asymmetry
  int val = map(raw, center - span, center + span, -255, 255);
  val = constrain(val, -255, 255);
  return (abs(val) < DEADZONE) ? 0 : val;
}