#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <U8g2lib.h>

// Initialize the display with the I2C address 0x3C (most common for SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

#define SERVO_PIN 13

const int SIGNAL_TIMEOUT = 1000;  // Signal timeout in milliseconds
unsigned long lastRecvTime = 0;
unsigned long lastButtonPressTime = 0;
const int buttonDebounceDelay = 50; // Debounce delay for button press

// State variables
int headlightState = 0; // 0-3 for 4 different states
bool headlightButtonPressed = false;

Servo myServo;
int currentAngle = 90;      // Initial angle set to 90 degrees
int targetAngle = 90;       // Target angle for smooth movement

// Define the SPI pins for ESP32
const int latchPin = 4;  // Latch pin (STCP)
const int clockPin = 18; // Clock pin (SHCP)
const int dataPin = 23;  // Data pin (DS)

// Light values setup
const uint8_t hazardLight = 0b00110000;
const uint8_t nightLight = 0b00001100;
const uint8_t dipperAndNightLight = 0b00001111;
const uint8_t offLight = 0b00000000;

// Define array of angles with a 15-degree gap
const int numAngles = 13;   // Total number of angles (0 to 180 with 15-degree gap)
int angles[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180};
int *anglePointer = &angles[6];  // Start at index 6 (90 degrees initially)

struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
  byte headlightController; // For peer 2 only
  byte faceExpressionController; // For peer 2 only
  byte dipper; // For peer 2 only
};

PacketData receiverData;

// Linear interpolation variables
const float smoothingFactor = 0.2;  // Smoothing factor (adjust as needed)
int interpolatedAngle = 90;         // Interpolated angle value

// Animation variables
const int numFrames = 20;
const int frameWidth = 30;
const int frameHeight = 30;
const int minHeight = 10; // Minimum height of the rectangles
const float frameStep = (float)(frameHeight - minHeight) / numFrames; // Calculate the frame step as a float for precision
int currentFrame = 0;
bool closing = true; // True if animating closing, false if opening
unsigned long previousMillis = 0;
const long frameInterval = 50; // Interval between frames (80 ms)

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 0) {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData = "values " + String(receiverData.xAxisValue) + "  " + String(receiverData.yAxisValue) + "  " + String(receiverData.switchPressed) + " " + String(receiverData.headlightController) + " " + String(receiverData.faceExpressionController) + " " + String(receiverData.dipper);
  Serial.println(inputData);

  // Update target angle based on joystick input
  if (receiverData.yAxisValue >= 175) {
    if (anglePointer < &angles[numAngles - 1]) {
      anglePointer++;  // Move pointer to the right if not at the end
    }
  } else if (receiverData.yAxisValue <= 75) {
    if (anglePointer > &angles[0]) {
      anglePointer--;  // Move pointer to the left if not at the beginning
    }
  }

  // Set target angle based on pointer position
  targetAngle = *anglePointer;

  // Handle headlight button
  if (receiverData.headlightController == 1) {
    unsigned long now = millis();
    if (now - lastButtonPressTime > buttonDebounceDelay) {
      lastButtonPressTime = now;
      headlightState = (headlightState + 1) % 4; // Cycle through 0-3
    }
  }

  lastRecvTime = millis();
}

// Function to perform linear interpolation
void smoothMovement() {
  // Smoothly move currentAngle towards targetAngle
  interpolatedAngle = (1.0 - smoothingFactor) * interpolatedAngle + smoothingFactor * targetAngle;
  currentAngle = int(interpolatedAngle);

  // Apply the new servo angle
  myServo.write(currentAngle);
}

// Headlight state functions
void headlightState0() {
  digitalWrite(latchPin, LOW);
  SPI.transfer(offLight);
  digitalWrite(latchPin, HIGH);
}

void headlightState1() {
  digitalWrite(latchPin, LOW);
  SPI.transfer(hazardLight);
  digitalWrite(latchPin, HIGH);
}

void headlightState2() {
  digitalWrite(latchPin, LOW);
  SPI.transfer(nightLight);
  digitalWrite(latchPin, HIGH);
}

void headlightState3() {
  digitalWrite(latchPin, LOW);
  SPI.transfer(dipperAndNightLight);
  digitalWrite(latchPin, HIGH);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  // Set the pin modes
  pinMode(latchPin, OUTPUT);

  // Initialize SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  digitalWrite(latchPin, LOW);
  SPI.transfer(offLight); // Initial light state
  digitalWrite(latchPin, HIGH);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Initialize the display
  u8g2.begin();
}

void loop() {
  unsigned long now = millis();

  // Check for signal loss
  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    targetAngle = 90;  // Set to neutral position
  }

  // Control servo movement smoothly
  smoothMovement();

  // Update light based on headlight state
  switch (headlightState) {
    case 0:
      headlightState0();
      break;
    case 1:
      headlightState1();
      break;
    case 2:
      headlightState2();
      break;
    case 3:
      headlightState3();
      break;
  }

  // SSD1306 display animation
  if (now - previousMillis >= frameInterval) {
    // Save the last time the frame was updated
    previousMillis = now;

    // Update to the next frame
    if (closing) {
      currentFrame++;
      if (currentFrame >= numFrames) {
        closing = false; // Start opening animation
        currentFrame = numFrames - 1; // Start from the last frame
      }
    } else {
      currentFrame--;
      if (currentFrame < 0) {
        closing = true; // Start closing animation
        currentFrame = 0; // Start from the first frame
      }
    }

    // Clear the display
    u8g2.firstPage();
    do {
      // Draw the rounded boxes with varying heights based on the current frame
      for (int i = 0; i < 2; ++i) {
        int x = 20 + (i * 58); // X position for the two boxes
        int y = 16; // Fixed Y position

        int boxHeight = frameHeight - (int)(frameStep * currentFrame);
        if (boxHeight < minHeight) {
          boxHeight = minHeight; // Ensure the height does not go below the minimum height
        }
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        u8g2.drawRBox(x, y + (frameHeight - boxHeight) / 2, frameWidth, boxHeight, 4); // Center the box vertically
      }
    } while (u8g2.nextPage());
  }

  delay(10); // Small delay to avoid excessive updates
}
