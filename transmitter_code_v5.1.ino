#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>
#include<SPI.h>



//joystick1
#define X_AXIS_PIN_JOY1 33
#define Y_AXIS_PIN_JOY1 32
#define SWITCH_PIN_JOY1 25

//joystick2
#define X_AXIS_PIN_JOY2 34
#define Y_AXIS_PIN_JOY2 35
#define SWITCH_PIN_JOY2 26

// button pins (total 12 extra buttons====> used 3/12);
#define HEADLIGHT_PIN 4
#define FACE_EXPRESSION_PIN 13
#define DIPPER_PIN 16


// Define LED parameters
#define DATA_PIN 5    // Data pin connected to SM16703
#define NUM_LEDS 6    // Number of SM16703 LEDs connected
#define BUTTON_PIN 22 // Pin connected to the button
#define LONG_PRESS_DURATION 3000 // Duration for a long press in milliseconds (3 seconds)
#define DURATION_FADE 1000       // Duration for fading between colors

// Create LED array
CRGB leds[NUM_LEDS];

// Define colors for the fading pattern
const CRGB colors[] = {
CRGB::Red,
  CRGB(255, 69, 0),     // Red-Orange
  CRGB::Orange,
  CRGB(255, 140, 0),    // Dark Orange
  CRGB(255, 165, 0),    // Light Orange
  CRGB::Yellow,
  CRGB(255, 255, 102),  // Light Yellow
  CRGB(255, 255, 153),  // Lemon Yellow
  CRGB::Lime,
  CRGB(0, 255, 127),    // Spring Green
  CRGB::Green,
  CRGB(0, 204, 102),    // Sea Green
  CRGB(0, 153, 76),     // Dark Green
  CRGB::Teal,
  CRGB(64, 224, 208),   // Turquois
  CRGB::Cyan,
  CRGB(0, 204, 204),    // Dark Cyan
  CRGB::Blue,
  CRGB(0, 0, 255),      // Bright Blue
  CRGB(0, 0, 139),      // Dark Blue
  CRGB::Purple, 
  CRGB(128, 0, 128),    // Dark Purple
  CRGB(139, 0, 139),    // Dark Magenta
  CRGB::Magenta,
  CRGB(255, 0, 255),    // Bright Magenta
  CRGB::Pink,
  CRGB(255, 20, 147),   // Deep Pink
  CRGB(255, 105, 180),  // Hot Pink
  CRGB(255, 0, 127),    // Rose 
  CRGB::White, 
  CRGB::Aqua, 
  CRGB::Coral, 
  CRGB::Gold
};

const int numColors = sizeof(colors) / sizeof(colors[0]);

// Button state variables for leds
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
bool isLongPress = false;
int colorIndex = 0;
bool inFadeMode = false;



// RECEIVER MAC Addresses
uint8_t receiverMacAddress1[] = {0xC8, 0xF0, 0x9E, 0xF1, 0x8A, 0x70};
uint8_t receiverMacAddress2[] = {0xC8, 0xF0, 0x9E, 0xF1, 0x7D, 0x00};

// Structure to hold packet data
struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
  byte headlightController; // For peer 2 only
  byte faceExpressionController; // For peer 2 only
  byte dipper; // For peer 2 only
};

PacketData data1;
PacketData data2;

// This function maps joystick values and adjusts the deadband
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 2200)
  {
    value = map(value, 2200, 4095, 127, 254);
  }
  else if (value <= 1800)
  {
    value = map(value, 1800, 0, 127, 0);  
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  return value;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("Success: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer 1
  esp_now_peer_info_t peerInfo1 = {};
  memcpy(peerInfo1.peer_addr, receiverMacAddress1, 6);
  peerInfo1.channel = 0;
  peerInfo1.encrypt = false;

  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    Serial.println("Failed to add peer 1");
    return;
  } else {
    Serial.println("Success: Added peer 1");
  }

  // Register peer 2
  esp_now_peer_info_t peerInfo2 = {};
  memcpy(peerInfo2.peer_addr, receiverMacAddress2, 6);
  peerInfo2.channel = 0;
  peerInfo2.encrypt = false;

  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer 2");
    return;
  } else {
    Serial.println("Success: Added peer 2");
  }
  FASTLedSetup();
  pinModeSetup();
  


}

void loop() {
  // Data for peer 1
  data1.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN_JOY1), false);
  data1.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN_JOY1), false);
  data1.switchPressed = digitalRead(SWITCH_PIN_JOY1) == LOW ? 1 : 0;

  // Data for peer 2
  data2.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN_JOY2), false);
  data2.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN_JOY2), false);
  data2.switchPressed = digitalRead(SWITCH_PIN_JOY2) == LOW ? 1 : 0;
  data2.headlightController = digitalRead(HEADLIGHT_PIN) == LOW ? 1 : 0;
  data2.faceExpressionController = digitalRead(FACE_EXPRESSION_PIN) == LOW ? 1 : 0;
  data2.dipper = digitalRead(DIPPER_PIN) == LOW ? 1 : 0;
  Serial.println(data2.dipper);

  //for led control (start**************)

  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = millis();
    } else {
      if ((millis() - buttonPressTime) >= LONG_PRESS_DURATION) {
        isLongPress = true;
      }
    }
  } else {
    if (buttonPressed) {
      buttonPressed = false;
      if (isLongPress) {
        isLongPress = false;
        inFadeMode = !inFadeMode;  // Toggle fade mode
        if (!inFadeMode) {
          FastLED.clear();
          setLEDColor(CRGB::Cyan);
          FastLED.show();
        }
      } else if (!inFadeMode) {
        cycleColors();
      }
    }
  }

  if (inFadeMode) {
    executeFadeToColor();
  }

  delay(10);
  // for led control (over********)//

  // Sending data to peer 1
  esp_err_t result1 = esp_now_send(receiverMacAddress1, (uint8_t *) &data1, sizeof(data1));
  if (result1 == ESP_OK) {
    Serial.println("Sent with success to peer 1");
  } else {
    Serial.print("Error sending the data to peer 1: ");
    Serial.println(result1);
    seek_error(result1);
  }

  // Sending data to peer 2
  esp_err_t result2 = esp_now_send(receiverMacAddress2, (uint8_t *) &data2, sizeof(data2));
  if (result2 == ESP_OK) {
    Serial.println("Sent with success to peer 2");
  } else {
    Serial.print("Error sending the data to peer 2: ");
    Serial.println(result2);
    seek_error(result2);
  }

  if (data1.switchPressed == 1 || data2.switchPressed == 1) {
    delay(500);
  } else {
    delay(50);
  }
}

void FASTLedSetup(){
  //****************************************//
    // Initialize FastLED library
  FastLED.addLeds<SM16703, DATA_PIN, RGB>(leds, NUM_LEDS);

  // Set brightness (optional)
  FastLED.setBrightness(255);

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor

  // Example initialization sequence
  initializeLEDs();
  //*****************************************//

}

void initializeLEDs() {
  // Send initialization sequence if needed
  // For example, you might want to clear LEDs
  FastLED.clear();   
  FastLED.show();// Refresh LEDs
  setLEDColor(CRGB::Cyan);
  FastLED.show();
}

void pinModeSetup(){
  pinMode(SWITCH_PIN_JOY1, INPUT_PULLUP);
  pinMode(SWITCH_PIN_JOY2, INPUT_PULLUP);
  pinMode(HEADLIGHT_PIN, INPUT_PULLUP);
  pinMode(FACE_EXPRESSION_PIN, INPUT_PULLUP);
  pinMode(DIPPER_PIN, INPUT_PULLUP);
}


void fadeToColor(CRGB fromColor, CRGB toColor, int duration) {
  // Interpolate between fromColor and toColor over duration
  for (int i = 0; i <= 256; ++i) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      unsigned long pressStart = millis();
      while (digitalRead(BUTTON_PIN) == LOW) {
        if (millis() - pressStart > LONG_PRESS_DURATION) {
          inFadeMode = false;
          return;
        }
      }
    }
    if (!inFadeMode) return; // Exit if not in fade mode
    CRGB color = blend(fromColor, toColor, i);
    setLEDColor(color);
    FastLED.show();
    delay(duration / 256);
  }
}


void setLEDColor(CRGB color) {
  // Set color to all LEDs
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = color;
  }
}

void cycleColors() {
  colorIndex = (colorIndex + 1) % numColors;
  setLEDColor(colors[colorIndex]);
  FastLED.show();
}


void executeFadeToColor() {
  for (int i = 0; i < numColors; ++i) {
    if (!inFadeMode) return; // Exit if not in fade mode
    fadeToColor(colors[i], colors[(i + 1) % numColors], DURATION_FADE);
  }
}



void seek_error(esp_err_t result) {
  if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESP-NOW is not initialized");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found");
  } else if (result == ESP_ERR_ESPNOW_IF) {
    Serial.println("Interface error");
  }

  
}
