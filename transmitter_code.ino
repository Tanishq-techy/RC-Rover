#include <esp_now.h>
#include <WiFi.h>

#define X_AXIS_PIN_JOY1 32
#define Y_AXIS_PIN_JOY1 33
#define SWITCH_PIN_JOY1 25



// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0xC8, 0xF0, 0x9E, 0xF1, 0x8A, 0x70};

// Structure to hold packet data
struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData data;

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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Success: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Success: Added peer");
  }

  pinMode(SWITCH_PIN_JOY1, INPUT_PULLUP);
}

void loop() 
{
  data.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN_JOY1), false);
  data.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN_JOY1), false);
  data.switchPressed = digitalRead(SWITCH_PIN_JOY1) == LOW ? 1 : 0;

  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else 
  {
    Serial.print("Error sending the data: ");
    Serial.println(result);
    if (result == ESP_ERR_ESPNOW_NOT_INIT)
    {
      Serial.println("ESP-NOW is not initialized");
    }
    else if (result == ESP_ERR_ESPNOW_ARG)
    {
      Serial.println("Invalid argument");
    }
    else if (result == ESP_ERR_ESPNOW_INTERNAL)
    {
      Serial.println("Internal error");
    }
    else if (result == ESP_ERR_ESPNOW_NO_MEM)
    {
      Serial.println("Out of memory");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    {
      Serial.println("Peer not found");
    }
    else if (result == ESP_ERR_ESPNOW_IF)
    {
      Serial.println("Interface error");
    }
  }

  if (data.switchPressed == 1)
  {
    delay(500);
  }
  else
  {
    delay(50);
  }
}
