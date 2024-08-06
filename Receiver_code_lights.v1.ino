#include <esp_now.h>
#include <WiFi.h>

// Motor Pins
int enableRightMotor = 22; 
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;

int enableLeftMotor = 23;
int leftMotorPin1 = 18;
int leftMotorPin2 = 19;

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

#define SIGNAL_TIMEOUT 1000  // Signal timeout in milliseconds
unsigned long lastRecvTime = 0;

// 74HC595 Pin Definitions for LEDs
int latch_Pin = 12;  // Latch pin (GPIO 12)
int clock_Pin = 13;  // Clock pin (GPIO 13)
int data_Pin = 14;   // Data pin (GPIO 14)

// Data packet structure
struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData receiverData;

bool throttleAndSteeringMode = false;

// Color data for LEDs
uint16_t redData = 0b0000010010010010;    // Red LEDs
uint16_t greenData = 0b0000100100100100;  // Green LEDs
uint16_t blueData = 0b0000001001001001;   // Blue LEDs
uint16_t yellowData = 0b0000110110110110; // Yellow LEDs (Red + Green)
uint16_t cyanData = 0b0000101101101101;   // Cyan LEDs (Blue + Green)
uint16_t pinkData = 0b0000011011011011;   // Pink LEDs (Red + Blue)
uint16_t offData = 0b0000000000000000;    // All LEDs off

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    blinkLed(blueData);
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  // Update LEDs based on data
  showLEDs();
  // Update motors based on data
  if (receiverData.switchPressed == 1)  // Checking if the switch is pressed
  {
    throttleAndSteeringMode = !throttleAndSteeringMode;  // Toggle mode
  }

  if (throttleAndSteeringMode)
  {
    throttleAndSteeringMovements();
  }
  else
  {
    simpleMovements();
  }
  
  lastRecvTime = millis();   
}

void showLEDs() {
  uint16_t colorData;
  if (receiverData.switchPressed == 1) {
    colorData = pinkData;  // Example for switchPressed = 1
  } else if (receiverData.yAxisValue <= 75) {
    colorData = pinkData; // Move car forward
  } else if (receiverData.yAxisValue >= 175) {
    colorData = redData;  // Move car backward
  } else if (receiverData.xAxisValue >= 175) {
    colorData = cyanData; // Move car right
  } else if (receiverData.xAxisValue <= 75) {
    colorData = greenData; // Move car left
  } else {
    colorData = offData;  // Stop the car
  }

  showColor(colorData);
}

void showColor(uint16_t data) {
  digitalWrite(latch_Pin, LOW);
  shiftOut(data_Pin, clock_Pin, MSBFIRST, highByte(data)); // Shift out high byte of data
  shiftOut(data_Pin, clock_Pin, MSBFIRST, lowByte(data));  // Shift out low byte of data
  digitalWrite(latch_Pin, HIGH); // Latch the data to output pins
}
void blinkLed(uint16_t data){
  digitalWrite(latch_Pin,LOW);
  shiftOut(data_Pin, clock_Pin, MSBFIRST, highByte(data));
  shiftOut(data_Pin, clock_Pin, MSBFIRST, lowByte(data));  // Shift out low byte of data
  digitalWrite(latch_Pin, HIGH);
  delay(10);
  digitalWrite(latch_Pin,LOW);
  shiftOut(data_Pin, clock_Pin, MSBFIRST, highByte(offData));
  shiftOut(data_Pin, clock_Pin, MSBFIRST, lowByte(offData));  // Shift out low byte of data
  digitalWrite(latch_Pin, HIGH);
  

}
void fadeLed(uint16_t data){
  digitalWrite(latch_Pin,LOW);
  shiftOut(data_Pin, clock_Pin, MSBFIRST, highByte(data));
  shiftOut(data_Pin, clock_Pin, MSBFIRST, lowByte(data));  // Shift out low byte of data
  digitalWrite(latch_Pin, HIGH);
  delay(50);
  digitalWrite(latch_Pin,LOW);
  shiftOut(data_Pin, clock_Pin, MSBFIRST, highByte(offData));
  shiftOut(data_Pin, clock_Pin, MSBFIRST, lowByte(offData));  // Shift out low byte of data
  digitalWrite(latch_Pin, HIGH);
  delay(50);

}

// Function to shift out data to the 74HC595 IC
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
  for (int i = 0; i < 8; i++) {
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(val & (1 << i)));  // Write the bit value
    } else {
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));  // Write the bit value
    }
    digitalWrite(clockPin, HIGH);  // Clock the bit
    digitalWrite(clockPin, LOW);
  }
}

void simpleMovements()
{
  if (receiverData.yAxisValue <= 75)       // Move car forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.yAxisValue >= 175)   // Move car backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue >= 175)  // Move car right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValue <= 75)   // Move car left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                      // Stop the car
  {
    rotateMotor(0, 0);
    blinkLed(blueData);
  }   
}

void throttleAndSteeringMovements()
{
  int throttle = map(receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map(receiverData.xAxisValue, 0, 254, -255, 255);  
  int motorDirection = 1;
  
  if (throttle < 0)  // Move car backward
  {
    motorDirection = -1;    
  }

  int rightMotorSpeed = abs(throttle) - steering;
  int leftMotorSpeed = abs(throttle) + steering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  } 

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));    
}

void setUpPinModes()
{
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Set up PWM for motor speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel); 
  
  rotateMotor(0, 0);
  
  // Set up 74HC595 pins
  pinMode(latch_Pin, OUTPUT);
  pinMode(clock_Pin, OUTPUT);
  pinMode(data_Pin, OUTPUT);
  
  // Initialize the pins to a known state
  digitalWrite(latch_Pin, LOW);
  digitalWrite(clock_Pin, LOW);
  digitalWrite(data_Pin, LOW);
}

void setup() 
{
  setUpPinModes();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
  // Check for signal loss
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) 
  {
    showColor(offData); // Turn off all LEDs if signal is lost
  }
}


