#include <esp_now.h>
#include <WiFi.h>

// Right motor
int enableRightMotor = 22; 
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;

// Left motor
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

struct PacketData
{
  byte xAxisValueJoyStick1;
  byte yAxisValueJoyStick1;
  byte switchPressedJoyStick1;
};
PacketData receiverData;

bool throttleAndSteeringMode = false;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValueJoyStick1 + "  " + receiverData.yAxisValueJoyStick1 + "  " + receiverData.switchPressedJoyStick1;
  Serial.println(inputData);
  
  if (receiverData.switchPressedJoyStick1 == 1)  // Checking if the switch is pressed
  {
    throttleAndSteeringMode = !throttleAndSteeringMode;  // Toggle mode
  }0sw

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

void simpleMovements()
{
  if (receiverData.yAxisValueJoyStick1 <= 75)       // Move car forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.yAxisValueJoyStick1 >= 175)   // Move car backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValueJoyStick1 >= 175)  // Move car right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (receiverData.xAxisValueJoyStick1 <= 75)   // Move car left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                      // Stop the car
  {
    rotateMotor(0, 0);
  }   
}

void throttleAndSteeringMovements()
{
  int throttle = map(receiverData.yAxisValueJoyStick1, 254, 0, -255, 255);
  int steering = map(receiverData.xAxisValueJoyStick1, 0, 254, -255, 255);  
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
    rotateMotor(0, 0);
  }
}
