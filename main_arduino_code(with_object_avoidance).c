#include <Arduino_BMI270_BMM150.h>
#include <math.h>
#include <ArduinoBLE.h>
#include <Servo.h>

/*************BLUETOOTH*******************/
// Define UUIDs for BLE Service and Characteristic
BLEService controlService("69042F00-F8C3-4F4A-A8F4-15CD926DA146");  // Main BLE Service

// Use a String characteristic to handle movement commands
BLEStringCharacteristic movementCharacteristic("69042F04-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 20);

// Initialize PWM value (0 - 255)
int motorSpeed = 255*80; // Maximum speed for testing
float Turn_Var_Left = 0;
float Turn_Var_Right = 0;
/****************************************/

/* Queue for fixing forward*/



enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  CLAW_OPEN = 5,
  CLAW_CLOSE = 6,
  DISTANCE = 7,
  UPRAMP_5 = 8,
  UPRAMP_10 = 9
};

// acc readings
float a, b, c;
// gyro readings
float a1, b1, c1;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

static unsigned long lastTime = 0; 

bool movingBackward = false;
bool f = false;
bool balance1 = false;
bool balance2 = false;
bool balance3 = false;
bool balance4 = false;
bool balance5 = false;
bool balance6 = false;
bool turningRight = false;
bool turningLeft = false;
bool forward = false;
bool forward1 = false;
bool forward2 = false;
float backwardStartTime =0;
float balanceTime = 0;
float turnedAngle = 0;
float forwardTime = 0;

float theta1 = 0.0;
float theta = 0.0;


int AIN1PIN = 4;
int AIN2PIN = 5;
int BIN1PIN = 7;
int BIN2PIN = 6;

// PID variables
float deltaTime2 = 0;
float error1 = 0;
float k = 0.0;
float out = 0.0;
float errorSum = 0.0;
float lastError = 0.0;
float targetAngle = 0;
float k_calc = 0.0;
float derivative = 0.0;

float wheel_offset = 1.0;
float right_wheel_offset = 1.05;


int SIGNAL_PIN = 3;
int SERVO_PIN = 8;
int SERVO_PIN1 = 9;
bool FORWARD_FLAG = false;
bool DISTANCE_FLAG = false;

int PREVSTATE = STOP;
int STOPLOOPCOUNT = 10;
bool STOPFORWARD = false;
bool STOPBACKWARD = false;
bool RAMP_MODE_FLAG = false;

void motorStop() {
  analogWrite(AIN1PIN, 255);
  analogWrite(AIN2PIN, 255);
  analogWrite(BIN2PIN, 255);
  analogWrite(BIN1PIN, 255);
  //Serial.println("Stop");
  return;
}

void motorInput() {
  /********B wheel is slower**********/
  if (k_calc < 0) {
      analogWrite(AIN1PIN, 255+k_calc*wheel_offset-Turn_Var_Left);
      digitalWrite(AIN2PIN, HIGH);
      analogWrite(BIN1PIN, 255+k_calc*right_wheel_offset-Turn_Var_Right);
      digitalWrite(BIN2PIN, HIGH);
  } 
  else if (k_calc > 0) {
      digitalWrite(AIN1PIN, HIGH);
      analogWrite(AIN2PIN, 255-k_calc*wheel_offset-Turn_Var_Right);
      digitalWrite(BIN1PIN, HIGH);
      analogWrite(BIN2PIN, 255-k_calc*right_wheel_offset-Turn_Var_Left);
  }
  return;
}

float calculatePID(float out) {
  static unsigned long lastTime = 0; 
  unsigned long currentTime = millis();
  deltaTime2 = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  float Kp = 7.9; //550
  float Ki = 96; //200
  float Kd = 0.3; //12.5
  

  // PID Control
  error1 = targetAngle - out;
  errorSum += error1 * deltaTime2;  // Integrate with respect to time
  errorSum = constrain(errorSum, -5, 5);  // Anti-windup
  
  // Calculate PID output with proper derivative term
  derivative = (error1 - lastError) / deltaTime2;
  lastError = error1;
  k = Kp * error1 + Ki * errorSum + Kd * derivative;
  k = constrain(k, -255, 255);
  return k;
}

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(a, b, c);
    if (b != 0) {
      theta = atan(b/c)*180/M_PI;
    } else {
      theta = 90;
    }
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(a1, b1, c1);
    theta1 = 0; // Start with zero integrated angle
  }

  // pin (setup)
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN1, OUTPUT);
  pinMode(SIGNAL_PIN, INPUT);
  pinMode(AIN1PIN, OUTPUT);
  pinMode(AIN2PIN, OUTPUT);
  pinMode(BIN1PIN, OUTPUT);
  pinMode(BIN2PIN, OUTPUT);

  /*********BLUETOOTH*******/
  // Start BLE
  if (!BLE.begin()) {
      Serial.println("Starting BLE failed!");
      while (1);
  }

  // Set device name and add service
    BLE.setLocalName("Arduino Nano 33 BLE Sense");
    BLE.setAdvertisedService(controlService);
    controlService.addCharacteristic(movementCharacteristic);
    BLE.addService(controlService);

    // Set initial movement value
    movementCharacteristic.writeValue("stop");

    // Start advertising
    BLE.advertise();
    //Serial.println("Waiting for BLE connection...");
  /************************/

  lastTime = millis();
}

void loop() {
  unsigned long lastTimePID = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  if (deltaTime < 0.001) {
    deltaTime = 0.001;
  }
  
  if (FORWARD_FLAG){
    targetAngle = 0.3;
  }
  if ((digitalRead(SIGNAL_PIN) == HIGH) && DISTANCE_FLAG) {
      f = true;
      DISTANCE_FLAG = false;
      FORWARD_FLAG = false;
      Serial.println("Object detected within 20 cm!");
  }
  float x, y, z; // acc readings
  float x1, y1, z1; // gyro readings
  float theta2 = 0;
  
  // Read accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    
    if (y != 0 && z != 0) {  // Check both to avoid division by zero
      theta = atan(y/z)*180/M_PI; //rad
    }
  }
  
  // Read gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x1, y1, z1);
    x1 -= gyroBiasX;
    theta2 = (x1) * deltaTime; //rad/sec
  }

  // Calculate new output - use complementary filter approach
  float alpha = 0.98;  // Weight factor for gyro vs accelerometer
  out = alpha * (out - theta2) + (1-alpha) * theta;
  
  k_calc = calculatePID(out);
  motorInput();

  //Obstacle Detection
  if (f && !movingBackward) {
      Serial.println("Balancing");
      targetAngle = -1;
      f = false;
      movingBackward = true;
      backwardStartTime = millis();  // Record start time
    }
    
    if (movingBackward && (millis() - backwardStartTime >= 1000)) {
        Serial.println("Backward movement complete. Balancing...");
        movingBackward = false;
        //balance2 = true;
        targetAngle = 0;
        //balanceTime = millis();
        Turn_Var_Left = 25;
        Turn_Var_Right = -25;
        turningRight = true;

    }
    
    
    // Handle turning right after moving backward
    if (turningRight) {
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(a1, b1, c1);
            turnedAngle += c1 * deltaTime;
        }

        if (turnedAngle <= -65) {  // Stop turning when 90-degree turn is complete
            motorStop();
            Serial.println("Turn complete, resuming balance.");
            Turn_Var_Left = 0;
            Turn_Var_Right = 0;
            turningRight = false;
            turnedAngle = 0;
            targetAngle = -1;
            forward = true;
            forwardTime = millis();
            
        }
    }
    if (forward && (millis() - forwardTime >= 800)){
      Serial.println("Forward2 Complete");
      forward = false;
      targetAngle = 0; 
      turningLeft = true;
      Turn_Var_Left = 25;
      Turn_Var_Right = -25; 
    }
    if (turningLeft) {
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(a1, b1, c1);
            turnedAngle += c1 * deltaTime;
        }

        if (turnedAngle <= -65)  {  // Stop turning when 90-degree turn is complete
            motorStop();
            Serial.println("Turn complete, resuming balance.");
            Turn_Var_Left = 0;
            Turn_Var_Right = 0;
            turningLeft = false;
            turnedAngle = 0;
            forward1 = true;
            targetAngle = 1;
            forwardTime = millis();
        }
    }
    if (forward1 && (millis() - forwardTime >= 1000)){
      Serial.println("Forward5 Complete");
      forward1 = false;
      balance6 = true;
      targetAngle = 0; 
      balanceTime = millis();
    }
    if (balance6 && (millis() - balanceTime >= 1000)){
        Serial.println("Balance6 Complete");
        //forward1 = true;
        balance6 = false;
        //targetAngle = -1.3; 
        //forwardTime = millis();
    }
  

/***********BLUETOOTH*************/
// Wait for BLE connection
  BLEDevice central = BLE.central();

  if (central) {
          // Check if a new movement command is received
          if (movementCharacteristic.written()) {
              //String movement = movementCharacteristic.value();
              uint8_t movement;
              movementCharacteristic.readValue(&movement, 1);
               // Handle movement commands
              switch(movement) {
                case FORWARD:
                  targetAngle = 1;
                  //Serial.println("Moving Forward");
                  //PREVSTATE = FORWARD;
                  break;
                case BACKWARD:
                  targetAngle = -1;
                  Serial.println("Moving Backward");
                  //PREVSTATE = BACKWARD;
                  break;
                case LEFT:
                  //Serial.println("Turning Left");
                  Turn_Var_Left = -20;
                  Turn_Var_Right = 20;
                  PREVSTATE = LEFT;
                  break;
                case RIGHT:
                  //Serial.println("Turning Right");
                  Turn_Var_Left = 20;
                  Turn_Var_Right = -20; //-10
                  PREVSTATE = RIGHT;
                  break;
                case STOP:
                  targetAngle = 0;
                  Turn_Var_Left = 0;
                  Turn_Var_Right = 0;
                  digitalWrite(SERVO_PIN, LOW); 
                  digitalWrite(SERVO_PIN1, LOW);

                  if (PREVSTATE == LEFT) {
                    motorStop();
                  }
                  else if (PREVSTATE == RIGHT) {
                    motorStop();
                  }
                  PREVSTATE = STOP;
                  break;
                case CLAW_OPEN:
                  digitalWrite(SERVO_PIN, HIGH); 
                  digitalWrite(SERVO_PIN1, LOW); 
                  break;
                case CLAW_CLOSE:
                  digitalWrite(SERVO_PIN, LOW); 
                  digitalWrite(SERVO_PIN1, HIGH);                  
                  break;
                case DISTANCE:
                  DISTANCE_FLAG = !DISTANCE_FLAG;
                  FORWARD_FLAG = !FORWARD_FLAG;
                  //Serial.println(digitalRead(SIGNAL_PIN));
                  //targetAngle = -1.3;
                  break;
                case UPRAMP_5:
                  targetAngle = 5;
                  //RAMP_MODE_FLAG = !RAMP_MODE_FLAG;
                  break;
                case UPRAMP_10:
                  targetAngle = 10;
                  //RAMP_MODE_FLAG = !RAMP_MODE_FLAG;
                  break;
                default:
                  targetAngle = 0.0;
                  Turn_Var_Left = 0;
                  Turn_Var_Right = 0;
                  DISTANCE_FLAG = false;
                  FORWARD_FLAG = false;
                  break;
              }
          }

    if (RAMP_MODE_FLAG) {
                    targetAngle = out - 2;
                    Serial.print("RAMP");
                    Serial.print("\t");
    }

  }

/********************************/
 

  // Debug output (reminder- printing stuff makes everything slower!!!)
  Serial.print("TA: ");
  Serial.print(targetAngle);
  Serial.print('\t');
  Serial.print("out: ");
  Serial.println(out);
  //Serial.print('\t');
  //Serial.print("errorSum: ");
  //Serial.print(errorSum);
  //Serial.print('\t');
  //Serial.print("Derivative: ");
  //Serial.print(derivative);
  //Serial.print('\t');
  //Serial.println(deltaTime2);
  
  //Serial.print(theta);
  //Serial.print('\t');
  //Serial.println(out - theta2);
  //Serial.print('\t');
  //Serial.print(Kp);
  //Serial.print('\t');
  //Serial.print('\t');
  //Serial.print(error);
  //Serial.print(deltaTime);
  //Serial.print(errorSum);
  //Serial.print('\t');
  //Serial.println(255-constrain(abs(k_calc), 0, 255));
  //Serial.println(60);
  //Serial.print(",");
  //Serial.println(-60);
}