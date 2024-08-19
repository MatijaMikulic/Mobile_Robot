#pragma region Includes
//################### <INCLUDE> #############################
#include <BluetoothSerial.h>
#include "L298N.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_P.h"
//#include <MadgwickAHRS.h>
#include <esp32-hal-bt.h>
#include "CommandQueue.h"
#include "..\lib\IMU\MPU6050Imp\MPU6050Imp.h"
#include "..\lib\IMU\MPU9250Imp\MPU9250Imp.h"
#include <memory>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
/*################### <\INCLUDE> #############################*/
#pragma endregion

#pragma region Global Constants
/*################### <GLOBAL CONSTANTS> ####################*/
// Motor A
const uint8_t enA = 15;
const uint8_t in1 = 2;
const uint8_t in2 = 4;

// Motor B
const uint8_t enB = 19;
const uint8_t in3 = 5;
const uint8_t in4 = 18;

//IR sensor A
const uint8_t ir_A = 33;
//IR sensor B
const uint8_t ir_B = 25;
//Encoder disc
const uint8_t discHoles = 20;
//Wheel radius [mm]
const float wheelRadius = 32.5f; //31
//Complementary filter
const float alpha = 0.98f;

const float factor = 0.91f;

const int manualCommandLength = 3;
/*################### <\GLOBAL CONSTANTS> ####################*/
#pragma endregion

#pragma region Timers
/*################### <TIMERS> ##############################*/

// Timer for calculating traversed distance (and sending data to app)
hw_timer_t *Timer0_Cfg = NULL;

// Timer for P control (straight line control and rotate control)
hw_timer_t *Timer1_Cfg = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
/*################### <\TIMERS> ##############################*/
#pragma endregion

#pragma region Global Variables
/*################### <GLOBAL VARIABLES> ####################*/
MPU6050 mpu6050_1(0x69);
MPU9250Impl imu;

// Classic bluetooth connection
BluetoothSerial SerialBT;
L298N motors(enA, enB, in1, in2, in3, in4, 8);

// Initial motor speeds
volatile uint16_t AMotorSpeed = 170;
volatile uint16_t BMotorSpeed = 170;

// Estimated speed at which mobile robot moves in a straight line when BMotorSpeed = 170
// This speed will be adjusted inside control straight line function
uint16_t adjustedSpeed = 150; 

// Madgwickfilter 
// Madgwick filter;
unsigned long microsPrevious, currentTime, previousTime;

// Orientation, desired orientation ,angular speed
double heading_6050, heading_9250;
double targetAngle;

// Yaw rate for MPU6050 and MPU9250
float gyroX_degs, gyroZ_degs;

// IR holes counter
volatile int numOfHolesA=0;
volatile int numOfHolesB=0;

// Flags for activating controlStraightLine and rotate
volatile bool activateLineController = false;
volatile bool control_needed=false;
volatile bool activateRotateController = false;
volatile bool isRotateNeeded = false;
volatile bool isAngleCorrectionNeeded = false;

// Distance travelled 
double distance = 0;
double targetDistance=0;

// For controlStraightLine and controlRotate
int countRotate = 0;
int countStraight = 0;

// For storing commands that arrive via Bluetooth
CommandQueue queue;

// Current Command executing
Command currentCommand;

// PID controller for controlling target angle
// Only Kp is used
float kp = 4;
float ki = 5;
float kd = 2;
// This params need to be reset every time control algorithm starts.
// These are not used as only P controller is used
float integral = 0.0f;
int previousError = 0.0f;
/*################### <\GLOBAL VARIABLES> ####################*/
#pragma endregion

#pragma region Declarations
/*################### <DECLARATIONS> ########################*/

void setOffsetsMPU6050();
void setupMPU6050();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void updateSpeed(uint16_t speedA, uint16_t speedB);
void stop();
void setupTimer();
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param);
void ExecuteManualCommand(const char* command);
void ProcessData(const char* data, size_t length);
Command parseCommand(const char* commandString);
void executeAutoCommand();
void execute();
void MPUCalculation(void *parameter);
void runPID();
void countHolesA();
void countHolesB();
void controlStraightLine();
void controlRotate();
int  calculateAngleOffset(float target_angle, float current_angle);
bool should_turn_right(int deltaAngle);
bool should_turn_left(int deltaAngle);
void IRAM_ATTR Timer1_ISR();
void IRAM_ATTR Timer0_ISR();

/*################### <\DECLARATIONS> ########################*/
#pragma endregion

void setup() {

  // This is for I2C communication (for IMU sensors)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  
  /* Setting up MPU 9250 sensor */
  imu.initializeMPU();
  /* This function isn't used because MPU9250 will automatically recalibrate gyro sensor on start.
     Accel and magnetometer calibration data are used inside method for calculating angles*/
  imu.setOffsets();

  /* Setting up MPU 6250 sensor */
  setupMPU6050();
  //setOffsetsMPU6050();
  mpu6050_1.CalibrateAccel(6);
  mpu6050_1.CalibrateGyro(6);

  /* Setting bluetooth serial connection */
  SerialBT.begin("ESP32test");
  /* Callback function will handle incoming data and connection loss */
  SerialBT.register_callback(callback);

  // Setting up both timers
  setupTimer();

  // Setup task for second core. This task will continiously calculate heading angle from both
  // IMU sensors 
  xTaskCreatePinnedToCore(MPUCalculation, "MPUCalculation", 4096, NULL, 1, NULL, 0);

  currentTime = micros();

  // Attaching interrupts for encoders
  pinMode(ir_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir_A), countHolesA, RISING);  
  pinMode(ir_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir_B), countHolesB, RISING); 

  // Initialy theere is no command to be executed
  currentCommand.type = CommandType::NONE;
  currentCommand.angle = -1;
  currentCommand.distance = -1;
  currentCommand.state = CommandState::COMPLETED;   
}

void loop() {
  // Pooling implementation

  // For straight line control...
  if (control_needed && !isAngleCorrectionNeeded) {
    
    // If the angle error is too high, correct fast using rotate
    if(abs(round(targetAngle - heading_9250)) > 15){
      isAngleCorrectionNeeded = true;
    }
  	
    // This will calculate the speed of motors at which mobile robot will move in a straight line
    // This is needed because one of the motors is stronger than the other and because of the caster wheel
    // which might make the robot move in wrong direction
    controlStraightLine();

    // Set the speed of both motors (pwm signal)
    moveForward();
    ledcWrite(0, AMotorSpeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(1, BMotorSpeed);
    digitalWrite(in3,HIGH);
    digitalWrite(in4, LOW);

    //Serial.print(motors.getSpeedA());
    //Serial.print(", ");
    //Serial.println(motors.getSpeedB());
    
    // Stop the robot if robot has travelled desired distance
    // If the robot has reached the desired destination current command will be completed 
    // The next command (if there is any) will be ready to be executed
    if(targetDistance > 0 && distance >=targetDistance && currentCommand.type == CommandType::FORWARD){
      stop();
      control_needed = false;
      activateLineController = false;
      currentCommand.state = CommandState::COMPLETED;
    }
    control_needed = false;
  }

  // For rotation in place
  if (isRotateNeeded){
    controlRotate();
    isRotateNeeded = false;
  }

  // Pooling mode for command execution
  executeAutoCommand();
}

/**
 * @brief Initializes and sets up the MPU6050 sensor.
 */
void setupMPU6050(){
  mpu6050_1.initialize();
  if(mpu6050_1.testConnection()){
    Serial.println("Success");
  }
  else{
    Serial.println("Error");
  }
  //mpu6050_1.setRate(20);
  mpu6050_1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu6050_1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

/**
 * @brief Function to set calibration offsets for MPU6050 sensor.
 */
void setOffsetsMPU6050(){
  mpu6050_1.setXAccelOffset(AX_OFFSET);
  mpu6050_1.setYAccelOffset(AY_OFFSET);
  mpu6050_1.setZAccelOffset(AZ_OFFSET);
  mpu6050_1.setXGyroOffset (GX_OFFSET);
  mpu6050_1.setYGyroOffset (GY_OFFSET);
  mpu6050_1.setZGyroOffset (GZ_OFFSET);
}

/**
 * @brief Initializes timers.
 */
void setupTimer(){
    // 80 MHz frequency for system clock

    // Every 50 ms
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 50000, true);
    timerAlarmEnable(Timer0_Cfg);

    // Every 10 ms
    Timer1_Cfg = timerBegin(1, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 10000, true);
    timerAlarmEnable(Timer1_Cfg);
} 

/**
 * @brief Interrupt Service Routine for Timer0.
 *  Calculates the distance traveled by the robot.
 */
void IRAM_ATTR Timer0_ISR(){
  double avgNumOfHoles = (numOfHolesA + numOfHolesB) / 2;
  // DiscHoles * 2 because for every hole, IR sensor on RISING picks up 2
  // In total, for one rotation with 20 holes, the sensor reads 40 RISING edges
  double numOfRotations = avgNumOfHoles/(discHoles*2);
  distance = numOfRotations * wheelRadius * 2 * PI / 10.0; //in cm

  
  // Sending data via bluetooth
  char buffer[30]; 
  itoa((int16_t)adjustedSpeed, buffer, 10);              
  strcat(buffer, ",");                     
  itoa((int16_t)distance, buffer + strlen(buffer), 10); 
  strcat(buffer, ",");
  itoa((int16_t)heading_9250, buffer + strlen(buffer), 10);
  //dtostrf(roll, 0, 6, buffer + strlen(buffer));
  buffer[sizeof(buffer) - 1] = '\0';
  SerialBT.println(buffer);

}

/**
 * @brief Interrupt Service Routine for Timer1.
 *  Activates robot control based on conditions.
 */
void IRAM_ATTR Timer1_ISR(){
  if(activateLineController){
    control_needed = true;
  }
  if(activateRotateController){
    isRotateNeeded = true;
  }
}

/**
 * @brief Task to run on a separate core for MPU calculation
 * @param parameter Unused parameter
 */
void MPUCalculation(void *parameter) {
  (void) parameter; // Unused parameter
  for (;;) {

      // Calculate all angles from MPU 9250 (uses mangnetometer, accel, gyro data)
      Angles angles = imu.calculateAngles();
      
      // Calculating angles from MPU 6050
      int16_t accX_raw_1, accY_raw_1, accZ_raw_1;
      int16_t gyroX_raw_1, gyroY_raw_1, gyroZ_raw_1; 

      mpu6050_1.getMotion6(&accX_raw_1, &accY_raw_1, &accZ_raw_1, &gyroX_raw_1, &gyroY_raw_1, &gyroZ_raw_1);

      // Acceleration [m/s^2] for MPU1
      float accX_ms2_1 = accX_raw_1 / ACCEL_DIVIDER_2G;
      float accY_ms2_1 = accY_raw_1 / ACCEL_DIVIDER_2G;
      float accZ_ms2_1 = accZ_raw_1 / ACCEL_DIVIDER_2G;

      // Angular velocity [deg/s] for MPU1
      gyroX_degs = gyroX_raw_1 / GYRO_DIVIDER_250;
      float gyroY_degs_1 = gyroY_raw_1 / GYRO_DIVIDER_250;
      float gyroZ_degs_1 = gyroZ_raw_1 / GYRO_DIVIDER_250;

      // Find the angle formed by the projection of the Y-axis acceleration onto the plane formed by the X and Z axes.
      float accYaw = (atan(accY_ms2_1 / sqrt(pow(accX_ms2_1, 2) + pow(accZ_ms2_1, 2))) * 180 / PI);
      float gyroYaw;

      // Calculating yaw angle based on gyrosope data by integrating
      previousTime = currentTime;
      currentTime = micros();
      float DT = (currentTime - previousTime) / 1000000.0; // convert to seconds
      gyroYaw += gyroX_degs * DT;

      // Combine accelerometer and gyroscope yaw to get the final yaw (complementary filter)
      double yaw = alpha * gyroYaw + (1-alpha) * accYaw;

      // Sets the angle between 0 and 360
      yaw = fmod(yaw, 360.0);

      // Sets the angle between -180 and 180
      if(yaw > 180){
        yaw -=360;
      }
      else if(yaw<-180){
        yaw +=360;
      }
      heading_6050 = yaw;
      heading_9250 = -angles.yaw;
      gyroZ_degs   = -angles.rotationSpeed;

      //Serial.print(-angles.yaw);
      //Serial.print(",");
      //Serial.println(yaw);
      //vTaskDelay(pdMS_TO_TICKS(10)); 
    
  }
}

/**
 * @brief Bluetooth callback function
 * @param event Bluetooth callback event
 * @param param Structure containing data.
 */
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  uint8_t* sourceData; 
  // Every command starts with character '&' and ends with ';'
  char* command; 
  size_t length;

  switch (event) {
    // If connection to client is lost
    case ESP_SPP_NO_CONNECTION:
    case ESP_SPP_CLOSE_EVT:
     ExecuteManualCommand("&x;"); //stop the mobile robot
     break; 

    // If new data has arrived
    case ESP_SPP_DATA_IND_EVT:
      // Fetch data
      sourceData = param->data_ind.data;
      length = param->data_ind.len;
      command = (char*) malloc(length+1);
      memcpy(command,sourceData,length);
      command[length]='\0';
      
      // Chekc if the command is for manual control
      if(length == manualCommandLength){ 
        ExecuteManualCommand(command); 
      }
      // If the size of command is greater than 3 it means that the command
      // is for auto control
      // e.g. "&w100;" (move forward 100 cm)
      else{ 
        ProcessData(command,length);
      }
      free(command);
      break;
    default:
      break;
  }
}

/**
 * @brief ExecuteCommand function for processing manual commands
 * @param command Bluetooth command to execute
 */
void ExecuteManualCommand(const char* command){
  // Move forward, straight line control and reset all the variables
  if(strcmp(command,"&w;")==0){
    AMotorSpeed = adjustedSpeed;
    BMotorSpeed = 170;
    targetAngle = heading_6050;
    integral = 0.0f;
    previousError = 0;
    targetDistance = 0;
    numOfHolesA=0;
    numOfHolesB=0;
    distance=0;
    activateLineController = true;
    moveForward();
    updateSpeed(AMotorSpeed,BMotorSpeed);
  }
  // Move backwards
  else if(strcmp(command,"&s;")==0){
    moveBackward();
    updateSpeed(150,150);
  }
  // Move left
  else if(strcmp(command,"&a;")==0){
    turnLeft();
    updateSpeed(130,130);
    Serial.println("left");

  }
  // Move right
  else if(strcmp(command,"&d;")==0){
    turnRight();
    updateSpeed(130,130);
    Serial.println("right");

  }
  // Force stop robot and clear queue
  else if(strcmp(command,"&x;")==0){
    activateLineController = false;
    activateRotateController = false;
    isRotateNeeded = false;
    control_needed=false;
    stop();
    queue.Clear();
    Serial.println("stop");
  }
}

/**
 * @brief Function to process commands for automatic control
 * @param data Data to be processed
 * @param length Length of data
 */
void ProcessData(const char* data, size_t length){
  char *buffer = (char*)malloc(length+1);

  int bufferIndex = 0;
  // This will parse command and add it to the queue
  for(int i=0;i<length;i++){
      if(data[i]=='&'){
        bufferIndex=0;
      }

      buffer[bufferIndex++] = data[i];

      if(data[i]==';'){
        buffer[bufferIndex] ='\0';
        Command command = parseCommand(buffer);
        Serial.println(buffer);
        queue.push(command);
      }
  }
}

/**
 * @brief Function to parse string to Command type
 * @param commandString String to be parsed
 * @return Parsed command
 */
Command parseCommand(const char* commandString){
  // Create command and set it to default values for now
  Command command;
  command.type = CommandType::NONE; 
  command.distance = -1;
  command.angle = -1;
  command.state = CommandState::IDLE;
  // Check if the command is of right format
  if(commandString[0]=='&' && commandString[strlen(commandString)-1] == ';'){
     char type;
     int value;
     int result = sscanf(commandString, "&%c%d;", &type, &value);
     // Every command should have two values, first value is command type
     // the second value is eithe distance to be travelled or angle change
     // hence why the result should be 2
     if(result == 2) 
     {
        switch (type)
        {
        case 'w':
          command.type=CommandType::FORWARD;
          command.distance=value;
          command.angle = -1;
          break;
        case 's':
          command.type=CommandType::BACKWARD;
          command.distance=value;
          command.angle = -1;
          break;
        case 'a':
          command.type=CommandType::LEFT;
          command.angle=value;
          command.distance=-1;
          break;
        case 'd':
          command.type=CommandType::RIGHT;
          command.angle=value;
          command.distance=-1;   
          break;
        default:
          break;
        }
     }
  }
  return command;
}
/**
 * @brief Checks if there is a new command to be executed
 */
void executeAutoCommand(){
  // If previous command was completed and there are new commands to be executed...
  // Find the next command that isn't in state COMPLETED
  if(!queue.isEmpty() && currentCommand.state == CommandState::COMPLETED){
    currentCommand = queue.pop();
    if(currentCommand.state != CommandState::COMPLETED){
      // Start command execution
      execute();
    }
  }
}

/**
 * @brief Implementation of command execution
 */
void execute(){
  switch (currentCommand.type)
  {
  // If the command is forward, activate straight line control
  // Reset all the variables needed for line control
  case CommandType::FORWARD:
    AMotorSpeed = adjustedSpeed;
    BMotorSpeed = 170;

    // Set the target angle to currently calculated angle
    targetAngle = heading_9250;

    // The factor was experimentally set to make sure the robot travels the desired distance
    // as closely as possible
    targetDistance = currentCommand.distance * factor;
    integral = 0.0f;
    previousError = 0;
    distance=0;
    numOfHolesA=0;
    numOfHolesB=0;
    activateLineController = true;
    moveForward();
    updateSpeed(AMotorSpeed,BMotorSpeed);
    break;

  // If the command is left, start rotating in place
  case CommandType::LEFT:
    AMotorSpeed = 128;
    BMotorSpeed = 128;
    // Set the target angle by adding desired angle change to current orientation
    // The factor was experimentally set to minimize real angle difference
    targetAngle = heading_9250 + currentCommand.angle * factor;

    // The angle should be between -180 and 180
    if (targetAngle <= -180) {
        targetAngle += 360;
    }
    else if (targetAngle > 180) {
        targetAngle -= 360;
    }
    integral = 0.0f;
    previousError = 0;
    activateRotateController=true;
    break;
  
  // If the command is right, start rotating in place
  // Same implementation as LEFT
  case CommandType::RIGHT:
    AMotorSpeed = 128;
    BMotorSpeed = 128;
    targetAngle = heading_9250 - currentCommand.angle * factor;
    if (targetAngle <= -180) {
        targetAngle += 360;
    }
    else if (targetAngle > 180) {
        targetAngle -= 360;
    }
    integral = 0.0f;
    previousError = 0;
    activateRotateController=true;
    break;
  default:
    break;
  }
}

/**
 * @brief Change the motor speed by the specified increment.
 * @param motorSpeed Current motor speed.
 * @param increment Amount by which to increment the motor speed.
 * @return The updated motor speed.
 */
int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  // Ensure the new speed is inside the interval [120,255].
  // Those are the minimum and maximum pwm values 
  if (motorSpeed > 255){ 
    motorSpeed = 255;
  } else if (motorSpeed <120){
    motorSpeed = 120;
  }
  return motorSpeed;
}


/**
 * @brief Control the robot to move in a straight line.
 */
void controlStraightLine(){

  int deltaAngle = round(targetAngle - heading_6050);
  int absDifference = abs(deltaAngle);
  if (absDifference < 2){
    // Indicates that the robot has been aligned for a long enough duration
    if (countStraight < 10){
      countStraight ++;
    } 
    else {
      countStraight = 0;
      adjustedSpeed = AMotorSpeed; 
    }
  } 
  else {
    countStraight = 0;
  }  

  // Target angle change is used to achieve desired motor speed and achieve the target direction.
  // Angle increases as the robot turns right (rotates right). GyroX also increases (has positive sign).
  // If robot is turning left gyroX increases and has a negative sign.
  int targetGyroX;
  
  // If the angle offset is very large than than set the targetGyroX (desired angle change to 120)
  // The robot needs to make a sharper turn (stronger response). It is restricted to a max value of 120
  if (deltaAngle > 60){
      targetGyroX = 120;
  } 
  else if (deltaAngle < -60){
    targetGyroX = -120;
  } 
  // Proportional control
  // For smaller delta angles, we need smaller rotation speed (yaw rate)
  else {
    targetGyroX = 2 * deltaAngle;
  }

  /* This is PID control*/

  /* float dt = 0.01f;
  int error = targetGyroX - gyroX_degs;
  integral += error * dt;
  float output = kp*error + ki*integral + kd*(error - previousError)/dt; 
  previousError = error;
  AMotorSpeed = changeSpeed(AMotorSpeed,output);  */
  
  // Motor A is stronger than motor B. Speed of motor B will remain the same. Speed of motor A will be adjusted accordingly.
  // If the targetGyroX is greater than the current reading, the motor speed (AMotorSpeed) is adjusted by calling the 
  // changeSpeed function with a positive increment (proportional control). This will increase the motor speed to rotate towards the target angle.
  int diff = round(targetGyroX - gyroX_degs);
  if (diff != 0) {
    AMotorSpeed = (targetGyroX > gyroX_degs) ? changeSpeed(AMotorSpeed, kp*absDifference) : changeSpeed(AMotorSpeed, -kp*absDifference);
  }
    
}

volatile int rotateControlCounter = 0;
/**
 * @brief Control the robot to rotate based on the target angle.
 */
void controlRotate() {
  // Increment the counter
  rotateControlCounter++;  

  // Execute the control logic only every 2 counts (10 ms might not be enoug for angle to converge, so this function will be run every 20 ms)
  if (rotateControlCounter % 2 != 0) {
    return;
  }

  int deltaAngle = calculateAngleOffset(targetAngle, heading_9250);
  int targetGyroX;

  // Check if the angle difference is small
  if (abs(deltaAngle) <= 2) {
    stop();
    // Indicates that the robot has been within the acceptable range for a period of time
    // This also means that the current command has finished executing
    if (countRotate < 5) {
      countRotate++;
    } else {
      stop();
      activateRotateController = false;
      isRotateNeeded = false;
      countRotate = 0;
      currentCommand.state = CommandState::COMPLETED;
    }
  } else {
    countRotate = 0;
    if (should_turn_left(deltaAngle)) {
      turnLeft();
    } else if (should_turn_right(deltaAngle)) {
      turnRight();
    }

    // The robot needs to make a sharper turn (stronger response).
    // With targetGyroX we can influence speed of rotation
    // By increasing or decreasing speed we can change speed of rotation
    if (abs(deltaAngle) > 18) {
      targetGyroX = 18;
    } else {
      // For smaller delta angles, we need smaller rotation speed
      targetGyroX = 0.5 * abs(deltaAngle);
    }

    /* PID control*/
    /* int error = targetGyroX - abs(gyroX_degs);
    integral += error * dt;
    float output = kp*error + ki*integral + kd*(error - previousError)/dt; 
    previousError = error;
    AMotorSpeed = changeSpeed(AMotorSpeed,output); */
    
    // similar logic to controlStraightLine
    if (round(targetGyroX - abs(gyroZ_degs)) != 0) {
      AMotorSpeed = (targetGyroX > abs(gyroZ_degs)) ? changeSpeed(AMotorSpeed, +1) : changeSpeed(AMotorSpeed, -1);
    } 

    // B motor has the same speed as motorA so that robot rotates in place.
    BMotorSpeed = AMotorSpeed;
    updateSpeed(AMotorSpeed, BMotorSpeed);
  }
}

// Helper function to change speed with bounds checking
int changeSpeed(int motorSpeed, float increment) {
    // Use round to handle small increments properly
    motorSpeed += round(increment); 
    if (motorSpeed > 255) {
        motorSpeed = 255;
    } else if (motorSpeed < 120) {
        motorSpeed = 120;
    }
    return motorSpeed;
}


/**
 * @brief Count the holes for encoder A.
 */
void countHolesA(){
  // Count holes only when robot is going foward or backwards
  if ((motors.getMotorAStatus() == motors.FORWARD  && motors.getMotorBStatus() == motors.FORWARD) 
      ||  (motors.getMotorAStatus() == motors.BACKWARD  && motors.getMotorBStatus() == motors.BACKWARD)){
    numOfHolesA++;
  }
}

/**
 * @brief Count the holes for encoder B.
 */
void countHolesB(){
  //count holes only when robot is going foward or backwards
  if ((motors.getMotorAStatus() == motors.FORWARD  && motors.getMotorBStatus() == motors.FORWARD) 
      ||  (motors.getMotorAStatus() == motors.BACKWARD  && motors.getMotorBStatus() == motors.BACKWARD)){
    numOfHolesB++;
  }
}

/**
 * @brief Calculate the angle offset between the target and current angles.
 * @param target_angle Target angle in degrees.
 * @param current_angle Current angle in degrees.
 * @return The angle offset in degrees.
 */
int calculateAngleOffset(float target_angle, float current_angle) {
    // the sign of delta will decide whether the robot should turn right or left.
    int delta;
    if(current_angle > 0){
        delta = (int)(target_angle - current_angle + 180) % 360 - 180;
        if (delta <= -180) {
            delta += 360;
        }
        else if (delta > 180) {
            delta -= 360;
        }
    }
    else if(current_angle < 0 ){
        delta =(int)(current_angle - target_angle + 180) % 360 - 180;
        if (delta <= -180) {
            delta += 360;
        }
        else if (delta > 180) {
            delta -= 360;
        }
        delta = delta*-1;
    }
    
    return delta;
}

/**
 * @brief Check if the robot should turn right based on the delta angle.
 * @param deltaAngle Delta angle in degrees.
 * @return True if the robot should turn right, false otherwise.
 */
bool should_turn_right(int deltaAngle) {
    return deltaAngle < 0;
}

/**
 * @brief Check if the robot should turn left based on the delta angle.
 * @param deltaAngle Delta angle in degrees.
 * @return True if the robot should turn left, false otherwise.
 */
bool should_turn_left(int deltaAngle){
    return deltaAngle > 0;
}

/**
 * @brief Move the robot forward.
 */
void moveForward(){
  motors.moveForward_A();
  motors.moveForward_B();
}

/**
 * @brief Move the robot backward.
 */
void moveBackward(){
  motors.moveBackward_A();
  motors.moveBackward_B();
}

/**
 * @brief Rotate the robot right
 */
void turnRight(){
  motors.moveBackward_A();
  motors.moveForward_B();
}

/**
 * @brief Rotate the robot left
 */
void turnLeft(){
  motors.moveForward_A();
  motors.moveBackward_B();
}

/**
 * @brief Stop the robot
 */
void stop(){
  motors.stop_A();
  motors.stop_B();
}

/**
 * @brief Update the speeds of both motors.
 * @param speedA Speed of motor A.
 * @param speedB Speed of motor B.
 */
void updateSpeed(uint16_t speedA, uint16_t speedB){
  motors.updateSpeed_A(speedA);
  motors.updateSpeed_B(speedB);
}