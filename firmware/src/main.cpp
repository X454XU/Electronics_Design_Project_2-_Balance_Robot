#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <mpu6050.h>
#include <PIDController.h>
#include <chrono> // For time functions


// The Stepper pins
#define STEPPER1_DIR_PIN 16   //Arduino D9
#define STEPPER1_STEP_PIN 17  //Arduino D8
#define STEPPER2_DIR_PIN 4    //Arduino D11
#define STEPPER2_STEP_PIN 14  //Arduino D10
#define STEPPER_EN 15         //Arduino D12

// Diagnostic pin for oscilloscope
#define TOGGLE_PIN  32        //Arduino A4

const int PRINT_INTERVAL = 500;
const int LOOP_INTERVAL = 10;
const int  STEPPER_INTERVAL_US = 20;

const int COMMAND_INTERVAL = 5000; // 5 seconds, for testing

//PID tuning parameters
double kp = 1000;
double ki = 1;
double kd = 15;
double setpoint = 0; // 0.0629; 

// PID tuning parameters for speed control
double speedKp = 100;
double speedKi = 0;
double speedKd = 0;
double speedSetpoint = 0; // Desired speed

double pidOutput;
double speedPidOutput;
double speedControlOutput;
double balanceControlOutput;
double desiredTiltAngle;


//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;  //Default pins for I2C are SCL: IO22/Arduino D3, SDA: IO21/Arduino D4

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

MPU6050Handler mpuHandler;
MPU6050_DATA mpu6050_data;

PID balancePid(kp, ki, kd, setpoint);
PID speedPid(speedKp, speedKi, speedKd, speedSetpoint);

// Complementary filter constant
const double alpha = 0.98; 
const double speedAlpha = 0.1;

double filteredAngle = 0.0;
double previousFilteredAngle = 0.0;

double filteredSpeed = 0.0;
double previousFilteredSpeed = 0.0;

double turningSpeed = 0;

double previousPosition = 0.0;
unsigned long previousTime = 0;

bool impulseApplied = false;
unsigned long commandTimer = 0;
int commandIndex = 0;

double pitchCalibration = 0.0;
double yawCalibration = 0.0;


// Function prototypes
void moveForward(double speed);
void moveBackward(double speed);
void turnLeft(double speed);
void turnRight(double speed);
void stop();
//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool timerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

void setup()
{
  Serial.begin(115200); // 115200 (kbps or bps?) transmission speed
  pinMode(TOGGLE_PIN,OUTPUT);
  mpuHandler.init();


  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, timerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
  step1.setAccelerationRad(0);
  step2.setAccelerationRad(0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN,OUTPUT);
  digitalWrite(STEPPER_EN, false);


  double yawSum = 0;
  double pitchSum = 0;

  for (int i = 0; i < 500; ++i) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    yawSum += g.gyro.x;
    pitchSum += g.gyro.y;

    delay(5);
  }
  yawCalibration = yawSum / 500.0;
  pitchCalibration = pitchSum / 500.0;

  // Initialize command timer
  commandTimer = millis();

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //double gyroPitchRaw = g.gyro.y;
    //gyroPitchRaw -= pitchCalibration;
    //double angleChange;
    //angleChange += gyroPitchRaw * 0.01;



    // Calculate Tilt using accelerometer (simplified for small angles)
    double accelAngle = a.acceleration.z/9.67;
    //double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  
    // Serial.print(currentTime);
    // Serial.print(",");
    // Serial.println(accelAngle);
    // Get the rate of change from the gyroscope
    double gyroRate = g.gyro.y; //* (PI / 180.0); 

    // Apply complementary filter
    double dt = LOOP_INTERVAL / 1000.0;  // Convert LOOP_INTERVAL to seconds
    filteredAngle = (1 - alpha) * accelAngle + alpha * (previousFilteredAngle + gyroRate * dt);
    previousFilteredAngle = filteredAngle;

    pidOutput = balancePid.compute(filteredAngle);

    double currentPosition = (step1.getPositionRad() + step2.getPositionRad()) / 2.0;
    unsigned long currentMillis = millis();

    // Calculate overall speed (rad/s) based on position change
    double positionChange = currentPosition - previousPosition;
    double timeChange = (currentMillis - previousTime) / 1000.0; // Convert to seconds
    double overallSpeed = positionChange / timeChange;

    // Update previous position and time
    previousPosition = currentPosition;
    previousTime = currentMillis;
    
    
    // Outer loop: Speed control
    speedPid.setSetpoint(speedSetpoint);
    speedControlOutput = speedPid.compute(overallSpeed);

    desiredTiltAngle = setpoint + (speedControlOutput * 0.01);

    // Inner loop: Balance control with speed control output as setpoint
    balancePid.setSetpoint(desiredTiltAngle);
    balanceControlOutput = balancePid.compute(filteredAngle);

    step1.setAccelerationRad(-balanceControlOutput);
    step2.setAccelerationRad(balanceControlOutput);


    if (balanceControlOutput > 0){
      step1.setTargetSpeedRad(-15);
      step2.setTargetSpeedRad(15);
    } else {
      step1.setTargetSpeedRad(15);
      step2.setTargetSpeedRad(-15);
    }

    // self-generated pulse

    // if (!impulseApplied && millis() > 10000) { //apply impulse after 10000 milliseconds
    //   Serial.println("Applying impulse!");
    //   step1.setTargetSpeedRad(100);
    //   step2.setTargetSpeedRad(-100);
    //   delay(250);  // Duration of the impulse
    //   step1.setTargetSpeedRad(0);
    //   step2.setTargetSpeedRad(0);
    //   impulseApplied = true;
    // }

  }

  
  // Print updates every PRINT_INTERVAL ms
  /*if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    mpuHandler.readData(mpu6050_data);
    Serial.print("Acceleration X: ");
    Serial.print(mpu6050_data.Acc_X);
    Serial.print(", Y: ");
    Serial.print(mpu6050_data.Acc_Y);
    Serial.print(", Z: ");
    Serial.println(mpu6050_data.Acc_Z);
    Serial.print("Gyro X: ");
    Serial.print(mpu6050_data.Angle_Velocity_R);
    Serial.print(", Y: ");
    Serial.print(mpu6050_data.Angle_Velocity_P);
    Serial.print(", Z: ");
    Serial.println(mpu6050_data.Angle_Velocity_Y);
    Serial.print("PID Output: ");
    Serial.println(pidOutput);
  }*/
  

  // Execute movement commands every COMMAND_INTERVAL ms
  // if (millis() - commandTimer > COMMAND_INTERVAL) {
  //   commandTimer = millis();

  //   switch (commandIndex) {
  //     case 0:
  //       Serial.println("Moving forward");
  //       moveForward(0.1);  
  //       break;
  //     case 1:
  //       Serial.println("Moving backward");
  //       moveBackward(0.1);  
  //       break;
  //     // case 2:
  //     //   Serial.println("Turning left");
  //     //   turnLeft(5.0);  
  //     //   break;
  //     // case 3:
  //     //   Serial.println("Turning right");
  //     //   turnRight(5.0); 
  //     //   break;
  //     default:
  //       stop(); 
  //       commandIndex = -1;  // Reset index
  //       break;
  //   }
  //   commandIndex++;
  // }

  /*if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print(" Filtered Angle: ");
    Serial.println(filteredAngle);
    Serial.print(" Filtered Speed: ");
    Serial.println(filteredSpeed);
    Serial.print(" Target Angle: ");
    Serial.println(setpoint);
    // Serial.print(" Target Speed: ");
    // Serial.println(speedSetpoint);
    Serial.print(" Target angle: ");
    Serial.println(desiredTiltAngle);

    Serial.print(" Speed Output: ");
    Serial.println(speedControlOutput);

    Serial.print(" Output: ");
    Serial.println(balanceControlOutput);
  }*/
    
}

// Functions to move forward, backward, and turn
void stop() {
  speedPid.setSetpoint(0);
}

void moveForward(double speed) {
  speedPid.setSetpoint(speed);
}

void moveBackward(double speed) {
  speedPid.setSetpoint(-speed);
}

void turnLeft(double speed) {
  step1.setTargetSpeedRad(-speed);
  step2.setTargetSpeedRad(speed);
}

void turnRight(double speed) {
  step1.setTargetSpeedRad(speed);
  step2.setTargetSpeedRad(-speed);
}



