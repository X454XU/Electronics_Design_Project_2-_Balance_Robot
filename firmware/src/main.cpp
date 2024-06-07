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

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

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
unsigned long previousMicros = 0;

bool impulseApplied = false;
unsigned long commandTimer = 0;
int commandIndex = 0;

double pitchCalibration = 0.0;
double yawCalibration = 0.0;

double overallSpeed = 0.0;
const double alphaSpeed = 0.1; // Smoothing factor for speed
double smoothedSpeed = 0.0;

float pitch = 0.0;
bool motorsEnabled = true; // Variable to track motor state

// Function prototypes
void moveForward(double speed);
void moveBackward(double speed);
void turnLeft(double speed);
void turnRight(double speed);
void stop();
void checkAndToggleMotors();

//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool timerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

void setup()
{
  Serial.begin(115200); // 115200 (kbps or bps?) transmission speed
  pinMode(TOGGLE_PIN, OUTPUT);
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
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, false);

  double yawSum = 0;
  double pitchSum = 0;

  for (int i = 0; i < 500; ++i) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    pitchSum += atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));

    delay(10);
  }
  pitchCalibration = pitchSum / 500.0;

  Serial.print(pitchCalibration);

  previousMicros = micros();

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

    pitch = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y)) - pitchCalibration;

    // Gyro rates (rate of change of tilt) in radians
    float gyroPitchRate = g.gyro.y; // Pitch rate

    // Apply complementary filter
    double dt = LOOP_INTERVAL / 1000.0;  // Convert LOOP_INTERVAL to seconds
    filteredAngle = (1 - alpha) * pitch + alpha * (previousFilteredAngle + gyroPitchRate * dt);
    previousFilteredAngle = filteredAngle;

    // Check and toggle motors based on tilt angle
    checkAndToggleMotors();

    if (motorsEnabled) {
      pidOutput = balancePid.compute(filteredAngle);

      double currentPosition = (step1.getPositionRad() + step2.getPositionRad()) / 2.0;
      unsigned long currentMicros = micros(); // Use micros() for better precision

      // Calculate overall speed (rad/s) based on position change
      double positionChange = currentPosition - previousPosition;
      double timeChange = (currentMicros - previousMicros) / 1000000.0; // Convert to seconds

      // Ensure timeChange is not zero to avoid division by zero
      if (timeChange > 0.001) { // Ensure a minimum time interval of 1ms to avoid large jumps
        double instantSpeed = positionChange / timeChange;
        // Apply exponential moving average to smooth the speed readings
        smoothedSpeed = (alphaSpeed * instantSpeed) + ((1 - alphaSpeed) * smoothedSpeed);
        overallSpeed = smoothedSpeed;
      } else {
        overallSpeed = smoothedSpeed; // Keep the previous smoothed speed if timeChange is too small
      }

      // Update previous position and time
      previousPosition = currentPosition;
      previousMicros = currentMicros;

      // Outer loop: Speed control
      speedPid.setSetpoint(speedSetpoint);
      speedControlOutput = speedPid.compute(overallSpeed);

      desiredTiltAngle = speedControlOutput;

      // Inner loop: Balance control with speed control output as setpoint
      balancePid.setSetpoint(setpoint);
      balanceControlOutput = balancePid.compute(filteredAngle);

      step1.setAccelerationRad(-balanceControlOutput);
      step2.setAccelerationRad(balanceControlOutput);

      if (balanceControlOutput > 0) {
        step1.setTargetSpeedRad(-15);
        step2.setTargetSpeedRad(15);
      } else {
        step1.setTargetSpeedRad(15);
        step2.setTargetSpeedRad(-15);
      }
    } else {
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
    }
  }

  // Print updates every PRINT_INTERVAL ms
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("Filtered Angle: ");
    Serial.println(filteredAngle);
    Serial.print("Speed: ");
    Serial.println(overallSpeed);
    Serial.print("Target Angle inner loop: ");
    Serial.println(setpoint);
    Serial.print("Target angle outer loop: ");
    Serial.println(desiredTiltAngle);
    Serial.print("Speed Output: ");
    Serial.println(speedControlOutput);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Output: ");
    Serial.println(balanceControlOutput);
  }
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

void checkAndToggleMotors() {
  if (abs(filteredAngle) > 0.7) {
    if (motorsEnabled) {
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
      step1.setAccelerationRad(0);
      step2.setAccelerationRad(0);
      digitalWrite(STEPPER_EN, true); // Disable the stepper motor drivers
      motorsEnabled = false;
    }
  } else {
    if (!motorsEnabled) {
      digitalWrite(STEPPER_EN, false); // Enable the stepper motor drivers
      motorsEnabled = true;
    }
  }
}
