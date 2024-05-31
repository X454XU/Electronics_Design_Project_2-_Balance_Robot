#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <mpu6050.h>
#include <PIDController.h>

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

//PID tuning parameters
double kp = 1500;
double ki = 0;
double kd = 0;
double setpoint = 0.08; // Adjust

double pidOutput;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;  //Default pins for I2C are SCL: IO22/Arduino D3, SDA: IO21/Arduino D4

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

MPU6050Handler mpuHandler;
MPU6050_DATA mpu6050_data;

PID pid(kp, ki, kd, setpoint);

// Complementary filter constant
const double alpha = 0.99; 
double filteredAngle = 0.0;
double previousFilteredAngle = 0.0;

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
  Serial.begin(115200);
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
  step1.setAccelerationRad(13.0);
  step2.setAccelerationRad(13.0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN,OUTPUT);
  digitalWrite(STEPPER_EN, false);

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float accelAngle = 0.0;        // Current tilt angle from accelerometer
  static float gyroRate = 0.0;          // Current rate of change from gyroscope
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate Tilt using accelerometer (simplified for small angles)
    accelAngle = a.acceleration.z/9.67;

    // Get the rate of change from the gyroscope
    gyroRate = g.gyro.y; //* (PI / 180.0); 

    // Apply complementary filter
    double dt = LOOP_INTERVAL / 1000.0;  // Convert LOOP_INTERVAL to seconds
    filteredAngle = (1 - alpha) * accelAngle + alpha * (previousFilteredAngle + gyroRate * dt);
    previousFilteredAngle = filteredAngle;

    pidOutput = pid.compute(filteredAngle);

    //Set target motor speed proportional to tilt angle
    //Note: this is for demonstrating accelerometer and motors - it won't work as a balance controller
    step1.setAccelerationRad(-pidOutput);
    step2.setAccelerationRad(pidOutput);


    if (pidOutput > 0){
      step1.setTargetSpeedRad(-10);
      step2.setTargetSpeedRad(10);
    }
    else{
      step1.setTargetSpeedRad(10);
      step2.setTargetSpeedRad(-10);
    }

  }
  
  //Print updates every PRINT_INTERVAL ms
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("Tilt (mrad): ");
    Serial.print(filteredAngle*1000);
    Serial.print(", Tilt (deg): ");
    Serial.print(filteredAngle*180/PI);
    Serial.print(", Step1 Speed: ");
    Serial.print(step1.getSpeedRad());
    Serial.print(", Step2 Speed: ");
    Serial.print(step2.getSpeedRad());
    Serial.print(", Setpoint: ");
    Serial.println(setpoint);

  }

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
  

    delay(100);
}




