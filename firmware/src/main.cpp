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

// The uart pins
#define RX2 16
#define TX2 17

// Diagnostic pin for oscilloscope
#define TOGGLE_PIN  32        //Arduino A4

const int PRINT_INTERVAL = 500;
const int LOOP_INTERVAL = 10;
const int  STEPPER_INTERVAL_US = 20;

const float kx = 20.0;

//tune the parameters here

double kp = 50;
double ki = 2;
double kd = 20;
double setpoint = 100;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22/Arduino D3, SDA: IO21/Arduino D4

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

MPU6050Handler mpuHandler;
MPU6050_DATA mpu6050_data;

PID pid(kp, ki, kd, setpoint);

//uart2 port object
HardwareSerial Serial2(2);

//change these values to tune the PID controller
double pidOutput;
//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
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

char listenToUart2() {
  // Check if data is available on Serial2
  if (Serial2.available()) {
    // Read and return the first character
    return Serial2.read();
  }
  // Return a null character if no data is available
  return '\0';
}
void sendFloatToUart2(float value) {
  // Convert the float to a string and send it via Serial2
  Serial2.print(value);
  Serial2.print("\n");  // Send a newline character to indicate the end of the float value
}

void setup()
{ 
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2); 

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
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
  step1.setAccelerationRad(10.0);
  step2.setAccelerationRad(10.0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN,OUTPUT);
  digitalWrite(STEPPER_EN, false);

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate Tilt using accelerometer and sin x = x approximation for a small tilt angle
    tiltx = a.acceleration.z/9.67;

    pidOutput = pid.compute(tiltx);

    //Set target motor speed proportional to tilt angle
    //Note: this is for demonstrating accelerometer and motors - it won't work as a balance controller
    step1.setTargetSpeedRad(pidOutput*kx);
    step2.setTargetSpeedRad(-pidOutput*kx);
  }
  
  //Print updates every PRINT_INTERVAL ms
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print(tiltx*1000);
    Serial.print(tiltx*180/PI);
    //print tilt angle in degrees
    Serial.print(' ');
    Serial.print(step1.getSpeedRad());
    Serial.println();
    Serial.print(step2.getSpeedRad());
    //print step2 speed
    Serial.println(setpoint);
    //print setpoint

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

    Serial.print("PIDOUTPUT: ");
    Serial.println(pidOutput);


    delay(100);
}
