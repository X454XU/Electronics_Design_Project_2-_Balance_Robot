#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <mpu6050.h>
#include <PIDController.h>
#include <chrono> // For time functions
#include <WebSocketsServer.h>


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
double kp = 1000; // 640
double ki = 15; //15
double kd = 95; //95
double setpoint = 0; 

// PID tuning parameters for speed control
double speedKp = 1;
double speedKi = 0.1;
double speedKd = 0.2;//1000;
double speedSetpoint = 0; // Desired speed

double pidOutput;
double speedPidOutput;
double speedControlOutput;
double balanceControlOutput;
double TargetTiltAngle;

//Global objects
WebSocketsServer webSocket = WebSocketsServer(81);

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

double filteredAngle = 0.0;
double previousFilteredAngle = 0.0;

// bool impulseApplied = false;
unsigned long commandTimer = 0;
int commandIndex = 0;

// double pitchCalibration = 0.0;
// double yawCalibration = 0.0;

float pitch = 0.0;

bool motorsEnabled = true; // Variable to track motor state

const float alphaEMA = 0.1; // Smoothing factor for EMA

float emaSpeed1 = 0;
float emaSpeed2 = 0;

const float wheelDiameter = 6.6;  // Diameter of the wheel in cm
const float wheelCircumference = PI * wheelDiameter;
const int stepsPerRevolution = 200 * 16; // 200 steps per revolution and 16 microsteps
const float trackWidth = 11.9;  // Distance between the centers of the two wheels in cm

float speedCmPerSecond;
float rotationalSpeedRadPerSecond;

float speedCmPerSecond1;
float speedCmPerSecond2;

double previousSpeedControlOutput = 0;
double prevAccel = 0;

// Maybe needs further tuning
const double deadBand = 0; // Dead-band threshold for ignoring small angle changes


// double currentPosition = 0.0;
// double speed = 0.0; 

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

//////////////////////////////////////////////////////////////////////
//////////////////////// Movement ////////////////////////////////////
//////////////////////////////////////////////////////////////////////

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
  step2.setTargetSpeedRad(-speed);
}

void turnRight(double speed) {
  step1.setTargetSpeedRad(speed);
  step2.setTargetSpeedRad(speed);
}

//////////////////////////////////////////////////////////////////////
//////////////////////// Communication ///////////////////////////////
//////////////////////////////////////////////////////////////////////
void handleCommand(char* cmd) {
    if (strcmp(cmd, "MOVE_FORWARD") == 0) {
        moveForward(15);
    } else if (strcmp(cmd, "MOVE_BACKWARD") == 0) {
        moveBackward(15);
    } else if (strcmp(cmd, "TURN_LEFT") == 0) {
        turnLeft(5);
    } else if (strcmp(cmd, "TURN_RIGHT") == 0) {
        turnRight(5);
    }
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] Text: %s\n", num, payload);
            // Handle received text (control commands)
            // Example: move robot based on command
            handleCommand((char *)payload);
            break;
    }
}


///////////////////////////////////////////////////////////////////////
////////////////////////////// Filtering //////////////////////////////
///////////////////////////////////////////////////////////////////////

// Butterworth filter variables
const int order = 2;
const float cutoffFrequency = 5.0; // Hz, cutoff frequency
const float samplingFrequency = 100.0; // Hz
float a[order + 1];
float b[order + 1];
float x1Butter[order + 1] = {0};
float y1Butter[order + 1] = {0};
float x2Butter[order + 1] = {0};
float y2Butter[order + 1] = {0};

void calculateButterworthCoefficients() {
  double pi = 3.141592653589793;
  double sqrt2 = 1.4142135623730951;

  double normalizedCutoff = cutoffFrequency / (samplingFrequency / 2.0);
  double theta = 2.0 * pi * normalizedCutoff;
  double d = 1.0 / cos(theta);
  double beta = 0.5 * ((1 - (d / sqrt2)) / (1 + (d / sqrt2)));
  double gamma = (0.5 + beta) * cos(theta);
  double alpha = (0.5 + beta - gamma) / 2.0;

  b[0] = alpha;
  b[1] = 2.0 * alpha;
  b[2] = alpha;
  a[0] = 1.0;
  a[1] = -2.0 * gamma;
  a[2] = 2.0 * beta;
}

float butterworthFilter(float *x, float *y, float input) {
  // Shift previous samples
  for (int i = order; i > 0; i--) {
    x[i] = x[i - 1];
    y[i] = y[i - 1];
  }
  x[0] = input;
  
  // Compute new output
  y[0] = 0;
  for (int i = 0; i <= order; i++) {
    y[0] += b[i] * x[i];
    if (i > 0) {
      y[0] -= a[i] * y[i];
    }
  }
  return y[0];
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Testing /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

class Incrementer {
public:
    Incrementer(float start=-0.01, float end=0.01, float step=0.0002, int incrementInterval=50)
        : start(start), current(start), end(end), step(step), incrementInterval(incrementInterval), counter(0), direction(1) {}

    float next_value() {
        float value = current;
        counter++;
        if (counter >= incrementInterval) {
            counter = 0;
            current += step * direction;
            if (current > end || current < start) {
                direction *= -1; // Reverse direction
                current += step * direction; // Correct overshoot
            }
        }
        return value;
    }

private:
    float start;
    float current;
    float end;
    float step;
    int incrementInterval;
    int counter;
    int direction;
};

Incrementer pitchIncrementer;



//////////////////////////////////////////////////////////////////////
//////////////////////// Useful Functions ////////////////////////////
//////////////////////////////////////////////////////////////////////

void resetSteppers() {
  step1.setTargetSpeedRad(0);
  step2.setTargetSpeedRad(0);
  step1.setAccelerationRad(0);
  step2.setAccelerationRad(0);
  delay(10);  // Allow some time for the steppers to stop
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

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Main Functionality /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

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

  // double yawSum = 0;
  // double pitchSum = 0;

  // for (int i = 0; i < 500; ++i) {
  //   sensors_event_t a, g, temp;
  //   mpu.getEvent(&a, &g, &temp);

  //   pitchSum += atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));

  //   delay(10);
  // }
  // pitchCalibration = pitchSum / 500.0;

  // Serial.print(pitchCalibration);

  emaSpeed1 = step1.getSpeed() / 2000.0;
  emaSpeed2 = step2.getSpeed() / 2000.0;

  calculateButterworthCoefficients();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Initialize command timer
  commandTimer = millis();
}


void loop()
{
  //Static variables are initialised once and then the value is remembered between subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update

  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    pitch = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y)) - 0.08837433;
    //pitch = pitchIncrementer.next_value();
    // Gyro rates (rate of change of tilt) in radians
    float gyroPitchRate = g.gyro.y; // Pitch rate

    // Apply complementary filter
    double dt = LOOP_INTERVAL / 1000.0;  // Convert LOOP_INTERVAL to seconds
    filteredAngle = (1 - alpha) * pitch + alpha * (previousFilteredAngle + gyroPitchRate * dt);
    previousFilteredAngle = filteredAngle;

    // Check and toggle motors based on tilt angle
    checkAndToggleMotors();

    if (motorsEnabled) {
      // Get raw speed readings
      float rawSpeed1 = step1.getSpeed() / 2000.0;
      float rawSpeed2 = step2.getSpeed() / 2000.0;

      // Apply exponential moving average
      emaSpeed1 = alphaEMA * rawSpeed1 + (1 - alphaEMA) * emaSpeed1;
      emaSpeed2 = alphaEMA * rawSpeed2 + (1 - alphaEMA) * emaSpeed2;

      // Apply Butterworth low-pass filter
      float butterSpeed1 = butterworthFilter(x1Butter, y1Butter, emaSpeed1);
      float butterSpeed2 = butterworthFilter(x2Butter, y2Butter, emaSpeed2);

      // Average the speeds of the two motors 
      float averageSpeedSteps = (butterSpeed1 - butterSpeed2) / 2.0;

      // Convert speed from steps per second to cm per second
      float distancePerStep = wheelCircumference / stepsPerRevolution;
      speedCmPerSecond = averageSpeedSteps * distancePerStep;

      speedCmPerSecond1 = butterSpeed1 * distancePerStep;
      speedCmPerSecond2 = butterSpeed2 * distancePerStep;

      // Calculate the rotational speed in radians per second
      rotationalSpeedRadPerSecond = (speedCmPerSecond1 + speedCmPerSecond2) / trackWidth;

      // Outer loop: Speed control
      speedControlOutput = speedPid.compute(speedCmPerSecond);

      TargetTiltAngle = speedControlOutput * 0.001;

      // Inner loop: Balance control with speed control output as setpoint
      balancePid.setSetpoint(TargetTiltAngle);
      balanceControlOutput = balancePid.compute(filteredAngle);

      // Apply dead-band
      if (abs(balanceControlOutput) < deadBand) {
        balanceControlOutput = 0;
      }

      step1.setAccelerationRad(-balanceControlOutput);
      step2.setAccelerationRad(balanceControlOutput);

      if (balanceControlOutput > 0) {
        step1.setTargetSpeedRad(-20);
        step2.setTargetSpeedRad(20);
      } 
      if(balanceControlOutput < 0) {
        step1.setTargetSpeedRad(20);
        step2.setTargetSpeedRad(-20);
      }
      /*if(setpoint == 0){
        if(balanceControlOutput < 0){setpoint += 0.0015;
        Serial.print("hi");}
        if(balanceControlOutput > 0){setpoint -= 0.0015;}
      }*/

    } 
  }

  // Print updates every PRINT_INTERVAL ms
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    // Serial.print("Output: ");
    // Serial.println(balanceControlOutput, 6);
  }

  webSocket.loop();
}

