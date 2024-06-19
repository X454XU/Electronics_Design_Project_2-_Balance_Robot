#include <Arduino.h>
#include <WiFi.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <mpu6050.h>
#include <PIDController.h>
#include <chrono> // For time functions
#include <WiFi.h>
#include <HTTPClient.h>


// The Stepper pins
#define STEPPER1_DIR_PIN 16   //Arduino D9
#define STEPPER1_STEP_PIN 17  //Arduino D8
#define STEPPER2_DIR_PIN 4    //Arduino D11
#define STEPPER2_STEP_PIN 14  //Arduino D10
#define STEPPER_EN 15         //Arduino D12

// Diagnostic pin for oscilloscope
#define TOGGLE_PIN  32        //Arduino A4

// Task handles
TaskHandle_t Balance;
TaskHandle_t Communication;

// WiFi credentials
const char* ssid = "EEE";
const char* password = "Opklopkl123123@";

// Flask server IP address and port
const char* serverIP = "192.68.0.100";
const uint16_t serverPort = 5000;

const int PRINT_INTERVAL = 500;
const int LOOP_INTERVAL = 10;
const int STEPPER_INTERVAL_US = 20;

const int COMMAND_INTERVAL = 5000; // 5 seconds, for testing

// PID tuning parameters
double kp = 900; // 1000
double ki = 17; //15
double kd = 80; //95
double setpoint = 0; 

// PID tuning parameters for speed control
double speedKp = 1; //3
double speedKi = 0.38; //0.35
double speedKd = 0.23; //0.2
double speedSetpoint = 0; // Desired speed

// PID tuning parameters for Yaw control
double yawKp = 2.7; // 2.5 
double yawKi = 0.1; //0.1
double yawKd = 1.9; // 1.5
double yawSetpoint = 0.0; 

int countr;

double pidOutput;
double speedPidOutput;
double speedControlOutput;
double balanceControlOutput;
double TargetTiltAngle;


// WiFiServer server(80);  // Create a server on port 80

//WebSocketsClient webSocket;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;  //Default pins for I2C are SCL: IO22/Arduino D3, SDA: IO21/Arduino D4

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

MPU6050Handler mpuHandler;
MPU6050_DATA mpu6050_data;

PID balancePid(kp, ki, kd, setpoint);
PID speedPid(speedKp, speedKi, speedKd, speedSetpoint);
PID yawPID(yawKp, yawKi, yawKd, yawSetpoint);

//function prototypes
void BalanceCode(void * parameter);
void CommunicationCode(void * parameter);

// Complementary filter constant
const double alpha = 0.98; 

double filteredAngle = 0.0;
double previousFilteredAngle = 0.0;

float filteredAngleYaw = 0.0;
float previousFilteredAngleYaw = 0.0;

float gyroBiasX = 0;
unsigned long lastTime = 0; 

const int calibrationSamples = 500;


unsigned long commandTimer = 0;
int commandIndex = 0;

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
const double deadBand = 7; // Dead-band threshold for ignoring small angle changes


// Interrupt Service Routine for motor update
// Note: ESP32 doesn't support floating point calculations in an ISR
bool timerHandler(void * timerNo)
{
  static bool toggle = false;

  // Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  // Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}


//////////////////////////////////////////////////////////////////////
//////////////////////// Useful Functions ////////////////////////////
//////////////////////////////////////////////////////////////////////
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  float sumGyroX = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumGyroX += g.gyro.x;

    delay(10); // Adjust delay as needed
  }

  gyroBiasX = sumGyroX / calibrationSamples;

  Serial.println("Sensor calibration complete.");
  Serial.print("Gyro Bias X: "); Serial.println(gyroBiasX);
}

float normalizeAngle(float angle) {
    // Adjust the angle to be within the range [-π, π)
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}


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

void turnLeft(double heading) {
  yawSetpoint += heading;
  yawSetpoint = normalizeAngle(yawSetpoint);
  yawPID.setSetpoint(yawSetpoint);
}

void turnRight(double heading) {
  yawSetpoint -= heading;
  yawSetpoint = normalizeAngle(yawSetpoint);
  yawPID.setSetpoint(yawSetpoint);
}


//////////////////////////////////////////////////////////////////////
//////////////////////// Communication ///////////////////////////////
//////////////////////////////////////////////////////////////////////

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void sendSensorData(float speed, float angle) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String("http://") + serverIP + ":" + serverPort + "/post_speed");
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"speed\":" + String(speed) + ", \"angle\":" + String(angle) + "}";
    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.println("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
}

String receiveCommand() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String("http://") + serverIP + ":" + serverPort + "/command");
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
      return response;
    } else {
      Serial.println("Error on sending GET: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
  return "";
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


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Main Functionality //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200); // 115200 (kbps or bps?) transmission speed
  connectToWiFi();


  Serial.println("Starting setup...");
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

  // Calibrate sensors
  calibrateSensors();

  // Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, timerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  Serial.println("Initialised Interrupt for Stepper");

  // Set motor acceleration values
  step1.setAccelerationRad(0);
  step2.setAccelerationRad(0);

  // Enable the stepper motor drivers
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, false);

  // Connect to WiFi
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("Connected!");

  // // Start the server
  // server.begin();
  // Serial.println("Server started");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());

  emaSpeed1 = step1.getSpeed() / 2000.0;
  emaSpeed2 = step2.getSpeed() / 2000.0;

  calculateButterworthCoefficients();

  yawPID.isYawFn(true);

  commandTimer = millis();

  

  countr = 0;

  xTaskCreate(
    BalanceCode,    // Function that should be called
    "Balance",     // Name of the task
    10000,        // Stack size (bytes)
    NULL,         // Parameter to pass
    1,            // Task priority
    &Balance      // Task handle
  );

  xTaskCreate(
    CommunicationCode, 
    "Movement",   
    10000,        
    NULL,         
    1,            
    &Communication     
  );

}


void CommunicationCode(void * parameter)
{
  for (;;){
    sendSensorData(speedCmPerSecond, pitch);
    String command = receiveCommand(); // temporary
    if (command == "forward") {
      moveForward(10);
    } else if (command == "backward") {
      moveBackward(10);
    }
        
  }
}
void BalanceCode(void * parameter)
{
  for (;;){
    //Static variables are initialised once and then the value is remembered between subsequent calls to this function
    static unsigned long printTimer = 0;  //time of the next print
    static unsigned long loopTimer = 0;   //time of the next control update

    // Run the control loop every LOOP_INTERVAL ms
    if (millis() > loopTimer) {
      loopTimer += LOOP_INTERVAL;

      // Fetch data from MPU6050
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float gyroX = g.gyro.x - gyroBiasX;

      //////////////// YAW ///////////////////////
      double dt = LOOP_INTERVAL / 1000.0; // Convert LOOP_INTERVAL to seconds
      
      filteredAngleYaw = (1 - alpha) * filteredAngleYaw + alpha * (previousFilteredAngleYaw + gyroX * dt);
      
      filteredAngleYaw = normalizeAngle(filteredAngleYaw);
      
      previousFilteredAngleYaw = filteredAngleYaw;
      
      double yawCorrection = yawPID.compute(filteredAngleYaw);


      //////////////// PITCH ///////////////////////
      pitch = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y)) - 0.08837433;
      // pitch = pitchIncrementer.next_value();
      // Gyro rates (rate of change of tilt) in radians
      float gyroPitchRate = g.gyro.y; // Pitch rate

      // Apply complementary filter
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

        step1.setAccelerationRad(-balanceControlOutput - yawCorrection);
        step2.setAccelerationRad(balanceControlOutput - yawCorrection);

        if (balanceControlOutput > 0) {
          step1.setTargetSpeedRad(-20);
          step2.setTargetSpeedRad(20);
        } 
        if (balanceControlOutput < 0) {
          step1.setTargetSpeedRad(20);
          step2.setTargetSpeedRad(-20);
        }
        if(countr > 1000){
          resetSteppers();
          countr = 0;
        }
        countr += 1;
        

        // TEST
        //  if (millis() > 10000 && millis() < 20000){
        //   Serial.print("Left");
        //   turnLeft(0.003);
        //  }
        //  else if(millis() > 20000 && millis() < 30000){
        //   //  Serial.print("Turning right");
        //   Serial.print("Right");
        //   turnRight(0.003);
        //  }
        //  else if (millis() > 30000 && millis() < 30010){
        //   Serial.print("Forward");
        //   moveForward(10);
        //  }
        //  else if (millis() > 40000 && millis() < 40010){
        //   Serial.print("Backward");
        //   moveBackward(10);
        //  }

      } 
    }
    

    // Print updates every PRINT_INTERVAL ms
    if (millis() > printTimer) {
      printTimer += PRINT_INTERVAL;
      Serial.print(" Yaw : ");
      Serial.println(filteredAngleYaw, 6);
    }
  }
}

void loop(){

}

