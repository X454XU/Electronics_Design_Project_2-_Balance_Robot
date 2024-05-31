#include "PIDController.h"
#include <algorithm> // For std::max and std::min
#include <chrono> // For time functions
#include <iostream>

// Constructor
PID::PID(double kp, double ki, double kd, double setpoint) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->setpoint = setpoint;
    this->outputMin = 0.0;
    this->outputMax = 100.0;
    this->prevInput = 0.0;
    this->integral = 0.0;
    this->sampleTime = 0.01; // Default sample time in seconds
    this->lastTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    this->lastOutput = 0.0;
}

// Set tuning parameters
void PID::setTunings(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Set the desired setpoint
void PID::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

// Set the time interval between calculations
void PID::setSampleTime(double sampleTime) {
    this->sampleTime = sampleTime;
}

// Set output limits
void PID::setOutputLimits(double min, double max) {
    if (min >= max) return;
    this->outputMin = min;
    this->outputMax = max;
}


// Compute the PID output based on the input
double PID::compute(double input) {
    double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double timeChange = currentTime - lastTime;

    if (timeChange >= sampleTime) {
        // Calculate error
        double error = setpoint - input;
        
        // Integral term calculation
        integral += (ki * error * timeChange);
        //integral = std::max(outputMin, std::min(integral, outputMax)); // Clamp integral to output limits

        // Derivative term calculation
        double derivative = (input - prevInput) / timeChange;

        // Calculate output
        double output = (kp * error) + integral + (kd * derivative);
        // should it be plus between the second and the third term?
        //output = std::max(outputMin, std::min(output, outputMax)); // Clamp output to output limits
        

        // Remember current input and time for next calculation
        prevInput = input;
        lastTime = currentTime;
        lastOutput = output;

        return output;
        
    }

    // If not enough time has passed, return the last output (or 0 if first run)
    return (lastOutput == 0.0) ? 0.0 : lastOutput;
}
