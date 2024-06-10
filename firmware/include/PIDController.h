#ifndef PIDController_H
#define PIDController_H

class PID {
public:
    PID(double kp, double ki, double kd, double setpoint);
    void setTunings(double kp, double ki, double kd);
    void setSetpoint(double setpoint);
    void setSampleTime(double sampleTime);
    void setOutputLimits(double min, double max);
    double compute(double input);

private:
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
    double setpoint; // Desired value

    double outputMin; // Minimum output limit
    double outputMax; // Maximum output limit

    double prevError; // Last error value
    double integral; // Integral term
    double lastTime; // Last time compute() was called

    double sampleTime; // Time interval between calculations

    double lastOutput; // Last output
};

#endif // PID_H
