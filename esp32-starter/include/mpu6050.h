// MPU6050Handler.h
#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct MPU6050_DATA {
    double Temperature = 0.0;

    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;

    double Acc_X = 0.0;
    double Acc_Y = 0.0;
    double Acc_Z = 0.0;

    double Angle_Velocity_R = 0.0;
    double Angle_Velocity_P = 0.0;
    double Angle_Velocity_Y = 0.0;
};

class MPU6050Handler {
public:
    MPU6050Handler();
    void init();
    void readData(MPU6050_DATA &data);

private:
    Adafruit_MPU6050 mpu;
};

#endif // MPU6050HANDLER_H
