// MPU6050Handler.cpp
#include "mpu6050.h"

MPU6050Handler::MPU6050Handler() {}

void MPU6050Handler::init() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void MPU6050Handler::readData(MPU6050_DATA &data) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    data.Temperature = temp.temperature;

    data.Acc_X = a.acceleration.x;
    data.Acc_Y = a.acceleration.y;
    data.Acc_Z = a.acceleration.z;

    data.Angle_Velocity_R = g.gyro.x;
    data.Angle_Velocity_P = g.gyro.y;
    data.Angle_Velocity_Y = g.gyro.z;
}
