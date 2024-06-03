#include "mpu6050.h"

void setup() {
  Serial.begin(115200);
  Init_mpu6050();
}

void loop() {

  ReadMPU6050();
  //
  //  Serial.print("Acc_x:");
  //  Serial.print(mpu6050_data.Acc_X);
  //  Serial.print(",");
  //  Serial.print("Acc_Y:");
  //  Serial.print(mpu6050_data.Acc_Y);
  //  Serial.print(",");
  //  Serial.print("Acc_Z:");
  //  Serial.println(mpu6050_data.Acc_Z);

  // 
  Serial.print("Angle_velocity_X:");
  Serial.println(mpu6050_data.Angle_Velocity_Rotation);
  Serial.print(",");
  Serial.print("Angle_velocity_Y:");
  Serial.print(mpu6050_data.Angle_Velocity_Pitch);
  Serial.print(",");
  Serial.print("Angle_velocity_Z:");
  Serial.println(mpu6050_data.Angle_Velocity_Yaw);

  delay(50);

}
