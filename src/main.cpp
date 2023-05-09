#include <Arduino.h>
#include <MPU_6050.hpp>
#include <calibrate_MPU.hpp>
int16_t gyro_roll, gyro_pitch, gyro_yaw;
unsigned count;
void setup()
{
    Wire.begin();
    calibrate_gyro();
}
void loop()
{
}