#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "MPU_6050.hpp"

#define NUMBER_OF_SAMPLES 1000

void calibrate_accel(void)
{
    MPU.setup();
    Serial.begin(9600);
    
    accel_output accelData;
    int32_t tempX = 0, tempY = 0, tempZ = 0;

    Serial.println("Taking Samples ... Don't move the accelerometer");
    for (unsigned count = 0u; count < NUMBER_OF_SAMPLES; ++count)
    {
        MPU.read_accel_data(accelData);

        tempX += accelData.x;
        tempY += accelData.y;
        tempZ += accelData.z;
    }
    Serial.println("Calibration Complete.");

    // take the mean
    accelData.x = tempX / NUMBER_OF_SAMPLES;
    accelData.y = tempY / NUMBER_OF_SAMPLES;
    accelData.z = tempZ / NUMBER_OF_SAMPLES;

    /// Write into EEPROM
    EEPROM.write(0, accelData.x >> 8);
    EEPROM.write(1, accelData.x bitand 0x00ff);
    EEPROM.write(2, accelData.y >> 8);
    EEPROM.write(3, accelData.y bitand 0x00ff);
    EEPROM.write(4, accelData.z >> 8);
    EEPROM.write(5, accelData.z bitand 0x00ff);

    // print the calibration results
    Serial.print(accelData.x);
    Serial.print(' ');
    Serial.print(accelData.y);
    Serial.print(' ');
    Serial.println(accelData.z);
    Serial.println("**************************************");

    Serial.end();
}
void calibrate_gyro(void)
{
    MPU.setup();
    Serial.begin(9600);

    gyro_output gyroData;
    int32_t tempX = 0, tempY = 0, tempZ = 0;

    Serial.println("Taking Samples ... Don't move the gyroscope");
    for (unsigned count = 0u; count < NUMBER_OF_SAMPLES; ++count)
    {
        MPU.read_gyro_data(gyroData);

        tempX += gyroData.roll;
        tempY += gyroData.pitch;
        tempZ += gyroData.yaw;
    }
    Serial.println("Calibration Complete.");

    // Take the mean
    gyroData.roll = tempX / NUMBER_OF_SAMPLES;
    gyroData.pitch = tempY / NUMBER_OF_SAMPLES;
    gyroData.yaw = tempZ / NUMBER_OF_SAMPLES;

    // Write into EEPROM
    EEPROM.write(6, gyroData.roll >> 8);
    EEPROM.write(7, gyroData.roll bitand 0x00ff);
    EEPROM.write(8, gyroData.pitch >> 8);
    EEPROM.write(9, gyroData.pitch bitand 0x00ff);
    EEPROM.write(10, gyroData.yaw >> 8);
    EEPROM.write(11, gyroData.yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(gyroData.roll);
    Serial.print(' ');
    Serial.print(gyroData.pitch);
    Serial.print(' ');
    Serial.println(gyroData.yaw);
    Serial.println("**************************************");

    Serial.end();
}
