#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "MPU_6050.hpp"

#define NUMBER_OF_SAMPLES 1000

void calibrate_accel(void)
{
    MPU.setup();
    Serial.begin(9600);
    int16_t accel_roll, accel_pitch, accel_yaw;
    int32_t tempX = 0, tempY = 0, tempZ = 0;

    Serial.println("Taking Samples ... Don't move the accelerometer");
    for (unsigned count = 0u; count < NUMBER_OF_SAMPLES; ++count)
    {
        MPU.read_accel_data(accel_roll, accel_pitch, accel_yaw);

        tempX += accel_roll;
        tempY += accel_pitch;
        tempZ += accel_yaw;
    }
    Serial.println("Calibration Complete.");

    // take the mean
    accel_roll = tempX / NUMBER_OF_SAMPLES;
    accel_pitch = tempY / NUMBER_OF_SAMPLES;
    accel_yaw = tempZ / NUMBER_OF_SAMPLES;

    /// Write into EEPROM
    EEPROM.write(0, accel_roll >> 8);
    EEPROM.write(1, accel_roll bitand 0x00ff);
    EEPROM.write(2, accel_pitch >> 8);
    EEPROM.write(3, accel_pitch bitand 0x00ff);
    EEPROM.write(4, accel_yaw >> 8);
    EEPROM.write(5, accel_yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(accel_roll);
    Serial.print(' ');
    Serial.print(accel_pitch);
    Serial.print(' ');
    Serial.print(accel_yaw);
    Serial.println("**************************************");

    Serial.end();
}
void calibrate_gyro(void)
{
    MPU.setup();
    Serial.begin(9600);

    int16_t gyro_roll, gyro_pitch, gyro_yaw;
    int32_t tempX = 0, tempY = 0, tempZ = 0;

    Serial.println("Taking Samples ... Don't move the gyroscope");
    for (unsigned count = 0u; count < NUMBER_OF_SAMPLES; ++count)
    {
        MPU.read_gyro_data(gyro_roll, gyro_pitch, gyro_yaw);

        tempX += gyro_roll;
        tempY += gyro_pitch;
        tempZ += gyro_yaw;
    }
    Serial.println("Calibration Complete.");

    // Take the mean
    gyro_roll = tempX / NUMBER_OF_SAMPLES;
    gyro_pitch = tempY / NUMBER_OF_SAMPLES;
    gyro_yaw = tempZ / NUMBER_OF_SAMPLES;

    // Write into EEPROM
    EEPROM.write(6, gyro_roll >> 8);
    EEPROM.write(7, gyro_roll bitand 0x00ff);
    EEPROM.write(8, gyro_pitch >> 8);
    EEPROM.write(9, gyro_pitch bitand 0x00ff);
    EEPROM.write(10, gyro_yaw >> 8);
    EEPROM.write(11, gyro_yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(gyro_roll);
    Serial.print(' ');
    Serial.print(gyro_pitch);
    Serial.print(' ');
    Serial.print(gyro_yaw);
    Serial.println("**************************************");

    Serial.end();
}
