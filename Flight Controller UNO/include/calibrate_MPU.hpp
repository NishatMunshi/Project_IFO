#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "MPU_6050.hpp"

void calibrate_gyro(void)
{
    Serial.begin(9600);
    int16_t gyro_roll, gyro_pitch, gyro_yaw;
    int32_t tempX = 0, tempY = 0, tempZ = 0;
    MPU.reset();
    MPU.setup_gyro(3);

    Serial.println("Taking Samples ... Don't move the gyro");
    for (unsigned count = 0u; count < 10000u; count++)
    {
        MPU.get_gyro_data(gyro_roll, gyro_pitch, gyro_yaw);

        tempX += gyro_roll;
        tempY += gyro_pitch;
        tempZ += gyro_yaw;
    }
    Serial.println("Calibration Complete.");
    gyro_roll = tempX / 10000;
    gyro_pitch = tempY / 10000;
    gyro_yaw = tempZ / 10000;
    EEPROM.write(0, gyro_roll >> 8);
    EEPROM.write(1, gyro_roll bitand 0x00ff);
    EEPROM.write(2, gyro_pitch >> 8);
    EEPROM.write(3, gyro_pitch bitand 0x00ff);
    EEPROM.write(4, gyro_yaw >> 8);
    EEPROM.write(5, gyro_yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(gyro_roll);
    Serial.print(' ');
    Serial.print(gyro_pitch);
    Serial.print(' ');
    Serial.print(gyro_yaw);
    Serial.println();

    Serial.end();
}
void calibrate_accl(void)
{
    Serial.begin(9600);
    int16_t accl_roll, accl_pitch, accl_yaw;
    int32_t tempX = 0, tempY = 0, tempZ = 0;
    MPU.reset();
    MPU.setup_accl(3);

    Serial.println("Taking Samples ... Don't move the accl");
    for (unsigned count = 0u; count < 10000u; count++)
    {
        MPU.get_accl_data(accl_roll, accl_pitch, accl_yaw);

        tempX += accl_roll;
        tempY += accl_pitch;
        tempZ += accl_yaw;
    }
    Serial.println("Calibration Complete.");
    accl_roll = tempX / 10000;
    accl_pitch = tempY / 10000;
    accl_yaw = tempZ / 10000;
    EEPROM.write(6, accl_roll >> 8);
    EEPROM.write(7, accl_roll bitand 0x00ff);
    EEPROM.write(8, accl_pitch >> 8);
    EEPROM.write(9, accl_pitch bitand 0x00ff);
    EEPROM.write(10, accl_yaw >> 8);
    EEPROM.write(11, accl_yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(accl_roll);
    Serial.print(' ');
    Serial.print(accl_pitch);
    Serial.print(' ');
    Serial.print(accl_yaw);
    Serial.println();

    Serial.end();
}
