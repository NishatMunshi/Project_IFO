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
    EEPROM.write(0, gyro_roll bitand 0xff00);
    EEPROM.write(1, gyro_roll bitand 0x00ff);
    EEPROM.write(2, gyro_pitch bitand 0xff00);
    EEPROM.write(3, gyro_pitch bitand 0x00ff);
    EEPROM.write(4, gyro_yaw bitand 0xff00);
    EEPROM.write(5, gyro_yaw bitand 0x00ff);

    // print the calibration results
    Serial.print(EEPROM.read(0));
    Serial.print(' ');
    Serial.print(EEPROM.read(1));
    Serial.print(' ');
    Serial.print(EEPROM.read(2));
    Serial.print(' ');
    Serial.print(EEPROM.read(3));
    Serial.print(' ');
    Serial.print(EEPROM.read(4));
    Serial.print(' ');
    Serial.println(EEPROM.read(5));

    Serial.end();
}
