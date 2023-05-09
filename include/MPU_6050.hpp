#pragma once
#include <Wire.h>
#include <Arduino.h>

class MPU_6050
{
    const uint8_t MPU_ADDR = 0x68;
    const uint8_t PWR_MGMT_1 = 0x6b;
    const uint8_t GYRO_CONFIG = 0x1b;
    const uint8_t GYRO_XOUT_H = 0x43;

public:
    void reset(void)
    {
        Wire.beginTransmission(MPU_ADDR);
        // reset the chip
        Wire.write(PWR_MGMT_1);
        Wire.write(0x00);
        Wire.endTransmission();
    }


public: // gyro
    // full scale range 0 for 250 deg, 1 for 500 deg, 2 for 1000 deg, 3 for 2000 deg
    void setup_gyro(const uint8_t _fullScaleRange)
    {
        Wire.beginTransmission(MPU_ADDR);
        // reset the chip
        Wire.write(GYRO_CONFIG);
        Wire.write(_fullScaleRange << 3);
        Wire.endTransmission();
    }

    void get_gyro_data(int16_t &gyro_roll, int16_t &gyro_pitch, int16_t &gyro_yaw)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, uint8_t(6));
        gyro_roll = Wire.read() << 8 bitor Wire.read();
        gyro_pitch = Wire.read() << 8 bitor Wire.read();
        gyro_yaw = Wire.read() << 8 bitor Wire.read();
    }

public:
void get_accl_data(int16_t &gyro_roll, int16_t &gyro_pitch, int16_t &gyro_yaw)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, uint8_t(6));
        gyro_roll = Wire.read() << 8 bitor Wire.read();
        gyro_pitch = Wire.read() << 8 bitor Wire.read();
        gyro_yaw = Wire.read() << 8 bitor Wire.read();
    }
};

MPU_6050 MPU;