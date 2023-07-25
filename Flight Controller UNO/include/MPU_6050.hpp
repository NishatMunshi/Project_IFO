#pragma once
#include <Wire.h>
#include <Arduino.h>

class MPU_6050
{
private:
    const byte MPU_ADDR = 0x68;
    enum Registers : byte
    {
        PWR_MGMT_1 = 0x6b,
        GYRO_CONFIG = 0x1b,
        ACCL_CONFIG = 0x1c,
        GYRO_XOUT_H = 0x43,
        ACCL_XOUT_H = 0x3b
    };

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

        Wire.requestFrom(MPU_ADDR, 6);
        gyro_roll = (Wire.read() << 8) bitor Wire.read();
        gyro_pitch = (Wire.read() << 8) bitor Wire.read();
        gyro_yaw = (Wire.read() << 8) bitor Wire.read();
    }

public: // accl
    // 0 for 2g, 1 for 4g, 2 for 8g, 3 for 16g
    void setup_accl(const uint8_t _fullScaleRange)
    {
        Wire.beginTransmission(MPU_ADDR);
        // reset the chip
        Wire.write(ACCL_CONFIG);
        Wire.write(_fullScaleRange << 3);
        Wire.endTransmission();
    }
    void get_accl_data(int16_t &accl_roll, int16_t &accl_pitch, int16_t &accl_yaw)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCL_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, 6);
        accl_roll = (Wire.read() << 8) bitor Wire.read();
        accl_pitch = (Wire.read() << 8) bitor Wire.read();
        accl_yaw = (Wire.read() << 8) bitor Wire.read();
    }
};

MPU_6050 MPU;