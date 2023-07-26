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
        ACCEL_CONFIG = 0x1c,
        GYRO_XOUT_H = 0x43,
        ACCEL_XOUT_H = 0x3b,
        TEMP_OUT_H = 0x41
    };
    byte lowByte, highByte;
    const byte howManyBytes = 6;

public:
    // to be called in setup once
    void setup(void)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_CONFIG);
        Wire.write(0b00011000); // +-16g full scale range
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_CONFIG);
        Wire.write(0b00011000); // +-2000 deg full scale range
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(PWR_MGMT_1);
        Wire.write(0b00001011); // dont reset the device, dont sleep, dont cycle, temp sensor disable, use z axis gyro clock as reference
        Wire.endTransmission();
    }

public:
    void read_accel_data(int16_t &accel_roll, int16_t &accel_pitch, int16_t &accel_yaw)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, howManyBytes);

        highByte = Wire.read();
        lowByte = Wire.read();
        accel_roll = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        accel_pitch = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        accel_yaw = (highByte << 8) bitor lowByte;
    }

public:
    void read_gyro_data(int16_t &gyro_roll, int16_t &gyro_pitch, int16_t &gyro_yaw)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, howManyBytes);

        highByte = Wire.read();
        lowByte = Wire.read();
        gyro_roll = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        gyro_pitch = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        gyro_yaw = (highByte << 8) bitor lowByte;
    }
};

MPU_6050 MPU;