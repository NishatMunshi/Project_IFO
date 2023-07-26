#pragma once
#include <Wire.h>
#include <Arduino.h>

struct gyro_output
{
    int16_t roll, pitch, yaw;
    void operator-=(const gyro_output &_other)
    {
        this->roll -= _other.roll;
        this->pitch -= _other.pitch;
        this->yaw -= _other.yaw;
    }
};
struct accel_output
{
    int16_t x, y, z;
    void operator-=(const accel_output &_other)
    {
        this->x -= _other.x;
        this->y -= _other.y;
        this->z -= _other.z;
    }
};

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
    void read_accel_data(accel_output &_accelData)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, howManyBytes);

        highByte = Wire.read();
        lowByte = Wire.read();
        _accelData.x = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        _accelData.y = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        _accelData.z = (highByte << 8) bitor lowByte;
    }

public:
    void read_gyro_data(gyro_output &_gyroData)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission();

        Wire.requestFrom(MPU_ADDR, howManyBytes);

        highByte = Wire.read();
        lowByte = Wire.read();
        _gyroData.roll = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        _gyroData.pitch = (highByte << 8) bitor lowByte;

        highByte = Wire.read();
        lowByte = Wire.read();
        _gyroData.yaw = (highByte << 8) bitor lowByte;
    }
};

MPU_6050 MPU;