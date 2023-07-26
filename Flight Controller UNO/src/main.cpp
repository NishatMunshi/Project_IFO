#include <Arduino.h>
#include <EEPROM.h>
#include <nRF24L01.hpp>
#include <MPU_6050.hpp>
#include <calibrate_MPU.hpp>

#if false
#define __CALIBRATE__
#endif

constexpr uint32_t REFRESH_RATE = 3496; // 3500 - 4

receiver_output receiverData;

accel_output accelData;
gyro_output gyroData;

accel_output accelCalibData;
gyro_output gyroCalibData;

uint16_t PW_motor_0, PW_motor_1, PW_motor_2, PW_motor_3; // final motor pulse widths

uint32_t loop_timer, elapsed_time; // two timers to take care of the timing and refresh rate

void setup()
{
#ifdef __CALIBRATE__
    calibrate_gyro();
    calibrate_accl();
#else
    // set up the radio receiver with the correct settings.
    receiver.setup();

    // reset and setup the gyro and accl
    MPU.setup();

    // Read the zero errors from EEPROM
    accelCalibData.x = (EEPROM.read(0) << 8) bitor EEPROM.read(1);
    accelCalibData.y = (EEPROM.read(2) << 8) bitor EEPROM.read(3);
    accelCalibData.z = (EEPROM.read(4) << 8) bitor EEPROM.read(5);

    gyroCalibData.roll = (EEPROM.read(6) << 8) bitor EEPROM.read(7);
    gyroCalibData.pitch = (EEPROM.read(8) << 8) bitor EEPROM.read(9);
    gyroCalibData.yaw = (EEPROM.read(10) << 8) bitor EEPROM.read(11);

    // remove later section
    Serial.begin(9600);
#endif
}

void loop()
{
#ifndef __CALIBRATE__
    // this loop ensures a steady refresh rate
    for (elapsed_time = 0; elapsed_time < REFRESH_RATE; elapsed_time = micros() - loop_timer) // corresponding to the refresh rate of the flight controller; subject to
    {                                                                                         // change according to the performance needs.
        __asm__ __volatile__("nop\n\t");                                                      // do nothing in the refresh period, nop is used to remove compiler
                                                                                              // optimisation
    }

    PORTD or_eq 0b11110000;                  // pull the pins that execute PWM high
    loop_timer = micros(), elapsed_time = 0; // start the timer

    // unsigned long timer = micros();

    // reading the receiver inputs ( about 40 us)
    receiver.read_rx_payload(receiverData);
    // -------------------------------------------------

    // timer = micros() - timer;

    // reading the gyro and accl data and subtracting the offsets (about 384 us)
    MPU.read_accel_data(accelData);
    accelData -= accelCalibData;

    MPU.read_gyro_data(gyroData);
    gyroData -= gyroCalibData;
    // -------------------------------------------------

    // one PWM pulse FALLING EDGE for each motor
    for (; true; elapsed_time = micros() - loop_timer) // we reinitiate the loop timer at the instant PWM pins go high
    {
        if (elapsed_time >= PW_motor_0)
            PORTD and_eq 0b11101111; // assuming motor 0 on pin D4
        if (elapsed_time >= PW_motor_1)
            PORTD and_eq 0b11011111; // assuming motor 1 on pin D5
        if (elapsed_time >= PW_motor_2)
            PORTD and_eq 0b10111111; // assuming motor 2 on pin D6
        if (elapsed_time >= PW_motor_3)
            PORTD and_eq 0b01111111; // assuming motor 3 on pin D7

        if (not(PORTD bitand 0b11110000))
            break; // when all 4 pulses have finished, break out of the loop
    }
    // -------------------------------------------------

    // -------------------------------------------------

    // PID SECTION

    // -------------------------------------------------

    // Pulse Width calculation section

    // MODIFY later
    PW_motor_0 = receiverData.roll;
    PW_motor_1 = receiverData.pitch;
    PW_motor_2 = 1000;
    PW_motor_3 = 1000;

    // -------------------------------------------------
    // Serial.println(micros());
#endif
}