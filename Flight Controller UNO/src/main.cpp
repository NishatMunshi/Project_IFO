#include <Arduino.h>
#include <EEPROM.h>
#include <nRF24L01.hpp>
#include <MPU_6050.hpp>
#include <calibrate_MPU.hpp>
#include <PID.hpp>
// #include<Servo.h>

// Servo servo;
#if 0
#define __CALIBRATE__
#endif

constexpr uint32_t REFRESH_RATE = 2000; // 3500 - 4

receiver_output receiverData;

accel_output accelData;
gyro_output gyroData;

accel_output accelOffset;
gyro_output gyroOffset;

uint16_t motorPW[4] = {1000u, 1000u, 1000u, 1000u}; // final motor pulse widths

uint32_t loop_timer, elapsed_time; // two timers to take care of the timing and refresh rate

void setup()
{
    // servo.attach(5);
#ifdef __CALIBRATE__
    calibrate_accel();
    calibrate_gyro();
#else
    // remove later section
    Serial.begin(9600);

    // set up the radio receiver with the correct settings.
    receiver.setup();

    // reset and setup the gyro and accl
    MPU.setup();

    // Read the zero errors from EEPROM
    accelOffset.x = (EEPROM.read(0) << 8) bitor EEPROM.read(1);
    accelOffset.y = (EEPROM.read(2) << 8) bitor EEPROM.read(3);
    accelOffset.z = (EEPROM.read(4) << 8) bitor EEPROM.read(5);

    gyroOffset.roll = (EEPROM.read(6) << 8) bitor EEPROM.read(7);
    gyroOffset.pitch = (EEPROM.read(8) << 8) bitor EEPROM.read(9);
    gyroOffset.yaw = (EEPROM.read(10) << 8) bitor EEPROM.read(11);

    // clear the PID variable memory
    PID.reset();
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

    PORTD or_eq 0b11110000; // pull the pins that execute PWM high
    elapsed_time = 0;       // start the timer for a pulsewidth
    loop_timer = micros();  // reinitiate the loop_timer

    // reading the receiver inputs ( about 40 us)
    receiver.read_rx_payload(receiverData);
    // -------------------------------------------------

    // unsigned long timer = micros();
    // reading the gyro and accl data and subtracting the offsets (about 384 us)
    MPU.read_accel_data(accelData);
    accelData -= accelOffset;

    MPU.read_gyro_data(gyroData);
    gyroData -= gyroOffset;
    // -------------------------------------------------
    // timer = micros() - timer;

    // one PWM pulse FALLING EDGE for each motor
    for (; true; elapsed_time = micros() - loop_timer) // we reinitiate the loop timer at the instant PWM pins go high
    {
        if (elapsed_time >= motorPW[0])
            PORTD and_eq 0b11101111; // assuming motor 0 on pin D4
        if (elapsed_time >= motorPW[1])
            PORTD and_eq 0b11011111; // assuming motor 1 on pin D5
        if (elapsed_time >= motorPW[2])
            PORTD and_eq 0b10111111; // assuming motor 2 on pin D6
        if (elapsed_time >= motorPW[3])
            PORTD and_eq 0b01111111; // assuming motor 3 on pin D7

        if (not(PORTD bitand 0b11110000))
            break; // when all 4 pulses have finished, break out of the loop
    }
    // -------------------------------------------------

    // -------------------------------------------------

    // unsigned long timer = micros();
    // PID SECTION
    PID.calc_PWs(receiverData, gyroData, motorPW);

    //    Serial.print(receiverData.throttle) ;
    //    Serial.print(' ');
    //    Serial.print(receiverData.roll) ;
    //    Serial.print(' ');
    //    Serial.print(receiverData.pitch) ;
    //    Serial.print(' ');
    //    Serial.print(receiverData.yaw) ;
    //    Serial.print(' ');
    //    Serial.print(receiverData.camera) ;
    //    Serial.print(' ');
    //    Serial.print(receiverData.autopilot) ;
    //    Serial.println();

    //    Serial.print(gyroData.roll) ;
    //    Serial.print(' ');
    //    Serial.print(gyroData.pitch) ;
    //    Serial.print(' ');
    //    Serial.print(gyroData.yaw) ;
    //    Serial.println();

    // motorPW[0] = receiverData.roll;
    // motorPW[1] = receiverData.roll;
    // motorPW[2] = receiverData.roll;
    // motorPW[3] = receiverData.roll;

    // timer = micros() - timer;
    // -------------------------------------------------
    // Serial.print(motorPW[0]);
    // Serial.print(' ');
    // Serial.print(motorPW[1]);
    // Serial.print(' ');
    // Serial.print(motorPW[2]);
    // Serial.print(' ');
    // Serial.println(motorPW[3]);

    // Serial.println(timer);

#endif
    // servo.write(100);
    // delay(200);
    // servo.write(20);
    // delay(200);
}