#include <Arduino.h>
#include <EEPROM.h>
#include <nRF24L01.hpp>
#include <MPU_6050.hpp>
#include <calibrate_MPU.hpp>

#if false
#define CALIBRATE
#endif

constexpr uint32_t REFRESH_RATE = 3496; /// 3500 -4
byte array[12];
uint16_t throttle, roll, pitch, yaw, camera, autopilot;                                                                                     // receiver inputs
int16_t gyro_roll, gyro_pitch, gyro_yaw, accl_roll, accl_pitch, accl_yaw;                                                                   // sensor inputs
int16_t gyro_roll_zero_error, gyro_pitch_zero_error, gyro_yaw_zero_error, accl_roll_zero_error, accl_pitch_zero_error, accl_yaw_zero_error; // zero errors

uint16_t PW_motor_0, PW_motor_1, PW_motor_2, PW_motor_3; // final motor pulse widths

uint32_t loop_timer, elapsed_time; // two timers to take care of the timing and refresh rate

void setup()
{
#ifdef CALIBRATE
    calibrate_gyro();
    calibrate_accl();
#endif

#ifndef CALIBRATE
    // reset and setup the gyro and accl
    Wire.begin();
    MPU.reset();
    MPU.setup_gyro(3);
    MPU.setup_accl(3);

    // Read the zero errors from EEPROM
    gyro_roll_zero_error = (EEPROM.read(0) << 8) bitor EEPROM.read(1);
    gyro_pitch_zero_error = (EEPROM.read(2) << 8) bitor EEPROM.read(3);
    gyro_yaw_zero_error = (EEPROM.read(4) << 8) bitor EEPROM.read(5);

    accl_roll_zero_error = (EEPROM.read(6) << 8) bitor EEPROM.read(7);
    accl_pitch_zero_error = (EEPROM.read(8) << 8) bitor EEPROM.read(9);
    accl_yaw_zero_error = (EEPROM.read(10) << 8) bitor EEPROM.read(11);

    // set up the radio receiver with the correct settings.
    receiver.setup();

    // remove later section
    Serial.begin(9600);
#endif
}

void loop()
{
#ifndef CALIBRATE
    // this loop ensures a steady refresh rate
    for (elapsed_time = 0; elapsed_time < REFRESH_RATE; elapsed_time = micros() - loop_timer) // corresponding to the refresh rate of the flight controller; subject to
    {                                                                                         // change according to the performance needs.
        __asm__ __volatile__("nop\n\t");                                                      // do nothing in the refresh period, nop is used to remove compiler optimisation
    }

    PORTD or_eq 0b11110000;                  // pull the pins that execute PWM high
    loop_timer = micros(), elapsed_time = 0; // start the timer

    // reading the receiver inputs ( about 36 us)
    receiver.read_rx_payload(array);

    // decompose the array into 6 channels and save it ( less than 4 us)
    // throttle, roll, pitch, yaw, camera, autopilot
    // array[0] = autopilot least significant byte
    // array[1] = autopilot most significant byte
    autopilot = array[1];
    autopilot = array[0] bitor (autopilot << 8);
    camera = array[3];
    camera = array[2] bitor (camera << 8);
    yaw = array[5];
    yaw = array[4] bitor (yaw << 8);
    pitch = array[7];
    pitch = array[6] bitor (pitch << 8);
    roll = array[9];
    roll = array[8] bitor (roll << 8);
    throttle = array[11];
    throttle = array[10] bitor (throttle << 8);
    // -------------------------------------------------

    // reading the gyro and accl data (about 560 us)
    MPU.get_gyro_data(gyro_roll, gyro_pitch, gyro_yaw);
    gyro_roll -= gyro_roll_zero_error;
    gyro_pitch -= gyro_pitch_zero_error;
    gyro_yaw -= gyro_yaw_zero_error;

    MPU.get_accl_data(accl_roll, accl_pitch, accl_yaw);
    accl_roll -= accl_roll_zero_error;
    accl_pitch -= accl_pitch_zero_error;
    accl_yaw -= accl_yaw_zero_error;
    // -------------------------------------------------

    // one PWM pulse for each motor
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
    PW_motor_0 = throttle;
    PW_motor_1 = roll;
    PW_motor_2 = pitch;
    PW_motor_3 = yaw;

// -------------------------------------------------
#endif
}