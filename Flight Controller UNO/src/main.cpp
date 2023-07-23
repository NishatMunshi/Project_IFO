#include <Arduino.h>
#include <nRF24L01.hpp>
#include <MPU_6050.hpp>

byte array[12];
uint16_t throttle, roll, pitch, yaw, camera, autopilot;                   // receiver inputs
int16_t gyro_roll, gyro_pitch, gyro_yaw, accl_roll, accl_pitch, accl_yaw; // sensor inputs

uint16_t PW_motor_0, PW_motor_1, PW_motor_2, PW_motor_3; // final motor pulse widths

uint32_t loop_timer, elapsed_time; // two timers to take care of the timing and refresh rate

void setup()
{
    // reset and setup the gyro and accl
    Wire.begin();
    MPU.reset();
    MPU.setup_gyro(3);
    MPU.setup_accl(3);

    // set up the radio receiver with the correct settings.
    receiver.setup();

    // remove later section
    Serial.begin(9600);
}

void loop()
{
    // unsigned long timer = micros();

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
    MPU.get_accl_data(accl_roll, accl_pitch, accl_yaw);
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

    // this loop ensures a steady 4000 us refresh rate
    for (elapsed_time = 0; elapsed_time < 4000; elapsed_time = micros() - loop_timer) // 4000 us corresponding to the refresh rate of the flight controller; subject to
    {                                                                                 // change according to the performance needs.
        __asm__ __volatile__("nop\n\t");                                              // do nothing in the refresh period, nop is used to remove compiler optimisation
    }

    // one PWM pulse for each motor
    PORTD or_eq 0b11110000;                                                                   // pull the pins that execute PWM high
    for (loop_timer = micros(), elapsed_time = 0; true; elapsed_time = micros() - loop_timer) // we reinitiate the loop timer at the instant PWM pins go high
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

    // Serial.println(micros() - timer);
}