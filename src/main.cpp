#include <Arduino.h>
#include <MPU_6050.hpp>
#include <calibrate_MPU.hpp>
int16_t accl_roll, accl_pitch, accl_yaw;
void setup()
{
    Wire.begin();
    Serial.begin(9600);

    MPU.reset();
    MPU.setup_accl(3);
}
void loop()
{
    MPU.get_accl_data(accl_roll, accl_pitch, accl_yaw);
    Serial.print(accl_roll);
    Serial.print(' ');
    Serial.print(accl_pitch);
    Serial.print(' ');
    Serial.println(accl_yaw);
}