#pragma once
#include <MPU_6050.hpp>
#include <nRF24L01.hpp>
#include <vector.hpp>

class PID_Controller
{
    const Vector3<float> P_gain = {0.5f, 0.5f, 0.5f};
    const Vector3<float> I_gain = {0.02f, 0.02f, 0.02f};
    const Vector3<float> D_gain = {10.f, 10.f, 10.f};

    Vector3<float> error = {0.f, 0.f, 0.f}, previous_error = error;

    Vector3<float> P_correction, I_correction, D_correction;

    Vector3<float> PID_output;

    const float PID_max_output = 200.f;

public:
    // call in setup once
    void reset()
    {
        P_correction = {0.f, 0.f, 0.f};
        I_correction = {0.f, 0.f, 0.f};
        D_correction = {0.f, 0.f, 0.f};
    }

public:
    void calc_PWs(receiver_output &_receiverData, const gyro_output &_gyroData, uint16_t *_motorPW)
    {
        // remove later (simulates movement)
        static int i = 0, k =1;
        _receiverData.roll = _gyroData.roll;
        _receiverData.pitch = _gyroData.pitch;
        _receiverData.yaw = _gyroData.yaw + i;
        i+= k;
        if(i== 200 or i == -200) k*=-1;

        // calculate the errors
        error.roll = _receiverData.roll - _gyroData.roll;
        error.pitch = _receiverData.pitch - _gyroData.pitch;
        error.yaw = _receiverData.yaw - _gyroData.yaw;

        // calculate the PID corrections
        P_correction = P_gain * error;
        I_correction += I_gain * error;
        D_correction = D_gain * (error - previous_error);

        // Save this error in the previous_error varable
        previous_error = error;

       // the total PID output is the sum of P, I and D outputs
        PID_output = P_correction + I_correction + D_correction;

        // limit the PID outputs
        if (PID_output.roll > PID_max_output)
            PID_output.roll = PID_max_output;
        if (PID_output.pitch > PID_max_output)
            PID_output.pitch = PID_max_output;
        if (PID_output.yaw > PID_max_output)
            PID_output.yaw = PID_max_output;

        // limit the throttle to make room for corrections
        if (_receiverData.throttle > 1800)
            _receiverData.throttle = 1800;

        // calculate the PWS
        _motorPW[0] = _receiverData.throttle - PID_output.roll - PID_output.pitch - PID_output.yaw ;
        _motorPW[1] = _receiverData.throttle + PID_output.roll - PID_output.pitch + PID_output.yaw ;
        _motorPW[2] = _receiverData.throttle + PID_output.roll + PID_output.pitch - PID_output.yaw ;
        _motorPW[3] = _receiverData.throttle - PID_output.roll + PID_output.pitch + PID_output.yaw ;

        for (unsigned i = 0; i < 4; ++i)
        {
            if (_motorPW[i] < 1000)
                _motorPW[i] = 1000;
            if (_motorPW[i] > 2000)
                _motorPW[i] = 2000;
        }
    }
};

PID_Controller PID;