#pragma once

template <typename _Tp>
struct Vector3
{
    _Tp roll, pitch, yaw;

    void operator=(const Vector3 &_other)
    {
        this->roll = _other.roll;
        this->pitch = _other.pitch;
        this->yaw = _other.yaw;
    }
    void operator+=(const Vector3 &_other)
    {
        this->roll += _other.roll;
        this->pitch += _other.pitch;
        this->yaw += _other.yaw;
    }
    Vector3 operator*(const Vector3 &_other) const// dot product
    {
        Vector3 product;
        product.roll = this->roll * _other.roll;
        product.pitch = this->pitch * _other.pitch;
        product.yaw = this->yaw * _other.yaw;
        return product;
    }
    Vector3 operator+(const Vector3 &_other) const
    {
        Vector3 product;
        product.roll = this->roll + _other.roll;
        product.pitch = this->pitch + _other.pitch;
        product.yaw = this->yaw + _other.yaw;
        return product;
    }
    Vector3 operator-(const Vector3 &_other) const
    {
        Vector3 product;
        product.roll = this->roll - _other.roll;
        product.pitch = this->pitch - _other.pitch;
        product.yaw = this->yaw - _other.yaw;
        return product;
    }
    
};