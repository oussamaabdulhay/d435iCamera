#pragma once
#include "common_srv/Vector3D.hpp"

class Matrix3by3
{
    protected:
    Vector3D<float> v1, v2, v3;
    public:
    Matrix3by3();
    Matrix3by3(Vector3D<float>, Vector3D<float>, Vector3D<float>);
    Vector3D<float> TransformVector(Vector3D<float>);
    void Transpose();
};