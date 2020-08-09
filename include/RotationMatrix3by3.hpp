#pragma once
#include "Matrix3by3.hpp"
#include "common_srv/Vector3D.hpp"
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;

class RotationMatrix3by3 : public Matrix3by3
{
    private:
        /* data */
    public:
        RotationMatrix3by3(/* args */);
        ~RotationMatrix3by3();
        MatrixXd Update(Vector3D<float>);
};