#include "RotationMatrix3by3.hpp"
using namespace std;
using Eigen::MatrixXd;

RotationMatrix3by3::RotationMatrix3by3(/* args */){
}

RotationMatrix3by3::~RotationMatrix3by3(){
}

MatrixXd RotationMatrix3by3::Update(Vector3D<float> t_input)
{ 
    
    Matrix3by3::v1.x =  cos(t_input.x)*cos(t_input.z) - sin(t_input.x)*sin(t_input.y)*sin(t_input.z);
    Matrix3by3::v1.y =  sin(t_input.z)*cos(t_input.y);
    Matrix3by3::v1.z = -sin(t_input.x)*cos(t_input.z)-cos(t_input.x)*sin(t_input.y)*sin(t_input.z);

    Matrix3by3::v2.x = -cos(t_input.x)*sin(t_input.z) - sin(t_input.x)*sin(t_input.y)*cos(t_input.z);
    Matrix3by3::v2.y =  cos(t_input.z)*cos(t_input.y);
    Matrix3by3::v2.z =  sin(t_input.x)*sin(t_input.z)-cos(t_input.x)*sin(t_input.y)*cos(t_input.z);

    Matrix3by3::v3.x =  sin(t_input.x)*cos(t_input.y);
    Matrix3by3::v3.y =  sin(t_input.y);
    Matrix3by3::v3.z =  cos(t_input.x)*cos(t_input.y);

    MatrixXd m(3,3);

    m(0,0)=v1.x;
    m(1,0)=v1.y;
    m(2,0)=v1.z;
    m(0,1)=v2.x;
    m(1,1)=v2.y;
    m(2,1)=v2.z;
    m(0,2)=v3.x;
    m(1,2)=v3.y;
    m(2,2)=v3.z;

    return m;
}