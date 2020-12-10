#pragma once
#include "HEAR_core/DataMsg.hpp"
#include "HEAR_math/Vector3D.hpp"
#include "HEAR_math/Quaternion.hpp"

class OptitrackMsg : public DataMsg {
public:
    Vector3D<double> position;
    double time;
    Quaternion attitude_heading; 
    msg_type getType() {return msg_type::optitrack;};
    const int getSize() {return sizeof(*this);};
    OptitrackMsg() {};
    ~OptitrackMsg() {};
};