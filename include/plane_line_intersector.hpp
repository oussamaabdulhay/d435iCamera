#pragma once
#include "HEAR_math/RotationMatrix3by3.hpp"
#include <opencv2/core/types.hpp>
#include <iostream>
#include "std_msgs/UInt64.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "HEAR_math/Matrix3by3.hpp"
#include "HEAR_msg/FloatMsg.hpp"
#include "HEAR_math/Vector3D.hpp"
#include "HEAR_msg/Vector2DMsg.hpp"
#include "HEAR_msg/Vector3DMsg.hpp"
#include "HEAR_core/InputPort.hpp"
#include "HEAR_core/OutputPort.hpp"
#include "HEAR_core/Block.hpp"
#include "HEAR_math/Plane3D.hpp"
using Eigen::MatrixXd;

class plane_line_intersector: public Block
{
    private:
        Port* _input_port_0;
        Port* _input_port_1;
        Port* _output_port_0;
        Plane3D<double> projection_plane;

        double inertial_plane_offset;

        Vector3D<double> rotated_unit_vector;

    public:
        float depth;
        Vector3D<float> plane_point1,plane_point2, plane_point3;
        void process(DataMsg* t_msg, Port* t_port);
              
         enum ports_id {IP_0_UNIT_VEC,IP_1_DEPTH_DATA, OP_0_DATA};
         plane_line_intersector();
        ~plane_line_intersector();
};