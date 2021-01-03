#pragma once
#include "RotationMatrix3by3.hpp"
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
using Eigen::MatrixXd;

class pixeltometer: public Block
{
    private:
        Port* _input_port_0;
        Port* _input_port_1;
        Port* _input_port_2;
        Port* _input_port_3;
        Port* _output_port;

        float depth, f_x,f_y;
    public:
        cv::Point2f ball_location;
        Vector3D<float> drone_orientation,object_location;
        Vector3D<float> update_location (Vector3D<float>);
        void process(DataMsg* t_msg, Port* t_port);
        void convert_pixel_to_meters();
        void update_rotation_matrices();
        Eigen::Matrix<double,3,3> R_inverse;
        


        
         enum ports_id {IP_0_CAMERA,IP_1_ROLL,IP_2_PITCH,IP_3_YAW,OP_0_DATA};
         pixeltometer();
        ~pixeltometer();
};