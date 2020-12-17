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

class rayrotation_rgb: public Block
{
    private:
        Port* _input_port_0;
        Port* _input_port_1;
        Port* _input_port_2;
        Port* _input_port_3;
        Port* _output_port;
    public:
        cv::Point2f ball_location;
        RotationMatrix3by3 R_o_d,R_d_c;
        Vector3D<float> drone_orientation,U_v,P_b,camera_angle,rotated_unit_vector;
        MatrixXd MultiplyMatrices(MatrixXd R_inertia, MatrixXd R_drone);
        void scale_and_translate();
        FloatMsg z_parameter,y_parameter,x_parameter;
        Vector3DMsg camera_parameters;
        Vector3DMsg all_parameters;
        Vector3D<float> obj_pos;
        Vector3D<float> Update_unit_vector(MatrixXd);
        void process(DataMsg* t_msg, Port* t_port);
        void update_camera_angles();
        void update_rotation_matrices();
        


        
         enum ports_id {IP_0_CAMERA,IP_1_ROLL,IP_2_PITCH,IP_3_YAW,OP_0_DATA};
         rayrotation_rgb();
        ~rayrotation_rgb();
};