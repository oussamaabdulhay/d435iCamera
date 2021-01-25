#pragma once
#include "HEAR_math/RotationMatrix3by3_camera.hpp"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "HEAR_math/Matrix3by3.hpp"
#include "HEAR_math/Vector3D.hpp"
#include "HEAR_math/Vector2D.hpp"
#include "HEAR_msg/Vector2DMsg.hpp"
#include "HEAR_msg/Vector3DMsg.hpp"
#include "HEAR_core/InputPort.hpp"
#include "HEAR_core/OutputPort.hpp"
#include "HEAR_core/Block.hpp"
using Eigen::MatrixXd;

class test_rotation: public Block
{
    private:
        Port* _input_port_0;
        Port* _input_port_1;
        Port* _input_port_2;
        Port* _input_port_3;
        Port* _output_port_0;
        Port* _output_port_1;
        Port* _output_port_2;
    public:
        Vector2D<float> ball_location;
        RotationMatrix3by3_camera R_i_d,R_d_c;
        Vector3D<float> drone_orientation,U_v,camera_angle,rotated_unit_vector;
        MatrixXd MultiplyMatrices(MatrixXd R_1, MatrixXd R_2);
        void scale_and_translate();
        Vector3DMsg camera_parameters;
        Vector3DMsg all_parameters;
        Vector3D<float> obj_pos;
        void Update_unit_vector(MatrixXd);
        void process(DataMsg* t_msg, Port* t_port);
        void update_camera_angles();
        void update_rotation_matrices();
        


        
         enum ports_id {IP_0_CAMERA,IP_1_ROLL,IP_2_PITCH,IP_3_YAW,OP_0_DATA, OP_CAMERA_ANGLES_DATA, OP_PIXEL_DATA};
         test_rotation();
        ~test_rotation();
};