#pragma once
#include "HEAR_math/RotationMatrix3by3.hpp"
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

//The current resolution being used in the camera is 640 x 480. (60fps)
//the intrinsic camera matrix is =[ 616.437  0.0        319.82  ]
//                                [ 0.0     616.655     241.773 ]
//                                [ 0.0     0.0         1.0     ]

//using the matrix above the Horizantal field of view is 54.8429 degrees (0.9572 rad) and the Vertical field of view is 42.817 degrees (0.7473 rad).

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

         float f_c;
    public:
        Vector2D<float> ball_location;
        RotationMatrix3by3 R_i_d,R_d_c;
        Vector3D<float> drone_orientation,camera_vector,rotated_unit_vector;
        void scale_and_translate();
        Vector3DMsg camera_parameters;
        Vector3D<float> obj_pos;
        void Update_unit_vector(Eigen::Matrix<float,3,3>);
        void process(DataMsg* t_msg, Port* t_port);
        void update_camera_vector();
        void update_rotation_matrices();
        


        
         enum ports_id {IP_0_CAMERA,IP_1_ROLL,IP_2_PITCH,IP_3_YAW,OP_0_DATA, OP_CAMERA_ANGLES_DATA, OP_PIXEL_DATA};
         test_rotation();
        ~test_rotation();
};