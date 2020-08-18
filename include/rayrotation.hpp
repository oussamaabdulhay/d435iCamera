#pragma once
#include "Matrix3by3.hpp"
#include "common_srv/Vector3D.hpp"
#include "RotationMatrix3by3.hpp"
#include <opencv2/core/types.hpp>
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Vector2DMsg.hpp"
#include "OptitrackMessage.hpp"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "common_srv/FloatMsg.hpp"
#include <ros/ros.h>
using Eigen::MatrixXd;

class rayrotation: public MsgEmitter, public MsgReceiver
{
    public:
        cv::Point2f ball_location;
        RotationMatrix3by3 R_o_d,R_d_c;
        ros::Time time;
        Vector3D<float> drone_position, drone_orientation,U_v,P_b,camera_angle,rotated_unit_vector;
        MatrixXd MultiplyMatrices(MatrixXd R_inertia, MatrixXd R_drone);
        void scale_and_translate();
        FloatMsg z_parameter,y_parameter,x_parameter;
        Vector3DMessage all_parameters,camera_parameters;
        Vector3D<float> obj_pos;
        Vector3D<float> Update_unit_vector(MatrixXd);
        void receiveMsgData(DataMessage* t_msg) {};
        void receiveMsgData(DataMessage* t_msg, int t_channel);
        void update_camera_angles();
        void update_rotation_matrices();
        rayrotation();
        bool x,y;

        

        enum receiving_channels {ch_x, ch_y, ch_z, ch_roll, ch_pitch, ch_yaw,camera,ch_camera_x,ch_camera_y};
        ~rayrotation();
};