#include <ros/ros.h>
#include <iostream>
#include "BallDetectorRgb.hpp"
#include "rayrotation_rgb.hpp"
#include "pixeltometer.hpp"
#include <opencv2/core/types.hpp>
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"


int main(int argc, char **argv)
{
ros::init(argc, argv, "rgb_node");
ros::NodeHandle nh_;
ROSUnit_Factory ROSUnit_Factory_main{nh_};
BallDetectorRgb* detection=new BallDetectorRgb(nh_);
ROSUnit_Optitrack* position_in_z=new ROSUnit_Optitrack(nh_);
ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh_);





ROSUnit* rosunit_camera = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/camera_provider");
ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/yaw");
                                                                

// rayrotation_rgb* rotate = new rayrotation_rgb();

// detection->getPorts()[(int)BallDetectorRgb::ports_id::OP_0_DATA]->connect(rotate->getPorts()[(int)rayrotation_rgb::ports_id::IP_0_CAMERA]);
// rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(rotate->getPorts()[(int)rayrotation_rgb::ports_id::IP_1_ROLL]);
// rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(rotate->getPorts()[(int)rayrotation_rgb::ports_id::IP_2_PITCH]);
// rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(rotate->getPorts()[(int)rayrotation_rgb::ports_id::IP_3_YAW]);

// rotate->getPorts()[(int)rayrotation_rgb::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

pixeltometer* locate = new pixeltometer();

detection->getPorts()[(int)BallDetectorRgb::ports_id::OP_0_DATA]->connect(locate->getPorts()[(int)pixeltometer::ports_id::IP_0_CAMERA]);
myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(locate->getPorts()[(int)pixeltometer::ports_id::IP_1_ROLL]);
myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(locate->getPorts()[(int)pixeltometer::ports_id::IP_2_PITCH]);
rosunit_yaw_provider->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(locate->getPorts()[(int)pixeltometer::ports_id::IP_3_YAW]);


locate->getPorts()[(int)pixeltometer::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

ros::Rate r(60);
while (ros::ok())
{
  r.sleep();
  ros::spinOnce();
}
return 0;
}
