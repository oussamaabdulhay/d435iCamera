#include <ros/ros.h>
#include <iostream>
#include "BallDetectorRgb.hpp"
#include "test_rotation.hpp"
#include <opencv2/core/types.hpp>
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "plane_line_intersector.hpp"


int main(int argc, char **argv)
{
ros::init(argc, argv, "test_node");
ros::NodeHandle nh_;
ROSUnit_Factory ROSUnit_Factory_main{nh_};
BallDetectorRgb* detection=new BallDetectorRgb(nh_);
ROSUnit_Optitrack* position_in_z=new ROSUnit_Optitrack(nh_);
ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh_);





ROSUnit* rosunit_camera = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/camera_provider");
ROSUnit* rosunit_camera_before_rotation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/camera_angles_rotation");
ROSUnit* rosunit_pixel = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point2D,
                                                                  "/pixel_data");
ROSUnit* rosunit_servoing_object_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/servoing_object_position");
ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/yaw");
ROSUnit* rosunit_depth = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/y"); //TODO ADD x and y subscribers in case the orientation changes
                                                                
test_rotation* locate = new test_rotation();

detection->getPorts()[(int)BallDetectorRgb::ports_id::OP_0_DATA]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_0_CAMERA]);
myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_1_ROLL]);
myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_2_PITCH]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_3_YAW]);

locate->getPorts()[(int)test_rotation::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
locate->getPorts()[(int)test_rotation::ports_id::OP_CAMERA_ANGLES_DATA]->connect(rosunit_camera_before_rotation->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
locate->getPorts()[(int)test_rotation::ports_id::OP_PIXEL_DATA]->connect(rosunit_pixel->getPorts()[(int)ROSUnit_Point2DPub::ports_id::IP_0]);

plane_line_intersector* estimate = new plane_line_intersector();

locate->getPorts()[(int)test_rotation::ports_id::OP_0_DATA]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_0_UNIT_VEC]);
rosunit_depth->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_1_DEPTH_DATA]);

estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_0_DATA]->connect(rosunit_servoing_object_position->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);


ros::Rate r(70);
while (ros::ok())
{
  r.sleep();
  ros::spinOnce();
}
return 0;
}
