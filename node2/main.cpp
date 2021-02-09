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
ROSUnit* rosunit_pixel = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/pixel_data");
ROSUnit* rosunit_servoing_object_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/servoing_object_position");
ROSUnit* rosunit_servoing_object_position_with_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/servoing_object_position_with_offset");
ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/yaw"); //0
ROSUnit* rosunit_depth = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/y"); //1 //TODO ADD x and y subscribers in case the orientation changes
ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/pitch");  //2
ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/roll");  //3
                                                                
test_rotation* locate = new test_rotation();

detection->getPorts()[(int)BallDetectorRgb::ports_id::OP_0_DATA]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_0_CAMERA]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_3_YAW]);
rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_2_PITCH]);
rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(locate->getPorts()[(int)test_rotation::ports_id::IP_1_ROLL]);

locate->getPorts()[(int)test_rotation::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
locate->getPorts()[(int)test_rotation::ports_id::OP_PIXEL_DATA]->connect(rosunit_pixel->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

plane_line_intersector* estimate = new plane_line_intersector();

locate->getPorts()[(int)test_rotation::ports_id::OP_0_DATA]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_0_UNIT_VEC]);
rosunit_depth->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_1_DEPTH_DATA]);
rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_2_ROLL]);
rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_3_PITCH]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_4_YAW]);

estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_0_DATA]->connect(rosunit_servoing_object_position->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_1_DATA]->connect(rosunit_servoing_object_position_with_offset->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);


ros::Rate r(70);
while (ros::ok())
{
  r.sleep();
  ros::spinOnce();
}
return 0;
}
