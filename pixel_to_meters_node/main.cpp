#include <ros/ros.h>
#include <iostream>
#include "pixel_rotation.hpp"
#include <opencv2/core/types.hpp>
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "plane_line_intersector.hpp"


int main(int argc, char **argv)
{
ros::init(argc, argv, "pixel_to_meters_node");
ros::NodeHandle nh_;
ROSUnit_Factory ROSUnit_Factory_main{nh_};
ROSUnit_Optitrack* position_in_z=new ROSUnit_Optitrack(nh_);
ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh_);





ROSUnit* rosunit_camera = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/camera_provider");
ROSUnit* rosunit_p_i_d = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/p_i_d");
ROSUnit* rosunit_p1 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/p1");
ROSUnit* rosunit_p2 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/p2");                                                                  
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
ROSUnit* rosunit_pixel = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/pixel_data");//4
                                                                
pixel_rotation* locate = new pixel_rotation();

rosunit_pixel->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_4]->connect(locate->getPorts()[(int)pixel_rotation::ports_id::IP_0_CAMERA]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(locate->getPorts()[(int)pixel_rotation::ports_id::IP_3_YAW]);
rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(locate->getPorts()[(int)pixel_rotation::ports_id::IP_2_PITCH]);
rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(locate->getPorts()[(int)pixel_rotation::ports_id::IP_1_ROLL]);

locate->getPorts()[(int)pixel_rotation::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

plane_line_intersector* estimate = new plane_line_intersector();

locate->getPorts()[(int)pixel_rotation::ports_id::OP_0_DATA]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_0_UNIT_VEC]);
rosunit_depth->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_1_DEPTH_DATA]);
rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_2_ROLL]);
rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_3_PITCH]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(estimate->getPorts()[(int)plane_line_intersector::ports_id::IP_4_YAW]);

estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_0_DATA]->connect(rosunit_p_i_d->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_1_DATA]->connect(rosunit_p1->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
estimate->getPorts()[(int)plane_line_intersector::ports_id::OP_2_DATA]->connect(rosunit_p2->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);


ros::Rate r(200);
while (ros::ok())
{
  r.sleep();
  ros::spinOnce();
}
return 0;
}
