#include <ros/ros.h>
#include <iostream>
#include "BallDetectorDepth.hpp"
#include "rayrotation_depth.hpp"
#include <opencv2/core/types.hpp>
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "ROSUnit_Optitrack.hpp"


int main(int argc, char **argv)
{
ros::init(argc, argv, "depth_node");
ros::NodeHandle nh_;
ROSUnit_Factory ROSUnit_Factory_main{nh_};
BallDetectorDepth* detection=new BallDetectorDepth(nh_);
ROSUnit_Optitrack* position_in_z=new ROSUnit_Optitrack(nh_);


ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/x");
ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/y");
ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/roll");
ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/pitch");
ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/yaw");
ROSUnit* rosunit_camera = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/camera_provider");
                                                                

rayrotation_depth* rotate = new rayrotation_depth();

detection->getPorts()[(int)BallDetectorDepth::ports_id::OP_0_DATA]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_0_CAMERA]);
rosunit_x_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_1_X_POSITION]);
rosunit_y_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_2_Y_POSITION]);
position_in_z->getPorts()[(int)ROSUnit_Optitrack::ports_id::OP_0_OPT]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_3_Z_POSITION]);
rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_4_ROLL]);
rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_5_PITCH]);
rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_4]->connect(rotate->getPorts()[(int)rayrotation_depth::ports_id::IP_6_YAW]);


rotate->getPorts()[(int)rayrotation_depth::ports_id::OP_0_DATA]->connect(rosunit_camera->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

ros::Rate r(200);
while (ros::ok())
{
  r.sleep();
  ros::spinOnce();
}
return 0;
}
