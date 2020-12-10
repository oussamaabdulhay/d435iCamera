#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "HEAR_ROS_BRIDGE/ROSUnit.hpp"
#include "HEAR_msg/OptitrackMsg.hpp"

class ROSUnit_Optitrack : public ROSUnit {
private:
    static ROSUnit_Optitrack* _instance_ptr;
    ros::Subscriber _sub_attitude;
    static OptitrackMsg optitrack_msg; 
    static void callbackOptitrack(const geometry_msgs::PoseStamped& msg);
    Port* _output_port_0;        
public:
    enum ports_id {OP_0_OPT};
    void process(DataMsg* t_msg, Port* t_port) { };
    ROSUnit_Optitrack(ros::NodeHandle&);
    ~ROSUnit_Optitrack();
};