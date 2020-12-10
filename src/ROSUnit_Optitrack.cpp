#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"

ROSUnit_Optitrack* ROSUnit_Optitrack::_instance_ptr = NULL;
OptitrackMsg ROSUnit_Optitrack::optitrack_msg;

ROSUnit_Optitrack::ROSUnit_Optitrack(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){
    _sub_attitude = t_main_handler.subscribe("/Robot_1/pose", 2, callbackOptitrack, ros::TransportHints().tcpNoDelay()); //Queue sizes of 2 are better than 1
    _instance_ptr = this;
    this->_output_port_0 = new OutputPort(ports_id::OP_0_OPT, this);
    _ports = {_output_port_0};
}

ROSUnit_Optitrack::~ROSUnit_Optitrack() {

}

void ROSUnit_Optitrack::callbackOptitrack(const geometry_msgs::PoseStamped& msg){
    
    Vector3D<float> pos_data;
    pos_data.x = msg.pose.position.x;
    pos_data.y = msg.pose.position.y;
    pos_data.z = msg.pose.position.z;
    Quaternion att_data;
    att_data.x = msg.pose.orientation.x;
    att_data.y = msg.pose.orientation.y;
    att_data.z = msg.pose.orientation.z;
    att_data.w = msg.pose.orientation.w;

    ros::Time t_time = msg.header.stamp;
    double t_dt = t_time.toSec();
    optitrack_msg.position = pos_data;
    optitrack_msg.attitude_heading =  att_data;
    optitrack_msg.time = t_dt;
    _instance_ptr->_output_port_0->receiveMsgData(&optitrack_msg);
}