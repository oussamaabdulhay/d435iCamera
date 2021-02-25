#include "ChangeType.hpp"

ChangeType::ChangeType(ros::NodeHandle &t_nh)
{

    nh_ = t_nh;

    sub_roll = nh_.subscribe("/providers/roll", 1,&ChangeType::convert_roll,this);
    sub_pitch = nh_.subscribe("/providers/pitch", 1,&ChangeType::convert_pitch,this);
    sub_yaw = nh_.subscribe("/providers/yaw", 1,&ChangeType::convert_yaw,this);

    pub_roll = nh_.advertise<geometry_msgs::PoseStamped>("roll_angle", 1000);
    pub_pitch = nh_.advertise<geometry_msgs::PoseStamped>("pitch_angle", 1000);
    pub_yaw = nh_.advertise<geometry_msgs::PoseStamped>("yaw_angle", 1000);
}

ChangeType::~ChangeType()
{
}

void ChangeType::convert_roll(const geometry_msgs::PointConstPtr& msg_roll)
{
    geometry_msgs::PoseStamped roll;
    
    
    roll.pose.position.x = msg_roll->x;
    roll.pose.position.y = msg_roll->y;
    roll.header.stamp=ros::Time::now();

    pub_roll.publish(roll);

}

void ChangeType::convert_pitch(const geometry_msgs::PointConstPtr& msg_pitch)
{
    geometry_msgs::PoseStamped pitch;


    pitch.pose.position.x = msg_pitch->x;
    pitch.pose.position.y = msg_pitch->y;
    pitch.header.stamp=ros::Time::now();
 


    pub_pitch.publish(pitch);

}

void ChangeType::convert_yaw(const geometry_msgs::PointConstPtr& msg_yaw)
{
    geometry_msgs::PoseStamped yaw;

    yaw.pose.position.x = msg_yaw->x;
    yaw.pose.position.y = msg_yaw->y;
    yaw.header.stamp=ros::Time::now();

    pub_yaw.publish(yaw);
}