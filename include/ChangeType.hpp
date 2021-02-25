#include <iostream>
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>


class ChangeType
{
  public:
  ros::NodeHandle nh_;

  ros::Subscriber sub_roll;
  ros::Subscriber sub_pitch;
  ros::Subscriber sub_yaw;

  ros::Publisher pub_roll;
  ros::Publisher pub_pitch;
  ros::Publisher pub_yaw;

  void convert_roll(const geometry_msgs::PointConstPtr& );
  void convert_pitch(const geometry_msgs::PointConstPtr&);
  void convert_yaw(const geometry_msgs::PointConstPtr& );

  ChangeType(ros::NodeHandle&);
  ~ChangeType();

};