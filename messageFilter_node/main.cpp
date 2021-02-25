#include "visual_servoing_test.hpp"
#include "ChangeType.hpp"
#include <ros/ros.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  ChangeType ct(nh);
  visual_servoing_test ic(nh);

  ros::spin();

  return 0;
}
