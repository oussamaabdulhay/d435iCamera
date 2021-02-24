#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"


double pitch;
double roll;



void yawCallback(const geometry_msgs::Point& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now() + ros::Duration(0.025);
  // transformStamped.header.frame_id = "body_fixed";
  // transformStamped.child_frame_id = "body";
  tf2::Quaternion q;
  q.setRPY(roll, pitch, msg.x);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // br.sendTransform(transformStamped);
  transformStamped.header.frame_id = "camera_fixed";
  transformStamped.child_frame_id = "camera";
  
  br.sendTransform(transformStamped);
}

void rollCallback(const geometry_msgs::Point& msg){

    roll=msg.x;

}

void pitchCallback(const geometry_msgs::Point& msg){

    pitch=msg.x;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;
   
  

  ros::Subscriber sub_roll = node.subscribe("/providers/roll", 10, &rollCallback);
  ros::Subscriber sub_pitch = node.subscribe("/providers/pitch", 10, &pitchCallback);
  ros::Subscriber sub_yaw = node.subscribe("/providers/yaw", 10, &yawCallback);

  
  
  ros::spin();
  return 0;
};

