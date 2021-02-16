#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"


ros::Publisher pub_object_pos_i;
tf2_ros::Buffer tf_Buffer;
 


void pixelCallback(const geometry_msgs::Point& msg){

    geometry_msgs::PointStamped object_position_m,object_position_m_transformed;
    object_position_m.point.x= 3.9;
    object_position_m.point.x= msg.x * (3.9 / 616.5);
    object_position_m.point.x= msg.y * (3.9 / 616.5);
    object_position_m.header.stamp=ros::Time::now();
    object_position_m.header.frame_id= "camera";



    try{
        tf_Buffer.transform(object_position_m, object_position_m_transformed, "inertial", ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex) {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    pub_object_pos_i.publish(object_position_m_transformed);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "position_node");
  ros::NodeHandle node;
   
  ros::Subscriber sub_pixel = node.subscribe("/pixel_data", 10, &pixelCallback);
  pub_object_pos_i = node.advertise<geometry_msgs::PointStamped>("/position_data", 1);


  
  
  ros::spin();
  return 0;
};