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
geometry_msgs::TransformStamped cam_to_cam_fixed, t_stamped; // My frames are named "base_link" and "leap_motion"
geometry_msgs::PointStamped object_pos_m, obj_bf, object_pos_px, rotated_px;
float _depth = 3.8, _f = 616.5;


void pixelCallback(const geometry_msgs::Point& msg){
  static tf2_ros::TransformBroadcaster br;
  
  //object_pos_px.point = msg;
  object_pos_px.point.x = _f;
  object_pos_px.point.y = -1 * msg.x;
  object_pos_px.point.z = -1 * msg.y;

  try{
    cam_to_cam_fixed = tf_Buffer.lookupTransform("camera", "camera_fixed", ros::Time(0), ros::Duration(0.01) );    
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  }
  tf2::doTransform(object_pos_px, rotated_px, cam_to_cam_fixed);

  object_pos_m.point.x = _depth * rotated_px.point.x/rotated_px.point.y;
  object_pos_m.point.y = _depth;
  object_pos_m.point.z = _depth * rotated_px.point.z/rotated_px.point.y;
  object_pos_m.header = rotated_px.header;
  
  // try{
  //   tf_Buffer.transform(object_pos_m, obj_bf, "body_fixed", ros::Duration(0.02));    
  // }
  // catch (tf2::TransformException &ex) {
  //   ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  // }

  t_stamped.header = object_pos_m.header;
  t_stamped.child_frame_id = "object";
  t_stamped.transform.translation.x = object_pos_m.point.x;
  t_stamped.transform.translation.y = object_pos_m.point.y;
  t_stamped.transform.translation.z = object_pos_m.point.z;
  t_stamped.transform.rotation.w = 1.0;
  br.sendTransform(t_stamped);

  pub_object_pos_i.publish(object_pos_m);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "position_node");
  ros::NodeHandle node;
  tf2_ros::TransformListener tfListener(tf_Buffer);
   
  ros::Subscriber sub_pixel = node.subscribe("/pixel_data", 10, &pixelCallback);
  pub_object_pos_i = node.advertise<geometry_msgs::PointStamped>("/position_data", 1);


  
  
  ros::spin();
  return 0;
};