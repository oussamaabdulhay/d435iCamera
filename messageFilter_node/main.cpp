// #include "geometry_msgs/Vector3.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/Point32.h"
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/types.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/features2d.hpp>
// #include <image_transport/image_transport.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <cmath>
// #include <sstream>
// #include <iostream>
// #include "medianFilter.hpp"
// #include <sys/stat.h>
// #include "HEAR_math/RotationMatrix3by3.hpp"
// #include <math.h>
// #include <eigen3/Eigen/Dense>
// #include "visual_servoing_test.hpp"
// #include <ros/ros.h>
#include "visual_servoing_test.hpp"
#include "ChangeType.hpp"
#include <ros/ros.h>

// using namespace sensor_msgs;
// using namespace message_filters;
// using namespace geometry_msgs;


// void callback(const ImageConstPtr& msg, const PoseStampedConstPtr& roll, const PoseStampedConstPtr& pitch, const PoseStampedConstPtr& yaw)
// {
//   // Solve all of perception here...
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  ChangeType ct(nh);
  visual_servoing_test ic(nh);
//   message_filters::Subscriber<Image> image_sub(nh, "/camera/color/image_raw", 1);
//   message_filters::Subscriber<PoseStamped> roll_sub(nh, "/providers/roll", 1);
//   message_filters::Subscriber<PoseStamped> pitch_sub(nh, "/providers/pitch", 1);
//   message_filters::Subscriber<PoseStamped> yaw_sub(nh, "/providers/yaw", 1);

//   TimeSynchronizer<Image, PoseStamped, PoseStamped, PoseStamped> sync(image_sub, roll_sub, pitch_sub, yaw_sub, 10);
//   sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
