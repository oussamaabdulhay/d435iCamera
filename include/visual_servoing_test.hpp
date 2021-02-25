#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include <sstream>
#include <iostream>
#include "medianFilter.hpp"
#include "geometry_msgs/Point.h"
#include <sys/stat.h>
#include "HEAR_math/RotationMatrix3by3.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>


class visual_servoing_test
{
  public:
    ros::NodeHandle nh_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped,geometry_msgs::PoseStamped,geometry_msgs::PoseStamped> sync_poilicy;
    message_filters::Synchronizer<sync_poilicy> *sync;
    message_filters::Subscriber<sensor_msgs::Image> *image1_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *roll_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *pitch_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *yaw_sub;
    ros::Publisher pixel_center_location,drone_pose;
    float threshold, f_c, depth;
    int file;
    cv::Point2d center_point;
    std::vector<cv::Point2f> list_of_positions;
    const std::string OPENCV_WINDOW = "Image window";
    std::vector<cv::KeyPoint> keypoints;
    medianFilter* filter=new medianFilter();
    Vector3D<float> drone_orientation,p_drone_camera;

    void ImageProcess(const sensor_msgs::ImageConstPtr& ,const geometry_msgs::PoseStampedConstPtr& , const geometry_msgs::PoseStampedConstPtr& , const geometry_msgs::PoseStampedConstPtr& );    
    void rotate_camera_vector(Vector3D<float> );
    void update_rotation_matrices(Vector3D<float>);
    Vector3D<float> rotate_offset();
    Vector3D<float> get_object_location(Vector3D<float>);
    void saveImage(cv::Mat&);  
        
    int iLowH = 0;
    int iHighH = 49;

    int iLowS = 56;
    int iHighS = 203;

    int iLowV = 186;
    int iHighV = 255;

    int imageIndex = 0;
   

    std::string Path = "/home/osama/noDetectionFrames/frame";
    cv::SimpleBlobDetector::Params params;

    visual_servoing_test(ros::NodeHandle&);
    ~visual_servoing_test();

};