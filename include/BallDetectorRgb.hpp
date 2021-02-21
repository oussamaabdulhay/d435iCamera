#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include <iostream>
#include "medianFilter.hpp"
#include "HEAR_math/Vector2D.hpp"
#include "HEAR_msg/Vector3DMsg.hpp"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <sys/stat.h>


class BallDetectorRgb
{


public:
    ros::NodeHandle nh_;
    ros::Publisher pixel_center_location;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    float threshold;
    Vector3D<float> obj_pos;
    cv::Point2d _c_;
    std::vector<cv::Point2f> temp;
    const std::string OPENCV_WINDOW = "Image window";
    std::vector<cv::KeyPoint> keypoints;
    medianFilter* filter=new medianFilter();
    float point_of_interest;
    int iLowH = 0;
    int iHighH = 49;

    int iLowS = 56;
    int iHighS = 203;

    int iLowV = 186;
    int iHighV = 255;

    int imageIndex = 0;
    int file;
    
    std::string Path = "/home/osama/noDetectionFrames/frame";

    BallDetectorRgb(ros::NodeHandle&);
    ~BallDetectorRgb();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    void saveImage(cv::Mat&);
    cv::SimpleBlobDetector::Params params;
};