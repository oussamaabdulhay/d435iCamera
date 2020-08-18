#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cmath>
#include "std_msgs/Float32.h"
#include <opencv2/photo.hpp>
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include <iostream>
#include "common_srv/Vector2D.hpp"
#include "common_srv/Vector2DMsg.hpp"
#include <opencv2/features2d.hpp>
#include <vector>
#include <algorithm>
#include "medianFilter.hpp"

class BallDetectorDepth : public MsgEmitter, public MsgReceiver
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher puby,pubx,pubsize,pub_array_size;
    cv::Point2d _c_,crop_size;
    Vector2D<float> obj_pos;
    std::vector<cv::Point2f> temp;
    float threshold;
    Vector2DMsg pixel_location;
    double crop_col,crop_row,change_in_loc_x,change_in_loc_y;
    bool first_iteration, rest;
    

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgOriginal,adjMap,falseColorsMap,imgThresholded,croppedFrame,im_with_keypoints;
    std::vector<cv::KeyPoint> keypoints;
    int minimum_dis;
    float point_of_interest;
    medianFilter* filter=new medianFilter();

    const std::string OPENCV_WINDOW = "Image window";
    int iLowH = 261;//200 //97
    int iHighH = 3500;//256 //179

    int iLowS = 255;//96 //164
    int iHighS = 256;//255 //255

    int iLowV = 0; //0 //0
    int iHighV = 129; //15 //30

    BallDetectorDepth(ros::NodeHandle &);
    ~BallDetectorDepth();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    //cv::Point2d changeCrop(cv::Point2d ball_location_temp);
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
};