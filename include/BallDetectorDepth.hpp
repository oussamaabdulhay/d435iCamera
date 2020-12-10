#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/photo.hpp>
#include <cmath>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt64.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "medianFilter.hpp"
#include "HEAR_core/InputPort.hpp"
#include "HEAR_core/OutputPort.hpp"
#include "HEAR_math/Vector2D.hpp"
#include "HEAR_msg/Vector2DMsg.hpp"
#include "HEAR_core/Block.hpp"

class BallDetectorDepth : public Block
{
private:
    Port* _output_port;
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher puby,pubx,pubsize,pub_array_size,pub_sec,pub_nano;
    cv::Point2d _c_,crop_size;
    Vector2D<float> obj_pos;
    std::vector<cv::Point2f> temp;
    float threshold;
    Vector2DMsg pixel_location;
    double crop_col,crop_row,change_in_loc_x,change_in_loc_y;
    enum ports_id {OP_0_DATA};
    void process(DataMsg* t_msg, Port* t_port){};
    

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgOriginal,adjMap,falseColorsMap,imgThresholded,croppedFrame,im_with_keypoints;
    std::vector<cv::KeyPoint> keypoints;
    int minimum_dis;
    float point_of_interest;
    medianFilter* filter=new medianFilter();

    const std::string OPENCV_WINDOW = "Image window";
    int iLowH = 700;//200 //97
    int iHighH = 3000;//256 //179

    int iLowS = 0;//96 //164
    int iHighS = 256;//255 //255

    int iLowV = 0; //0 //0
    int iHighV = 256; //15 //30

    BallDetectorDepth(ros::NodeHandle &);
    ~BallDetectorDepth();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    cv::SimpleBlobDetector::Params params;
};