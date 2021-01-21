#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
#include <cmath>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt64.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include <iostream>
#include "medianFilter.hpp"
#include "HEAR_core/InputPort.hpp"
#include "HEAR_core/OutputPort.hpp"
#include "HEAR_math/Vector2D.hpp"
#include "HEAR_msg/Vector2DMsg.hpp"
#include "HEAR_core/Block.hpp"

class BallDetectorRgb : public Block
{
private:
    Port* _output_port;

public:
    ros::NodeHandle nh_;
    ros::Publisher puby, pubx;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    Vector2DMsg pixel_location;
    float threshold;
    Vector2D<float> obj_pos;
    cv::Point2d _c_;
    std::vector<cv::Point2f> temp;
    const std::string OPENCV_WINDOW = "Image window";
    std::vector<cv::KeyPoint> keypoints;
    medianFilter* filter=new medianFilter();
    float point_of_interest;
    int iLowH = 0;
    int iHighH = 57;

    int iLowS = 58;
    int iHighS = 255;

    int iLowV = 125;
    int iHighV = 255;

    enum ports_id {OP_0_DATA};
    void process(DataMsg* t_msg, Port* t_port){};

    BallDetectorRgb(ros::NodeHandle&);
    ~BallDetectorRgb();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    cv::SimpleBlobDetector::Params params;
};