#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <cmath>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ros/ros.h>
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include <iostream>
#include "common_srv/Vector2D.hpp"
#include "common_srv/Vector2DMsg.hpp"


class BallDetectorRgb: public MsgEmitter, public MsgReceiver
{
public:
    ros::NodeHandle nh_;
    ros::Publisher puby, pubx;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    Vector2DMsg pixel_location;
    const std::string OPENCV_WINDOW = "Image window";
    int iLowH = 0;
    int iHighH = 18;

    int iLowS = 131;
    int iHighS = 255;

    int iLowV = 82;
    int iHighV = 255;
    Vector2D<float> _c_;

    BallDetectorRgb(ros::NodeHandle&);
    ~BallDetectorRgb();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
};