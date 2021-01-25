#include "BallDetectorRgb.hpp"



BallDetectorRgb::BallDetectorRgb(ros::NodeHandle &main_nodehandle)
    : it_(nh_)
{
  nh_=main_nodehandle;

  this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
  _ports = {_output_port};

  image_sub_ = it_.subscribe("/camera/color/image_raw", 1,&BallDetectorRgb::imageCb, this);
    
  // puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1);
  // pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1);
  
  //cv::namedWindow(OPENCV_WINDOW);

  params.filterByArea = true;
  params.minArea = 800;
  params.maxArea = 5000;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.5;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.6;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.6;

  threshold = 10;
}

BallDetectorRgb::~BallDetectorRgb()
{
    //cv::destroyWindow(OPENCV_WINDOW);
}

void BallDetectorRgb::imageCb(const sensor_msgs::ImageConstPtr &msg)

{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {

        ROS_ERROR("cv_bridge exception: %s", e.what());

        return;
    }
    cv::Mat imgOriginal = cv_ptr->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat blurred,im_with_keypoints;
    cv::GaussianBlur(imgHSV, blurred, cv::Size(11, 11), 0, 0);

    cv::Mat imgThresholded;

    cv::inRange(blurred, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);


    cv::bitwise_not(imgThresholded, imgThresholded);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(imgThresholded, keypoints);
    // cv::drawKeypoints(imgThresholded, keypoints, im_with_keypoints, cv::Scalar(128, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // std_msgs::Float32 msg_y;
    // std_msgs::Float32 msg_x;
    std::cout<<keypoints.size()<<std::endl;
  if (keypoints.size() == 0)
  {
    std::cout << "EMPTY KEYPOINTS\n";
    // puby.publish(msg_y);
    // pubx.publish(msg_x);
    Vector2DMsg output_msg;
    output_msg.data = obj_pos;
    this->_output_port->receiveMsgData((DataMsg*) &output_msg);
  }

  else
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 3)
    {
      std_dev = filter->getStdDev(temp);
      //std::cout << std_dev << std::endl;
      if (std_dev < threshold)
      {
        _c_.x = temp.back().x;
        _c_.y = temp.back().y;
        // msg_y.data = _c_.y - 240;
        // msg_x.data = _c_.x - 320;
        obj_pos.y = _c_.y-240;
        obj_pos.x = _c_.x-320;
        Vector2DMsg output_msg;
        output_msg.data = obj_pos;
        this->_output_port->receiveMsgData((DataMsg*) &output_msg);
        // puby.publish(msg_y);
        // pubx.publish(msg_x);
      }

      else
      {
        //std::cout << "standard dev too high\n";
        _c_ = filter->getMedian(temp, _c_);
        // msg_y.data = _c_.y - 240;
        // msg_x.data = _c_.x - 320;
        // puby.publish(msg_y);
        // pubx.publish(msg_x);
        obj_pos.y = _c_.y-240;
        obj_pos.x = _c_.x-320;
        Vector2DMsg output_msg;
        output_msg.data = obj_pos;
        this->_output_port->receiveMsgData((DataMsg*) &output_msg);
      }
      temp.erase(temp.begin());
    }
  }

    // cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 179);
    // cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 179);

    // cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255);
    // cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    // cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);
    // cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);
    // cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv::imshow("Original", imgOriginal);             //show the original image
    // cv::imshow("im_with_keypoints", im_with_keypoints); 
    // cv::waitKey(1);
}
