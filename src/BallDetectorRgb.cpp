#include "BallDetectorRgb.hpp"



  BallDetectorRgb::BallDetectorRgb(ros::NodeHandle &main_nodehandle)
      : it_(nh_)
  {
    nh_=main_nodehandle;
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1000,
                               &BallDetectorRgb::imageCb,this);
    puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1000);
    pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1000);

    cv::namedWindow(OPENCV_WINDOW);
  }

  BallDetectorRgb::~BallDetectorRgb()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    cv::Mat blurred;
    cv::GaussianBlur(imgHSV, blurred, cv::Size(11, 11), 0, 0);

    cv::Mat imgThresholded;

    cv::inRange(blurred, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

    std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;

    findContours(imgThresholded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE); // Find the contours in the image
    int largest_area = 100;
    int largest_contour_index = 0;
    cv::Size s = imgThresholded.size();
    int rows = s.height;
    int cols = s.width;
    // std::cout<<rows<<"\n";
    // std::cout<<cols<<"\n";
    std_msgs::Float32 msg_y;
    std_msgs::Float32 msg_x;
    std::cout<<contours.size()<<std::endl;
    for (int i = 0; i < contours.size(); i++) // iterate through each contour.
    {
      double a = cv::contourArea(contours[i], false); //  Find the area of contour
      if (a > largest_area)
      {
        largest_area = a;
        //std::cout << a << std::endl;
        largest_contour_index = i;
        cv::Point2f center;
        float radius;                                        //Store the index of largest contour
        cv::minEnclosingCircle(contours[i], center, radius); //
        cv::circle(imgOriginal, center, radius, (255, 0, 0), 5, 8);
        cv::Point2f c;
        c.x = center.x;
        c.y = center.y;
        _c_.y= c.y - 240;
        _c_.x= c.x - 310;
        msg_y.data = _c_.y;
        msg_x.data = _c_.x;
        puby.publish(msg_y);
        pubx.publish(msg_x);
        pixel_location.setVector2DMsg(_c_);
        this->emitMsgUnicastDefault((DataMessage*) &pixel_location);  
      }

    if (contours.size() == 0)
    {
      _c_.y = 230.8;
      _c_.x = 320;
      msg_y.data = _c_.y;
      msg_x.data = _c_.x;
      puby.publish(msg_y);
      pubx.publish(msg_x);
      pixel_location.setVector2DMsg(_c_);
      this->emitMsgUnicastDefault((DataMessage*) &pixel_location);  
      std::cout << "NOT DETECTED NO CONTOURS\n";
    }
    }

    cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 255);
    cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 255);

    cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255);
    cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);
    cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);
    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    cv::imshow("Original", imgOriginal);             //show the original image
    cv::waitKey(1);
  }
