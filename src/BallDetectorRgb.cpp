#include "BallDetectorRgb.hpp"



BallDetectorRgb::BallDetectorRgb(ros::NodeHandle &main_nodehandle)
    : it_(nh_)
{
    nh_=main_nodehandle;
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1000,
        &BallDetectorRgb::imageCb, this);
    puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1000);
    pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1000);

    //cv::namedWindow(OPENCV_WINDOW);

  params.filterByArea = true;
  params.minArea = 200;
  params.maxArea = 2000;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.5;

  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.3;

  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.3;

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
    ros::Time begin = ros::Time::now();
    obj_pos.time=begin;
    cv::Mat imgOriginal = cv_ptr->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat blurred,im_with_keypoints;
    cv::GaussianBlur(imgHSV, blurred, cv::Size(11, 11), 0, 0);

    cv::Mat imgThresholded;

    cv::inRange(blurred, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

    // std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
    // std::vector<cv::Vec4i> hierarchy;

    // findContours(imgThresholded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE); // Find the contours in the image
    // int largest_area = 100;
    // int largest_contour_index = 0;
    // cv::Size s = imgThresholded.size();
    // int rows = s.height;
    //int cols = s.width;
    // std::cout<<rows<<"\n";
    // std::cout<<cols<<"\n";
    cv::bitwise_not(imgThresholded, imgThresholded);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(imgThresholded, keypoints);
    cv::drawKeypoints(imgThresholded, keypoints, im_with_keypoints, cv::Scalar(128, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std_msgs::Float32 msg_y;
    std_msgs::Float32 msg_x;
    //std::cout<<contours.size()<<std::endl;
    // for (int i = 0; i < contours.size(); i++) // iterate through each contour.
    // {
    //     double a = cv::contourArea(contours[i], false); //  Find the area of contour
    //     if (a > largest_area)
    //     {
    //         largest_area = a;
    //         //std::cout << a << std::endl;
    //         largest_contour_index = i;
    //         cv::Point2f center;
    //         float radius;                                        //Store the index of largest contour
    //         cv::minEnclosingCircle(contours[i], center, radius); //
    //         cv::circle(imgOriginal, center, radius, (255, 0, 0), 5, 8);
    //         cv::Point2f c;
    //         c.x = center.x;
    //         c.y = center.y;
    //         _c_.y= c.y - 240;
    //         _c_.x= c.x - 310;
    //         msg_y.data = _c_.y;
    //         msg_x.data = _c_.x;
    //         puby.publish(msg_y);
    //         pubx.publish(msg_x);
    //         pixel_location.setVector2DMsg(_c_);
    //         this->emitMsgUnicastDefault((DataMessage*)&pixel_location);
    //     }

    //     if (contours.size() == 0)
    //     {
    //         _c_.y = 230.8;
    //         _c_.x = 320;
    //         msg_y.data = _c_.y;
    //         msg_x.data = _c_.x;
    //         puby.publish(msg_y);
    //         pubx.publish(msg_x);
    //         pixel_location.setVector2DMsg(_c_);
    //         this->emitMsgUnicastDefault((DataMessage*)&pixel_location);
    //         std::cout << "NOT DETECTED NO CONTOURS\n";
    //     }
    // }


  if (keypoints.size() == 0)
  {
    // msg_y.data = 240;
    // msg_x.data = 165;
    // puby.publish(msg_y);
    // pubx.publish(msg_x);
    std::cout << "EMPTY KEYPOINTS\n";
  }

  else
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 3)
    {
      std_dev = filter->getStdDev(temp);
      std::cout << std_dev << std::endl;
      if (std_dev < threshold)
      {
        _c_.x = temp.back().x;
        _c_.y = temp.back().y;
        //if(_c_.x!=0 && _c_.y!=0)
        // {
        // crop_size=changeCrop(_c_);
        // }
        msg_y.data = _c_.y - 240;
        msg_x.data = _c_.x - 320;
        obj_pos.x = _c_.x-320;
        obj_pos.y = _c_.y-240;
        pixel_location.setVector2DMsg(obj_pos);
        this->emitMsgUnicastDefault((DataMessage *)&pixel_location);
        puby.publish(msg_y);
        pubx.publish(msg_x);
      }

      else
      {
        std::cout << "standard dev too high\n";
        _c_ = filter->getMedian(temp, _c_);
        //if(_c_.x!=0 && _c_.y!=0)
        // {
        // crop_size=changeCrop(_c_);
        // }
        msg_y.data = _c_.y - 240;
        msg_x.data = _c_.x - 320;
        point_of_interest = sqrt((pow(_c_.x, 2)) + (_c_.y, 2));
        puby.publish(msg_y);
        pubx.publish(msg_x);
        obj_pos.x = _c_.x-320;
        obj_pos.y = _c_.y-240;
        pixel_location.setVector2DMsg(obj_pos);
        this->emitMsgUnicastDefault((DataMessage *)&pixel_location);
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
