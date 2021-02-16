#include "BallDetectorRgb.hpp"



BallDetectorRgb::BallDetectorRgb(ros::NodeHandle &main_nodehandle)
    : it_(nh_)
{
  nh_=main_nodehandle;

  image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &BallDetectorRgb::imageCb, this);
  pixel_center_location = nh_.advertise<geometry_msgs::Point>("/pixel_data", 1);

    
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

  threshold = 10000.0;
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
    cv::drawKeypoints(imgThresholded, keypoints, im_with_keypoints, cv::Scalar(128, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // std_msgs::Float32 msg_x;
    // std_msgs::Float32 msg_y;
    std::cout<<keypoints.size()<<std::endl;
    if (keypoints.size() == 0)
    {
      std::cout << "EMPTY KEYPOINTS\n";
      // pubx.publish(msg_x);
      // puby.publish(msg_y);
      geometry_msgs::Point point_pub;
      point_pub.x=obj_pos.x;
      point_pub.y=obj_pos.y;
      pixel_center_location.publish(point_pub);
    }

  else if (keypoints.size() == 1) // to be removed
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 1)
    {
      std_dev = filter->getStdDev(temp);
      //std::cout << std_dev << std::endl;
      if (std_dev < threshold)
      {
        _c_.x = temp.back().x;
        _c_.y = temp.back().y;
        // msg_x.data = _c_.x - 320;
        // msg_y.data = _c_.y - 240;
        obj_pos.x = _c_.x-320;
        obj_pos.y = _c_.y-240;
        geometry_msgs::Point point_pub;
        point_pub.x=obj_pos.x;
        point_pub.y=obj_pos.y;
        
        pixel_center_location.publish(point_pub);
        // pubx.publish(msg_x);
        // puby.publish(msg_y);
      }

      else
      {
        std::cout << "standard dev too high\n";
        _c_ = filter->getMedian(temp, _c_);
        // msg_x.data = _c_.x - 320;
        // msg_y.data = _c_.y - 240;
        // pubx.publish(msg_x);
        // puby.publish(msg_y);
        obj_pos.x = _c_.x-320;
        obj_pos.y = _c_.y-240;
        geometry_msgs::Point point_pub;
        point_pub.x=obj_pos.x;
        point_pub.y=obj_pos.y;
        pixel_center_location.publish(point_pub);
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Object_detection_rgb_node");
  ros::NodeHandle nh_;
  BallDetectorRgb* detection=new BallDetectorRgb(nh_);

  ros::Rate r(60); 
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
}