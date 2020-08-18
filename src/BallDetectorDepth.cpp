#include "BallDetectorDepth.hpp"

BallDetectorDepth::BallDetectorDepth(ros::NodeHandle &main_nodehandle) : it_(nh_)
{
  // Subscribe to input video feed and publish output video feed
  nh_ = main_nodehandle;
  image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
                             &BallDetectorDepth::imageCb, this);
  //image_pub_ = it_.advertise("/detection_depth/output_video", 1);
  puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1);
  pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1);
  //pubsize = nh_.advertise<std_msgs::Float32>("size", 100);
  // pub_array_size = nh_.advertise<std_msgs::Float32>("array_size", 100);
  cv::namedWindow(OPENCV_WINDOW);

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 400;
  params.maxArea = 2000;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.6;

  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.6;

  threshold = 0.5;

  crop_size.x = 300; //640
  crop_size.y = 150; //480

  first_iteration = true;
  rest = false;
}

BallDetectorDepth::~BallDetectorDepth()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void BallDetectorDepth::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception &e)
  {

    ROS_ERROR("cv_bridge exception: %s", e.what());

    return;
  }

  cv_ptr->image.convertTo(imgOriginal, CV_64F);
  cv::inRange(imgOriginal, cv::Scalar(iLowH), cv::Scalar(iHighH), imgThresholded);
  croppedFrame = imgThresholded(cv::Rect(0, 70,640,300)); //70 80 500 300  //70 0 500 480
  cv::bitwise_not(croppedFrame, croppedFrame);
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  detector->detect(croppedFrame, keypoints);
  cv::drawKeypoints(croppedFrame, keypoints, im_with_keypoints, cv::Scalar(128, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std_msgs::Float32 msg_x;
  std_msgs::Float32 msg_y;
  std_msgs::Float32 msg_size;
  std_msgs::Float32 msg_array_size;

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
        msg_x.data = _c_.x - 215;
        obj_pos.x = _c_.x-215;
        obj_pos.y = _c_.y-240;
        float x_temp;
        x_temp = keypoints[0].size;
        msg_size.data = x_temp;
        //pubsize.publish(msg_size);
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
        msg_x.data = _c_.x - 215;
        msg_y.data = _c_.y - 240;
        obj_pos.x = _c_.x-215;
        obj_pos.y = _c_.y-240;
        point_of_interest = sqrt((pow(_c_.x, 2)) + (_c_.y, 2));
        puby.publish(msg_y);
        pubx.publish(msg_x);
        pixel_location.setVector2DMsg(obj_pos);
        this->emitMsgUnicastDefault((DataMessage *)&pixel_location);
      }
      temp.erase(temp.begin());
    }
  }

  cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 10000);
  cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 10000);

  cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 256);
  cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 256);

  cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 256);
  cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 256);
  cv::imshow("imgOriginal", imgOriginal);
  cv::imshow("imgThresholded", imgThresholded);
  cv::imshow("im_with_keypoints", im_with_keypoints);
  cv::imshow("croppedFrame", croppedFrame);

  cv::waitKey(1);
}

// cv::Point2d BallDetectorDepth::changeCrop(cv::Point2d ball_location_temp)
// {
//   double temp_x, temp_y;
//   std::cout<<"ball location x="<<ball_location_temp.x<<"\n";
//   std::cout<<"ball location y="<<ball_location_temp.y<<"\n";
//   temp_x = ball_location_temp.x+crop_size.x;
//   temp_y = ball_location_temp.y+crop_size.y;
//   if (first_iteration)
//   {
//     change_in_loc_x = temp_x;
//     change_in_loc_y = temp_y;
//     first_iteration = false;
//     rest = true;  
//   }
//   if (rest)
//   {
//     std::cout<<"change x="<<change_in_loc_x<<"\n";
//     std::cout<<"change y="<<change_in_loc_y<<"\n";
//     change_in_loc_x = (temp_x-change_in_loc_x);
//     change_in_loc_y = (temp_y-change_in_loc_y);
//     std::cout<<"change x after="<<change_in_loc_x<<"\n";
//     std::cout<<"change y after="<<change_in_loc_y<<"\n";
//     crop_size.x = (crop_size.x + change_in_loc_x);
//     crop_size.y = (crop_size.y + change_in_loc_y);
//     std::cout<<"crop_size x="<<crop_size.x<<"\n";
//     std::cout<<"crop_size y="<<crop_size.y<<"\n";
//   }

//   return crop_size;
// }