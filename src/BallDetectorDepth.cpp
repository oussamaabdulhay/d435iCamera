#include "BallDetectorDepth.hpp"

BallDetectorDepth::BallDetectorDepth(ros::NodeHandle &main_nodehandle) : it_(nh_)
{
  nh_ = main_nodehandle;

  this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
  _ports = {_output_port};
  image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &BallDetectorDepth::imageCb, this);
  puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1);
  pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1);
  //cv::namedWindow(OPENCV_WINDOW);

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 800;
  params.maxArea = 2100;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.2;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.2;

  threshold = 0.7;

  crop_size.x = 300; //640
  crop_size.y = 150; //480

}

BallDetectorDepth::~BallDetectorDepth()
{
  //cv::destroyWindow(OPENCV_WINDOW);
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
  std::cout<<keypoints.size()<<std::endl;
  if (keypoints.size() == 0)
  {
    std::cout << "EMPTY KEYPOINTS\n";
    puby.publish(msg_y);
    pubx.publish(msg_x);
    Vector2DMsg output_msg;
    output_msg.data = obj_pos;
    this->_output_port->receiveMsgData((DataMsg*) &output_msg);
    std::cout << "EMPTY KEYPOINTS\n";
  }

  else
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 3)
    {
      std_dev = filter->getStdDev(temp);
      if (std_dev < threshold)
      {
        _c_.x = temp.back().x;
        _c_.y = temp.back().y;
        msg_y.data = _c_.y - 150;
        msg_x.data = _c_.x - 320;
        obj_pos.y = _c_.y-150;
        obj_pos.x = _c_.x-320;
        Vector2DMsg output_msg;
        output_msg.data = obj_pos;
        this->_output_port->receiveMsgData((DataMsg*) &output_msg);
        puby.publish(msg_y);
        pubx.publish(msg_x);
      }

      else
      {
        _c_ = filter->getMedian(temp, _c_);
        msg_y.data = _c_.y - 150;
        msg_x.data = _c_.x - 320;
        obj_pos.y = _c_.y-150;
        obj_pos.x = _c_.x-320;
        point_of_interest = sqrt((pow(_c_.x, 2)) + (_c_.y, 2));
        puby.publish(msg_y);
        pubx.publish(msg_x);
       Vector2DMsg output_msg;
        output_msg.data = obj_pos;
        this->_output_port->receiveMsgData((DataMsg*) &output_msg);
      }
      temp.erase(temp.begin());
    }
  }

  // cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 10000);
  // cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 10000);

  // cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 256);
  // cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 256);

  // cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 256);
  // cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 256);

  // cv::imshow("imgOriginal", imgOriginal);
  // cv::imshow("imgThresholded", imgThresholded);
  // cv::imshow("im_with_keypoints", im_with_keypoints);
  // cv::imshow("croppedFrame", croppedFrame);

  // cv::waitKey(1);
}

