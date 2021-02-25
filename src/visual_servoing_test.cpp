#include "visual_servoing_test.hpp"


visual_servoing_test::visual_servoing_test(ros::NodeHandle &main_nodehandle)
{   
    f_c=616.5;
      
    nh_=main_nodehandle;
    //cv::namedWindow(OPENCV_WINDOW);
    image1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/color/image_raw", 100000);
    roll_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/roll_angle", 100000);
    pitch_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/pitch_angle", 100000);
    yaw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/yaw_angle", 100000);
    sync = new message_filters::Synchronizer<sync_poilicy>(sync_poilicy(100),*image1_sub, *roll_sub, *pitch_sub, *yaw_sub);
    sync->registerCallback(boost::bind(&visual_servoing_test::ImageProcess, this, _1, _2, _3, _4));


    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 5000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.5;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.5;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.6;

    threshold = 10000.0;
    file = mkdir("/home/osama/noDetectionFrames", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    pixel_center_location = nh_.advertise<geometry_msgs::Point>("/pixel_data", 1);
    drone_pose = nh_.advertise<geometry_msgs::Point>("/drone_position", 1);
}

visual_servoing_test::~visual_servoing_test()
{
  //cv::destroyWindow(OPENCV_WINDOW);
}

void visual_servoing_test::ImageProcess(const sensor_msgs::ImageConstPtr& msg,const geometry_msgs::PoseStampedConstPtr& roll, const geometry_msgs::PoseStampedConstPtr& pitch, const geometry_msgs::PoseStampedConstPtr& yaw)

{
    //std::cout<<"HELLOOOOOOOO\n";
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
    std::cout<<keypoints.size()<<std::endl;
    if (keypoints.size() == 0)
    {
      std::cout << "EMPTY KEYPOINTS\n";
      geometry_msgs::Point point_pub;
      point_pub.x=obj_pos.x;
      point_pub.y=obj_pos.y;
      pixel_center_location.publish(point_pub);
      saveImage(im_with_keypoints);
    }

  else if (keypoints.size() == 1) // to be removed
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 1)
    {
      std_dev = filter->getStdDev(temp);
      if (std_dev < threshold)
      {
        _c_.x = temp.back().x;
        _c_.y = temp.back().y;
        obj_pos.x = _c_.x-320;
        obj_pos.y = _c_.y-240;
        geometry_msgs::Point point_pub;
        point_pub.x=obj_pos.x;
        point_pub.y=obj_pos.y;
        drone_orientation.x=roll->pose.position.x;
        drone_orientation.y=pitch->pose.position.x;
        drone_orientation.z=yaw->pose.position.x;
        rotate_camera_vector();
        pixel_center_location.publish(point_pub);
      }

      else
      {
        std::cout << "standard dev too high\n";
        _c_ = filter->getMedian(temp, _c_);
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

    // cv::createTrackbar("LowH", OPENCV_WINDOW, &iLowH, 255);
    // cv::createTrackbar("HighH", OPENCV_WINDOW, &iHighH, 255);

    // cv::createTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255);
    // cv::createTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    // cv::createTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);
    // cv::createTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);
    // cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv::imshow("Original", imgOriginal);             //show the original image
    // cv::imshow("im_with_keypoints", im_with_keypoints); 
    // cv::waitKey(1);
}

void visual_servoing_test::saveImage(cv::Mat &img )
{
  Path=Path+std::to_string(imageIndex)+ ".jpg";
  cv::imwrite(Path,img);
  Path="/home/osama/noDetectionFrames/frame";
  imageIndex++;
}


void visual_servoing_test::rotate_camera_vector()
{
    camera_vector.x = f_c;
    camera_vector.y = -1 * obj_pos.x;
    camera_vector.z = -1 * obj_pos.y;

    RotationMatrix3by3 R_d_i;
    R_d_to_i_temp = R_d_i.Update(drone_orientation); //Create the rotation matrices
    R_d_to_i_temp.transposeInPlace(); //drone to inertial
    
    Vector3D<float> t_results;
    t_results.x = camera_vector.x * R_d_to_i_temp(0, 0) + camera_vector.y * R_d_to_i_temp(0, 1) + camera_vector.z * R_d_to_i_temp(0, 2);
    t_results.y = camera_vector.x * R_d_to_i_temp(1, 0) + camera_vector.y * R_d_to_i_temp(1, 1) + camera_vector.z * R_d_to_i_temp(1, 2);
    t_results.z = camera_vector.x * R_d_to_i_temp(2, 0) + camera_vector.y * R_d_to_i_temp(2, 1) + camera_vector.z * R_d_to_i_temp(2, 2);

    get_object_location();

}



Vector3D<float> visual_servoing_test::rotate_offset()    
{
    Eigen::Matrix<float, 3, 3> R_body_to_inertial_temp(3, 3);
    RotationMatrix3by3 R_b_i;

    R_body_to_inertial_temp = R_b_i.Update(drone_orientation); //Create the rotation matrices
    R_body_to_inertial_temp.transposeInPlace(); //drone to inertial

    Vector3D<float> t_results;
    t_results.x = p_d_c.x * R_body_to_inertial_temp(0, 0) + p_d_c.y * R_body_to_inertial_temp(0, 1) + p_d_c.z * R_body_to_inertial_temp(0, 2);
    t_results.y = p_d_c.x * R_body_to_inertial_temp(1, 0) + p_d_c.y * R_body_to_inertial_temp(1, 1) + p_d_c.z * R_body_to_inertial_temp(1, 2);
    t_results.z = p_d_c.x * R_body_to_inertial_temp(2, 0) + p_d_c.y * R_body_to_inertial_temp(2, 1) + p_d_c.z * R_body_to_inertial_temp(2, 2);

    return t_results;
}

Vector3D<float> visual_servoing_test::get_object_location()    
{

    Vector3D<float> object_location,data_transmitted_bo, data_transmitted_ao;

    object_location.x= (rotated_pixel_vector.x * 3.80) / rotated_pixel_vector.y;
    object_location.y= 3.80;
    object_location.z=(rotated_pixel_vector.z * 3.80) / rotated_pixel_vector.y;

    geometry_msgs::Point object_pos;
    object_pos.x=object_location.x;
    object_pos.y=object_location.y;
    object_pos.z=object_location.y;


    drone_pose.publish(object_pos);

    return object_location;
}

