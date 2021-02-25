#include "visual_servoing_test.hpp"


visual_servoing_test::visual_servoing_test(ros::NodeHandle &main_nodehandle)
{   
    
      
    nh_=main_nodehandle;
    //cv::namedWindow(OPENCV_WINDOW);
    image1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/color/image_raw", 100); // TODO: look into queue size
    roll_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/roll_angle", 100);
    pitch_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/pitch_angle", 100);
    yaw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/yaw_angle", 100);
    sync = new message_filters::Synchronizer<sync_poilicy>(sync_poilicy(100),*image1_sub, *roll_sub, *pitch_sub, *yaw_sub);
    sync->registerCallback(boost::bind(&visual_servoing_test::ImageProcess, this, _1, _2, _3, _4));

    pixel_center_location = nh_.advertise<geometry_msgs::Point>("/pixel_data", 1);
    drone_pose = nh_.advertise<geometry_msgs::Point>("/drone_position", 1);

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

    f_c=616.5;
    p_drone_camera.x = 0.1308;
    p_drone_camera.y = 0.038;
    p_drone_camera.z = -0.1137;


}

visual_servoing_test::~visual_servoing_test()
{
  //cv::destroyWindow(OPENCV_WINDOW);
}

void visual_servoing_test::ImageProcess(const sensor_msgs::ImageConstPtr& msg,const geometry_msgs::PoseStampedConstPtr& roll, const geometry_msgs::PoseStampedConstPtr& pitch, const geometry_msgs::PoseStampedConstPtr& yaw)

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
    drone_orientation.x=roll->pose.position.x;
    drone_orientation.y=pitch->pose.position.x;
    drone_orientation.z=yaw->pose.position.x;

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
      Vector3D<float> obj_pos;
      point_pub.x=obj_pos.x;
      point_pub.y=obj_pos.y;
      pixel_center_location.publish(point_pub);
      saveImage(im_with_keypoints);
    }

  else if (keypoints.size() == 1) // to be removed
  {
    float std_dev;
    list_of_positions.push_back(keypoints[0].pt);

    if (list_of_positions.size() == 1)
    {
      std_dev = filter->getStdDev(list_of_positions);
      if (std_dev < threshold)
      {
        center_point.x = list_of_positions.back().x;
        center_point.y = list_of_positions.back().y;

        Vector3D<float> pixel_pos;
        pixel_pos.x = center_point.x-320;
        pixel_pos.y = center_point.y-240;
        rotate_camera_vector(pixel_pos);
        
        geometry_msgs::Point pixel_pub;
        pixel_pub.x=pixel_pos.x;
        pixel_pub.y=pixel_pos.y;
        pixel_center_location.publish(pixel_pub);
      }

      else
      {
        std::cout << "standard dev too high\n";
        center_point = filter->getMedian(list_of_positions, center_point);
        
        Vector3D<float> pixel_pos;
        pixel_pos.x = center_point.x-320;
        pixel_pos.y = center_point.y-240;
        rotate_camera_vector(pixel_pos);

        geometry_msgs::Point pixel_pub;
        pixel_pub.x=pixel_pos.x;
        pixel_pub.y=pixel_pos.y;
        pixel_center_location.publish(pixel_pub);
      }
      list_of_positions.erase(list_of_positions.begin());
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


void visual_servoing_test::rotate_camera_vector(Vector3D<float> pixel_pos)
{

    Vector3D<float> camera_pixel_vector;
    camera_pixel_vector.x = f_c;
    camera_pixel_vector.y = -1 * pixel_pos.x;
    camera_pixel_vector.z = -1 * pixel_pos.y;

    RotationMatrix3by3 R_d_i;
    Eigen::Matrix<float, 3, 3> R_d_to_i_temp;
    R_d_to_i_temp = R_d_i.Update(drone_orientation); //Create the rotation matrices
    R_d_to_i_temp.transposeInPlace(); //drone to inertial
    
    Vector3D<float> t_results;
    t_results.x = camera_pixel_vector.x * R_d_to_i_temp(0, 0) + camera_pixel_vector.y * R_d_to_i_temp(0, 1) + camera_pixel_vector.z * R_d_to_i_temp(0, 2);
    t_results.y = camera_pixel_vector.x * R_d_to_i_temp(1, 0) + camera_pixel_vector.y * R_d_to_i_temp(1, 1) + camera_pixel_vector.z * R_d_to_i_temp(1, 2);
    t_results.z = camera_pixel_vector.x * R_d_to_i_temp(2, 0) + camera_pixel_vector.y * R_d_to_i_temp(2, 1) + camera_pixel_vector.z * R_d_to_i_temp(2, 2);

    get_object_location(t_results);

}



Vector3D<float> visual_servoing_test::rotate_offset()    
{
    Eigen::Matrix<float, 3, 3> R_body_to_inertial_temp(3, 3);
    RotationMatrix3by3 R_b_i;

    R_body_to_inertial_temp = R_b_i.Update(drone_orientation); //Create the rotation matrices
    R_body_to_inertial_temp.transposeInPlace(); //drone to inertial

    Vector3D<float> t_results;
    t_results.x = p_drone_camera.x * R_body_to_inertial_temp(0, 0) + p_drone_camera.y * R_body_to_inertial_temp(0, 1) + p_drone_camera.z * R_body_to_inertial_temp(0, 2);
    t_results.y = p_drone_camera.x * R_body_to_inertial_temp(1, 0) + p_drone_camera.y * R_body_to_inertial_temp(1, 1) + p_drone_camera.z * R_body_to_inertial_temp(1, 2);
    t_results.z = p_drone_camera.x * R_body_to_inertial_temp(2, 0) + p_drone_camera.y * R_body_to_inertial_temp(2, 1) + p_drone_camera.z * R_body_to_inertial_temp(2, 2);

    return t_results;
}

Vector3D<float> visual_servoing_test::get_object_location(Vector3D<float> rotated_pixel_vector)    
{

    Vector3D<float> object_location;

    object_location.x= (rotated_pixel_vector.x * 3.80) / rotated_pixel_vector.y;
    object_location.y= 3.80;
    object_location.z=(rotated_pixel_vector.z * 3.80) / rotated_pixel_vector.y;

    geometry_msgs::Point object_pos;
    object_pos.x=object_location.x;
    object_pos.y=object_location.y;
    object_pos.z=object_location.z;

    Vector3D<float> rotated_offset = rotate_offset();

    geometry_msgs::Point object_pos_with_offset;
    object_pos_with_offset.x = object_location.x + rotated_offset.x; 
    object_pos_with_offset.y = object_location.y + rotated_offset.y;
    object_pos_with_offset.z = object_location.z + rotated_offset.x;



    drone_pose.publish(object_pos);

    return object_location;
}

