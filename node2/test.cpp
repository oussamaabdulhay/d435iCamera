#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> 
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
ros::init(argc, argv, "image_converter");
ros::NodeHandle nh;

ros::Rate loop_rate(5);



while (nh.ok()) {
            
if( argc != 2)
{
    cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
}

Mat image;
image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

if(! image.data )                              // Check for invalid input
{
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
}

Size patternsize(7,7); //interior number of corners
Mat gray ; //source image
vector<Point2f> corners; //this will be filled by the detected corners

bool patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FILTER_QUADS);
cout<<patternfound<<"\n";
if (patternfound){
    cvtColor(image,gray,COLOR_BGR2GRAY);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
}


circle(gray, Point(352.385, 128.914), 3, CV_RGB(255,0,0));
circle(gray, Point(352.447, 138.027), 3, CV_RGB(255,0,0));
//drawChessboardCorners(gray, patternsize, Mat(corners), patternfound);
cout<<corners.size()<<"\n";
for(int i=0 ; i< corners.size(); i++)
{
    cout<<corners[i]<<"\n";
}



namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
resizeWindow("Display window",1280,720);
imshow( "Display window", gray );                   // Show our image inside it.


waitKey(0); 
destroyAllWindows(); 
waitKey(1);                                  // Wait for a keystroke in the window

ros::spin();
return 0;


}


}
