// PA_Gen2.cpp : Defines the entry point for the console application.
//

#include "stdio.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sstream> // for converting the command line parameter to integer

#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;

Mat lapangan, latar;
const double PI = 3.14159265;

// void all_estimate(float Xpos, float Ypos, float theta)
// {
// 	tha[1] = theta;
// 	if(tha[0]==NULL) tha[0] = theta;

// 	if (max_element(tha,tha+1) - min_element(tha,tha+1) > 90) zone = !zone;

// 	if(zone = false)
// 	{
// 		Xpos = 8 - Xpos;
// 		Ypos = 12 - Ypos;
// 		if (orient > 0) orient = orient - 180;
// 		else orient = orient + 180;
// 	}
	
// 	tha[0] = tha[1];
// }

int xDraw, yDraw;
void illustCallback(const geometry_msgs::Point& msg)
{

	//lapangan = imread("~/Documents/Data_PA/desain_Lapangan_new.png");
	//all_estimate(msg.x,msg.y,msg.z);

	latar.copyTo(lapangan);

	xDraw = sin(msg.z*PI/180)*12;
	yDraw = cos(msg.z*PI/180)*12;

	circle(lapangan, Point((msg.x*37.5+10),(msg.y*37.5+10)), 6, Scalar(0,0,255), 4);
	line(lapangan, Point((msg.x*37.5+10),(msg.y*37.5+10)), Point(msg.x*37.5+10+xDraw, msg.y*37.5+10-yDraw), Scalar(255,0,0), 2);

	ROS_INFO("X: %4f   Y: %4f   T: %4f", msg.x, msg.y, msg.z);

	//video.write(frame);
	imshow("Illustration", lapangan);
	//imshow("simulation", lapangan);
}

int main(int argc, char** argv) 
{
	if(argv[1] == NULL) return 1;

	latar = imread(argv[1]);
	lapangan = imread(argv[1]);

	ros::init(argc, argv, "opencv_illustration");
	ros::NodeHandle nh;

  	cv::namedWindow("Illustration");
  	cv::startWindowThread();
  	
  	ros::Subscriber sub = nh.subscribe("estimate", 1, illustCallback);

  	ros::spin();
  	destroyAllWindows();

	return 0;
}
//*/


