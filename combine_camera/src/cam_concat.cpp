#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_concatenation");
	ros::NodeHandle nh;

	ros::Publisher concat_pub = nh.advertise<sensor_msgs::Image>("usb_cam/image_raw", 100);
	ros::Rate loop_rate(10);

	// Video Concatenation
	VideoCapture cap0("/dev/video0");
	VideoCapture cap1("/dev/video4");

	if (!cap0.isOpened()) {
		cerr << "cap0 open failed" << endl;
		return 0;
	}

	if (!cap1.isOpened()) {
		cerr << "cap1 open failed" << endl;
		return 0;
	}

	cout << "Frame width: " << cvRound(cap0.get(CAP_PROP_FRAME_WIDTH)) << endl;
	cout << "Frame height: " << cvRound(cap0.get(CAP_PROP_FRAME_HEIGHT)) << endl;

	Mat frame0, frame1, concat;
	while (true) {
		cap0 >> frame0;
		cap1 >> frame1;

		if (frame0.empty()) break;
		if (frame1.empty()) break;

		hconcat(frame0, frame1, concat);

		//Publish the concatenated frame
		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg; // >> message to be sent

		std_msgs::Header header; // empty header
		header.seq = 1; // user defined counter
		header.stamp = ros::Time::now(); // time
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, concat);
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		
		concat_pub.publish(img_msg);

		imshow("concat", concat);

		if (waitKey(10) == 27) break;
	}

	destroyAllWindows();
	return 0;
}
