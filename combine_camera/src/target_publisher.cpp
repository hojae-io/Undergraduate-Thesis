#include "ros/ros.h"
#include "targetDetection_topic/TargetPosition.h"

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void find_HSign(ros::Publisher target_pub, targetDetection_topic::TargetPosition msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_publisher2");
	ros::NodeHandle nh;

	ros::Publisher target_pub = nh.advertise<targetDetection_topic::TargetPosition>("target_msg2", 100);
	ros::Rate loop_rate(10);

	targetDetection_topic::TargetPosition msg;
	find_HSign(target_pub, msg);
	return 0;
}

void find_HSign(ros::Publisher target_pub, targetDetection_topic::TargetPosition msg)
{
        VideoCapture cap(0);
        Mat templ = imread("LandingTarget.jpg", IMREAD_GRAYSCALE);

        if(!cap.isOpened()){
            cerr << "Camera open failed" << endl;
            return;
        }

        if (templ.empty()) {
            cerr << "Template image open failed" << endl;
            return;
	}

	cout << "Frame width: " << cvRound(cap.get(CAP_PROP_FRAME_WIDTH)) << endl;
	cout << "Frame height: " << cvRound(cap.get(CAP_PROP_FRAME_HEIGHT)) << endl;
	cout << "Template Size: " << templ.size << endl;


	int w = cvRound(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = cvRound(cap.get(CAP_PROP_FRAME_HEIGHT));
	double fps = cap.get(CAP_PROP_FPS);

        int fourcc = VideoWriter::fourcc('D', 'X', '5', '0');
	int delay = cvRound(1000 / fps);

	// ������ ���� ����

	VideoWriter writer;
        writer.open("./live3.mpeg", fourcc, fps, Size(w + templ.cols, h));

	Ptr<Feature2D> orb = ORB::create();
	vector<KeyPoint> keypoints_templ; // template�� Ư¡���� ����� ���� ���
	Mat desc_templ;
        orb->detectAndCompute(templ, Mat(), keypoints_templ, desc_templ);

	Mat img, img_color;
	while (ros::ok()) {
		cap >> img_color;
		if (img_color.empty()) break;
		cvtColor(img_color, img, COLOR_BGR2GRAY);

		vector<KeyPoint> keypoints_img; // �� ������(img) ���� Ư¡���� ����� ���
		Mat desc_img;
		orb->detectAndCompute(img, Mat(), keypoints_img, desc_img);

		Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);

		vector<DMatch> matches;
		matcher->match(desc_templ, desc_img, matches);

		std::sort(matches.begin(), matches.end());
		vector<DMatch> good_matches(matches.begin(), matches.begin() + 50);

		Mat dst;
		drawMatches(templ, keypoints_templ, img, keypoints_img, good_matches, dst, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		vector<Point2f> pts1, pts2;
		for (size_t i = 0; i < good_matches.size(); i++) {
			pts1.push_back(keypoints_templ[good_matches[i].queryIdx].pt); //keyoints_templ���� query Ư¡���� ����Ǿ� �ִ�. �� Ư¡���� pts1�� �����Ѵ�.
			pts2.push_back(keypoints_img[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(pts1, pts2, RANSAC);

		vector<Point2f> corners1, corners2;
		corners1.push_back(Point2f(0, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, templ.rows - 1.f));
		corners1.push_back(Point2f(0, templ.rows - 1.f));
		perspectiveTransform(corners1, corners2, H);

		vector<Point> corners_dst;
		float sumX = 0;
		float sumY = 0;
		for (Point2f pt : corners2) {
			corners_dst.push_back(Point(cvRound(pt.x + templ.cols), cvRound(pt.y)));
			sumX += pt.x + templ.cols;
			sumY += pt.y;
		}

		int X_coord = cvRound(sumX / 4.0);
		int Y_coord = cvRound(sumY / 4.0);
		cout << "X_coord: " <<  X_coord<< endl;
		cout << "Y_coord: " << Y_coord << endl << endl;

		msg.x_coord = X_coord;
		msg.y_coord = Y_coord;

		ROS_INFO("send msg = %d", msg.x_coord);
		ROS_INFO("send msg = %d", msg.y_coord);

		target_pub.publish(msg);

		polylines(dst, corners_dst, true, Scalar(0, 255, 0), 2, LINE_AA);

		writer << dst;

		imshow("dst", dst);

		if (waitKey(10) == 27) break;
	}
}
