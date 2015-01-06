#include <cassert>
#include <iostream>
#include <time.h>
#include "opencv2/opencv.hpp"
using namespace cv;
void processFrame(Mat& inFrame, Mat& outFrame) {
	Mat downFrame;
	resize(inFrame, downFrame, Size(), 0.5, 0.5, INTER_NEAREST);
	Mat hsvFrame;
	cvtColor(downFrame, hsvFrame, CV_BGR2HSV);
	// DEBUG
	//cvtColor(hsvFrame, outFrame, CV_HSV2BGR);
	Mat filteredFrame;
	inRange(hsvFrame, Scalar(0, 0, 0), Scalar(50, 255, 255), filteredFrame);
	cvtColor(filteredFrame, outFrame, CV_GRAY2BGR);
}
int main()
{
	VideoCapture cap(0);
	assert(cap.isOpened());
	Mat testFrame;
	cap >> testFrame;
	Mat testOut;
	processFrame(testFrame, testOut);
	Size outSize = Size(testOut.cols, testOut.rows);
	VideoWriter outVid("test.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	assert(outVid.isOpened());
	for (int i = 0; i < 50; ++i) {
		Mat frame;
		cap >> frame;
		std::cout << "Grabbed frame" << std::endl;
		Mat out;
		processFrame(frame, out);
		outVid << out;
		// 100ms
		struct timespec tim, tim2;
		tim.tv_sec = 0;
		tim.tv_nsec = 100000000;
		nanosleep(&tim, &tim2);
	}
}
