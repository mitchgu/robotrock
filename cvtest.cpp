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
	double alpha=2.0;
	int beta=80;
	//std::cout<<"* Enter the alpha value [1.0-3.0]: ";std::cin>>alpha;
	//std::cout<<"* Enter the beta value [0-100]: "; std::cin>>beta;
	Size outSize = Size(testOut.cols, testOut.rows);
	Size recSize = Size(testFrame.cols, testFrame.rows);
	VideoWriter outVid("test.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	VideoWriter recVid("rec.avi", CV_FOURCC('M','P','4','2'),10,recSize,true);
	VideoWriter incVid("inc.avi", CV_FOURCC('M','P','4','2'),10,recSize,true);
	assert(outVid.isOpened());
	for (int i = 0; i < 50; ++i) {
		Mat frame;
		cap >> frame;
		std::cout << "Grabbed frame" << std::endl;
		Mat out;
		processFrame(frame, out);
		outVid << out;
		recVid << frame;
		Mat new_image=Mat::zeros( frame.size(), frame.type() );
		for( int y = 0; y < frame.rows; y++ )
		{ 
			for( int x = 0; x < frame.cols; x++ )
			{ 
				for( int c = 0; c < 3; c++ )
				{
					new_image.at<Vec3b>(y,x)[c] =
						saturate_cast<uchar>( alpha*( frame.at<Vec3b>(y,x)[c] ) + beta );
				}
			}
		}
		incVid << new_image;
		// 100ms
		struct timespec tim, tim2;
		tim.tv_sec = 0;
		tim.tv_nsec = 100000000;
		nanosleep(&tim, &tim2);
	}
}
