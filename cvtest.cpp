#include <cassert>
#include <iostream>
#include <time.h>
#include "opencv2/opencv.hpp"

#define REP(i,n) for(int i=0;i<n;i++)

using namespace cv;

void downSize(Mat& inFrame, Mat& outFrame) 
{
	resize(inFrame, outFrame, Size(), 0.5, 0.5, INTER_NEAREST);
}
void hsv(Mat& inFrame, Mat& outFrame) 
{
	cvtColor(inFrame, outFrame, CV_BGR2HSV);
}
void maxFilter(Mat& frame, int ind) 
{
	Vec3b& cur;
	REP(i,frame.rows)
	{
		REP(j,frame.cols)
		{
			cur=frame.at(i,j);
			if(cur[ind]>cur[(ind+1)%3]&&cur[ind]>cur[(ind+2)%3])  cur[ind]=255;
			else cur[ind]=0;
			cur[(ind+1)%3]=0;
			cur[(ind+2)%3]=0;
		}
	}
}
Mat brigChange(Mat frame)
{
	double alpha=2.0;
	int beta=80;
	Mat new_image=Mat::zeros( frame.size(), frame.type() );
	for( int y = 0; y < frame.rows; y++ )
	{ 
		for( int x = 0; x < frame.cols; x++ )
		{ 
			for( int c = 0; c < 3; c++ ) new_image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( alpha*( frame.at<Vec3b>(y,x)[c] ) + beta );
		}
	}
}
Mat edgeDetect(Mat& inFrame, Mat& outFrame)
{
	Mat kern = (Mat_<char>(3,3) <<  0, -1,  0,
			-1,  4, -1,
			0, -1,  0);
	filter2D(inFrame, outFrame, inFrame.depth(), kern);
}
int main()
{
	VideoCapture cap(0);
	assert(cap.isOpened());
	Mat testFrame;
	cap >> testFrame;

	Mat testOut;

	downSize(testFrame, testOut);
	Size outSize = Size(testOut.cols, testOut.rows);

	VideoWriter outVid("test.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	VideoWriter recVid("rec.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	assert(outVid.isOpened());

	for (int i = 0; i < 50; ++i) 
	{
		Mat in;
		cap >> in;
		std::cout << "Grabbed frame" << std::endl;

		Mat frame;
		downSize(in,frame); //downsized
		recVid << frame; //recorded raw video

		Mat out;
		maxFilter(frame,out);
		outVid<<out; //recorded processed video

		struct timespec tim, tim2;
		tim.tv_sec = 0;
		tim.tv_nsec = 100000000;
		nanosleep(&tim, &tim2);
	}
}
