#include <cassert>
#include "data.cpp"
#include <math.h>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"
#include "cv.cpp"

#define REP(i,n) for(int i=0;i<n;i++)
#define mp std::make_pair
#define pb push_back

using namespace cv;

int main()
{
	VideoCapture cap(0);
	assert(cap.isOpened());
	Mat testFrame;
	cap >> testFrame;

	Mat testOut,testEdge;

	downSize(testFrame, testOut);
	Size outSize = Size(testOut.cols, testOut.rows);

	//testEdge=hough(testOut);
	//Size edgeSize=Size(testEdge.cols,testEdge.rows);

	VideoWriter outVid("test.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	VideoWriter recVid("rec.avi", CV_FOURCC('M','P','4','2'),10,outSize,true);
	//VideoWriter edgeVid("edge.avi", CV_FOURCC('M','P','4','2'),10,edgeSize,false);
	assert(outVid.isOpened());
	std::vector<int> inds;
	int N,m; std::cin>>N;
	REP(i,N)
	{
		std::cin>>m;
		inds.pb(m);
	}
	//inds.pb(3);
	for (int i = 0; i < 20; ++i) 
	{
		Mat in;
		cap >> in;
		std::cout << "Grabbed frame" << std::endl;

		Mat frame;
		downSize(in,frame); //downsized
		recVid << frame; //recorded raw video

		maxFilter(frame,inds);
		fill(frame,0,2);

		vector<Vec4i> lines=hough(frame);
		procHough(lines,frame);

		//frame.copyTo(edge,emap);


		outVid<<frame; //recorded processed video


		struct timespec tim, tim2;
		tim.tv_sec = 0;
		tim.tv_nsec = 100000000;
		nanosleep(&tim, &tim2);
	}
}
