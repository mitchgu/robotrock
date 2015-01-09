#include <cassert>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"

#define REP(i,n) for(int i=0;i<n;i++)
#define mp std::make_pair
#define pb push_back

using namespace cv;

void downSize(Mat& inFrame, Mat& outFrame) 
{
	resize(inFrame, outFrame, Size(), 0.5, 0.5, INTER_NEAREST);
}
int main(int argc, char** argv)
{
	char* fileName=argv[1];
	VideoCapture cap(0); 
	if(!cap.isOpened())  return -1;
	Mat frame,out;
	cap >> frame; // get a new frame from camera
	downSize(frame,out);
	imwrite( fileName, out );
	return 0;
}
