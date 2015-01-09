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
void hsv(Mat& inFrame, Mat& outFrame) 
{
	cvtColor(inFrame, outFrame, CV_BGR2HSV);
}
void maxFilter(Mat& frame, int ind, float mult=1.3)
{
	REP(i,frame.rows)
	{
		REP(j,frame.cols)
		{
			Vec3b& cur =frame.at<Vec3b>(i,j);
			if(cur[ind]>mult*cur[(ind+1)%3]&&cur[ind]>mult*cur[(ind+2)%3])  cur[ind]=255;
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
int main(int argc, char** argv)
{
	char* fileName=argv[1];
	Mat in=imread(fileName,1);
	int rows=in.rows,cols=in.cols;
	REP(k,3)
	{
		int tot=0,num=0, wht=0;
		REP(i,rows) REP(j,cols)
		{
			Vec3b& cur=in.at<Vec3b>(i,j);
			if(cur[0]==255&&cur[1]==255&&cur[2]==255)
			{
				wht++;
				continue;
			}
			tot+=cur[k];
			num++;
		}
		double res=tot/((double)num);
		std::cout<<k<<" "<<res<<" "<<wht<<std::endl;
	}
	return 0;
}
