#include <cassert>
#include <vector>
#include <iostream>
#include <time.h>
#include "opencv2/opencv.hpp"

#define REP(i,n) for(int i=0;i<n;i++)
#define mp make_pair
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
int** comp; std::vector<pair<double,double> > cents; int currentComp;
int dimR, dimC, toti, totj;
int[] dx={1,-1,0,0};
int[] dy={0,0,1,-1};
bool check(int i, int j, Mat &inFrame, int ind)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	if(comp[i][j]==-1) return true;	
	if(inFrame.at<Vec3b>(i,j)[ind]==0) return false;
	return false;
}
int dfs(int i, int j, Mat &inFrame, int ind)
{
	int ar=1;
	toti+=i, totj+=j;
	comp[i][j]=currentComp;
	REP(k,4)
	{
		int ni=i+dx[k],nj=j+dy[k];
		if(!check(ni,nj,inFrame,ind)) continue;
		ar+=dfs(ni,nj,inFrame,ind);
	}
	return ar;
}
void fill(Mat &inFrame, int ind)
{
	comp=new int*[inFrame.rows];
	cents.resize(0);
	currentComp=0;
	dimR=inFrame.rows, dimC=inFrame.cols;
	REP(i,inFrame.rows) comp[i]=new int[inFrame.cols];
	memset(comp,-1,sizeof(comp));
	REP(i,inFrame.rows) REP(j,inFrame.cols) 
	{
		if(!check(i,j,inFrame,ind)) continue;
		if(c=-1) continue;
		toti=0,totj=0;
		int ar=dfs(i,j,inFrame);
		double ci=toti/((double)ar);
		double cj=totj/((double)ar);
		if(ar>=1000) 
		{
			std::cout<<ci<<" "<<cj<<" "<<ar<<std::endl;
			cents.pb(mp(ci,cj));
			REP(x,8) REP(y,8) 
			{
				int ni=int(ci)+x,nj=int(cj)+y;
				if(check(ni,nj,inFrame,ind)) inFrame.at<Vec3b>(ni,nj)[ind]=0,inFrame.at<Vec3b>(ni,nj)[(ind+2)%2]=255;
			}
		}
		currentComp++;
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
		maxFilter(frame,1);
		fill(frame,1);
		outVid<<frame; //recorded processed video

		struct timespec tim, tim2;
		tim.tv_sec = 0;
		tim.tv_nsec = 100000000;
		nanosleep(&tim, &tim2);
	}
}
