#include <cassert>
#include <math.h>
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

const double distA=6.8327;
const double distB=0.0062;
const int distR=240;
const int distC=320;

std::pair<double, double> getDist(int i, int j)
{
	i=distR-i;
	j=j-(distC/2);
	double dist=distA*std::exp(distB*i);
	double angle=std::atan2(j,i);
	dist=dist/std::cos(angle);
	angle=angle*180/3.14159;
	return mp(dist,angle);
}
void downSize(Mat& inFrame, Mat& outFrame) 
{
	resize(inFrame, outFrame, Size(), 0.5, 0.5, INTER_NEAREST);
}
void hsv(Mat& inFrame, Mat& outFrame) 
{
	cvtColor(inFrame, outFrame, CV_BGR2HSV);
}
void maxFilter(Mat& frame, int ind, double multA=1.22, double multB=1.5)
{
	if(ind!=0) multA=multB=1.3;
	REP(i,frame.rows)
	{
		REP(j,frame.cols)
		{
			Vec3b& cur =frame.at<Vec3b>(i,j);
			if(cur[ind]>60&&cur[ind]>multA*cur[(ind+1)%3]&&cur[ind]>multB*cur[(ind+2)%3])  cur[ind]=255;
			else cur[ind]=0;
			cur[(ind+1)%3]=0;
			cur[(ind+2)%3]=0;
		}
	}
}
int** comp; std::vector<std::pair<double,double> > cents; int currentComp;
int dimR, dimC, toti, totj;
int dx[]={1,-1,0,0};
int dy[]={0,0,1,-1};
bool check(int i, int j, Mat &inFrame, int ind)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	if(inFrame.at<Vec3b>(i,j)[ind]==0) return false;
	if(comp[i][j]==-1) return true;	
	return false;
}
bool checkin(int i, int j)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	return true;
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
double fill(Mat &inFrame, int ind)
{
	comp=new int*[inFrame.rows];
	cents.resize(0);
	currentComp=0;
	dimR=inFrame.rows, dimC=inFrame.cols;
	std::cout<<dimR<<" "<<dimC<<std::endl;
	REP(i,inFrame.rows)
	{
		comp[i]=new int[inFrame.cols];
		REP(j,inFrame.cols) comp[i][j]=-1;
	}
	double ret=0;
	REP(i,inFrame.rows) REP(j,inFrame.cols) 
	{
		if(!check(i,j,inFrame,ind)) continue;
		toti=0,totj=0;
		int ar=dfs(i,j,inFrame,ind);
		int ci=toti/((double)ar);
		int cj=totj/((double)ar);
		if(ar>=1000) 
		{
			std::cout<<"("<<ci<<", "<<cj<<"), area: "<<ar<<std::endl;
			cents.pb(mp(ci,cj));
			std::pair<double,double> dist=getDist(ci,cj);
			std::cout<<"This point is "<<dist.first<<" inches away at angle "<<dist.second<<" to the normal \n";
			ret=dist.second;
			REP(x,5) REP(y,5) 
			{
				int ni=ci+x,nj=cj+y;
				if(checkin(ni,nj))
				{
					//std::cout<<ni<<"" <<nj<<std::endl;
					inFrame.at<Vec3b>(ni,nj)[ind]=0,inFrame.at<Vec3b>(ni,nj)[(ind+2)%3]=255;
				}
			}
		}
		currentComp++;
	}
	return ret;
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
