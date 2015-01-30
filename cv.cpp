#include <cassert>
#include <fstream>
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
typedef std::pair<double,double> pdd;
typedef std::pair<int,int> pii;

pdd getDist(int gi, int gj)
{
	std::ifstream file("out.txt");
	int x,y; double i,j;
	double minMatch=1e9;
	double oi,oj;
	while(!file.eof())
	{
		file>>x>>y;
		file>>i>>j;
		std::cout<<x<<" "<<y<<" "<<i<<" "<<j<<std::endl;
		double dist=(x-gi)*(x-gi)+(y-gj)*(y-gj);
		std::cout<<dist<<std::endl;
		if(dist<minMatch)
		{
			minMatch=dist;
			oi=i,oj=j;
		}
	}
	file.close();
	double angle=std::atan2(oi,oj)*180/3.14159;
	return mp(std::sqrt(oi*oi+oj*oj),angle);
}
pdd getOldDist(int i, int j)
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
void maxFilter(Mat& frame, std::vector<int> inds)
{
	int* mnblue=new int[frame.cols];
	REP(i,frame.cols) mnblue[i]=frame.rows;
	REP(i,frame.rows)
	{
		REP(j,frame.cols)
		{
			bool set=false;
			Vec3b& cur=frame.at<Vec3b>(i,j);
			for(int k=0;k<inds.size()&&!set;k++)
			{
				int ind=inds[k];
				double multA,multB;
				bool Y=false;
				if(ind==0) multA=1.22,multB=1.5;
				if(ind==1) multA=multB=1.15;
				if(ind==2) multA=multB=1.15;
				if(ind==3) Y=true,ind=1,multA=0.8,1.12;
				if(ind==4) Y=true,ind=2,multA=0.6,1.05;
				if(cur[ind]>75&&cur[ind]>multA*cur[(ind+1)%3]&&cur[ind]>multB*cur[(ind+2)%3])  
				{
					set=true;
					cur[ind]=255;
					cur[(ind+1)%3]=Y?255:0;
					cur[(ind+2)%3]=0;
					if(ind==0) mnblue[j]=min(mnblue[j],i);
				}
			}
			if(!set)
			{
				cur[0]=cur[1]=cur[2]=0;
			}
		}
	}
	REP(i,frame.cols) if(mnblue[i]==frame.rows) mnblue[i]=0;
	REP(j,frame.cols) for(int i=0;i<mnblue[j];i++) 
	{
		Vec3b& cur =frame.at<Vec3b>(i,j);
		cur[0]=cur[1]=cur[2]=0;
	}
}
void maxFilter(Mat& frame, int ind, double multA=1.22, double multB=1.5)
{
	bool Y=false;
	if(ind!=0) multA=multB=1.2;
	if(ind==3) Y=true,ind=1,multA=0.8,1.12;
	REP(i,frame.rows)
	{
		REP(j,frame.cols)
		{
			Vec3b& cur =frame.at<Vec3b>(i,j);
			if(cur[ind]>60&&cur[ind]>multA*cur[(ind+1)%3]&&cur[ind]>multB*cur[(ind+2)%3])  
			{
				cur[ind]=255;
				cur[(ind+1)%3]=Y?255:0;
				cur[(ind+2)%3]=0;
			}
			else 
			{
				cur[ind]=cur[(ind+1)%3]=cur[(ind+2)%3]=0;
			}
		}
	}
}
int** comp; std::vector<std::pair<double,double> > cents; int currentComp;
int dimR, dimC, toti, totj;
int dx[]={1,-1,0,0};
int dy[]={0,0,1,-1};
bool check(int i, int j, Mat &inFrame)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	Vec3b &cur=inFrame.at<Vec3b>(i,j);
	if((cur[0]+cur[1]+cur[2])==0) return false;
	if(comp[i][j]==-1) return true;	
	return false;
}
bool check(int i, int j, Mat &inFrame, int ind1, int ind2)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	Vec3b &cur=inFrame.at<Vec3b>(i,j);
	if(cur[ind1]==0) return false;
	if(cur[ind2]==0) return false;
	if(comp[i][j]==-1) return true;	
	return false;
}
bool checkin(int i, int j)
{
	if(i<0||i>=dimR) return false;
	if(j<0||j>=dimC) return false;
	return true;
}
int dfs(int i, int j, Mat &inFrame, int ind1, int ind2)
{
	int ar=1;
	toti+=i, totj+=j;
	comp[i][j]=currentComp;
	REP(k,4)
	{
		int ni=i+dx[k],nj=j+dy[k];
		if(!check(ni,nj,inFrame, ind1, ind2)) continue;
		ar+=dfs(ni,nj,inFrame,ind1,ind2);
	}
	return ar;
}
int dfs(int i, int j, Mat &inFrame )
{
	int ar=1;
	toti+=i, totj+=j;
	comp[i][j]=currentComp;
	REP(k,4)
	{
		int ni=i+dx[k],nj=j+dy[k];
		if(!check(ni,nj,inFrame)) continue;
		ar+=dfs(ni,nj,inFrame);
	}
	return ar;
}
struct centers 
{
	double dist;
	double angle;
	int type;
};
std::vector<centers> fill(Mat &inFrame)
{
	comp=new int*[inFrame.rows];
	currentComp=0;
	dimR=inFrame.rows, dimC=inFrame.cols;
	//std::cout<<dimR<<" "<<dimC<<std::endl;
	REP(i,inFrame.rows)
	{
		comp[i]=new int[inFrame.cols];
		REP(j,inFrame.cols) comp[i][j]=-1;
	}
	std::vector<centers> out;
	REP(i,inFrame.rows) REP(j,inFrame.cols) 
	{
		if(!check(i,j,inFrame)) continue;
		toti=0,totj=0;
		int ar=dfs(i,j,inFrame);
		int ci=toti/((double)ar);
		int cj=totj/((double)ar);
		if(ar>=500) 
		{
			//std::cout<<"("<<ci<<", "<<cj<<"), area: "<<ar<<std::endl;
			cents.pb(mp(ci,cj));
			std::pair<double,double> dist=getDist(ci,cj);
			//std::cout<<"This point is "<<dist.first<<" inches away at angle "<<dist.second<<" to the normal \n";
			centers add;
			add.dist=dist.first;
			add.angle=dist.second;
			Vec3b &cur=inFrame.at<Vec3b>(i,j);
			if(cur[1]!=0&&cur[2]!=0) add.type=3;
			else if(cur[0]!=0&&cur[2]!=0) add.type=4;
			else if(cur[0]!=0) add.type=0;
			else if(cur[1]!=0) add.type=1;
			else add.type=2;
			REP(x,5) REP(y,5) 
			{
				int ni=ci+x,nj=cj+y;
				if(checkin(ni,nj))
				{
					//std::cout<<ni<<"" <<nj<<std::endl;
					int ind=1;
					inFrame.at<Vec3b>(ni,nj)[ind]=0,inFrame.at<Vec3b>(ni,nj)[(ind+2)%3]=255;
				}
			}
			out.push_back(add);
		}
		currentComp++;
	}
	return out;
}
void fill(Mat &inFrame,int ind1, int ind2)
{
	comp=new int*[inFrame.rows];
	cents.resize(0);
	currentComp=0;
	dimR=inFrame.rows, dimC=inFrame.cols;
	//std::cout<<dimR<<" "<<dimC<<std::endl;
	REP(i,inFrame.rows)
	{
		comp[i]=new int[inFrame.cols];
		REP(j,inFrame.cols) comp[i][j]=-1;
	}
	vector<int> fcomp;
	REP(i,inFrame.rows) REP(j,inFrame.cols) 
	{
		if(!check(i,j,inFrame,ind1,ind2)) continue;
		toti=0,totj=0;
		int ar=dfs(i,j,inFrame,ind1,ind2);
		int ci=toti/((double)ar);
		int cj=totj/((double)ar);
		if(ar>=400) 
		{
			fcomp.pb(currentComp);
			cents.pb(mp(ci,cj));
		}
		currentComp++;
	}
	REP(i,inFrame.rows) REP(j,inFrame.cols) 
	{
		bool in=false;
		REP(k,fcomp.size() )
		{
			if(comp[i][j]==fcomp[k]) in=true;
		}
		if(!in)
		{
			Vec3b& cur =inFrame.at<Vec3b>(i,j);
			cur[0]=cur[1]=cur[2]=0;
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
	return new_image;
}
void edgeDetect(Mat& inFrame, Mat& outFrame)
{
	Mat kern = (Mat_<char>(3,3) <<  0, -1,  0,
			-1,  4, -1,
			0, -1,  0);
	filter2D(inFrame, outFrame, inFrame.depth(), kern);
}
vector<Vec4i> hough(Mat &inFrame)
{
	const int edgeThresh=5;
	Mat gray,edge,blu;
	cvtColor(inFrame, gray, COLOR_BGR2GRAY);
	blur(gray, blu, Size(3,3));
	Canny(blu, edge, edgeThresh, edgeThresh*3, 3);
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI/3600.0, 35, 40, 30 );
	//std::cout<<std::endl<<std::endl<<"HOUGHING"<<std::endl;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		//std::cout<<l[0]<<","<<l[1]<<" "<<l[2]<<","<<l[3]<<std::endl;
	}
	return lines;
	//out.create(inFrame.size(),inFrame.type());
	//out= Scalar::all(0);G
	//inFrame.copyTo(out,edge);
}
double angle(Vec4i a, Vec4i b)
{
	cPoint A(a[2]-a[0],a[3]-a[1]),B(b[2]-b[0],b[3]-b[1]),C(0,0);
	double out= B.three_points_angle(&C,&A);
	double ang=fabs(out);
	ang=min(ang,CV_PI-ang);
	/*if(ang>0.25)
	{
		std::cout<<a[0]<<","<<a[1]<<" "<<a[2]<<","<<a[3]<<std::endl;
		std::cout<<b[0]<<","<<b[1]<<" "<<b[2]<<","<<b[3]<<std::endl;
		std::cout<<"Angle is"<<ang<<std::endl;
	}*/
	return out;
}
Vec2i inter(Vec4i a, Vec4i b)
{
	double c1=(a[1]*a[2]-a[3]*a[0])/(1.0*(a[2]-a[0]) );
	double m1=(a[3]-a[1])/(1.0*(a[2]-a[0]) );
	double c2=(b[1]*b[2]-b[3]*b[0])/(1.0*(b[2]-b[0]) );
	double m2=(b[3]-b[1])/(1.0*(b[2]-b[0]) );
	int X=(c2-c1)/(m1-m2);
	int Y=(m1*c2-m2*c1)/(m1-m2);
	std::cout<<"INTERSECT at "<<X<<" "<<Y<<std::endl;
	return Vec2i(X,Y);
}
vector<Vec4i> bundle(vector<Vec4i> lines)
{
	vector<Vec4i> out;
	double mx=0;
	REP(i,lines.size() )
	{
		bool in;
		REP(j,i)
		{
			double ang=angle(lines[j],lines[i]);
			ang=fabs(ang);
			ang=min(ang,CV_PI-ang);
			if(ang>mx&&ang>0.25)
			{
				mx=ang;
				out.resize(0);
				out.pb(lines[i]);
				out.pb(lines[j]);
			}
		}
	}
	return out;
}
struct interior
{
	bool init;
	pdd point;
	Vec4i a,b;
};
void add(Vec2i &pt, double r, Vec4i line)
{
	if(line[2]==line[0])
	{
		double c=1,s=0;
		if(abs(line[3]-pt[1])<abs(line[1]-pt[1]) ) r=-r;
		return;
	}
	if(line[2]<line[0])
	{
		std::swap(line[3],line[1]);
		std::swap(line[2],line[0]);
	}
	if(abs(line[2]-pt[0])==abs(line[0]-pt[0]) )
	{
		if(abs(line[3]-pt[1])<abs(line[1]-pt[1]) ) r=-r;
	}
	else if(abs(line[2]-pt[0])<abs(line[0]-pt[0]) ) r=-r;
	double m=((line[3]-line[1])/(1.0*(line[2]-line[0]) ));
	double c=1/sqrt(1+m*m);
	double s=c*m;
	pt[0]+=r*c; pt[1]+=r*s;
}
pdd procHough(vector<Vec4i> lines, Mat &inFrame)
{
	lines=bundle(lines);
	std::cout<<"number of bundels is"<<lines.size()<<std::endl;
	REP(j,lines.size() )
	{
		line( inFrame, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), Scalar(0,255,255), 3, CV_AA);
	}
	pii ret(-1,-1);
	if(lines.size()>=2) 
	{
		Vec2i pt= inter(lines[0],lines[1]);
		std::cout<<lines[0][0]<<","<<lines[0][1]<<" "<<lines[0][2]<<","<<lines[0][3]<<std::endl;
		std::cout<<lines[1][0]<<","<<lines[1][1]<<" "<<lines[1][2]<<","<<lines[1][3]<<std::endl;
		rectangle( inFrame, Point( pt[0], pt[1] ), Point( pt[0]+5,pt[1]+5), Scalar( 0, 255, 0 ), -1, 8 );
		add(pt,100,lines[0]); add(pt,100,lines[1]);
		std::cout<<"interior point is "<<pt[0]<<" "<<pt[1]<<std::endl;
		rectangle( inFrame, Point( pt[0], pt[1] ), Point( pt[0]+5,pt[1]+5), Scalar( 0, 0, 255 ), -1, 8 );
		ret=mp(pt[0],pt[1]);
		std::cout<<"interior point is "<<ret.first<<" inches away to deg"<<ret.second<<std::endl;
	}
	return ret;
}
