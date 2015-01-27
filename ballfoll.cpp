#include <cassert>
#include <math.h>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"
#include <sys/time.h>
#include <mraa.hpp>
#include <signal.h>

#include "cv.cpp"
#include "servo.cpp"
#include "wallfollower.cpp"

const double wThresh=6;

int running=1;
void sig_handler(int signo)
{
	if (signo == SIGINT) {
		printf("closing PWM nicely\n");
		running = 0;
	}
}

class FinalRunner
{	
	Motor *left, *right, *base, *lift;
	Odometry *odo;
	Location* location;
	mraa::Gpio* upEnd,*rotEnd,*uirb,*cube,*midEnd;
	Motion* motion;
	VideoCapture* cap;
	Mat in,test,frame;
	std::vector<int> inds; 
	double lastDist, target;
	int lostCount;
	bool ballSeen,foundBall; int type,channel;
	Wallfollower* wf;
	IR *irf, *irr, *irlf, *irlb;
	VideoWriter* outVid,*recVid;
	struct timeval tv;
	double timeDiff() 
	{ 
		unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec) / 1000; 
		gettimeofday(&tv, NULL); 
		unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec) / 1000; 
		double msf = (double)(msl-ms); 
		return msf;
	}
	public: 
		FinalRunner()
		{
			inds.pb(0); inds.pb(1); inds.pb(2);
			irf = new IR(0);
			irr = new IR(1);
			irlf = new IR(3);
			irlb = new IR(2);
			uirb = new mraa::Gpio(8);
			left = new Motor(0,2,4,false);
			right = new Motor(4,6,2,true);
			odo = new Odometry(left, right, 0, 0, 0);
			location = new Location(0.0,0.0,0.0);
			cap=new VideoCapture(0);
			assert (cap->isOpened());

			cap->read(in);
			downSize(in,test);
			Size outSize=Size(test.cols,test.rows);
			outVid=new VideoWriter("log.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
			recVid=new VideoWriter("rlog.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
			lostCount=0;

			upEnd=new mraa::Gpio(9);
			upEnd->dir(mraa::DIR_IN);
			rotEnd=new mraa::Gpio(8);
			rotEnd->dir(mraa::DIR_IN);
			cube=new mraa::Gpio(0);
			cube->dir(mraa::DIR_IN);
			midEnd=new mraa::Gpio(1);
			midEnd->dir(mraa::DIR_IN);

			motion = new Motion(left,right,odo,location); 
			base = new Motor(10,11,6,false); //base motor -counterclockwise when forward
			lift = new Motor(8,9,3,false); //lift motor

			wf= new Wallfollower(left,right,irf,irr,irlf,irlb,uirb,location);
			channel=1;

			lift->backward(); lift->setSpeed(2);
			while(upEnd->read() ) usleep(1000);
			lift->stop();

			ballSeen=foundBall=false;

			for(int i=0;i<10;i++) cap->read(test);

		}
		void setupChase()
		{
			motion->setMConstants(0.5,0.0,0.0);
			motion->setBaseSpeed(0.75);
			motion->straight(true);
		}
		bool chaseBall()
		{
			if(!senseBall(6))
			{
				if(lostCount==3) lostCount=0, ballSeen=false;
				else
				{
					lostCount++;
					goForward(-20);
				}
			}
			else
			{
				motion->run();
				//channel = wf->run_follower(channel);
				if(target!=0&&fabs(target)>8)
				{
					std::cout<<"I see a a ball"<<lastDist<<" away at "<<target<<"degrees\n";
					motion->rotate(target*3.14/180);
					while(running&&!motion->run()) cap->read(test);
					//goForward(lastDist+2);
					//channel = wf->run_follower(channel);
					setupChase();
					//motion->run();
				}
				if(lastDist<=20) 
				{
					goForward(20);
					if(cube->read() ) pickUp();
				}
			}
		}
		bool senseBall(int samps=1)
		{
			gettimeofday(&tv, NULL); 
			REP(i,samps) cap->read(in);
			std::cout << "Grabbed frame" << std::endl;
			downSize(in,frame); //downsized
			//recVid->write(frame);
			maxFilter(frame,inds);
			std::vector<centers> ret=fill(frame);
			//outVid->write(frame);
			target=0,lastDist=1000000;
			bool sensed=false;
			for(int j=0;j<ret.size();j++)
			{
				if(ret[j].type!=0&&ret[j].dist<lastDist)
				{
					lastDist=ret[j].dist;
					target=ret[j].angle;
					type=ret[j].type;
					std::cout<<type<<std::endl;
					sensed=true;
				}
			}
			std::cout<<"Camera shit is taking "<<timeDiff()<<std::endl;
			return sensed;
		}
		void goForward(double dist)
		{
			motion->setBaseSpeed(1);
			motion->straight(dist,false);
			motion->setMConstants(3,0.1,0.15);
			while(running&&!motion->run()) 
			{
				if(dist>0&&irf->getDistance()<wThresh) break;
				if(dist<0&&uirb->read()) break;
				cap->read(in);
			}
			left->stop();
			right->stop();
			usleep(200000);	
		}
		void pickUp()
		{
			base->forward();
			base->setSpeed(2);
			while(rotEnd->read()) usleep(1000);

			base->backward() ;
			base->setSpeed(0.5);
			while(midEnd->read()) 
			{
				std::cout<<midEnd<<" "<<rotEnd<<std::endl;
				usleep(1000);
			}
			base->stop();
			usleep(100000);

			Servo claw(15);

			claw.write(0.2);

			lift->forward();
			lift->turnAngle(1700,5);
			usleep(200000);	

			claw.write(0.8);
			usleep(200000);	

			lift->turnAngle(215,5);
			usleep(200000);	

			claw.write(0.15);
			usleep(200000);	

			lift->backward(); lift->setSpeed(3);
			while(upEnd->read() ) usleep(1000);
			lift->stop();

			usleep(200000);	

			if(type==1) base->forward();
			else base->backward();
			base->setSpeed(2);
			while(rotEnd->read()) usleep(1000);
			base->setSpeed(1);

			lift->forward();
			lift->turnAngle(50,5);

			claw.write(0.7);
			base->stop();
			usleep(200000);	



			lift->backward(); lift->setSpeed(2);
			while(upEnd->read() ) usleep(1000);
			lift->stop();
			claw.release();
			usleep(200000);	

			if(type==1) base->backward();
			else base->forward();
			base->setSpeed(0.5);
			while(midEnd->read()) usleep(1000);
			base->stop();
			usleep(100000);
			channel=1;
		}
		void run()
	   	{
			if(channel<6)
			{
				channel = wf->run_follower(channel);
				if(channel==3)
				{
					if(senseBall()) channel=7;
				}
			}
			else if(channel==7) //seen ball 
			{
				chaseBall();
			}
		}
		void stop()
		{
			left->stop(); right->stop();
			sleep(1);
		}
};

int main()
{
	signal(SIGINT,sig_handler);
	//motion->rotate(1.57);	


	FinalRunner* fr=new FinalRunner();

	fr->pickUp();

	//fr->goForward(20);
	//fr->goForward(-20);

	while(running)
	{
		fr->run();
	}

	fr->stop();

	return 0;
}
