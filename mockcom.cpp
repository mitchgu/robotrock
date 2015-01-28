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

const float run_time = 1.5;
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
	mraa::Gpio* upEnd,*rotEnd,*uirb;
	Motion* motion;
	VideoCapture* cap;
	Mat in,test,frame;
	std::vector<int> inds; 
	double lastDist, target;
	bool ballSeen,foundBall; int type,channel;
	Wallfollower* wf;
	IR *irf, *irr, *irlf, *irlb;
	struct timeval rtv; float run_base_time; bool initialized;
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

			upEnd=new mraa::Gpio(9);
			upEnd->dir(mraa::DIR_IN);
			rotEnd=new mraa::Gpio(8);
			rotEnd->dir(mraa::DIR_IN);

			motion = new Motion(left,right,odo,location); 
			base = new Motor(10,11,6,false); //base motor -counterclockwise when forward
			lift = new Motor(8,9,3,false); //lift motor

			wf= new Wallfollower(left,right,irf,irr,irlf,irlb,uirb,location);
			channel=1;

			lift->backward(); lift->setSpeed(2);
			while(upEnd->read() ) usleep(1000);
			lift->stop();

			ballSeen=foundBall=false;
			initialized = false;
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
			if(!senseBall())
			{
				goForward(-12);
				setupChase();
			}
			else
			{
				motion->run();
				if(target!=0&&fabs(target)>8)
				{
					motion->rotate(target*3.14/180);
					while(running&&!motion->run());
					motion->straight(true);
				}
				if(lastDist<=20) 
				{
					foundBall=true;
					goForward(20);
					pickUp();
				}
			}
		}
		bool senseBall()
		{
			REP(i,6) cap->read(in);
			std::cout << "Grabbed frame" << std::endl;
			downSize(in,frame); //downsized
			//recVid<<frame;
			maxFilter(frame,inds);
			std::vector<centers> ret=fill(frame);
			target=0,lastDist=1000000;
			bool sensed=false;
			for(int j=0;j<ret.size();j++)
			{
				if(ret[j].type!=0&&ret[j].dist<lastDist)
				{
					lastDist=ret[j].dist;
					target=ret[j].angle;
					type=ret[j].type;
					sensed=true;
				}
			}
			return sensed;
		}
		void goForward(double dist)
		{
			motion->setBaseSpeed(1);
			motion->straight(dist,true);
			motion->setMConstants(3,0.1,0.15);
			while(running&&!motion->run() ) usleep(10000);
			left->stop();
			right->stop();
			usleep(200000);	
		}
		void pickUp()
		{
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

			lift->backward(); lift->setSpeed(2);
			while(upEnd->read() ) usleep(1000);
			lift->stop();

			usleep(200000);	

			if(type==1) base->forward();
			else base->backward();
			base->setSpeed(2);
			while(rotEnd->read()) usleep(1000);
			base->stop();
			usleep(200000);	

			lift->forward();
			lift->turnAngle(75,5);

			claw.write(0.7);
			usleep(200000);	


			lift->backward(); lift->setSpeed(2);
			while(upEnd->read() ) usleep(1000);
			lift->stop();
			claw.release();
			usleep(200000);	

			if(type==1) base->backward();
			else base->forward();
			base->turnAngle(92,3);

			ballSeen=foundBall=false;
		}
		void run()
	   	{
			if(!ballSeen) 
			{
				channel = wf->run_follower(channel);
				if (channel == 3) {
					gettimeofday(&rtv,NULL);
					if(!initialized) {
						run_base_time = (unsigned long long)(rtv.tv_sec)*1000 +
						(unsigned long long)(rtv.tv_usec) / 1000;
						initialized = true;
					}
					else {
						if((((unsigned long long)(rtv.tv_sec)*1000 +
							(unsigned long long)(rtv.tv_usec) / 1000)-run_base_time)> run_time*1000) {
							run_base_time = (unsigned long long)(rtv.tv_sec)*1000 +
								(unsigned long long)(rtv.tv_usec) / 1000;
							if(senseBall()) {
								ballSeen=true;
								setupChase();
							}
						}
					}
				}
				else{
					initialized = false;
				}
			}
			else if(!foundBall)
			{
				channel=1;
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

	//downSize(in,test);

	//Size outSize=Size(test.cols,test.rows);

	//VideoWriter outVid("log.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
	//VideoWriter recVid("rlog.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);

	FinalRunner* fr=new FinalRunner();

	fr->goForward(20);
	fr->goForward(-20);

	while(running)
	{
		fr->run();
	}

	fr->stop();

	return 0;
}
