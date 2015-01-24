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

#include "motion.cpp"
#include "servo.cpp"
#include "cv.cpp"

int running=1;

void sig_handler(int signo)
{
	if (signo == SIGINT) {
		printf("closing PWM nicely\n");
		running = 0;
	}
}

int main()
{
	signal(SIGINT,sig_handler);
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Odometry* odo = new Odometry(left, right, 0, 0, 0);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,odo,location); 
	//motion->rotate(1.57);	
	const float K=10;
	float target;
	VideoCapture cap(0);
	assert(cap.isOpened());

	Mat in,test,frame;

	cap>>in;

	//downSize(in,test);

	//Size outSize=Size(test.cols,test.rows);

	//VideoWriter outVid("log.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
	//VideoWriter recVid("rlog.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
	for(int i=0;i<10;i++) cap >> test;

	motion->straight(10000);
	while(running)
	{
		motion->run();
		for(int i=0;i<6;i++) cap >> in;
		std::cout << "Grabbed frame" << std::endl;

		downSize(in,frame); //downsized

		//recVid<<frame;

		maxFilter(frame,1);
		pdd ret=fill(frame,1);
		double target=ret.second;

		//outVid<<frame;

		if(target!=0&&fabs(target)>5)
		{
			left->stop(); right->stop();
			std::cout<<target<<" angle"<<std::endl;
			motion->rotate(target*3.14/180);
			while(running&&!motion->run()) usleep(1000);
			sleep(0.1);
			motion->straight(10000);
		}
		if(ret.first<22) running=0;
		usleep(10000);
	}

	Motor* base = new Motor(10,11,6,false); //base motor -counterclockwise when forward
	Motor* lift = new Motor(8,9,3,false); //lift motor
	left->stop(); right->stop();
	sleep(1);

	running=1;

	motion->setBaseSpeed(0.5);
	motion->straight(12);
	while(running&&!motion->run() ) usleep(10000);
	left->stop();
	right->stop();
	usleep(200000);	
	sleep(1);

	if(running)
	{
		Servo claw(15);

		claw.write(0.7);

		lift->forward();
		lift->turnAngle(1700,5);
		usleep(200000);	

		claw.write(0.8);
		usleep(200000);	

		lift->turnAngle(215,5);
		usleep(200000);	

		claw.write(0.15);
		usleep(200000);	

		lift->backward();
		lift->turnAngle(1950,5);

		usleep(200000);	

		base->forward();
		base->turnAngle(92,3);

		usleep(200000);	

		lift->forward();
		lift->turnAngle(75,5);

		claw.write(0.7);
		usleep(200000);	


		lift->backward();
		lift->turnAngle(75,5);

		claw.release();
		usleep(200000);	
		base->backward();
		base->turnAngle(92,3);
	}

	left->stop(); right->stop();
	sleep(1);
	return 0;
}
