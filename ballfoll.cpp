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
	Motor* left = new Motor(8,6,4,false);
	Motor* right = new Motor(9,5,2,true);
	Gyroscope* gyr = new Gyroscope(10);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,gyr,location); 
	//motion->rotate(1.57);	
	const float K=10;
	float target;
	VideoCapture cap(0);
	assert(cap.isOpened());

	motion->straight(10000);

	while(running)
	{
		motion->run();
		Mat in;
		cap >> in;
		std::cout << "Grabbed frame" << std::endl;

		Mat frame;
		downSize(in,frame); //downsized

		Mat out;
		maxFilter(frame,2);
		target=fill(frame,2)/3;

		if(target!=0&&fabs(target)>5)
		{
			std::cout<<target<<" angle"<<std::endl;
			motion->rotate(target*3.14/180);
			while(!motion->run()) usleep(10000);
			sleep(0.1);
			motion->straight(10000);
		}
		usleep(10000);
	}

	left->stop(); right->stop();
	sleep(1);
    	return 0;
}
