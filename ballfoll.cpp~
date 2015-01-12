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

#include "gyro.cpp"
#include "motor.cpp"
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
	Motor left(8,3,4,false),right(9,5,2,true);
	Gyroscope gyr;

	const float K=0.001, base=0.1;
	float target;
	int neg;
	std::cin>>neg;
	left.forward();
	right.forward();

	left.setSpeed(base);
	right.setSpeed(base);
	VideoCapture cap(0);
	assert(cap.isOpened());

	while(running) 
	{
		Mat in;
		cap >> in;
		std::cout << "Grabbed frame" << std::endl;

		Mat frame;
		downSize(in,frame); //downsized

		Mat out;
		maxFilter(frame,2);
		if(neg) target=-fill(frame,2);
		else target=fill(frame,2);
		float angle=gyr.run();
		float diff=(target)*K;
		if(target==0) diff=(angle-target)*K;
		else gyr.reset();
	

		std::cout<<target<<" "<<angle<<" "<<diff<<std::endl;
		left.setSpeed(base);
		right.setSpeed(base);
		if(diff>0) left.setSpeed(base+diff);
		else right.setSpeed(base-diff);
		sleep(0.9);
	}

	left.setSpeed(0);
	right.setSpeed(0);
    	sleep(1);
    	return 0;
}
