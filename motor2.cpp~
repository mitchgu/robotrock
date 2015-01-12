#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>


class Motor
{
	mraa::Gpio* dir;
	mraa::Pwm* pwm;
	mraa::Gpio* hall;
	bool side; //0 -left or 1-right
	float speed;
public:
	Motor(int dpin, int ppin, int hpin, bool _side)
	{
		dir=new mraa::Gpio(dpin);
		pwm=new mraa::Pwm(ppin);
		hall=new mraa::Gpio(hpin);
		pwm->enable(true);
		dir->dir(mraa::DIR_OUT);
		hall->dir(mraa::DIR_IN);
		side=_side;
		std::cout<<"set at "<<side<<std::endl;
		speed=0;
	}	
	void forward()
	{
		if(side) dir->write(1);
		else dir->write(0);
	}
	void backward()
	{
		if(side) dir->write(0);
		else dir->write(1);
	}
	void setSpeed(float set)
	{
		speed=set;
		pwm->write(speed);
	}
	float getSpeed() { return speed; }

	/*
	get the real speed
	*/
	float rps()
	{
		struct timeval tv;
		int reading = hall->read();
		while(reading == hall->read())
		{
			usleep(100);
		}
		gettimeofday(&tv,NULL);
		unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		int count=0;
		reading = hall->read();
		while(count<=10) 
		{
			if (reading != hall->read())
			{
				count++;
				reading = hall->read();
			}
			usleep(100);
		}
		gettimeofday(&tv,NULL);
		unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		ms = msl - ms;
		float msf = (float)ms;
		return (1/((msf/10000)*2))/1920;
	}
};


int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
};

int main()
{
	signal(SIGINT, sig_handler);
	Motor left(12,5,4,false),right(8,3,2,true);
	Gyroscope gyr(10);

	const float target=0, K=0.003, base=0.1;

	left.forward();
	right.forward();

	left.setSpeed(base);
	right.setSpeed(base);

	
	while(running) 
	{
		/*
		float angle=gyr.run();
		float diff=(angle-target)*K;
		std::cout<<angle<<" "<<diff<<std::endl;
		left.setSpeed(base);
		right.setSpeed(base);
		if(diff>0) left.setSpeed(base+diff);
		else right.setSpeed(base-diff);
		sleep(0.01);
		*/
		float realspeed = left.rps();
		std::cout<<realspeed<<" "<<std::endl;
	}

	left.setSpeed(0);
	right.setSpeed(0);
    	sleep(1);
    	return 0;
}
