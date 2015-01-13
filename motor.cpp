#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>
#include "data.cpp"


class Motor
{
	mraa::Gpio* dir;
	mraa::Pwm* pwm;
	mraa::Gpio* hall;
	bool side; //0 -left or 1-right
	double speed,volt;
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
		speed=0;
	}	
	void forward()
	{
		if(side) dir->write(0);
		else dir->write(1);
	}
	void backward()
	{
		if(side) dir->write(1);
		else dir->write(0);
	}
	void stop() 
	{
		volt=speed=0;
		pwm->write(volt);
	}
	void setSpeed(double set)
	{
		volt=0.010624*set+0.01136;
		speed=set;
		pwm->write(volt);
	}
	float getSpeed() { return speed; }

	/*
	get the real speed
	*/
	double rps()
	{
		if(volt<0.02) return 0;
		struct timeval tv;
		int reading = hall->read();
		int its=0;
		while(reading == hall->read()&&its<500)
		{
			usleep(10);
			its++;
		}
		if(its==500) return 0;
		gettimeofday(&tv,NULL);
		unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		int count=0;
		reading = hall->read();
		while(count<=5) 
		{
			its++;
			if (reading != hall->read())
			{
				count++;
				reading = hall->read();
				its=0;
			}
			if(its==500) return 0;
			usleep(10);
		}
		gettimeofday(&tv,NULL);
		unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		ms = msl - ms;
		std::cout<<ms<<" "<<std::endl;
		float msf = (float)ms;
		return 5.0072*((1/((msf/5000)*2))/1920)-0.0157;
	}
};


