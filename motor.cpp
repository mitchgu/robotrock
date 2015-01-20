#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>
#include "data.cpp"
#include "i2c.cpp"

struct timeval start,end;

double dt;

void edge_handler(void* args)
{
	mraa::Gpio* hall=(mraa::Gpio*) args;
	if(hall->read()) //rising
	{
		std::cout<<"rising"<<std::endl;
		gettimeofday(&start, NULL);
	}
	else
	{
		std::cout<<"falling"<<std::endl;
		gettimeofday(&end, NULL);
		int diffSec = end.tv_sec - start.tv_sec;
		int diffUSec = end.tv_usec - start.tv_usec;
		dt = (double)diffSec + 0.000001*diffUSec;
	}
}

class Motor
{
	mraa::I2c* i2c;
	int dpin;
	//mraa::Gpio* dir;
	int ppin;
	mraa::Gpio* hall;
	bool side; //0 -left or 1-right
	double speed,volt;
	bool forw;
public:
	Motor(int _dpin, int _ppin, int _hpin, bool _side)
	{
		i2c = new mraa::I2c(6);
		initPWM(i2c);
		ppin = _ppin;
		dpin = _dpin;
		//dir=new mraa::Gpio(_dpin);
		//dir->dir(mraa::DIR_OUT);
		hall= new mraa::Gpio(_hpin);
		hall->dir(mraa::DIR_IN);
		side=_side;
		speed=0;
	}	
	void forward()
	{
		forw = true;
		if(side) writePWM(i2c, dpin, 0);
		else writePWM(i2c, dpin, 1);
		/*
		if(side) dir->write(0);
		else dir->write(1);
		*/
	}
	void backward()
	{
		forw = false;
		if(side) writePWM(i2c, dpin, 1);
		else writePWM(i2c, dpin, 0);
		/*
		if(side) dir->write(1);
		else dir->write(0);
		*/
	}
	void stop() 
	{
		volt=speed=0;
		writePWM(i2c, ppin, 0);
	}
	void setSpeed(double set)
	{
		std::cout<<"i want to set the speed! :D"<<std::endl;
			
		float _set = set;
	/*	if (_set<0) {
			_set = -_set;
			if (forw && side) {
				writePWM(i2c, dpin, 1);
			}
			if (forw && !side) {
				writePWM(i2c, dpin, 0);
			}
			if (!forw && side) {
				writePWM(i2c, dpin, 0);
			}
			if (!forw && !side) {
				writePWM(i2c, dpin, 1);
			}
		}*/
		std::cout<<"speed:  "<<_set<<std::endl;
		volt=0.010624*_set+0.01136;
		speed=_set;
		volt=std::min(1.0,volt);
		volt=std::max(0.0,volt);
		writePWM(i2c, ppin, volt);
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
	double rpsi() 
	{
		hall->isr(mraa::EDGE_BOTH, edge_handler, hall);
		usleep(1000);
		hall->isrExit();
		return dt;
	}
};


