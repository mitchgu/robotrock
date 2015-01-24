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

#define MX 10
#define LEFT false
#define RIGHT true

struct timeval start,end;

double dt;
int count;

//const double sKp=0.04, sKi=0.000005, sKd=0.0005;
double sKp=0.12, sKi=0.000001, sKd=0.0003;

void tickCounter(void* args)
{
	//std::cout<<"falling"<<std::endl;
	count++;
}

void edge_handler(void* args)
{
	//std::cout<<"falling"<<std::endl;
	if(count==0) gettimeofday(&start, NULL);
	if(count==MX)
	{
		gettimeofday(&end, NULL);
		int diffSec = end.tv_sec - start.tv_sec;
		int diffUSec = end.tv_usec - start.tv_usec;
		dt = (double)diffSec + 0.000001*diffUSec;
	}
	count++;
}

class Motor
{
	mraa::I2c* i2c;
	int dpin, ppin;
	mraa::Gpio* hall;
	bool side; //0 -left or 1-right
	double targetRPS,integ,volt,prev;
	bool forw;
	struct timeval tv;
	long long timeDiff()
	{
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
			return msl-ms;
	}
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
		prev=targetRPS=volt=integ=0;
		gettimeofday(&tv, NULL);
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
		volt=targetRPS=0;
		writePWM(i2c, ppin, 0);
	}
	void writeVolt(double _volt)
	{
		volt=_volt;
		volt=std::min(1.0,volt);
		volt=std::max(0.0,volt);
		//std::cout<<"volt writing "<<volt<<std::endl;
		writePWM(i2c, ppin, volt);
	}
	double run()
	{
		double dt=timeDiff();
		double e=targetRPS-rps();
		//std::cout<<"error is"<<e<<std::endl;
		double deriv=(e-prev)/dt;
		integ+=e*dt;
		prev=e;
		double dx=sKp*e+sKi*integ+sKd*deriv;
		writeVolt(volt+dx);
		return volt;
	}
	void setTarget(double inc)
	{
		targetRPS=inc;
	}
	void setSpeed(double set)
	{
		targetRPS=set;
		if(side==LEFT) writeVolt(0.1489*set+0.0367);
		else writeVolt(0.1568*set+0.0375);
	}
	float getSpeed() { return targetRPS; }
	/*
	get the real speed
	*/
	void turnTicks(int ticks, double speed)
	{
		count=0;
		setSpeed(speed);
		hall->isr(mraa::EDGE_RISING, tickCounter, hall);
		while(count<=ticks)
		{
			//std::cout<<"ticks "<<ticks<<" count "<<count<<std::endl;
			usleep(100);
		}
		hall->isrExit();
		stop();
	}
	void turnAngle(int angle, double speed)
	{
		int ticks=(angle/360.0)*480.0;
		turnTicks(ticks,speed);
	}
	double rps()
	{
		count=0; int its=0;
		hall->isr(mraa::EDGE_RISING, edge_handler, hall);
		while(count<=MX&&(++its)<500) usleep(100);
		if(its==500) return 0;
		hall->isrExit();
		//std::cout<<"delay "<<dt<<std::endl;
		double out=4*MX/(1920.0*dt);
		std::cout<<"Rps is "<<out<<std::endl;
		return out;
	}
	double srps()
	{
		if(forw) return rps();
		else return -rps();
	}
};


