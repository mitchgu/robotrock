#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>
#include "gyro.cpp"
#include "odometry.cpp"


class Motion
{
	Motor *l,*r;
	Gyroscope* gyr;
	Odometry* odo;
	Location* current;
	double currentAngle;
	double targetAngle,moveDistance;
	const double Kp=0.001, Ki=0, Kd=0;
	bool rotating;
public:
	Motion( Motor* _l, Motor* _r, Gyroscope* _gyr, Location* _start) 
	{
		l = _l;
		r = _r;
		gyr = _gyr;
		float n = gyr->run();
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		current= _start;
		currentAngle = _start->theta();
		rotating=false;
		angleError=moveError=0;
	}
	void run()
	{
	}
	// rotate clockwise means angle > 0
	void rotate( float angle) 
	{
		currentAngle = gyr->run();
		targetAngle = currentangle + angle;
		if(angle>0)
		{
			l->forward();
			r->backward();
		}
		while (!((targetangle - currentangle) >-0.001 && (targetangle - currentangle) <0.001)) {
			currentangle = gyr->run();
			float speed = (targetangle-currentangle)*5;
			l->setSpeed(speed);
			r->setSpeed(speed);
			current = odo->run();
		}
		currentangle = gyr->run();
		l->setSpeed(0);
		r->setSpeed(0);
	}
	// forward distance straight
	void forward(float distance) 
	{
		float nowdistance = 0;
		float targetangle = gyr->run();
		float deltaangle = 0;
		float lspeed;
		float rspeed;
		float a_lspeed;
		float a_rspeed;
		struct timeval tv;
		l->forward();
		r->forward();
		gettimeofday(&tv,NULL);
		currentangle = gyr->run();
		while(!((distance - nowdistance)>-0.1 && (distance - nowdistance)<0.1)) {	
			deltaangle = currentangle-targetangle;
			lspeed = (distance - nowdistance)*0.5;
			rspeed = (distance - nowdistance)*0.5;
			lspeed = lspeed - deltaangle*0.1;
			rspeed = rspeed + deltaangle*0.1;
			if (lspeed > 6 || rspeed>6) {
				a_lspeed = 6;
				a_rspeed = 6*rspeed/lspeed;
			}
			else {
				a_lspeed = lspeed;
				a_rspeed = rspeed;
			}
			l->setSpeed(a_lspeed);
			r->setSpeed(a_rspeed);
			float _speed = ((l->rps()+r->rps())/2)*12.095;
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
			nowdistance = nowdistance+float((msl-ms))*_speed/1000;
			current = odo->run();
			currentangle = gyr->run();
		}
	}
	Location* getLocation() {
		return current;
	}
	float getAngle() {
		return currentangle;
	}
};
