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
	double intError,prevError;
	const double Kp=0.001, Ki=0, Kd=0;
	const double mKp=0.001, mKi=0, mKd=0;
	struct timeval tv;
	bool rotating;
	void cw()
	{
			l->forward();
			r->backward();
	}
	void ccw()
	{
			r->forward();
			l->backward();
	}	
	long long timeDiff()
	{
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
				(unsigned long long)(tv.tv_usec) / 1000;
	}
	void rotPID()
	{
			currentAngle = gyr->run();
			double error=(targetAngle-currentAngle);
			long long td=timeDiff();
			intError+=td*error;
			double diffError=(error-prevError)/(td);
			double speed = l->getSpeed()+(error*Kp+intError*Ki+diffError*Kd);
			if(speed>0) cw();
			else { ccw(); speed=-speed;}
			l->setSpeed(speed); r->setSpeed(speed);
			current = odo->run();
	}
	void movPID()
	{
			currentAngle = gyr->run();
			current = odo->run();
			long long td=timeDiff();

			if(moveDistance<0.01) { l->stop(); r->stop(); return;} //change with move pid later

			double error=(targetAngle-currentAngle);
			intError+=td*error;
			double diffError=(error-prevError)/(td);
			double diff = (error*mKp+intError*mKi+diffError*mKd);
			double baseSpeed=1.5;
			l->setSpeed(baseSpeed+diff);
			r->setSpeed(baseSpeed-diff);
			float _speed = ((l->rps()+r->rps())/2)*12.095;
			moveDistance = moveDistance-td*_speed/1000;
	}
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
		if(rotating) rotPID();
		else movPID();
	}
	// rotate clockwise means angle > 0
	void rotate( double angle) 
	{
		currentAngle = gyr->run();
		targetAngle = currentangle + angle;
		moveDistance=0;
		l->stop(); r->stop();
		rotating=true;
		gettimeofday(&tv, NULL);
		intError=0,prevError=angle;
	}
	// forward distance straight
	void straight(double distance) 
	{
		rotating=false;
		currentAngle=targetAngle = gyr->run();
		l->stop(); r->stop();
		if(distance>0)  { l->forward(); r->forward(); }
		moveDistance=std::fabs(distance);
		else  { l->backward(); r->backward(); }
		gettimeofday(&tv,NULL);
		intError=0,prevError=0;
	}
	Location* getLocation() {
		return current;
	}
	double getAngle() {
		return currentangle;
	}
};
