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

const double Kp=20, Ki=8, Kd=10;
const double mKp=10, mKi=0, mKd=0;

class Motion
{
	Motor *l,*r;
	Gyroscope* gyr;
	Odometry* odo;
	Location* current;
	double currentAngle;
	double targetAngle,moveDistance;
	double intError,prevError;
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
			return msl-ms;
	}
	bool rotPID()
	{
			currentAngle = gyr->run();
			double error=(targetAngle-currentAngle);
			current = odo->run();
			long long td=timeDiff();

			if(error<0.1) { l->stop(); r->stop(); return true;} //change with move pid later
			intError+=td*error/1000;
			double diffError=(error-prevError)/(td);
			double speed = (error*Kp+intError*Ki+diffError*Kd);
			/*std::cout<<"getting speed "<<l->getSpeed()<<std::endl;
			std::cout<<"setting error "<<error<<std::endl;
			std::cout<<"setting integral "<<intError<<std::endl;
			std::cout<<"setting speed "<<speed<<std::endl;*/
			if(speed>0) cw();
			else { ccw(); speed=-speed;}
			l->setSpeed(speed); r->setSpeed(speed);
			return false;
	}
	bool movPID()
	{
			currentAngle = gyr->run();
			current = odo->run();
			long long td=timeDiff();

			if(moveDistance<0.01) { l->stop(); r->stop(); return true;} //change with move pid later
			std::cout<<moveDistance<<std::endl;

			double error=(targetAngle-currentAngle);
			intError+=td*error/1000;
			double diffError=(error-prevError)/(td);
			double diff = (error*mKp+intError*mKi+diffError*mKd);
			double baseSpeed=8;
			l->setSpeed(baseSpeed+diff);
			r->setSpeed(baseSpeed-diff);
			float _speed = ((l->rps()+r->rps())/2)*12.095;
			moveDistance = moveDistance-td*_speed/1000;
			return false;
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
		targetAngle=currentAngle = _start->theta();
		intError=moveDistance=0;
		rotating=false;
	}
	bool run()
	{
		if(rotating) return rotPID();
		else return movPID();
	}
	// rotate clockwise means angle > 0
	void rotate( double angle) 
	{
		currentAngle = gyr->run();
		targetAngle = currentAngle + angle;
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
		else  { l->backward(); r->backward(); }
		moveDistance=fabs(distance);
		gettimeofday(&tv,NULL);
		intError=0,prevError=0;
	}
	Location* getLocation() {
		return current;
	}
	double getAngle() {
		return currentAngle;
	}
};
