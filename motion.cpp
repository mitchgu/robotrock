#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>

#include "odometry.cpp"

//const double Kp=0.5, Ki=0.15, Kd=0.2;
double Kp,Ki,Kd;
double bKp=0.6, bKi=0.0, bKd=0.05;
double smKp=0.4, smKi=0.0, smKd=0.05;
//double smKp=0.4, smKi=0.0, smKd=0.05; THIS WORKS PRETTY WELL
const double mKp=0.05, mKi=0.01, mKd=0.005;

class Motion
{
protected:
	Motor *l,*r;
	Odometry* odo;
	Location* current;
	double currentAngle;
	double baseSpeed;
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
		std::cout<<"rotating"<<std::endl;
			current = odo->run();
			currentAngle = odo->getAngle();
			double error=(targetAngle-currentAngle);
			long long td=timeDiff();
			if(fabs(error)<0.02)
			{ 
				//std::cout<<"TURNT!\n";
				l->stop(); r->stop();
				//std::cout<<"setting error "<<error<<std::endl;
				return true;
			} //change with move pid later
			intError+=td*error/1000;
			double diffError=(error-prevError)/(td);
			double speed = (error*Kp+intError*Ki+diffError*Kd);
			//std::cout<<"setting error "<<error<<std::endl;
			//std::cout<<"setting speed "<<speed<<std::endl;
			if(speed>0) cw();
			else { ccw(); speed=-speed;}
			l->setTarget(speed); r->setTarget(speed);
			l->run(); r->run();
			return false;
	}
	bool movPID()
	{
		std::cout<<"moving"<<std::endl;
		current = odo->run();
		currentAngle=odo->getAngle();
		long long td=timeDiff();
		if(moveDistance<0.01) { l->stop(); r->stop(); return true;} //change with move pid later

		double error=(targetAngle-currentAngle);
		intError+=td*error/1000;
		double diffError=(error-prevError)/(td);
		double diff = (error*mKp+intError*mKi+diffError*mKd);
		//diff=0;
		std::cout<<"diff is: "<<diff<<std::endl;
		l->setTarget(baseSpeed+diff);
		r->setTarget(baseSpeed-diff);
		l->run(); r->run();
		float _speed = ((l->rps()+r->rps())/2)*12.095;
		std::cout<<_speed<<" speed "<<std::endl;
		moveDistance = moveDistance-td*_speed/1000; return false;
	}
public:
	Motion( Motor* _l, Motor* _r,Odometry* _odo, Location* _start) 
	{
		l = _l;
		r = _r;
		odo = _odo;
		current= odo->run();
		targetAngle=currentAngle = _start->theta();
		intError=moveDistance=0;
		rotating=false;
		baseSpeed=1.5;
	}
	bool run()
	{
		if(rotating) return rotPID();
		else return movPID();
	}
	// rotate clockwise means angle > 0
	void rotate( double angle) 
	{
		std::cout<<"ROTATING WITH "<<angle<<std::endl;
		l->stop(); r->stop();
		sleep(0.1);
		current = odo->run();
		currentAngle=odo->getAngle();
		targetAngle = currentAngle + angle;
		moveDistance=0;
		rotating=true;
		gettimeofday(&tv, NULL);
		if(angle<=0.8) Kp=smKp,Ki=smKi,Kd=smKd;
		else Kp=bKp,Ki=bKi,Kd=bKd;
		intError=0,prevError=angle;
	}
	// forward distance straight
	void straight(double distance) 
	{
		l->stop(); r->stop();
		sleep(0.1);
		rotating=false;
		if(distance>0)  { l->forward(); r->forward(); }
		else  { l->backward(); r->backward(); }
		moveDistance=fabs(distance);
		gettimeofday(&tv,NULL);
		intError=0,prevError=0;
		current = odo->run();
		currentAngle=targetAngle = odo->getAngle();
		l->setSpeed(baseSpeed);
		r->setSpeed(baseSpeed);
	}
	Location* getLocation() {
		return current;
	}
	double getAngle() {
		return currentAngle;
	}
	void adjustAngle(double angle)
	{
		currentAngle+=angle;
	}
};
