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

class Motion
{
protected:
	double Kp,Ki,Kd;
	double bKp, bKi, bKd;
	double smKp, smKi, smKd;
	//double smKp=0.4, smKi=0.0, smKd=0.05; THIS WORKS PRETTY WELL
	double mKp, mKi, mKd;
	Motor *l,*r;
	Odometry* odo;
	Location* current;
	double currentAngle;
	double baseSpeed;
	double targetAngle,moveDistance;
	double intError,prevError;
	struct timeval tv;
	bool rotating,usePID,target;
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
		odo->run();
		currentAngle = odo->getAngle();
		double error=(targetAngle-currentAngle);
		long long td=timeDiff();
		if(fabs(error)<0.05)
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
		l->setSpeed(speed); r->setSpeed(speed);
		//l->run(); r->run();
		return false;
	}
	bool movPID()
	{
		odo->run();
		currentAngle=odo->getAngle();
		long long td=timeDiff();
		if(moveDistance<0.01) { l->stop(); r->stop(); return true;} //change with move pid later

		double error=(targetAngle-currentAngle);
		//std::cout<<"erorr "<<error<<std::endl;
		intError+=td*error/1000;
		double diffError=(error-prevError)/(td);
		double diff = (error*mKp+intError*mKi+diffError*mKd);
		if(!l->forw) diff=-diff;
		//diff=0;
		if(usePID)
		{
			l->setTarget(baseSpeed+diff); r->setTarget(baseSpeed-diff);
			l->run(); r->run();
		}
		else
		{
			l->setSpeed(baseSpeed+diff); r->setSpeed(baseSpeed-diff);
		}
		if(target)
		{
			double _speed = ((l->rps()+r->rps())/2)*12.095;
			moveDistance = moveDistance-td*_speed/1000; return false;
		}
	}
public:
	Motion( Motor* _l, Motor* _r,Odometry* _odo, Location* _current) 
	{
		bKp=2, bKi=0, bKd=0;
		smKp=3.6, smKi=0.03, smKd=1.6;
		mKp=3, mKi=0.1, mKd=0.15;
		l = _l;
		r = _r;
		odo = _odo;
		current=_current;
		targetAngle=currentAngle = current->theta();
		intError=moveDistance=0;
		rotating=false, usePID=false;
		baseSpeed=0.5;
	}
	void setBaseSpeed(double set)
	{
		baseSpeed=set;
	}
	bool run()
	{
		if(rotating) return rotPID();
		else return movPID();
	}
	// rotate clockwise means angle > 0
	void rotate( double angle) 
	{
		l->stop(); r->stop();
		usleep(100000);
		odo->run();
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
	void straight(double distance, bool useP=false)
	{
		l->stop(); r->stop();
		usleep(100000);
		rotating=false;
		usePID=useP;
		if(distance>0)  { l->forward(); r->forward(); }
		else  { l->backward(); r->backward(); }
		moveDistance=fabs(distance);
		gettimeofday(&tv,NULL);
		intError=0,prevError=0;
		odo->run();
		currentAngle=targetAngle = odo->getAngle();
		r->setSpeed(baseSpeed); l->setSpeed(baseSpeed);
		target=true;
	}
	void straight(bool useP=false)
	{
		straight(1000,useP);
		target=false;
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
	void setMConstants(double _Kp,double _Ki, double _Kd)
	{
		mKp=_Kp;
		mKi=_Ki;
		mKd=_Kd;
	}
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
};
