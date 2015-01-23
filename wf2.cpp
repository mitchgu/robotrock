#include "motion.cpp"
#include <math.h>
#include <iostream>
#include "shortIR.cpp"

#define PAR 1
#define SMC 2
#define SMC_align 22
#define BGC_align 33
#define BGC 3
#define LOST 0

double wrKp=0.4, wrKi=0.0, wrKd=0.00;
const double wKp=1;
const double wfSpeed=1;
const double frontThresh=8;
const double sideThresh=5;
const double alignThresh=0.2;

class Wallfollower {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	mraa::Gpio* uirb;
	//IR* irb;
	Motor* left;
	Motor* right;
	Odometry* odo;
	Location* start;
	Location* current;
	Motion* motion;
	int mode;
	bool nearWalls()
	{
		return (irlf->getDistance()<sideThresh)||(irlb->getDistance()<sideThresh)||(irf->getDistance()<frontThresh);
	}

	void parSetup()
	{
		mode=PAR;
	}
	void smcSetup()
	{
		mode=SMC;
		motion->rotate(1.57);
	}
	void smcAlignSetup()
	{
		mode=SMC_align;
		left->stop(); right->stop();
		usleep(100000);
		//current = odo->run();
	}
	void bgcAlignSetup()
	{
		mode=BGC_align;
	}
	void bgcSetup()
	{
		mode=BGC;
		left->stop(); right->stop();
	}
	void lostSetup()
	{
		mode=LOST;
		motion->straight(10000);
	}
	void bgcRun()
	{
		//motion->baseSpeed
	}
	void lostRun()
	{
		if(nearWalls()) smcSetup();
		else motion->run();
	}
	void parRun()
	{
		double f=irlf->getDistance(), b=irlb->getDistance();
		double e=f-b;
		double ds=Kp*e;
		right->setTarget(wfSpeed+ds);
		left->setTarget(wfSpeed-ds);
		left->run(); right->run();
		if(irlf->getDistance()>20) bgcSetup();
		else if(irf->getDistance()<frontThresh) smcSetup();
		else if(!nearWalls()) lostSetup();
	}
	void smcRun()
	{
		if(motion->run()) smcAlignSetup();
	}
	void smcAlign()
	{
		double f=irlf->getDistance(),b=irlb->getDistance();
		std::cout<<"front "<<f<<" back "<<b<<std::endl;
		double e=fabs(f-b);
		double speed = e*wrKp;
		if(speed>0) motion->ccw();
		else { motion->cw(); speed=-speed;}
		left->setTarget(speed); right->setTarget(speed);
		left->run(); right->run();
		if(fabs(f-b)<alignThresh) 
		{
			left->stop(); right->stop();
			usleep(100000);
			parSetup();
		}
	}
	void bgcAlign()
	{
	}
public:
	Wallfollower(Motor* _l, Motor* _r, IR* _irf, IR* _irr, IR* _irlf, IR* _irlb, mraa::Gpio* _uirb, Location* _start) {
		left = _l;
		right = _r;
		irlf = _irlf;
		irlb = _irlb;
		irr = _irr;
		irf = _irf;
		uirb = _uirb;
		//irb = _irb;
		start = _start;
		current = _start;
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		motion = new Motion(left,right,odo,_start);
		lostSetup();
	}
	void parallel()
	{
		mode=PAR;
		left->forward(); right->forward();
		left->setSpeed(wfSpeed);
		right->setSpeed(wfSpeed);
	}
	void run()
	{
		std::cout<<"Currently in mode "<<mode<<std::endl;
		switch(mode)
		{
			case PAR: parRun(); break;
			case SMC: smcRun(); break;
			case SMC_align: smcAlign(); break;
			case BGC_align: bgcAlign(); break;
			case BGC: bgcRun(); break;
			case LOST: lostRun(); break;
		}
	}
};
