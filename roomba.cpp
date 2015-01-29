#include "motion.cpp"
#include "servo.cpp"
#include "cv.cpp"
#include <iostream>
#include "shortIR.cpp"
#include "logger.cpp"

#define APPROACH 0
#define ALIGN 1
#define PARALLEL 2
#define CHASE 3
#define PICKUP 4

const float FORWARD_SPEED = .65;
const float ROTATE_SPEED = .75;
const float PARALLEL_DIST_TARGET = 5.0;
const float PARALLEL_DIST_P = .1;
const float PARALLEL_ANGLE_P = .2;
const float PARALLEL_ROTATE_P = 1.0;
const float FORWARD_SCALE_FACTOR = 1.5;
const int PIC_DURATION=200;
const double stopChaseBall=3;

class Roomba {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	Servo* claw;
	Servo* flip;
	mraa::Gpio* uirb;
	mraa::Gpio* rotEnd;
	mraa::Gpio* midEnd;
	mraa::Gpio* upEnd;
	mraa::Gpio* cube;

	VideoCapture* cap;
	Mat in,test,frame;
	std::vector<int> inds; 
	double cubeDist, cubeAngle;
	int lostCount,cubeType, checkCount;
	VideoWriter* outVid,*recVid;
	struct timeval tv,ts;

	Motor* left;
	Motor* right;
	Motor* base;
	Motor* lift;
	Odometry* odo;
	Location* start;
	Location* current;
	Motion* motion;
	Logger logger;

	float fdist;
	float lfdist;
	float lbdist;
	float rdist;
	float parallel_dist;
	float parallel_angle;
	float rotateSpeed;
	float forwardScale;

	bool homeDetected;

	// Returns whether a sensor distance is in range.
	bool inRange(float dist) {
		if (dist<20) {
			return true;
		}
		return false;
	}

	double timeDiff() { 
		unsigned long long ms = (unsigned long long)(ts.tv_sec)*1000 + (unsigned long long)(ts.tv_usec) / 1000; 
		gettimeofday(&tv, NULL); 
		unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec) / 1000; 
		double msf = (double)(msl-ms); 
		return ms;
	}
	void stop() {
		left->stop();
		right->stop();
	}

	void setMotor(string motor, float speed) {
		if (motor == "left") {
			if (speed < 0) {
				left->backward();
			}
			else {
				left->forward();
			}
			left->setSpeed(std::abs(speed));
		}
		if (motor == "right") {
			if (speed < 0) {
				right->backward();
			}
			else {
				right->forward();
			}
			right->setSpeed(std::abs(speed));
		}
	}

	public:
	Roomba(Motor* _l, Motor* _r, IR* _irf, IR* _irr, IR* _irlf, IR* _irlb, mraa::Gpio* _uirb, Location* _start, Logger _logger) {
		left = _l;
		right = _r;
		irlf = _irlf;
		irlb = _irlb;
		irr = _irr;
		irf = _irf;
		uirb = _uirb; 
		logger = _logger;

		current = _start;
		start=new Location(current);
		odo = new Odometry(_l, _r, current);
		motion = new Motion(left,right,odo,_start);

		inds.pb(0); inds.pb(1); inds.pb(2);
		cap=new VideoCapture(0);
		assert (cap->isOpened());
		/* for video logging
		   cap->read(in);
		   downSize(in,test);
		   Size outSize=Size(test.cols,test.rows);
		   outVid=new VideoWriter("log.avi", CV_FOURCC('M','P','4','2'),1,outSize,true);
		   recVid=new VideoWriter("rlog.avi", CV_FOURCC('M','P','4','2'),1,outSize,true); */

		upEnd=new mraa::Gpio(9);
		upEnd->dir(mraa::DIR_IN);
		rotEnd=new mraa::Gpio(8);
		rotEnd->dir(mraa::DIR_IN);
		cube=new mraa::Gpio(0);
		cube->dir(mraa::DIR_IN);
		midEnd=new mraa::Gpio(1);
		midEnd->dir(mraa::DIR_IN);

		base = new Motor(10,11,6,false); //base motor -counterclockwise when forward
		lift = new Motor(8,9,3,true); //lift motor
		claw = new Servo(15);
		flip = new Servo(3);

		lostCount=0,cubeType=-1,checkCount=PIC_DURATION;

		for(int i=0;i<10;i++) cap->read(test);
		motion->setMConstants(0.5,0,0);

		gettimeofday(&ts, NULL); 
	}
	void armExtend()
	{
		flip->write(0.3);
		usleep(500000);
		
	}
	void armSwing()
	{
		flip->write(1.0);
		usleep(500000);
		flip->release();
	}
	void armRetract()
	{
		flip->write(-1.0);
		usleep(500000);
		flip->release();
	}
	void pickUp()
	{
		int slp=100000;
		if(!cubeIn()) return;
		std::cout<<"Picking up type"<<cubeType<<std::endl;
		stop();

		base->forward(); base->setSpeed(2);
		while(rotEnd->read()) usleep(1000);

		base->backward() ; base->setSpeed(0.5);
		while(midEnd->read()) usleep(1000);
		base->stop(); usleep(slp);

		claw->write(0.5);
		lift->forward(); lift->turnAngle(1700,3); usleep(slp);
		claw->write(0.8); usleep(slp);
		lift->turnAngle(215,3); usleep(slp);
		claw->write(0.15); usleep(slp);

		resetLift(); usleep(slp);

		if(cubeType==1) base->forward();
		else base->backward();
		base->setSpeed(2);
		while(rotEnd->read()) usleep(1000);
		base->setSpeed(1);

		lift->forward(); lift->turnAngle(50,5);

		claw->write(0.7);
		base->stop(); usleep(slp);

		lift->backward(); lift->setSpeed(2);
		while(upEnd->read() ) usleep(1000);
		lift->stop(); claw->release(); usleep(slp);
	}
	void resetLift()
	{
		lift->backward(); lift->setSpeed(2);
		while(upEnd->read() ) usleep(1000);
		lift->stop();
	}

	void dropTower(bool tower)
	{

		if(tower) //left tower
		{
			Servo ltower(13);
			ldoor.write(-0.2); //open
			usleep(1000*1000);
			ldoor.write(-0.9); //close
			usleep(1000*1000);
		}

		else //right tower
		{
			Servo rtower(12);

		}


	}
	bool senseBall(int samps=1)
	{
		//gettimeofday(&tv, NULL); 
		REP(i,samps) cap->read(in);
		std::cout << "Grabbed frame" << std::endl;
		downSize(in,frame); //downsized
		//recVid->write(frame);
		maxFilter(frame,inds);
		std::vector<centers> ret=fill(frame);
		//outVid->write(frame);
		cubeType=-1,cubeDist=1000000;
		bool sensed=false;
		for(int j=0;j<ret.size();j++)
		{
			if(ret[j].type==4) homeDetected=true;
			else if(ret[j].type!=0&&ret[j].dist<cubeDist)
			{
				cubeDist=ret[j].dist;
				cubeAngle=ret[j].angle;
				cubeType=ret[j].type;
				std::cout<<"Ball "<<cubeDist<<" inches away at angle "<<cubeAngle<<"of type "<<cubeType<<std::endl;
				sensed=true;
			}
		}
		//std::cout<<"Camera shit is taking "<<timeDiff()<<std::endl;
		return sensed;
	}
	bool cubeIn(int times=5)
	{
		REP(i,times) if(cube->read()) return false;
		return true;
	}
	void goForward(double dist)
	{
		motion->setBaseSpeed(1);
		motion->straight(dist,false);
		motion->setMConstants(3,0.1,0.15);
		while(!motion->run()) 
		{
			if(dist>0&&irf->getDistance()<stopChaseBall) break;
			if(dist<0&&!uirb->read()) break;
			cap->read(in);
		}
		stop();
		usleep(20000); 
	}
	int step(int state) {
		fdist = irf->getDistance();
		lfdist = irlf->getDistance();
		lbdist = irlb->getDistance();
		rdist = irr->getDistance();
		bool in=cubeIn();
		logger.log("Inside State", std::to_string(in));
		logger.log("Roomba State", std::to_string(state));
		logger.log("Front IR", std::to_string(fdist));
		logger.log("Right IR", std::to_string(rdist));
		logger.log("Left Front IR", std::to_string(lfdist));
		logger.log("Left Back IR", std::to_string(lbdist));
		switch (state) {
			// State 0: Go forward ///////////////////////////////////////////////////
			case APPROACH: 
				setMotor("left", FORWARD_SPEED);
				setMotor("right", FORWARD_SPEED);

				if(in)
					return PICKUP;
				else if (fdist < 8 || rdist < 5) { // If close in front or on right
					stop();
					return ALIGN;
				}
				else if (lfdist < 9) { // If left is already close to wall
					stop();
					return PARALLEL;
				}
				else return APPROACH;
				// State 1: Rotate in place CW //////////////////////////////////////////
			case ALIGN: 
				setMotor("left", ROTATE_SPEED);
				setMotor("right", -ROTATE_SPEED);

				if(in)
					return PICKUP;
				else if (lfdist < 9 && fdist > 14){ //If close to wall on left, clear in front
					motion->rotate(0.6);
					while(!motion->run()) usleep(1000);
					stop();
					if(senseBall(6))
					{
						motion->straight(true);
						return CHASE;
					}
					motion->rotate(-0.45);
					while(!motion->run()) usleep(1000);
					stop();
					return PARALLEL;
				}
				else if (!inRange(lfdist) && !inRange(fdist) && !inRange(rdist)){
					stop();
					return APPROACH;
				}
				else{ //Not clear or parallel to wall, keep rotating
					return ALIGN;
				}
				// State 2: Drive parallel to wall //////////////////////////////////////
			case PARALLEL: 

				parallel_dist = 0.5 * lfdist + 0.5 * lbdist;
				parallel_angle = lfdist - lbdist;

				if (lfdist < 2 || lbdist < 2) {
					parallel_angle = -2;
				}

				//logger.log("Parallel Dist V", std::to_string(PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET)));
				//logger.log("Parallel Angle V", std::to_string(PARALLEL_ANGLE_P * parallel_angle));

				rotateSpeed = std::min(PARALLEL_ROTATE_P * (PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET) + PARALLEL_ANGLE_P * parallel_angle), 2.5f);
				forwardScale = std::max(1-FORWARD_SCALE_FACTOR*std::abs(rotateSpeed),-0.0f) * FORWARD_SPEED;

				logger.log("Angle Error", std::to_string(PARALLEL_ANGLE_P * parallel_angle));
				logger.log("Distance Error", std::to_string(PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET)));
				logger.log("Rotate Speed", std::to_string(rotateSpeed));
				logger.log("Forward Scale", std::to_string(forwardScale));

				if(in)
					return PICKUP;
				else if (rotateSpeed > 0) {
					setMotor("left", forwardScale);
					setMotor("right", forwardScale + rotateSpeed);
				}
				else {
					setMotor("left", forwardScale - rotateSpeed);
					setMotor("right", forwardScale);
				}


				if (fdist < 7 || rdist < 3.5) { // If small corner
					stop();
					return ALIGN;
				}
				else if (!inRange(lfdist) && !inRange(lbdist)) { // If IRLF misses
					//stop();
					return PARALLEL;
				}
				else { // Stay if anything else
					return PARALLEL;
				}
				//chase cube
			case CHASE:
				motion->run();
				//channel = wf->run_follower(channel);
				homeDetected=false;
				bool sensed=senseBall(6);
				if(homeDetected&&
				if(!sensed)
				{
					if(lostCount==3) 
					{
						lostCount=0,cubeType=-1;
						return APPROACH;
					}
					else
					{
						lostCount++;
						goForward(-10);
						motion->straight(false);
						return CHASE;
					}
				}
				std::cout<<"I see a ball"<<cubeDist<<" away at "<<cubeAngle<<"degrees\n";
				if(fabs(cubeAngle)>8)
				{
					motion->rotate(cubeAngle*3.14/180);
					while(!motion->run()); 
					stop(); usleep(10000);
					motion->straight(false);
					return CHASE;
				}
				
				if(cubeDist<=20) 
				{
					goForward(-7); stop();
					if(irf->getDistance()<10)
					{
						armExtend();
						goForward(irf->getDistance()-2); stop();
						motion->rotate(1.0);
						while(!motion->run()) usleep(10000);
						goForward(-5);
						armRetract();
					}
					goForward(25); stop();
					return PICKUP;
				} 
				else if(in) return PICKUP;
				else if(irf->getDistance()<stopChaseBall)
				{
					goForward(-10); cubeType=-1;
					return APPROACH;
				}
				return CHASE;
			case PICKUP:
				if(cubeType==-1)
				{
					goForward(-12);
					motion->straight(false);
					return CHASE;
				}
				pickUp();
				return APPROACH;
		}
	}
};
