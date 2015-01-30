#include "motion.cpp"
#include "servo.cpp"
#include "cv.cpp"
#include <iostream>
#include "shortIR.cpp"
#include "logger.cpp"

#define TROUBLE -1
#define APPROACH 0
#define ALIGN 1
#define PARALLEL 2
#define CHASE 3
#define PICKUP 4
#define HOMING 5
#define STOP 6
#define BIGCORNER 7

const float FORWARD_SPEED = .65;
const float ROTATE_SPEED = .75;
const float PARALLEL_DIST_TARGET = 5.0;
const float PARALLEL_DIST_P = .1;
const float PARALLEL_ANGLE_P = .2;
const float PARALLEL_ROTATE_P = .4;
const float FORWARD_SCALE_FACTOR = 1.2;
const int PIC_DURATION=200;
const double stopChaseBall=5;
const int HOMING_TIME=0;//120000;

class Roomba {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	Servo* claw,*doorl,*doorr;
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
	float toWall, base_gyro;
	VideoWriter* outVid,*recVid;
	struct timeval tv,ts,stktv;
	float stuckThreshold;
	unsigned long long checkBase;

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
		return msf;
	}
	int transition(int state)
	{
		gettimeofday(&stktv, NULL);
		unsigned long long ms = (unsigned long long)(stktv.tv_sec)*1000 + (unsigned long long)(stktv.tv_usec) / 1000;
		checkBase =  ms;
		if(state == APPROACH) stuckThreshold = 4;
		else if(state == ALIGN) stuckThreshold = 3;
		else if(state == PARALLEL) stuckThreshold = 7;
		else if(state == PICKUP) stuckThreshold = 30;
		else if(state == CHASE) 
		{
			setupChase();
			stuckThreshold = 15;
		}
		else if(state == BIGCORNER)
		{
			odo->run();
			stuckThreshold = 4;
			base_gyro = odo->getAngle(); 
		}
		else if(state == TROUBLE)
		{
			stop(); usleep(500000);
			setMotor("left",-FORWARD_SPEED);
			setMotor("right",-FORWARD_SPEED);
			usleep(500000);
			stop();
			setMotor("left",ROTATE_SPEED);
			setMotor("right",-ROTATE_SPEED);
		}
		return state;
	}
	bool checkStuck()
	{
		gettimeofday(&stktv, NULL); 
		unsigned long long ms = (unsigned long long)(stktv.tv_sec)*1000 + (unsigned long long)(stktv.tv_usec) / 1000;
		return (ms - checkBase)>1000*stuckThreshold;
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

	void setupChase()
	{
		motion->setBaseSpeed(0.5);
		motion->straight(false);
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

		inds.pb(0); inds.pb(1); inds.pb(2); inds.pb(4);
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
		doorl = new Servo(13);
		doorr = new Servo(12);
		flip = new Servo(3);

		doorl->write(-1.0);
		doorr->write(-1.1);

		lostCount=0,cubeType=-1,checkCount=PIC_DURATION;

		for(int i=0;i<10;i++) cap->read(test);
		motion->setMConstants(0.5,0,0);

		gettimeofday(&stktv, NULL);
		checkBase = (unsigned long long)(stktv.tv_sec)*1000 + (unsigned long long)(stktv.tv_usec) / 1000;
		stuckThreshold = 4;
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
			ltower.write(-0.7); //open slightly
			usleep(1000*1000);
			ltower.write(-0.9); //close
			usleep(1000*1000);
			ltower.write(-0.2); //open fully
			usleep(1000*1000);
			ltower.write(-0.9); //close
			usleep(1000*1000);
		}

		else //right tower
		{
			Servo rtower(12);
			rtower.write(-0.8); //open slightly
			usleep(1000*1000);
			rtower.write(-1.0); //close
			usleep(1000*1000);
			rtower.write(-0.3); //open fully
			usleep(1000*1000);
			rtower.write(-1.0); //close
			usleep(1000*1000); 
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
			if(ret[j].type==4)
			{
				std::cout<<"I SEEE PURPELEPL!"<<std::endl;
				homeDetected=true;
			}
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
	void goForward(double dist,double speed=1)
	{
		motion->setBaseSpeed(speed);
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
		double dt=timeDiff();
		bool cubein=cubeIn(),sensed;
		logger.log("Inside State", std::to_string(cubein));
		logger.log("Roomba State", std::to_string(state));
		logger.log("Time State", std::to_string(dt));
		logger.log("Front IR", std::to_string(fdist));
		logger.log("Right IR", std::to_string(rdist));
		logger.log("Left Front IR", std::to_string(lfdist));
		logger.log("Left Back IR", std::to_string(lbdist));
		switch (state) {
			// State 0: Go forward ///////////////////////////////////////////////////
			case APPROACH:
				if(checkStuck()) return transition(TROUBLE);
				setMotor("left", FORWARD_SPEED);
				setMotor("right", FORWARD_SPEED);

				if(cubein)
					return transition(PICKUP);
				else if (fdist < 8 || rdist < 5) { // If close in front or on right
					stop();
					return transition(ALIGN);
				}
				else if (lfdist < 9) { // If left is already close to wall
					stop();
					return transition(PARALLEL);
				}
				else return APPROACH;
				// State 1: Rotate in place CW //////////////////////////////////////////
			case ALIGN: 
				if(checkStuck()) return TROUBLE;
				setMotor("left", ROTATE_SPEED);
				setMotor("right", -ROTATE_SPEED);

				if(cubein)
					return PICKUP;
				else if (lfdist < 9 && fdist > 14){ //If close to wall on left, clear in front
					motion->rotate(0.4);
					while(!motion->run()) usleep(1000);
					stop();
					homeDetected=false;
					sensed=senseBall(6);
					if(homeDetected&&(dt>HOMING_TIME)) return HOMING;
					if(sensed)
					{
						return transition(CHASE);
					}
					motion->rotate(-0.3);
					while(!motion->run()) usleep(1000);
					stop();
					return transition(PARALLEL);
				}
				else if (!inRange(lfdist) && !inRange(fdist) && !inRange(rdist)){
					stop();
					return transition(APPROACH);
				}
				else{ //Not clear or parallel to wall, keep rotating
					return ALIGN;
				}
				// State 2: Drive parallel to wall //////////////////////////////////////
			case PARALLEL: 
				if(checkStuck()) return transition(TROUBLE);
				parallel_dist = 0.5 * lfdist + 0.5 * lbdist;
				parallel_angle = lfdist - lbdist;

				if (lfdist < 2 || lbdist < 2) {
					parallel_angle = -4;
				}

				//logger.log("Parallel Dist V", std::to_string(PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET)));
				//logger.log("Parallel Angle V", std::to_string(PARALLEL_ANGLE_P * parallel_angle));

				rotateSpeed = std::min(PARALLEL_ROTATE_P * (PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET) + PARALLEL_ANGLE_P * parallel_angle), 2.5f);
				forwardScale = std::max(1-FORWARD_SCALE_FACTOR*std::abs(rotateSpeed),-0.0f) * FORWARD_SPEED;

				logger.log("Angle Error", std::to_string(PARALLEL_ANGLE_P * parallel_angle));
				logger.log("Distance Error", std::to_string(PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET)));

				if(cubein)
					return transition(PICKUP);
				else if (rotateSpeed > 0) {
					setMotor("left", forwardScale - 0.5 * rotateSpeed);
					setMotor("right", forwardScale + 0.5 * rotateSpeed);
				}
				else {
					setMotor("left", forwardScale - 0.5 * rotateSpeed);
					setMotor("right", forwardScale + 0.5 * rotateSpeed);
				}
/*
				if (lfdist > 15) {
					setMotor("left",(lbdist+2)*0.06);
					setMotor("right",(lbdist+14)*0.06);
					usleep(300*1000);
				}
*/
				if (lfdist > 15 ) return BIGCORNER;
				if (fdist < 7) { // If small corner
					stop();
					return transition(ALIGN);
				}
				else if (!inRange(lfdist) && !inRange(lbdist)) { // If IRLF misses
					//stop();
					if (lbdist!=20) toWall = lbdist;
					return PARALLEL;
				}
				else { // Stay if anything else
					return PARALLEL;
				}
				//chase cube
			case CHASE:
				if(checkStuck()) return transition(TROUBLE);
				motion->run();
				//channel = wf->run_follower(channel);
				homeDetected=false;
				sensed=senseBall(6);
				if(homeDetected&&(dt>HOMING_TIME)) return HOMING;

				if(irf->getDistance()<stopChaseBall)
				{
					goForward(-10); cubeType=-1;
					motion->rotate(0.4);
					while(!motion->run() ) usleep(1000);
					return transition(APPROACH);
				}
				if(!sensed)
				{
					if(lostCount==3) 
					{
						lostCount=0,cubeType=-1;
						return transition(APPROACH);
					}
					else
					{
						lostCount++;
						goForward(-15);
						setupChase();
						return CHASE;
					}
				}
				std::cout<<"I see a ball"<<cubeDist<<" away at "<<cubeAngle<<"degrees\n";
				if(fabs(cubeAngle)>5)
				{
					motion->rotate(cubeAngle*3.14/180);
					while(!motion->run()); 
					stop(); usleep(10000);
					setupChase();
					return CHASE;
				}
				
				if(cubeDist<=22) 
				{
					goForward(25,0.5); stop();
					return transition(PICKUP);
				} 
				else if(cubein) return transition(PICKUP);
				setupChase();
				return CHASE;
			case PICKUP:
				if(checkStuck()) return transition(TROUBLE);
				if(!cubein) return transition(APPROACH);
				if(cubeType==-1)
				{
					goForward(6);
					goForward(-18);
					return transition(CHASE);
				}
				pickUp(); cubeType=-1;
				return transition(APPROACH);
			case HOMING:
				stop(); sleep(1);
				REP(i,6) cap->read(in);
				REP(k,20)
				{
					cap->read(in);
					std::cout << "Grabbed homing frame" << std::endl;
					downSize(in,frame); //downsized
					maxFilter(frame,inds);
					fill(frame,0,2);
					vector<Vec4i> lines=hough(frame);
					pii retp=procHough(lines,frame);
					pdd ret=getDist(retp.first,retp.second);
					if(retp.first==-1) continue;
					if(retp.first<60)
					{
						motion->straight(6,false);
						while(!motion->run()) usleep(1000);
						return HOMING;
					}
					motion->rotate(ret.second*3.14/180);
					while(!motion->run()) usleep(1000);
					stop(); usleep(1000);
					if(ret.first>22)
					{
						motion->straight(6,false);
						while(!motion->run()) usleep(1000);
						return HOMING;
					}
					motion->straight(ret.first,false);
					while(!motion->run()) usleep(1000);
					stop(); usleep(1000);
					return STOP;
				}
				return transition(APPROACH);

			case STOP:
				sleep(1);
				return STOP;
			case TROUBLE:
				if(fdist==20) return transition(APPROACH);
				else
				{
					usleep(10000);
					return TROUBLE;
				}
			case BIGCORNER:
				if(checkStuck()) return transition(TROUBLE);
				odo->run();
				setMotor("left",(toWall+1)*0.08);
				setMotor("right",(toWall+13)*0.08);
				if((fdist>9 && lfdist>7) || odo->getAngle()>base_gyro-1.5) // only move on if left is close and has rotated >90 degrees
				{
					usleep(10000);
					return BIGCORNER;
				}
				else return transition(ALIGN);
		}
	}
};
