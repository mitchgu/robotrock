#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
const float Kp = 3, Kd = 3, Ki = 2;
const float threshelddis = 9.0; //for the first approaching to wall
const float fm_angle = 0.3;   //
const float base_speed = 15;
class Wallfollower {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	IR* irb;
	Motor* left;
	Motor* right;
	Odometry* odo;
	Location* current;
	Motion* motion;
	int corner;
	float smooth_rotate_speed; float smooth_forward_speed;
	float base_angle; //the angle that parrallel to the wall
	float target; //the distance that you want the robot to stay from the wall
	float distance; //after running setAngle, the robot is this much from the wall
	bool det; bool cw; bool forw; bool forw; int cnt;int cntdec; //only for setAngle
	float integration,prerror; //only for parallelrun
	int cornercnt; // only for cornercnt
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
	Wallfollower(Motor* _l, Motor* _r, IR* _irlf, IR* _irlb, IR* _irr,IR* _irf,IR* _irb, Location* _start) {
		left = _l;
		right = _r;
		irlf = _irlf;
		irlb = _irlb;
		irr = _irr;
		irf = _irf;
		irb = _irb;
		current = _start;
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		motion = new Motion(left,right,odo,_start);
		corner = 0;
		cornercnt = 0;
	}

/*
	implement the motion in wallfollower
	*/
	bool run() {
		return motion->run();
	}
	void straight(float distance){
		motion->straight(distance);
	}
	void rotate(float angle){
		motion->rotate(angle);
	}
	void stop(){
		smooth_forward_speed=0;
		smooth_rotate_speed=0;
		left->stop();
		right->stop();
	}


	/*
	setting the basic parameters
	*/
	void setLocation(Location* _location){
		current = _location;
		odo->set(_location);
	}
	void setDistance(float _target) {
		target = _target;
	}

	/*
	basic implementations and action for robot
	*/
	void smoothforward(float speed) {
		if(speed>0)
		{
			forw=true;
			left->forward();
			right->forward();
			smooth_forward_speed = speed;
		}
		else
		{
			forw= false;
			right->backward();
			left->backward();
			smooth_forward_speed = -speed;
		}
	}
	void smoothrotate(float speed) {
		if(speed>0)
		{
			cw=true;
			left->forward();
			right->backward();
			smooth_rotate_speed = speed;
		}
		else
		{
			cw= false;
			right->forward();
			left->backward();
			smooth_rotate_speed = -speed;
		}
	}
	void smooth_rotate_run() {
		left->setSpeed(smooth_rotate_speed);
		right->setSpeed(smooth_rotate_speed);
		current = odo->run();
	}
	void smooth_forward_run() {
		left->setSpeed(smooth_forward_speed);
		right->setSpeed(smooth_forward_speed);
		current = odo->run();
	}
	void setup_parallelrun() {
		integration = 0;
		prerror = 0;
		float dt = timeDiff();
	}
	void setup_parallel_to_wall() {
		cntdec = 0, cnt = 0, det = false;
		prelfdis = irlf->getDistance();
		prelbdis = irlb->getDistance();
	}



	/*
	steps that used in the test
	*/
	int incorner(){   //0 for no corner, 1 for corner almost 90 degrees, 2 for corner almos
		if (irf->getDistance()<=9.0) { return 1;}
		if (irlf->getDistance()==100) { 
			if(cornercnt=0) {
				cornercnt++;
			} 
			else { return 2;}
		}
		cornercnt = 0;
		return 0;
	}
	bool close_to_wall(){
		bool f = (irf->getDistance())<threshelddis;
		bool b = (irb->getDistance())<threshelddis;
		bool lb = (irlb->getDistance())<threshelddis;
		bool lf = (irlf->getDistance())<threshelddis;
		bool r = (irr->getDistance())<threshelddis;
		return (f || b || l || r);
	}
	void parallelrun(){
		float irlf = irlf->getDistance();
		float irlb = irlb->getDistance();
		float estimatedis = (irlf+irlb)*cos(fm_angle)/2
		std::cout<<"running parallel wall"<<std::endl;
		std::cout<<"distance from the wall:  "<<estimatedis<<std::endl;
		float dt = timeDiff();
		float error = target - estimatedis;
		float dif = irlf-irlb;  //(error-prerror)/dt;
		integration = integration + error*dt;
		float dspeed = error*Kp+dif*Kd+integration*Ki;
		float lspeed=base_speed+dspeed,rspeed=base_speed-dspeed;
		left->setSpeed(lspeed);
		right->setSpeed(rspeed);
		prerror = error;
		current = odo->run();
	}
	bool parallel_to_wall() {
		gettimeofday(&tv, NULL);
		std::cout<<"cnt:"<<cnt<<std::endl;
		current = odo->run();
		usleep(10000);
		// do parallel to wall
		float lbdis = irlb->getDistance();
		float lfdis = irlf->getDistance();
		std::cout<<"nowlfdis:  "<<lfdis<<",  nowlbdis:  "<<lbdis<<std::endl;
		if ((lbdis==100) || (lfdis==100) ) {
			return false;
		}
		if (cw) {
			std::cout<<"paralleling clockwise"<<std::endl;
			else {
				if(irlb>irlf) {
					cnt=0;   
					if(cntdec<5) cntdec++;
					else {
						std::cout<<"detection mode"<<std::endl;
						det = true;
					}
				}
				else 
				{
					if(cntdec<5) cntdec = 0;
					if (!det) return false;
					else 
					{
						if (cnt>0) 
						{
							std::cout<<"this is left cnt = 1"<<std::endl;
							return true;
						}
						else cnt++;
					}
				}
			}
			return false;
		}
		else {
			std::cout<<"paralleling counterclockwise"<<std::endl;
			else {
				if(irlb<irlf) {
					cnt=0;   
					if(cntdec<5) cntdec++;
					else {
						std::cout<<"detection mode"<<std::endl;
						det = true;
					}
				}
				else 
				{
					if(cntdec<5) cntdec = 0;
					if (!det) return false;
					else 
					{
						if (cnt>0) 
						{
							std::cout<<"this is left cnt = 1"<<std::endl;
							return true;
						}
						else cnt++;
					}
				}
			}
			return false;
		}
	}
};
