#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
const float K1 = 5, K2 = 3, K3 = 2;
class Wallfollower {
	IR* irl;
	IR* irr;
	IR* irf;
	Motor* left;
	Motor* right;
	Gyroscope* gyr;
	Odometry* odo;
	Location* current;
	Motion* motion;
	int corner;
	float base_speed;
	float currentangle;
	float base_angle; //the angle that parrallel to the wall
	float target; //the distance that you want the robot to stay from the wall
	float distance; //after running setAngle, the robot is this much from the wall
	bool side_is_right; //use the right IR to detect
	float prerdis, preldis; bool det; int cnt;int cntdec; //only for setAngle
	float predif,prerror; //only for parallelrun
	int cornercnt; // only for cornercnt
	struct timeval tv;
	

public:
	Wallfollower(Motor* _l, Motor* _r, Gyroscope* _gyr, IR* _irl, IR* _irr,IR* _irf, Location* _start) {
		left = _l;
		right = _r;
		gyr = _gyr;
		irl = _irl;
		irr = _irr;
		irf = _irf;
		current = _start;
		motion = new Motion(left,right,gyr,_start);
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		prerdis = irr->getDistance();
		preldis = irl->getDistance();
		corner = 0;
		cnt = 0; cntdec = 0; det = false; prerror = 0; predif = 0; cornercnt = 0;
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
	void setDistance(float _target) {
		target = _target;
	}
	void setforward(float _base_speed) {
		left->forward();
		right->forward();
		base_speed = _base_speed;
	}
	void parallelrun(){
		float dt = timeDiff();
		float error = target-irr->getDistance();
		float dif = (error-prerror)/dt;
		float ddif = (predif-dif)/dt;
		float dspeed = error*K1+dif*K2+ddif*K3;
		left->setSpeed(base_speed+dspeed);
		right->setSpeed(base_speed-dspeed);
		prerror = error;
		predif = dif;
		currentangle = gyr->run();
		current = odo->run();
	}
	int incorner(){   //0 for no corner, 1 for corner almost 90 degrees, 2 for corner almos
		if (irf->getDistance()<=5.0) { return 1;}
		if (irr->getDistance()==100) { 
			if(cornercnt=0) {
				cornercnt++;
			} 
			else { return 2;}
		}
		cornercnt = 0;
		return 0;
	}
	bool setAngle() {
		gettimeofday(&tv, NULL);
		std::cout<<"cnt:"<<cnt<<std::endl;
		currentangle = gyr->run();
		current = odo->run();
		usleep(10000);
		std::cout<<"nowdr:  "<<prerdis<<"  &nowl:  "<<preldis<<std::endl;
		float rdis = irr->getDistance(); float ldis = irl->getDistance();
		if ((rdis==100) && (ldis==100) ) {
			prerdis = 100; preldis = 100;
			return false;
		}
		else {
			if (rdis !=100) {
				if(prerdis>rdis) {
					prerdis = rdis;
					if (cnt!=0) {
						cnt = 0;
					}
					if(cntdec<5) {
						cntdec++;
					}
					else {
						std::cout<<"detection mode right"<< std::endl;
						det = true;
					}
					return false;
				}
				if(prerdis<rdis) {
					prerdis = rdis;
					if (cntdec<5) {
						cntdec = 0;
					}
					if (!det) {return false;}
					else {
						if(cnt == 0){
							std::cout<<"this is right cnt = 0"<<std::endl;
							base_angle = gyr->run();
							distance = rdis;
						}
						if (cnt>0) {
							side_is_right = true;
							cnt = 0;
							det = false;
							std::cout<<"this is right cnt = 20"<<std::endl;
							return true;
						}
						else {cnt++;}
					}
				}
			}
			else {
				if(preldis>ldis) {
					preldis = ldis;
					if(cnt!=0) {
						cnt=0;
					}
					if(cntdec<5) {
						cntdec++;
					}
					else {
						std::cout<<"detection mode left"<<std::endl;
						det = true;
					}
					return false;
				}
				if(preldis<ldis) {
					preldis = ldis;
					if(cntdec<5) {
						cntdec = 0;
					}
					if (!det) {return false;}
					else {
						if(cnt == 0) {
							std::cout<<"this is left cnt = 0"<<std::endl;
							base_angle = gyr->run();
							distance = ldis;
						}
						if (cnt>0) {
							side_is_right = false;
							cnt = 0;
							det = false;
							std::cout<<"this is left cnt = 20"<<std::endl;
							return true;
						}
						else {cnt++;}
					}
				}
			}

		}
	}
	motion* getmotion(){
		return motion;
	}
	void stop(){
		left->stop();
		right->stop();
	}
	void setrotate(float speed) {
		left->forward();
		right->backward();
		left->setSpeed(speed);
		right->setSpeed(speed);



	}
};
