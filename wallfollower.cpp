#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
class Wallfollower {
	IR* irl;
	IR* irr;
	Motor* left;
	Motor* right;
	Gyroscope* gyr;
	Odometry* odo;
	Location* current;
	Motion* motion;
	float currentangle;
	float base_angle; //the angle that parrallel to the wall
	float target; //the distance that you want the robot to stay from the wall
	float distance; //after running setAngle, the robot is this much from the wall
	bool side_is_right; //use the right IR to detect
	float prerdis, preldis; bool det; int cnt;int cntdec; //only for setAngle
	

public:
	Wallfollower(Motor* _l, Motor* _r, Gyroscope* _gyr, IR* _irl, IR* _irr, Location* _start) {
		left = _l;
		right = _r;
		gyr = _gyr;
		irl = _irl;
		irr = _irr;
		current = _start;
		motion = new Motion(left,right,gyr,_start);
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		prerdis = irr->getDistance();
		preldis = irl->getDistance();
		cnt = 0;
		cntdec = 0;
		det = false;
	}
	void setDistance(float _target) {
		target = _target;
	}
	void run(){

	}
	bool setAngle() {
		std::cout<<"cnt:"<<cnt<<std::endl;
		gyr->run();
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
	void setrotate(float speed) {
		left->forward();
		right->backward();
		left->setSpeed(speed);
		right->setSpeed(speed);



	}
};
