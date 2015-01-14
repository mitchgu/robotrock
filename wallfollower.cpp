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
	Motion* motion
	float currentangle;
	float base_angle; //the angle that parrallel to the wall
	float target; //the distance that you want the robot to stay from the wall
	float distance; //after running setAngle, the robot is this much from the wall
	bool side_is_right; //use the right IR to detect
	float prerdis, preldis; bool det; int cnt; //only for setAngle
	

public:
	Wallfollower(Motor* _l, Motor* _r, Gyroscope* _gyr, IR* _irl, IR* _irr, Location* _start) {
		left = _l;
		right = _r;
		gyr = _gyr;
		irl = _irl;
		irr = _irr;
		side = _side;
		current = _start;
		motion = Motion(left,right,gyr,location);
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		prerdis = irr->getDistance();
		preldis = irl->getDistance();
		cnt = 0;
		det = false;
	}
	setDistance(float _target) {
		target = _taget;
	}
	void run(){

	}
	bool setAngle() {
		gyr->run();
		current = odo->run();
		motion->rotate(2);
		motion->run();
		usleep(10000);
		float rdis = irr->getDistance(); float ldis = irl->getDistance();
		if ((rdis==100) && (ldis==100) ) {
			prerdis = 100; preldis = 100;
			return false;
		}
		else {
			if (rdis !=100) {
				if(prerdis>rdis) {
					prerdis = rdis;
					det = true;
					return false;
				}
				if(prerdis<rdis) {
					if (!det) {return false;}
					else {
						if(cnt == 0){
							base_angle = gyr->run();
							distance = rdis;
						}
						if (cnt>20) {
							side_is_right = true;
							cnt = 0;
							det = false;
							return true;
						}
						else {cnt++;}
					}
				}
			}
			else {
				if(preldis>ldis) {
					preldis = ldis;
					det = true;
					return false;
				}
				if(preldis<ldis) {
					if (!det) {return false;}
					else {
						if(cnt == 0) {
							base_angle = gyr->run()
							distance = ldis;
						}
						if (cnt>20) {
							side_is_right = false;
							cnt = 0;
							det = false;
							return true;
						}
						else {cnt++;}
					}
				}
			}

		}
	}
	void turn() {

	}
}