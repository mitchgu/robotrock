#include <iostream>
#include "shortIR.cpp"
#include "odometry.cpp"
const float slp = 0.5;
const float forward_speed = 1.2;
const float forward_side_dis_threshold = 12;
const float forward_front_dis_threshold = 10;
const float block_size = 3.5;

class Planningwalk {
	Motor* left;
	Motor* right;
	IR* irf;
	IR* irlf;
	IR* irlb;
	IR* irr;
	Odometry* odo;
	mraa::Gpio* irb;
	Location* start;
	Location* current;
	bool initialized;
	bool r_init; bool l_init; int f_cnt; Location* l_start; Location* r_start; int wall_side; //wallside: 0:left; 1:right; 2:front


public:
	Planningwalk (Motor* _left,Motor* _right,IR* _irf,IR* _irlf,IR* _irlb,IR* _irr,mraa::Gpio* _irb, Location* _start) {
		left = _left; right = _right;
		irf = _irf; irlf = _irlf; irlb = _irlb; irr = _irr; irb = _irb;
		start = _start; current = _start;
		odo = new Odometry(_left, _right, _start->x(),_start->y(),_start->theta());
		initialized = false;
	}
	void channel_stop() {
		left->stop(); right->stop();
		initialized = false;
		sleep (slp);
	}
	void forward_setup() {
		left->forward();
		right->forward();
		left->setSpeed(forward_speed);
		right->setSpeed(forward_speed);
		r_init = false; l_init = false; f_cnt = 0; 
		initialized = true;
	}
	void forward_run() {
		left->run();
		right->run();
		current = odo->run();
		std::cout<<"x position: "<<current->x()<<" y position: "<<current->y()<<" theta: "<<current->theta()<<std::endl;
	}
	void getwallside() {
		if (wall_side==0) {
			std::cout<<"wall in left: "<<irlf->getDistance()<<std::endl;
		}
		if (wall_side==1) {
			std::cout<<"wall in right: "<<irr->getDistance()<<std::endl;
		}
		if (wall_side==2) {
			std::cout<<"wall in front: "<<irf->getDistance()<<std::endl;
		}
	}
	bool forward_next () {
		float dlf = irlf->getDistance();
		float dr = irr->getDistance();
		float df = irf->getDistance();
		std::cout<<"dlf: "<< dlf<<" dr: "<<dr<<" df: "<<df<<std::endl;
		if (!((dlf>forward_side_dis_threshold) && (dr>forward_side_dis_threshold) && (df>forward_front_dis_threshold) && (!irb->read()))) {
			if (df<=forward_front_dis_threshold) {   //front wall dealer
				if (f_cnt<0) f_cnt++;
				else {
					wall_side = 2;
					return true;
				}
			}
			else f_cnt = 0;

			if (dlf<=forward_side_dis_threshold) {   //left wall dealer
				if (!l_init) {
					l_start = current;
					l_init = true;
				}
				else {
					if ((current->distance(l_start))>block_size) {
						wall_side = 0;
						return true;
					}
				}
			}
			else l_init = false;

			if (dr<=forward_side_dis_threshold) {  //right wall dealer
				if (!r_init) {
					r_start = current;
					r_init = true;
				}
				else {
					if ((current->distance(r_start))>block_size) {
						wall_side = 1;
						return true;
					}
				}
			}
			else r_init = false;
		}
		else {
			r_init = false;
			l_init = false;
			f_cnt = 0;
		}
		return false;
	}
};
