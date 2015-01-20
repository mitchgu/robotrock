#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <deque>  
#include <algorithm>
#include "data.cpp"

const float gridsize = 2; //inch
const float angle_error = 0.2;
const float distance_error = 5;


class MonteCarlos {
	std::deque<Smallwall*>* smallwall_dq;
	std::deque<Smallwall*>* pinpoint_dq;
	Point* start;
	bool located;
	bool locating_error;
public:
	MonteCarlos (std::deque<Wall*>* wall_dq, Point* _start) {
		located = false;
		locating_error = false;
		start = _start;
		smallwall_dq = new std::deque<Smallwall*>();
		for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) {
			Point* target = new Point((*it)->xe(),(*it)->ye());
			float xs = (*it)->xs(); float xe = (*it)->xe(); float ys = (*it)->ys(); float ye = (*it)->ye();
			Smallwall* sm = new Smallwall(xs,ys,xs,ys,target);
			pinpoint_dq
			int small_wall_number = (int)(((*it)->length())/gridsize)+1;
			for (int i=0; i<small_wall_number; i++) {
				float sw_xs = (i*xe+(small_wall_number-i)*xs)/small_wall_number;
				float sw_ys = (i*ye+(small_wall_number-i)*ys)/small_wall_number;
				float sw_xe = ((i+1)*xe+(small_wall_number-(i+1))*xs)/small_wall_number;
				float sw_ye = ((i+1)*ye+(small_wall_number-(i+1))*ys)/small_wall_number;
				Smallwall* sm = new Smallwall(sw_xs,sw_ys,sw_xe,sw_ye,target);
				sm.setprevious(start);
				smallwall_dq->push_back(sm);
			}
		}
	}
	float get_final_angle() {
		return *(smallwall_dq->begin())->wall_angle();
	}
	Point* previous_location() {
		return *(smallwall_dq->begin())->get_previous();
	}
	void update (float angle, float distance) {
		std::deque<Smallwall*>* new_smallwall_dq = new std::deque<Smallwall*>();
		int count = 0
		for (std::deque<Smallwall*>::iterator it = smallwall_dq->begin(); it!=smallwall_dq->end(); ++it) {
			if(((*it)->getangle() > (angle-angle_error))&&((*it)->getangle() < (angle+angle_error))&&((*it)->getdistance()>(distance-distance_error))&&((*it)->getdistance()<(distance+distance_error))) {
				count++;
				for (std::deque<Smallwall*>::iterator it2 = pinpoint_dq->begin(); it2!=pinpoint_dq->end(); ++it2) {
					if ((*it2)->equalTo((*it)->get_target)) {
						Smallwall* sm= new Smallwall((*it2)->xs(),(*it2)->ys(),(*it2)->xe(),(*it2)->ye(),(*it2)->get_target());
						sm->setprevious((*it2)->get_center());
						new_smallwall_dq->push_back(sm);
					}
				}
			}
		}
		smallwall_dq = new_smalwall_dq;
		if (count == 1) {
			located = true;
			std::cout<<"I am located!!"<<std::endl;
			std::cout<<"I am located!!"<<std::endl;
			std::cout<<"I am located!!"<<std::endl;
			std::cout<<"I am located!!"<<std::endl;
		}
		if (count == 0) {
			locating_error = true;
			std::cout<<"locating error!!"<<std::endl;
			std::cout<<"locating error!!"<<std::endl;
			std::cout<<"locating error!!"<<std::endl;
			std::cout<<"locating error!!"<<std::endl;
		}
	}
	bool isLocated {
		return located;
	}
	bool locate_error{
		return locating_error;
	}
	
};

class Smallwall {
	Point* _start;
	Point* _end;
	Point* _center;
	Point* _target;
	Point* _previous;
public:
	Smallwall(float xs,float ys, float xe, float ye, Point* target) {
		_start = new Point(xs,ys);
		_end = new Point(xe,ye);
		_center = new Point(((xe+xs)/2),((ye+ys)/2));
		_target = target;
	}
	Point* get_target() {
		return _target;
	}
	Point* get_center() {
		return _center;
	}
	Point* get_previous() {
		return _previous;
	}
	float xs() { return _xs;}
	float ys() { return _ys;}
	float xe() { return _xe;}
	float ye() { return _ye;}
	void setprevious (Point* previous) {
		_previous = previous;
	}
	float getangle () {
		return three_points_angle(previous,_center,_target);
	}
	float wall_angle() {
		return (atan((_center->x()-_previous->x())/(_center->y()-_previous->y())));
	}
	float getdistance() {
		return _current->distance(_previous);
	}
	bool equalTo(Point* pt) {
		return pt->equalTo(_center);
	}
};



