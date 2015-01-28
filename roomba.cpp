#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"

class Roomba {
  IR* irlf;
  IR* irlb;
  IR* irr;
  IR* irf;
  mraa::Gpio* uirb;

  Motor* left;
  Motor* right;
  Odometry* odo;
  Location* start;
  Location* current;
  Motion* motion;

public:
  Roomba(Motor* _l, Motor* _r, IR* _irf, IR* _irr, IR* _irlf, IR* _irlb, mraa::Gpio* _uirb, Location* _start) {
    left = _l;
    right = _r;
    irlf = _irlf;
    irlb = _irlb;
    irr = _irr;
    irf = _irf;
    uirb = _uirb;

    current = _start;
    start=new Location(current);
    //odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
    odo = new Odometry(_l, _r, current);
    motion = new Motion(left,right,odo,_start);
  }

  int step(int state) {
    if (state == 0) {
      if (irf->getDistance() < 10) {
        left->stop();
        right->stop();
        return 1;
      }
      else {
        left->forward();
        right->forward();
        left->setSpeed(0.25);
        right->setSpeed(0.25);
        return 0;
      }
    }
    if (state == 1) {
      //do nothing
      return 1;
    }
  }

};
