#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
#include "logger.cpp"

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
  Logger logger;

  float fdist;
  float lfdist;
  float lbdist;
  float rdist;

// Returns whether a sensor distance is in range.
bool inRange(float dist) {
  if (dist<20 && dist>1) {
    return true;
  }
  return false;
}

void stop() {
  left->stop();
  right->stop();
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
    //odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
    odo = new Odometry(_l, _r, current);
    motion = new Motion(left,right,odo,_start);
  }

  int step(int state) {
    fdist = irf->getDistance();
    lfdist = irlf->getDistance();
    lbdist = irlb->getDistance();
    rdist = irr->getDistance();
    logger.log("Roomba State", std::to_string(state));
    logger.log("Front IR", std::to_string(fdist));
    logger.log("Right IR", std::to_string(rdist));
    logger.log("Left Front IR", std::to_string(lfdist));
    logger.log("Left Right IR", std::to_string(lbdist));
    switch (state) {
      // State 0: Go forward ///////////////////////////////////////////////////
      case 0: 
        left->forward();
        right->forward();
        left->setSpeed(0.25);
        right->setSpeed(0.25);

        if (fdist < 7 || rdist < 5) { // If close in front or on right
          stop();
          return 1;
        }
        else if (lfdist < 7) { // If left is already close to wall
          stop();
          return 2;
        }
        else { // Stay if anything else 
          return 0;
        }
        return 0;
      // State 1: Rotate in place CW //////////////////////////////////////////
      case 1: 
        /* BEHAVIOR
        */
        if (lfdist < 10 && !inRange(fdist)){
          stop();
          return 2;
        }
        else{
          return 1;
        }
      // State 2: Drive parallel to wall //////////////////////////////////////
      case 2: 
        //behavior
        if (fdist < 7) { // If small corner
          stop();
          return 1;
        }
        else if (!inRange(lfdist)) { // If IRLF misses
          stop();
          return 3;
        }
        else { // Stay if anything else
          return 2;
        }
      // State 3: Pivot CCW about corner ///////////////////////////////////////
      case 3:
        //behavior
        if (!inRange(lfdist)){
          return 3;
        }
        else{
          stop();
          return 2;
        }
    }
  }

};
