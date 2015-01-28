#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
#include "logger.cpp"

#define 0 APPROACH
#define 1 ALIGN
#define 2 PARALLEL

const float FORWARD_SPEED = .75;
const float ROTATE_SPEED = .75;
const float PARALLEL_DIST_TARGET = 5.0;
const float PARALLEL_DIST_P = .15;
const float PARALLEL_ANGLE_P = .2;
const float PARALLEL_ROTATE_P = 1.0;
const float FORWARD_SCALE_FACTOR = 0.75;

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
  float parallel_dist;
  float parallel_angle;
  float rotateSpeed;
  float forwardScale;

// Returns whether a sensor distance is in range.
bool inRange(float dist) {
  if (dist<20) {
    return true;
  }
  return false;
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
    logger.log("Left Back IR", std::to_string(lbdist));
    switch (state) {
      // State 0: Go forward ///////////////////////////////////////////////////
      case APPROACH: 
        setMotor("left", FORWARD_SPEED);
        setMotor("right", FORWARD_SPEED);

        if (fdist < 8 || rdist < 5) { // If close in front or on right
          stop();
          return ALIGN;
        }
        else if (lfdist < 9) { // If left is already close to wall
          stop();
          return PARALLEL;
        }
        else { // Stay if anything else 
          return APPROACH;
        }
        return 0;
      // State 1: Rotate in place CW //////////////////////////////////////////
      case ALIGN: 
        setMotor("left", ROTATE_SPEED);
        setMotor("right", -ROTATE_SPEED);
        
        if (lfdist < 9 && fdist > 14){ //If close to wall on left, clear in front
          stop();
          return 2;
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

        //logger.log("Parallel Dist V", std::to_string(PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET)));
        //logger.log("Parallel Angle V", std::to_string(PARALLEL_ANGLE_P * parallel_angle));

        rotateSpeed = std::min(PARALLEL_ROTATE_P * (PARALLEL_DIST_P * (parallel_dist - PARALLEL_DIST_TARGET) + PARALLEL_ANGLE_P * parallel_angle), 2.0f);
        forwardScale = std::max(1-FORWARD_SCALE_FACTOR*std::abs(rotateSpeed),-0.0f) * FORWARD_SPEED;

        logger.log("Rotate Speed", std::to_string(rotateSpeed));
        logger.log("Forward Scale", std::to_string(forwardScale));
        if (rotateSpeed > 0) {
          setMotor("left", forwardScale);
          setMotor("right", forwardScale + rotateSpeed);
        }
        else {
          setMotor("left", forwardScale - rotateSpeed);
          setMotor("right", forwardScale);
        }

        if (fdist < 7) { // If small corner
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
      // State 3: Pivot CCW about corner ///////////////////////////////////////
    }
  }

};
