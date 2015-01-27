#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
const float Kpw = 0.05, Kdw = 35, Kiw = 0;
const float threshelddis = 8; //for the first approaching to wall
const float distance_of_irlfb = 2.0;
const float big_corner_turn_omega = 0.08;
const float base_speed = 1.0; //parallel run base speed
const float slp=0.7;
const float distance_to_wall=4.5;
const float small_corner_rotate_angle=1.55;
const float big_corner_rotate_angle= -1.9;
const float robotwidth =12;
const float rotate_stuck_time = 10;
const float parallel_run_stuck_time = 15;
const float forward_stuck_time = 15;
const float small_corner_threshold_distance=7.0;
const float big_corner_constant = 0.7;
const float small_corner_turn_stuck_time = 10;
const float big_wall_rotate_stuck_time = 10;
const float parallel_to_wall_speed=0.5;

class Wallfollower {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	mraa::Gpio* uirb;
	//IR* irb;
	Motor* left;
	Motor* right;
	Odometry* odo;
	Location* start;
	Location* current;
	Motion* motion;
	int corner;
	float smooth_rotate_speed; float smooth_forward_speed;
	float base_angle; //the angle that parrallel to the wall
	float target; //the distance that you want the robot to stay from the wall
	float distance; //after running setAngle, the robot is this much from the wall
	bool det; bool cw; bool forw; int cnt;int cntdec; //only for setAngle
	unsigned long long backward_base_time; struct timeval btv; bool bdet; int bcnt; int bcntdec; float prebdis; // only for facing air
	unsigned long long check_stuck_base_time; struct timeval stktv;
	float integration, prerror; //only for parallelrun
	int cornercnt; // only for cornercnt
	struct timeval tv;
	float big_turn_rspeed; float big_turn_lspeed;float big_corner_distance; float base_turn_angle; float before_turn_distance;

	bool initialized;
	int channel4_mode;
	int channel5_mode;
	int mode;
	int locating_return_channel;


	void channel_stop(){
		stop();
		initialized = false;
		sleep(slp);
	}
	float estimatedistance() {    //for two ir sensors
		float irlfd = irlf->getDistance();
		float irlbd = irlb->getDistance();
		return (irlfd+irlbd)/2;
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
public:
	Wallfollower(Motor* _l, Motor* _r, IR* _irf, IR* _irr, IR* _irlf, IR* _irlb, mraa::Gpio* _uirb, Location* _start) {
		left = _l;
		right = _r;
		irlf = _irlf;
		irlb = _irlb;
		irr = _irr;
		irf = _irf;
		uirb = _uirb;
		//irb = _irb;
		current = _start;
		start=new Location(current);
		//odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		odo = new Odometry(_l, _r, current);
		motion = new Motion(left,right,odo,_start);
		corner = 0;
		cornercnt = 0;

		channel4_mode = 0;
		channel5_mode = 0;
		initialized = false;
		locating_return_channel = 0;
		before_turn_distance = 4.5;
	}

	/*
	tell Locating class which kind of signal is he passing
	return -1 means missed the wall, please give up all the previous data
	return 0 return nothing
	return 1 return the origin
	return 2 return nothing
	return 3 return the first turning point
	return 4 return small angle turning point
	return 5 return large angle turning point
	return 6 return after corner point
	*/

	int locating_channel() {
		if (locating_return_channel == 0) {
			return 0;
		}            
		else if (locating_return_channel == -1) {
			locating_return_channel = 0;
			return -1;
		}  
		else 
		{
			int return_channel = locating_return_channel;
			locating_return_channel = 0;
			return return_channel;
		}
	}

	int run_follower(int channel) {
			//std::cout<<"Position "<<location->x()<<" "<<location->y()<<" "<<location->theta()<<std::endl;
		mode = channel;
		if (channel == 0) {              //problem dealer, when you are stuck in a bad thing
			//std::cout<<"channel 0: I meat a problem "<<std::endl;
			//std::cout<<"channel 0: I meat a problem "<<std::endl;
			//std::cout<<"channel 0: I meat a problem "<<std::endl;
			if (!initialized) {
				setup_smoothforward(-1);
				gettimeofday(&stktv, NULL);
				check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +  (unsigned long long)(stktv.tv_usec) / 1000;
				initialized = true;
				locating_return_channel = -1;
				return 0;
			}
			else {
				gettimeofday(&stktv,NULL);
				if(((unsigned long long)(stktv.tv_sec)*1000 +  (unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time<1000) {
				smoothforward_run();
				return 0;
				}
				else {
					channel_stop();
					locating_return_channel = 1;
					return 1;
				}
			}
		}
		if (channel == 1) {               //step1 : move forward, until you see the wall
			//std::cout<<"channel 1: move forward to the wall "<<std::endl;
			if(!initialized){
				setup_smoothforward(1.0);
				initialized = true;
				return 1;
			}
			else{
				gettimeofday(&stktv, NULL); 
				if ((((unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> forward_stuck_time*100000) {
					//std::cout<<"running out of time"<<check_stuck_base_time<<"  "<<((unsigned long long)(stktv.tv_sec)*1000 + (unsigned long long)(stktv.tv_usec) / 1000)<<std::endl;
					initialized = false;
					return 0;
				}
				smoothforward_run();
				usleep(10000);
				if (!close_to_wall()) {
					//std::cout<<"through"<<std::endl;
					return 1;
				}
				else {
					channel_stop();
					locating_return_channel=2;
					return 2;
				}
			}
		}

		if(channel == 2) {                      //step2 : parallel to wall
			//std::cout<<"channel 2: parallel to wall! "<<std::endl;
			if (!initialized) {
				gettimeofday(&stktv,NULL);
				check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000;
				//std::cout<<"I am initialized!"<<std::endl;
				cw = true;
				setup_smoothrotate(parallel_to_wall_speed);
				setup_parallel_to_wall();
				initialized = true;
				return 2;
			}
			else {
				gettimeofday(&stktv, NULL); 
				unsigned long long time_range = (((unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time);
				//std::cout<<"time_range:"<<time_range<<std::endl;
				if(time_range>rotate_stuck_time*1000) {
					initialized = false;
					return 0;
				}
				smoothrotate_run();
				usleep(10000);
				if (!parallel_to_wall()) {
					return 2;
				}
				else {
					channel_stop();
					locating_return_channel = 3;
					return 3;
				}
			}
		}

		if(channel == 3) {                      // step3: parallel run along the wall
			//std::cout<<"channel 3: walking along the wall! "<<std::endl;
			if (!initialized) {
				setDistance(distance_to_wall);
				setup_parallelrun();
				initialized = true;
				return 3;
			}
			else {
				gettimeofday(&stktv, NULL); 
				if ((((unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> parallel_run_stuck_time*1000) {
					initialized = false;
					return 0;
				}
				parallelrun();
				usleep(10000);
				int cornersignal = incorner();
				if (cornersignal ==0 ){
					return 3;
				}
				if (cornersignal ==1) {
					channel_stop();
					locating_return_channel = 4;
					return 4;
				}
				if (cornersignal ==2){
					channel_stop();
					locating_return_channel = 5;
					return 5;
				}
			}
		}

		if (channel == 4) {                        //case4: small angle corner dealer
			//std::cout<<"channel 4: small angle corner dealer! "<<std::endl;
			if (channel4_mode == 0) {              //case4, mode 1: rotate near 90 degrees
				//std::cout<<"small angle corner rotating!  angle:  "<<small_corner_rotate_angle <<std::endl;
				if (!initialized) {
					gettimeofday(&stktv, NULL);
					check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000;
					rotate(small_corner_rotate_angle);
					initialized = true;
					return 4;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> small_corner_turn_stuck_time*1000) {
						channel4_mode =0;
						initialized = false;
						return 2;
					}
					if (!run()) {
						return 4;
					}
					else {
						channel_stop();
						channel4_mode =1;            //change to channel4 mode 1
						return 4;
					}
				}
			}
			if (channel4_mode ==1){               //case4, mode 2: go parallel to the wall 
				//std::cout<<"small angle corner parallel to wall! "<<std::endl;
				if (!initialized) {
					gettimeofday(&stktv,NULL);
					check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000;
					setup_parallel_to_wall();
					cw = false;
					setup_smoothrotate(-parallel_to_wall_speed);
					initialized = true;
					return 4;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> rotate_stuck_time*1000) {
						channel4_mode = 0;
						initialized = false;
						return 0;
					}
					if (!parallel_to_wall()) {
						smoothrotate_run();
						usleep(10000);
						return 4;
					}
					else {
						channel_stop();
						channel4_mode = 0;
						locating_return_channel = 6;
						return 3;
					}
				}
			}
		}

		if(channel == 5) {                                 //case 5: big angle corner dealer
			//std::cout<<"channel 5: big angle corner dealer! "<<std::endl;
			if (channel5_mode == 0){
				//std::cout<<"big angle corner first time straight for a little! "<<std::endl;
				if (!initialized) {
					setup_big_corner_dealer();
					initialized = true;
					return 5;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)>big_wall_rotate_stuck_time*1000) {
						channel5_mode = 0;
						initialized = false;
						return 0;
					}
					if (!big_corner_dealer()) {
						usleep(10000);
						return 5;
					}
					else {
						channel_stop();
						channel5_mode = 1;
						return 5;
					}
				}
			}
			if (channel5_mode == 1){
				//std::cout<<"big angle corner go parallel to the wall! "<<std::endl;
				if (!initialized) {
					gettimeofday(&stktv,NULL);
					check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000;
					setup_parallel_to_wall();
					cw = true;
					setup_smoothrotate(parallel_to_wall_speed);
					initialized = true;
					return 5;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> rotate_stuck_time*1000) {
						channel5_mode = 0;
						initialized = false;
						return 0;
					}
					if (!parallel_to_wall()) {
						smoothrotate_run();
						usleep(10000);
						return 5;
					}
					else {
						channel_stop();
						channel5_mode = 0;
						locating_return_channel = 6;
						return 3;
					}
				}
			}
		}                          //channel 5 ends
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
	void setup_smoothforward(float speed) {
		gettimeofday(&stktv, NULL);
		check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
		if(speed>0)
		{
			forw=true;
			left->forward();
			right->forward();
			smooth_forward_speed = speed;
			left->setSpeed(smooth_forward_speed);
			right->setSpeed(smooth_forward_speed);
		}
		else
		{
			forw= false;
			right->backward();
			left->backward();
			smooth_forward_speed = -speed;
			left->setSpeed(smooth_forward_speed);
			right->setSpeed(smooth_forward_speed);
		}
	}
	void setup_smoothrotate(float speed) {
		if(speed>0)
		{
			left->forward();
			right->backward();
			smooth_rotate_speed = speed;
			left->setTarget(smooth_rotate_speed);
			right->setTarget(smooth_rotate_speed);
		}
		else
		{
			right->forward();
			left->backward();
			smooth_rotate_speed = -speed;
			left->setTarget(smooth_rotate_speed);
			right->setTarget(smooth_rotate_speed);
		}
	}
	void smoothrotate_run() {
		left->run();
		right->run();
		odo->run();
	}
	void smoothforward_run() {
		left->run();
		right->run();
		odo->run();
	}
	void setup_parallelrun() {
		check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
		prerror = 0;
		integration = 0;
		left->forward();
		right->forward();
		left->setSpeed(base_speed);
		right->setSpeed(base_speed);
	}
	void setup_parallel_to_wall() {
		float lbdis = irlb->getDistance();
		float lfdis = irlf->getDistance();
	}
	void setup_big_corner_dealer() {
		gettimeofday(&stktv, NULL);
		check_stuck_base_time = (unsigned long long)(stktv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
		left->forward();
		right->forward();
		float big_corner_distance = before_turn_distance+2;
		big_turn_rspeed = (big_corner_distance+robotwidth)*big_corner_turn_omega;
		big_turn_lspeed = big_corner_distance * big_corner_turn_omega;
		left->setSpeed(big_turn_lspeed);
		right->setSpeed(big_turn_rspeed);
		base_turn_angle = odo->getAngle();
	}
	bool big_corner_dealer() {
		//std::cout<<"turned"<<odo->getAngle()-base_turn_angle<<std::endl;
		if ((odo->getAngle()-base_turn_angle)<-1.5){
			if (((irlf->getDistance())<5)||(irf->getDistance()<9)) {
				return true;
			}
			else {
				odo->run();
				left->run();
				right->run();
				return false;
			}
		}
		else {
			odo->run();
			left->run();
			right->run();
			return false;
		}
	}



	/*
	steps that used in the test
	*/
	int incorner(){   //0 for no corner, 1 for corner almost 90 degrees, 2 for corner almos
		if (irf->getDistance()<=small_corner_threshold_distance) { return 1;}
		//std::cout<<"irlf:  "<<irlf->getDistance()<<std::endl;
		if (irlf->getDistance()==100) { 
			if(cornercnt>1) {
				cornercnt++;
			} 
			else { 
				cornercnt = 0;
				return 2;
			}
		}
		cornercnt = 0;
		return 0;
	}
	bool close_to_wall(){
		float df = irf->getDistance();
		float dlb = irlb->getDistance();
		float dlf = irlf->getDistance();
		float dr = irr->getDistance();
		std::cout<<"front: "<<df<<" lb: "<<dlb<<" lf: "<<dlf<<" right: "<<dr<<std::endl;
		bool f = (df)<threshelddis;
		//bool b = (irb->getDistance())<threshelddis;
		bool lb = (dlb)<threshelddis;
		bool lf = (dlf)<threshelddis;
		bool r = (dr)<threshelddis;
		return (f  || lb || lf || r);
	}
	void parallelrun(){
		float dt = timeDiff();
		float dis = estimatedistance();
		float error = target-dis;
		float dif = (error-prerror)/dt;
		integration = integration + error*dt;
		float dspeed = error*Kpw+dif*Kdw+integration*Kiw;
		float irlbd = irlb->getDistance();
		if (irlbd!=100) before_turn_distance = irlbd;
		//std::cout<<"dspeed is: "<<dspeed<<"  dt:  "<<dt<<" dis: "<<dis<<std::endl;
		//dspeed = 0;
		//std::cout<<"error: "<<error*Kpw<<" dif: "<<dif*Kdw<<" int: "<<integration*Kiw<<std::endl;
		left->setTarget(base_speed+dspeed);
		right->setTarget(base_speed-dspeed);
		left->run();
		right->run();
		prerror = error;
		odo->run();
	}
	bool parallel_to_wall() {
		odo->run();
		// do parallel to wall
		float lbdis = irlb->getDistance();
		float lfdis = irlf->getDistance();
		//std::cout<<"nowlfdis:  "<<lfdis<<",  nowlbdis:  "<<lbdis<<std::endl;
		if ((lbdis>10) || (lfdis>10) ) {
			if (cw == true) {
				setup_smoothrotate(parallel_to_wall_speed);
			}
			else {
				setup_smoothrotate(-parallel_to_wall_speed);
			}
			return false;
		}
		if (((lbdis-lfdis)<0.1) && ((lfdis-lbdis)<0.1)) {
			return true;
		}
		else {
			setup_smoothrotate((lbdis-lfdis)*0.7);
			return false;
		}
		return false;
	}
};
