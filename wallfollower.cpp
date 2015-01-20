#include "motion.cpp"
#include <iostream>
#include "shortIR.cpp"
const float Kpw = 3, Kdw = 3, Kiw = 2;
const float threshelddis = 9.0; //for the first approaching to wall
const float fm_angle = 0.3;   //
const float base_speed = 15;
const float slp=0.7;
const float big_corner_straight_distance=22;
const float distance_to_wall=10;
const float small_corner_rotate_angle=1.9;
const float big_corner_rotate_angle= -1.9;
const float robotwidth =13;
const float rotate_stuck_time = 10;
const float parallel_run_stuck_time = 15;
const float forward_stuck_time = 15;


class Wallfollower {
	IR* irlf;
	IR* irlb;
	IR* irr;
	IR* irf;
	IR* irb;
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
	float integration,prerror; //only for parallelrun
	int cornercnt; // only for cornercnt
	struct timeval tv;

	bool initialized;
	int channel4_mode;
	int channel5_mode;
	int mode;
	int locating_return_channel;
	Location* crossing_point;
	Location* return_point; // the data that is going to pass to Locating class


	void channel_stop(){
		stop();
		initialized = false;
		sleep(slp);
	}
	float estimatedistance() {    //for two ir sensors
		float irlfd = irlf->getDistance();
		float irlbd = irlb->getDistance();
		return (irlfd+irlbd)*cos(fm_angle)/2;
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
	Wallfollower(Motor* _l, Motor* _r, IR* _irlf, IR* _irlb, IR* _irr,IR* _irf,IR* _irb, Location* _start) {
		left = _l;
		right = _r;
		irlf = _irlf;
		irlb = _irlb;
		irr = _irr;
		irf = _irf;
		irb = _irb;
		start = _start;
		current = _start;
		odo = new Odometry(_l, _r, _start->x(),_start->y(),_start->theta());
		motion = new Motion(left,right,odo,_start);
		corner = 0;
		cornercnt = 0;

		channel4_mode = 0;
		channel5_mode = 0;
		initialized = false;
		locating_return_channel = 0;
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
		if (locating_return_channel == -1) {
			locating_return_channel = 0;
			return -1;
		}  
		if ((locating_return_channel !=0) &&(locating_return_channel !=(-1))) {
			float shortside = (robotwidth/2)+estimatedistance();
			crossing_point = current->move(shortside, -1.5708);
			return_point = crossing_point;
			float return_channel = locating_return_channel;
			locating_return_channel = 0;
			return return_channel;
		}
	}
	Location* data() {
		return return_point;
	}

	int run_follower(int channel) {
		mode = channel;
		if (channel == 0) {               //problem dealer, when you are stuck in a bad thing
			std::cout<<"channel 0: I meat a problem "<<std::endl;
			if(!initialized) {
				setup_smoothrotate(10);
				setup_back_facing_wall();
				initialized = true;
				locating_return_channel = -1;
				return 0;
			}
			else {
				smoothrotate_run();
				usleep(10000);
				if (!back_facing_wall()) {
					return 0;
				}
				else {
					stop();
					locating_return_channel = 1;
					return 1;
				}
			}
		}
		if (channel == 1) {               //step1 : move forward, until you see the wall
			std::cout<<"channel 1: move forward to the wall "<<std::endl;
			if(!initialized){
				setup_smoothforward(10);
				initialized = true;
				return 1;
			}
			else{
				gettimeofday(&stktv, NULL); 
				if ((((unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> forward_stuck_time*1000) {
					return 0;
				}
				smoothforward_run();
				usleep(10000);
				if (!close_to_wall()) {
					return 1;
				}
				else {
					stop();
					locating_return_channel=2;
					return 2;
				}
			}
		}

		if(channel == 2) {                      //step2 : parallel to wall
			std::cout<<"channel 2: parallel to wall! "<<std::endl;
			if (!initialized) {
				setup_smoothrotate(10);
				setup_parallel_to_wall();
				initialized = true;
				return 2;
			}
			else {
				gettimeofday(&stktv, NULL); 
				if ((((unsigned long long)(stktv.tv_sec)*1000 +
					(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> rotate_stuck_time*1000) {
					return 0;
				}
				smoothrotate_run();
				usleep(10000);
				if (!parallel_to_wall()) {
					return 2;
				}
				else {
					stop();
					locating_return_channel = 3;
					return 3;
				}
			}
		}

		if(channel == 3) {                      // step3: parallel run along the wall
			std::cout<<"channel 3: walking along the wall! "<<std::endl;
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
					return 0;
				}
				parallelrun();
				usleep(10000);
				int cornersignal = incorner();
				if (cornersignal ==0 ){
					return 3;
				}
				if (cornersignal ==1) {
					stop();
					locating_return_channel = 4;
					return 4;
				}
				if (cornersignal ==2){
					stop();
					locating_return_channel = 5;
					return 5;
				}
			}
		}

		if (channel == 4) {                        //case4: small angle corner dealer
			std::cout<<"channel 4: small angle corner dealer! "<<std::endl;
			if (channel4_mode == 0) {              //case4, mode 1: rotate near 90 degrees
				std::cout<<"small angle corner rotating!  angle:  "<<small_corner_rotate_angle <<std::endl;
				if (!initialized) {
					rotate(small_corner_rotate_angle);
					initialized = true;
					return 4;
				}
				else {
					if (!run()) {
						usleep(10000);
						return 4;
					}
					else {
						stop();
						channel4_mode =1;            //change to channel4 mode 1
						return 4;
					}
				}
			}
			if (channel4_mode ==1){               //case4, mode 2: go parallel to the wall 
				std::cout<<"small angle corner parallel to wall! "<<std::endl;
				if (!initialized) {
					setup_parallel_to_wall();
					setup_smoothrotate(-10);
					initialized = true;
					return 4;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> rotate_stuck_time*1000) {
						return 0;
					}
					if (!parallel_to_wall()) {
						smoothrotate_run();
						usleep(10000);
						return 4;
					}
					else {
						stop();
						channel4_mode = 0;
						locating_return_channel = 6;
						return 3;
					}
				}
			}
		}

		if(channel == 5) {                                 //case 5: big angle corner dealer
			std::cout<<"channel 4: big angle corner dealer! "<<std::endl;
			if (channel5_mode == 0){
				std::cout<<"big angle corner first time straight for a little! "<<std::endl;
				if (!initialized) {
					straight(big_corner_straight_distance);
					initialized = true;
					return 5;
				}
				else {
					if (!run()) {
						usleep(10000);
						return 5;
					}
					else {
						stop();
						channel5_mode = 1;
						return 5;
					}
				}
			}
			if (channel5_mode == 1){
				std::cout<<"big angle corner turn! "<<std::endl;
				if (!initialized) {
					rotate(big_corner_rotate_angle);
					initialized = true;
					return 5;
				}
				else {
					if (!run()) {
						usleep(10000);
						return 5;
					}
					else {
						stop();
						channel5_mode = 2;
						return 5;
					}
				}
			}
			if (channel5_mode == 2){
				std::cout<<"big angle corner second time straight for a little! "<<std::endl;
				if (!initialized) {
					straight(big_corner_straight_distance);
					initialized = true;
					return 5;
				}
				else {
					if (!run()) {
						usleep(10000);
						return 5;
					}
					else {
						stop();
						channel5_mode = 3;
						return 5;
					}
				}
			}
			if (channel5_mode == 3){
				std::cout<<"big angle corner go parallel to the wall! "<<std::endl;
				if (!initialized) {
					setup_parallel_to_wall();
					setup_smoothrotate(10);
					initialized = true;
					return 5;
				}
				else {
					gettimeofday(&stktv, NULL); 
					if ((((unsigned long long)(stktv.tv_sec)*1000 +
						(unsigned long long)(stktv.tv_usec) / 1000)-check_stuck_base_time)> rotate_stuck_time*1000) {
						return 0;
					}
					if (!parallel_to_wall()) {
						smoothrotate_run();
						usleep(10000);
						return 5;
					}
					else {
						stop();
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
		check_stuck_base_time = (unsigned long long)(tv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
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
	void setup_smoothrotate(float speed) {
		check_stuck_base_time = (unsigned long long)(tv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
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
	void smoothrotate_run() {
		left->setSpeed(smooth_rotate_speed);
		right->setSpeed(smooth_rotate_speed);
		current = odo->run();
	}
	void smoothforward_run() {
		left->setSpeed(smooth_forward_speed);
		right->setSpeed(smooth_forward_speed);
		current = odo->run();
	}
	void setup_parallelrun() {
		check_stuck_base_time = (unsigned long long)(tv.tv_sec)*1000 +
			(unsigned long long)(stktv.tv_usec) / 1000;
		integration = 0;
		prerror = 0;
		float dt = timeDiff();
	}
	void setup_parallel_to_wall() {
		cntdec = 0, cnt = 0, det = false;
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
		return (f || b || lb || lf || r);
	}
	void parallelrun(){
		float irlfd = irlf->getDistance();
		float irlbd = irlb->getDistance();
		float estimatedis = (irlfd+irlbd)*cos(fm_angle)/2;
		std::cout<<"running parallel wall"<<std::endl;
		std::cout<<"distance from the wall:  "<<estimatedis<<std::endl;
		float dt = timeDiff();
		float error = target - estimatedis;
		float dif = irlf-irlb;  //(error-prerror)/dt;
		integration = integration + error*dt;
		float dspeed = error*Kpw+dif*Kdw+integration*Kiw;
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
			if(irlb>irlf) {
				cnt=0;   
				if(cntdec<5) cntdec++;
				else {
					std::cout<<"detection mode"<<std::endl;
					det = true;
				}
			}
			else {
				if(cntdec<5) cntdec = 0;
				if (!det) return false;
				else {
					if (cnt>0) 	{
						std::cout<<"this is left cnt = 1"<<std::endl;
						return true;
					}
					else cnt++;
				}
			}
			return false;
		}
		else {
			std::cout<<"paralleling counterclockwise"<<std::endl;
			if(irlb<irlf) {
				cnt=0;   
				if(cntdec<5) cntdec++;
				else {
					std::cout<<"detection mode"<<std::endl;
					det = true;
				}
			}
			else {
				if(cntdec<5) cntdec = 0;
				if (!det) return false;
				else {
					if (cnt>0)	{
						std::cout<<"this is left cnt = 1"<<std::endl;
						return true;
					}
					else cnt++;
				}				
			}
			return false;
		}
	}
	void setup_back_facing_wall() {
		bcntdec = 0, bcnt = 0, bdet = false;
		prebdis = irb->getDistance();
		gettimeofday(&btv, NULL);
		backward_base_time = (unsigned long long)(btv.tv_sec)*1000 +
				(unsigned long long)(btv.tv_usec) / 1000;
	}
	bool back_facing_wall() {
		gettimeofday(&btv, NULL);                //make sure won't go into deadlock
		if ((((unsigned long long)(btv.tv_sec)*1000 +
				(unsigned long long)(btv.tv_usec) / 1000)-backward_base_time)> 5000) {
			return true;
		}
		float bdis =irb->getDistance();
		if(prebdis>bdis) {
			prebdis = bdis, bcnt=0;
			if(bcntdec<5) bcntdec++;
			else {
				std::cout<<"detection to face the air"<<std::endl;
				bdet = true;
			}
		}
		else 
		{
			prebdis = bdis;
			if(bcntdec<5) bcntdec = 0;
			if (!bdet) return false;
			else 
			{
				if (bcnt>0) 
				{
					std::cout<<"now facing backward!!"<<std::endl;
					return true;
				}
				else bcnt++;
			}
		}
		return false;
	}
};



/*





*/


