#include <iostream>
#include "wallfollower.cpp"

const float slp=0.5;
const float big_corner_straight_distance=22;
const float distance_to_wall=6;
const float small_corner_rotate_angle=1.9;
const float big_corner_rotate_angle= -1.9;
int running=1;
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
};

int main()
{
	signal(SIGINT, sig_handler);
	int cornersignal;
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Gyroscope* gyr = new Gyroscope(10);
	Location* location = new Location(0.0,0.0,0.0);
	IR* irf = new IR(0);
	IR* irr = new IR(1);
	IR* irb = new IR(2);
	IR* irl = new IR(3);
	Wallfollower* wf = new Wallfollower(left,right,gyr,irl,irr,irf,irb,location); 

	//start
	//step1 : move forward, until you see the wall
	wf->smoothforward(10);
	while(running&&!wf->close_to_wall()) { 
		wf->smoothrun(10);
		usleep(10000);
	}
	wf->stop();
	sleep(slp);

	//step2: go parallel to the wall
	wf->smoothrotate(10);    //move parallel
	while(running&&!wf->setAngle()) { usleep(10000);}
	wf->stop();
	sleep(slp);

	//step3: run parallel to the wall and turn if there is a corner;
	wf->smoothforward(12);       //walk along wall
	wf->setDistance(distance_to_wall);
	cornersignal = wf->incorner();
	while(running) {
		if (cornersignal==0) {
			wf->parallelrun();
		}
		if (cornersignal==1) {    //small corner dealer
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			std::cout<<"I detect a small wall!!"<<std::endl;
			//rotate for a almost 90 degrees
			wf->rotate(small_corner_rotate_angle);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//go parallel to the wall again
			wf->smoothrotate(-12);
			while(running&&!wf->setAngle()) {}
			wf->stop();
			sleep(slp);
		}
		if (cornersignal==2) {     //large corner dealer
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			std::cout<<"I detect a big wall!!"<<std::endl;
			//go straight for a little
			wf->straight(big_corner_straight_distance);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//rotate for almost -90 degrees
			wf->rotate(big_corner_rotate_angle);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//go straight for a little
			wf->straight(big_corner_straight_distance);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//
			wf->smoothrotate(10);
			while(running&&!wf->setAngle()) {}
			wf->stop();
			sleep(slp);
		}
		cornersignal = wf->incorner();   //check if there is a corner
	}

	/*
	wf->setrotate(50);
	while(running&&!wf->setAngle()) {}
	
	wf->setforward(60);
	wf->setDistance(8);
	while(running)
	{
		wf->parallelrun();
	}
	*/
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
