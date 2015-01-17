#include <iostream>
#include "wallfollower.cpp"

const float slp=0.7;
const float big_corner_straight_distance=22;
const float distance_to_wall=10;
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
	Location* location = new Location(0.0,0.0,0.0);
	IR* irf = new IR(0);
	IR* irr = new IR(1);
	IR* irb = new IR(2);
	IR* irlf = new IR(3);
	IR* irlb = new IR(4);
	Wallfollower* wf = new Wallfollower(left,right,irlf,irlb,irr,irf,irb,location); 

	//start
	//step1 : move forward, until you see the wall
	wf->smoothforward(10);
	while(running&&!wf->close_to_wall()) { 
		wf->smooth_forward_run();
		usleep(10000);
	}
	wf->stop();
	sleep(slp);

	//step2: go parallel to the wall
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	std::cout<<"first parallel to wall! "<< <<std::endl;
	wf->smoothrotate(10);
	wf->setup_parallel_to_wall();    
	while(running&&!wf->parallel_to_wall()) {   //move parallel
	    smooth_rotate_run(); 
		usleep(10000);
	}
	wf->stop();
	sleep(slp);

	//step3: run parallel to the wall and turn if there is a corner;
	
	cornersignal = wf->incorner();
	wf->setDistance(distance_to_wall);
	wf->setup_parallelrun();
	while(running) {
		if (cornersignal==0) {
			wf->parallelrun();
			usleep(10000);
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
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			std::cout<<"small wall rotating!  angle:  small_corner_rotate_angle"<< <<std::endl;
			wf->rotate(small_corner_rotate_angle);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//go parallel to the wall again
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			std::cout<<"small wall parallel to wall! "<< <<std::endl;
			wf->setup_parallel_to_wall();
			wf->smoothrotate(-10);
			while(running&&!wf->parallel_to_wall()) {
				wf->smooth_rotate_run();
				usleep(10000);
			}
			wf->stop();
			sleep(slp);
			wf->setup_parallelrun();
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
			
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step1:  "<<big_corner_straight_distance<<std::endl;
			wf->straight(big_corner_straight_distance);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//rotate for almost -90 degrees
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			std::cout<<"big wall rotate step2:  "<<big_corner_rotate_angle<<std::endl;
			wf->rotate(big_corner_rotate_angle);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//go straight for a little
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			std::cout<<"big wall go straight step3:  "<<big_corner_straight_distance<<std::endl;
			wf->straight(big_corner_straight_distance);
			while(running&&!wf->run()) usleep(10000);
			wf->stop();
			sleep(slp);

			//
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			std::cout<<"big wall parallel to wall! "<< <<std::endl;
			wf->setup_parallel_to_wall();
			wf->smoothrotate(10);
			while(running&&!wf->parallel_to_wall()) {
				wf->smooth_rotate_run();
				usleep(10000);
			}
			wf->stop();
			sleep(slp);
			wf->setup_parallelrun();
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
