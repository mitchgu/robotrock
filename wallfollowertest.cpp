#include <iostream>
#include "wallfollower.cpp"

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
	IR* irl = new IR(3);
	Wallfollower* wf = new Wallfollower(left,right,gyr,irl,irr,irf,location); 
	wf->setrotate(50);    //move parallel
	while(running&&!wf->setAngle()) {};
	wf->stop();
		
	wf->setforward(60); //walk along wall
	wf->setDistance(8);
	cornersignal = wf->incorner();
	while(running) {
		if (cornersignal==0) {
			wf->parallelrun();
		}
		if (cornersignal==1) { //small corner dealer
			(wf->getmotion())->rotate(-1.9);
			while(running&&!(wf->getmotion())->run()) usleep(10000);
			sleep(1.0);
			wf->setrotate(50);
			while(running&&!wf->setAngle()) {}
			wf->stop();
			sleep(1.0);
		}
		if (cornersignal==2) { //large corner dealer
			(wf->getmotion())->straight(25);
			while(running&&!(wf->getmotion())->run()) usleep(10000);
			sleep(1.0);
			(wf->getmotion())->rotate(-1.5);
			while(running&&!(wf->getmotion())->run()) usleep(10000);
			sleep(1.0);
			(wf->getmotion())->straight(25);
			while(running&&!(wf->getmotion())->run()) usleep(10000);
			sleep(1.0);
			wf->stop();
		}
		cornersignal = wf->incorner();
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
