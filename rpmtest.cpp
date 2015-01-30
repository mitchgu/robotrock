#include <iostream>
#include "data.cpp"
#include "motion.cpp"

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
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Location* location = new Location(0.0,0.0,0.0);
	Odometry* odo = new Odometry(left, right, 0, 0, 0);

	Motion* motion = new Motion(left,right,odo,location); 
	//motion->rotate(1.57);	
	left->forward();
	right->forward();
	int reps=25;
	/*
	for(double i=0.1;running&&i<=0.5;i+=0.05)
	{
		double ravg=0,lavg=0;
		right->writeVolt(i);
		left->stop();
		usleep(500000);
		for(int j=0;running&&j<reps;j++)
		{
			ravg+=right->rps();
			usleep(10000);
		}
		ravg/=reps;
		std::cout<<ravg<<std::endl;
		left->stop(); right->stop();
		sleep(1);
	}
	*/
	left->setSpeed(0.5);
	right->setSpeed(0.5);
	while(running) 
	{
		//motion->run();
		usleep(100000);
	}
	left->stop(); right->stop();

	sleep(1);
	return 0;
}
