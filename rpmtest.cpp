#include <iostream>
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
	Odometry* odo = new Odometry(left, right, 0, 0, 0);
	Location* location = new Location(0.0,0.0,0.0);

	//motion->rotate(1.57);	
	left->forward();
	right->forward();
	int reps=25;
	for(double i=0.1;running&&i<=0.5;i+=0.05)
	{
		double ravg=0,lavg=0;
		left->writeVolt(i);
		usleep(500000);
		for(int j=0;running&&j<reps;j++)
		{
			ravg+=left->rps();
			usleep(10000);
		}
		ravg/=reps;
		std::cout<<i<<" "<<ravg<<std::endl;
		left->stop(); right->stop();
		sleep(1);
	}
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
