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
	Odometry* odo = new Odometry(left, right, 0, 0, 0);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,odo,location); 
	left->forward();
	right->forward();
	//motion->rotate(1.57);	
	motion->straight(24,false);
	while(running&&!motion->run()) usleep(800000);
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
