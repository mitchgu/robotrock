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
	Motor* left = new Motor(8,6,4,false);
	Motor* right = new Motor(9,5,2,true);
	Gyroscope* gyr = new Gyroscope(10);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,gyr,location); 
	//motion->rotate(1.57);	
	motion->straight(20);
	
	for(int s=0;s<4;s++)
	{
		motion->straight(24);
		while(!motion->run()) usleep(10000);
		motion->rotate(1.57);
		while(!motion->run()) usleep(10000);
	}
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
