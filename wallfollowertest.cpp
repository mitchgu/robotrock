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
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Gyroscope* gyr = new Gyroscope(10);
	Location* location = new Location(0.0,0.0,0.0);
	IR* irr = new IR(1);
	IR* irl = new IR(3);
	Wallfollower* wf = new Wallfollower(left,right,gyr,irr,irl,location); 
	while(running&&!wf->setAngle()) {}
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
