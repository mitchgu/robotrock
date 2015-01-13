#include <iostream>
#include "motor.cpp"
#include "gyro.cpp"

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
	Motor* left = new Motor(8,6,4,false)
	Motor* right = new Motor(9,5,2,true);
	Gyroscope* gyr = new Gyroscope(10);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,gyr,location); 
	motion->rotate(1.57);	
	

	/*const float target=0, K=0.003, base=0.15;

	left.forward();
	right.forward();

	left.setSpeed(base);
	right.setSpeed(base);

	
	while(running) 
	{
		
		float angle=gyr.run();
		float diff=(angle-target)*K;
		std::cout<<angle<<" "<<diff<<std::endl;
		left.setSpeed(base);
		right.setSpeed(base);
		if(diff>0) left.setSpeed(base+diff);
		else right.setSpeed(base-diff);
		sleep(0.01);
		
		float realspeed = right.rps();
		std::cout<<realspeed<<" "<<std::endl;
	}
	*/

	left.setSpeed(0);
	right.setSpeed(0);
    	sleep(1);
    	return 0;
}
