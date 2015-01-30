#include "servo.cpp"
#include <csignal>
#include <iostream>
#include "mraa.hpp"
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
	Servo claw(15),doorl(13,false),doorr(12,false);
	while(running)
	{
		claw.write(0.2);
		doorl.write(0.2);
		doorr.write(0.2);
		std::cout<<"wriitng 0.2"<<std::endl;
		sleep(1);
		claw.write(0.9);
		doorl.write(0.9);
		doorr.write(0.9);
		sleep(1);
	}
	claw.release();
	doorl.release();
	doorr.release();
	return 0;
}
