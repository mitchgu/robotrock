#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>

#include "mraa.hpp"

int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
}

int main() {
	signal(SIGINT, sig_handler);
	mraa::Pwm leftPwm = mraa::Pwm(3);
	mraa::Pwm rightPwm = mraa::Pwm(5);
	mraa::Gpio leftDir=mraa::Gpio(11);
	mraa::Gpio rightDir=mraa::Gpio(12);
	mraa::Gpio leftHall=mraa::Gpio(2);
	mraa::Gpio rightHall=mraa::Gpio(4);

	rightDir.dir(mraa::DIR_OUT);
	leftDir.dir(mraa::DIR_OUT);
	rightHall.dir(mraa::DIR_IN);
	leftHall.dir(mraa::DIR_IN);
	rightPwm.enable(true);
	leftPwm.enable(true);

	float value = 0.5f;

	leftPwm.write(value);
	rightPwm.write(value);
	rightDir.write(0);
	leftDir.write(1);
	int i = rightHall.read();
	std::cout<<"first read "<<i<<std::endl;
	sleep(0.001);
	int j = 0,prev=0;
	while(running)
	{
		int k = rightHall.read();
		//std::cout<<k<<std::endl;
		if (k!=i) {
			fprintf(stdout,"change in %d ms\n", j-prev);
			i = k,prev=j;
		}
		j++;
		sleep(0.001);
	}
	leftPwm.write(0);
	rightPwm.write(0);
	return 0;
}
