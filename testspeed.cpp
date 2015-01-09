#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>

#include "mraa.hpp"

int running=1;

mraa::Pwm jpwm = mraa::Pwm(5);
mraa::Gpio jdir = mraa::Gpio(12);
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
}

int main() 
{
	signal(SIGINT, sig_handler);
	jpwm.enable(true);
	jdir.dir(mraa::DIR_OUT);
	double speed = 0.3;
	jdir.write(1);
	jpwm.write(speed);
	while(running) sleep(0.001);
	jpwm.write(0);
	sleep(5);
	return 0;
}
