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
	rightDir.dir(mraa::DIR_OUT);
	leftDir.dir(mraa::DIR_OUT);
	rightPwm.enable(true);
	leftPwm.enable(true);

	float value = 0.1f;

	leftPwm.write(value);
	rightPwm.write(value);
	rightDir.write(1);
	leftDir.write(1);
	while(running);
	leftPwm.write(0);
	rightPwm.write(1);
	return 0;
}
