#include <cassert>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>

int running=1;
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
}

/*
set the pin first, dir, pwm are already well set before using this 
function
set the motor turning clockwise in speed (round/s)
not done yet*/
void clockwise(mraa::Gpio& dir, mraa::Pwm& pwm, double speed ) 
{
	dir.write(1);
	if (speed== 0.0) 
	{
		pwm.write(speed);
	}

}
/*
moving forward for time and all the other pin are well set before using 
this function
time (s)
and then stop
*/
void forward(mraa::Gpio& vdir, mraa::Pwm& vpwm, mraa::Gpio& jdir, 
mraa::Pwm& jpwm, double time) 
{
	double speed = 0.2;
	vdir.write(1);
	jdir.write(0);
	jpwm.write(speed);
	vpwm.write(speed);
	sleep(time);
	vpwm.write(0);
	jpwm.write(0);

}
int main()
{
	signal(SIGINT, sig_handler);
	mraa::Gpio vdir = mraa::Gpio(8);
	mraa::Gpio jdir = mraa::Gpio(12);
	mraa::Pwm jpwm = mraa::Pwm(5);
	mraa::Pwm vpwm = mraa::Pwm(3);
	vpwm.enable(true);
	jpwm.enable(true);
	vdir.dir(mraa::DIR_OUT);
	jdir.dir(mraa::DIR_OUT);
	forward(vdir,vpwm,jdir,jpwm,4);
	while(running) sleep(0.001);
    	jpwm.write(0);	
    	sleep(1);
    	return 0;
}
