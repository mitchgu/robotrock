#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>

#include "mraa.hpp"

int running=1;

mraa::Gpio input = mraa::Gpio(12);
mraa::Gpio output= mraa::Gpio(11);
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
	output.write(0);
        running = 0;
    }
}

int main() {
	signal(SIGINT, sig_handler);
	input.dir(mraa::DIR_IN);
	output.dir(mraa::DIR_OUT);
	output.write(1);
	int i = input.read();
	std::cout<<"first read "<<i<<std::endl;
	sleep(0.001);
	int j = 0,prev=0;
	while(running)
	{
		int k = input.read();
		if (k!=i) {
			fprintf(stdout,"change in %d ms\n", j-prev);
			i = k,prev=j;
		}
		j++;
		sleep(0.001);
	}
	return 0;
}
