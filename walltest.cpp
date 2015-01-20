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

int main(){
	signal(SIGINT, sig_handler);
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Location* location = new Location(0.0,0.0,0.0);
	IR* irf = new IR(0);
	IR* irr = new IR(1);
	//IR* irb = new IR(2);
	IR* irlf = new IR(3);
	IR* irlb = new IR(2);
	Wallfollower* wf= new Wallfollower(left,right,irlf,irlb,irr,irf,location);
	int channel=1;
	bool localized = false;
	while(running &&!localized) {
		channel = wf->run_follower(channel);
	} 
	left->stop();
	right->stop();
	sleep(1);	
};
