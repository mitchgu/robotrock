#include <iostream>
#include "wallfollower.cpp"
#include "localize.cpp"

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
	IR* irlf = new IR(3);
	IR* irlb = new IR(2);
	mraa::Gpio* uirb = new mraa::Gpio(8);
	Wallfollower* wf= new Wallfollower(left,right,irf,irr,irlf,irlb,uirb,location);
	Localize* loc=new Localize("example.txt",location);
	int channel=1,mode=1,pm=1;
	bool localized = false;
	while(running &&!localized) {
		pm=mode;
		channel = wf->run_follower(channel);
		mode=wf->locating_channel();
		std::cout<<"Returned channel "<<mode<<std::endl;
		if(mode!=pm)
		{
			if(pm==3&&!localized)
			{
				loc->wallFound(irlf->getDistance());
				running=0;
				localized=true;
			}
			else if(mode==4||mode==5)
			{
			}
		}
	} 
	left->stop();
	right->stop();
	sleep(1);	
	return 0;
}

