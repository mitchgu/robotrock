#include <iostream>
#include "roomba.cpp"

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
  Roomba* roomba= new Roomba(left,right,irf,irr,irlf,irlb,uirb,location);
  int state=0;
  //bool localized = false;
  while(running) {// &&!localized) {
    state = roomba->step(state);
    usleep(100*1000);
    //int mode=wf->locating_channel();
  } 
  left->stop();
  right->stop();
  sleep(1); 
  return 0;
}

