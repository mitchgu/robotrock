#include <iostream>
#include "plan.cpp"

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
	Mapdealer* map = new Mapdealer("example.txt");
//	std::cout<<"i HAVE BEEN HERE"<<std::endl;
	std::vector<Wall> outerWall = map->allWalls();
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Location* location = new Location(0.0,0.0,0.0);
	Location* start(96,96,0);
	cPoint target(144,96);
	Plan plan(outerWall,left,right,location);
	plan.setStart(start);
	plan.setTarget(target);
//	std::cout<<"i HAVE BEEN HERE"<<std::endl;
	int state = 0;
	while(running && (channel!=3)) {
		state = plan.run(state);
	}
	left->stop(); right->stop();
	sleep(1);
	return 0;
}
