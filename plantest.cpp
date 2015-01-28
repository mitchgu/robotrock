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
	std::vector<Wall> outerWall = map->allWalls();
	Plan plan(outerWall);
	cPoint start(72,120);
	cPoint target(168,144);
	plan.setStart(start);
	plan.setTarget(target);
	std::vector<cPoint> myplan = plan.plan();
	std::cout<<"finish planning"<<std::endl;
	for (int i=0;i<myplan.size();i++) {
		std::cout<<"x: "<<myplan[i].x()<<"y: "<<myplan[i].y()<<std::endl;
	}
}
