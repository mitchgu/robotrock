#include <iostream>
#include "plan.cpp"

int main()
{
	Mapdealer* map = new Mapdealer("example.txt");
//	std::cout<<"i HAVE BEEN HERE"<<std::endl;
	std::vector<Wall> outerWall = map->allWalls();
	Plan plan(outerWall);
	cPoint start(96,96);
	cPoint target(144,96);
	plan.setStart(start);
	plan.setTarget(target);
//	std::cout<<"i HAVE BEEN HERE"<<std::endl;
	std::vector<cPoint> myplan = plan.plan();
	std::cout<<"finish planning"<<std::endl;
	for (int i=0;i<myplan.size();i++) {
		std::cout<<"x: "<<myplan[i].x()<<"y: "<<myplan[i].y()<<std::endl;
	}
}
