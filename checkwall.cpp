#include <iostream>
#include <vector>
#include "data.cpp"
#include <math.h>
class Checkwall {
	IR* ir1;
	IR* ir2;
	IR* ir3;
	IR* ir4;
	float dis1, dis2, dis3, dis4;
	Location* wall;
	bool runinwall;
	const float DT=10.0;
public:
	Checkwall (IR* _ir1, IR* _ir2, IR* _ir3, IR* _ir4) {
		ir1 = _ir1;
		ir2 = _ir2;
		ir3 = _ir3;
		ir4 = _ir4;
		dis1 = 20.0; dis2 = 20.0; dis3 = 20.0; dis4 = 15.0;
		runinwall = false;
	}
	bool check() {
		float dis1 = ir1->getDistance();
		float dis2 = ir2->getDistance();
		float dis3 = ir3->getDistance();
		float dis4 = ir4->getDistance();
		return ((dis1>DT)||(dis2>DT)||(dis3>DT)||(dis4>DT));
	}
	std::vector<Location> location_of_wall(){
		std::vector<Location> list;
		list.push_back(asdfasdf);
		list.push_back(asdffad);
		return list;
std::vector<Location> list = wall.location_of_wall();
		for (int i = 0; i < list.size(); ++i) {
			Location loc = list[i];
		}
	}
}