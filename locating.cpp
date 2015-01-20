#include "wallfollower.cpp"
#include "wallfollowertest.cpp"
#include "data.cpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

class Locating {
	Location* start;
	Point* mapstart;
	Point* first; Point* second; Point* third; Point* forth;
	Point* previous;
	int current_channel;
	Location* update_location;
	Mapdealer* map;
	MonteCarlos* monte ;
	std::deque<Wall*>* outer_wall;
public:
	Locating(string str, Location* _start){
		start = _start;
		map = new Mapdealer(str);
		std::deque<Wall*>* walls = map->stack_of_wall();
		Polygon* polygon = new Polygon(walls);
		outer_wall = polygon->field_wall();
		monte = new MonteCarlos(outer_wall);
		first = start->point();
		previous = start->point();
		mapstart = map->return_start();
	}
	bool decision(){
		if (!((current_channel == 4)||(current_channel == 5))) return true;
		else {
			return !monte->isLocated();
		}

	}
	void channel_switch(int channel){
		current_channel = channel;
	}
	void data(Location* location){
		if ((current_channel ==3)) { 
			second = location->point();
			third = location->point();
		}      
		if ((current_channel ==4)|| (current_channel ==5)) {  //calculate the angle and the distance
			forth = location->point();
			Point* update_point = first->four_points_crossing(second,third,forth);
			float update_angle = first->three_points_angle(update_point,forth);
			float update_distance = previous->distance(update_point);
			previous = update_point;
			monte->update(update_angle,update_distance);
		}
		if (current_channel==6) { // calculate nothing
			first = third;
			second = forth;
			third = location->point();
		}
	}
	bool locate_failure(){
		return monte->locate_error();
	}
	void setup_startpoint(Location* _start){
		start = _start;
	}
	Location*  get_location() {
		float dis = forth->distance(previous);

		float return_angle = monte->get_final_angle;
		float return_x = monte->previous_location+dis*sin(return_angle);
		float return_y = monte->previous_location+dis*cos(return_angle);
		Location* return_location = new Location(return_x,return_y,return_angle);
	}
}