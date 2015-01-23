#include "wallfollower.cpp"
#include "wallfollowertest.cpp"
#include "data.cpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#define pb push_back
using namespace std;

class Localize {
	Mapdealer* map;
	Point *startLocation;
	Location *start;
	int numCorners;
	bool localized,foundWall;
	//MonteCarlos* monte ;
	std::vector<Wall> outerWall;
	std::vector<Point> visPoints;
	std::vector<pair<Point,Wall*> > potentialLoc;
public:
	Localize(string filename, Location _start)
	{
		start = _start;
		map = new Mapdealer(filename);
		outerWall = map->getPolygon();
		startLocation=map->getStart();
		numCorners=localized=0;
		foundWall=false;
		///monte = new MonteCarlos(outer_wall);
	}
	void wallFound(Location* cur)
	{
		visPoints.pb(cur->point());
		foundWall=true;
		tr(outerWall)
		{
			
		}
	}
	void atCorner(Location* cur)
	{

	}
};
