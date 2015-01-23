//#include "data.cpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#define pb push_back
using namespace std;

class Localize {
	Mapdealer* map;
	Point *startLocation;
	Location *location;
	int numCorners;
	bool localized,foundWall;
	//MonteCarlos* monte ;
	std::vector<Wall> outerWall;
	std::vector<Point> visPoints;
	std::vector<pair<Point,Wall*> > potentialLoc;
public:
	Localize(string filename, Location _start)
	{
		location = _start;
		map = new Mapdealer(filename);
		outerWall = map->getPolygon();
		startLocation=map->getStart();
		numCorners=localized=0;
		foundWall=false;
		///monte = new MonteCarlos(outer_wall);
	}
	void wallFound()
	{
		Point* at=location->point();
		visPoints.pb(*at);
		foundWall=true;
		double distTravelled=at.abs();
		std::cout"At "<<at->x()<<" "<<at->y()<<std::endl;
		std::cout<<"Travelled "<<distTravelled<<std::endl;
		tr(outerWall)
		{
			double A=it->ye()-it->ys();
			double B=it->xs()-it->xe();
			double C=it->ys()*it->xe()-it->ye()*it->xs();
			double m=at->x(),n=at->y();
			double dist=A*m+B*n+C;
			double sqcoeff=A*A+B*B;
			double coeff=-dist/sqcoeff;
			sqcoeff=sqrt(sqcoeff);
			dist=fabs(dist)/sqcoeff;
			if(distTravelled<dist) continue;
			double fx=a*(coeff)+m, fy=b*(coeff)+n;
			double r=sqrt(distTravelled*distTravelled-dist*dist);
			double dx=r*B/sqcoeff, dy=-r*A/sqcoeff;
			int sign=1;
			for(int k=0;k<2;k++)
			{
				double x=fx+sign*dx,y=fy+sign*dy;
				if(x>=min(it->xs(),it->xe())&&x<=max(it->xs(),it->xe()))
				{
					cout<<"Could be at ("<<x<<","<<y<<") on wall "<<it->xs()<<" "<<it->ys()<<" "<<it->xe()<<" "<<it->ye()<<"\n";
					potentialLoc.pb(mp(Point(x,y),it));
				}
				sign*=-1;
			}
		}
	}
	void atCorner(Location* cur)
	{

	}
};
