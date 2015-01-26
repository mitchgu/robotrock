//#include "data.cpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include "mapdealer.cpp"
#include <string>

#define pb push_back
#define mp make_pair
using namespace std;

const double robWidth=16;
const double errorMargin=6;
const double angleError=0.1;


class Localize {
	Mapdealer* map;
	cPoint *startLocation;
	Location *location;
	int numCorners;
	bool localized,foundWall,exactPoint;
	//MonteCarlos* monte ;
	std::vector<Wall> outerWall;
	std::vector<cPoint> viscPoints;
	std::vector<pair<cPoint,Wall*> > potentialLoc;
public:
	Localize(char* filename, Location *_start)
	{
		location = _start;
		viscPoints.pb(*(start->point()));
		map = new Mapdealer(filename);
		outerWall = map->allWalls();
		startLocation=map->getStart();
		numCorners=localized=0;
		foundWall=false;
		tr(outerWall) std::cout<<"Wall: "<<((it)->xs())<<" "<<((it)->ys())<<" "<<((it)->xe())<<" "<<((it)->ye())<<std::endl;
		///monte = new MonteCarlos(outer_wall);
	}
	void wallFound(double leftDist)
	{
		cPoint* at=location->point();
		//cPoint diff=(*at-*startLocation);
		viscPoints.pb(*at);
		foundWall=true;
		double distTravelled=at->abs()+(robWidth/2+leftDist);///cos(location->theta());
		std::cout<<"Left distance is "<<leftDist<<std::endl;
		std::cout<<"At "<<at->x()<<" "<<at->y()<<std::endl;
		std::cout<<"Travelled "<<distTravelled<<std::endl;
		std::cout<<"Started at "<<startLocation->x()<<" "<<startLocation->y()<<std::endl;
		tr(outerWall)
		{
			double A=it->ye()-it->ys();
			double B=it->xs()-it->xe();
			double C=it->ys()*it->xe()-it->ye()*it->xs();
			double m=startLocation->x(),n=startLocation->y();
			double dist=A*m+B*n+C;
			double sqcoeff=A*A+B*B;
			double coeff=-dist/sqcoeff;
			sqcoeff=sqrt(sqcoeff);
			dist=fabs(dist)/sqcoeff;
			//cout<<"Could be on wall "<<it->xs()<<" "<<it->ys()<<" "<<it->xe()<<" "<<it->ye()<<"\n";
			//cout<<"Distance to wall is "<<dist<<"\n";
			if(distTravelled<(dist-errorMargin)) continue;
			else if(distTravelled<dist) distTravelled=dist;
			double fx=A*(coeff)+m, fy=B*(coeff)+n;
			double r=sqrt(distTravelled*distTravelled-dist*dist);
			double dx=r*B/sqcoeff, dy=-r*A/sqcoeff;
			//cout<<"foot is "<<fx<<" "<<fy<<std::endl;
			int sign=1;
			for(int k=0;k<2;k++)
			{
				double x=fx+sign*dx,y=fy+sign*dy;
				if(x>=(min(it->xs(),it->xe())-errorMargin)&&x<=(max(it->xs(),it->xe())+errorMargin))
				{
					cout<<"Could be at ("<<x<<","<<y<<") on wall "<<it->xs()<<" "<<it->ys()<<" "<<it->xe()<<" "<<it->ye()<<"\n";

					potentialLoc.pb(mp(cPoint(x,y),&(*it)));
					if(r==0) break;
				}
				sign*=-1;
			}
		}
	}
	void atCorner()
	{
		cPoint* at=location->point();
		int vs=viscPoints.size();
		double angle=at->three_point_angle(viscPoints[vs-1],viscPoints[i-2]);
		angle=fabs(angle);
		if(angle<10)
		viscPoints.pb(*at);
	}
};
