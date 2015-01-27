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
	std::vector<std::pair<cPoint,Wall> > potentialLoc;
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
					double angle=viscPoints[0].three_point_angle(it->s(), it->e());
					//double angle2=viscPoints[0].point()->three_point_angle(new Point(x,y), it->e());
					//double actAngle=fabs(location->theta()-viscPoints[0].theta());
					//std::cout<<"Angles "<<angle1<<" "<<angle2<<" "<<actAngle<<std::endl;
					Wall add=*it;
					if(angle>0) add.swap();
					cout<<"Could be at ("<<x<<","<<y<<") on wall "<<it->xs()<<" "<<it->ys()<<" "<<it->xe()<<" "<<it->ye()<<"\n";
					potentialLoc.pb(mp(cPoint(x,y),add));
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
		if(angle<10) return;
		double distTravelled=((*at)-viscPoints[vs-1]).abs();
		int pl=potentialLoc.size();
		std::vector<std::pair<cPoint,Wall> > newLoc;
		REP(i,pl)
		{
			double wallDist=(potentialLoc[i].second.e()-potentialLoc[i].first).abs();
			if(wallDist-errorMargin>distTravelled||wallDist+errorMargin<distTravelled) continue;
			tr(outerWall)
			{
				if(it->s()==potentialLoc[i].second.e()&&it->e()!=potentialLoc[i].second.s())
				{
					potentialLoc.pb(mp(potentialLoc[i].second.e(),*it));
				}
				else if(it->s()==potentialLoc[i].second.e()&&it->e()!=potentialLoc[i].second.s())
				{
					Wall add=*it;
					add.swap();
					potentialLoc.pb(mp(potentialLoc[i].second.e(),add));
				}
			}
		}
		potentialLoc=newLoc;
		viscPoints.pb(*at);
		debug();
	}
	void debug()
	{
		REP(i,pl)
		{
			std::cout<<"Maybe on wall"<<potentialLoc[i].second.xs()<<"  "<<potentialLoc[i].second.ys()<<" to "<<potentialLoc[i].second.xe()<<"  "<<potentialLoc[i].second.ye()<<std::endl;
		}
	}
};
