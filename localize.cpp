//#include "data.cpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include "mapdealer.cpp"
#include <string>

#define pb push_back
#define mp std::make_pair
#define REP(i,n) for(int i=0;i<n;i++)

const double robWidth=16;
const double robFLength=7;
const double robBLength=7;
double errorMargin=10;
const double pErrorMargin=0.00;
const double angleError=0.4;


class Localize {
	Mapdealer* map;
	cPoint *startLocation;
	Location *location;
	int numCorners,pmode;
	bool localized,foundWall,exactPoint;
	//MonteCarlos* monte ;
	std::vector<Wall> outerWall;
	std::vector<cPoint> viscPoints;
	std::vector<std::pair<cPoint,Wall> > potentialLoc;
public:
	Localize(char* filename, Location *_start)
	{
		location = _start;
		viscPoints.pb(*(_start->point()));
		map = new Mapdealer(filename);
		outerWall = map->allWalls();
		startLocation=map->getStart();
		numCorners=localized=0;
		foundWall=false;
		tr(outerWall) std::cout<<"Wall: "<<((it)->xs())<<" "<<((it)->ys())<<" "<<((it)->xe())<<" "<<((it)->ye())<<std::endl;
		pmode=0;
		///monte = new MonteCarlos(outer_wall);
	}
	void wallFound(double leftDist)
	{
		if(leftDist==100) leftDist=2;
		cPoint* at=location->point();
		//cPoint diff=(*at-*startLocation);
		viscPoints.pb(*at);
		foundWall=true;
		double distWall=at->abs()+(robWidth/2+leftDist);///cos(location->theta());
		std::cout<<"Left distance is "<<leftDist<<std::endl;
		std::cout<<"At "<<at->x()<<" "<<at->y()<<std::endl;
		std::cout<<"Travelled "<<distWall<<std::endl;
		std::cout<<"Started at "<<startLocation->x()<<" "<<startLocation->y()<<std::endl;
		errorMargin+=pErrorMargin*distWall;
		tr(outerWall)
		{
			double A=it->ye()-it->ys();
			double B=it->xs()-it->xe();
			double C=it->ys()*it->xe()-it->ye()*it->xs();
			double m=startLocation->x(),n=startLocation->y();
			double dist=A*m+B*n+C;
			double sqcoeff=A*A+B*B;
			double distTravelled=distWall;
			double coeff=-dist/sqcoeff;
			sqcoeff=sqrt(sqcoeff);
			dist=fabs(dist)/sqcoeff;
			std::cout<<"Could be on wall "<<it->xs()<<" "<<it->ys()<<" "<<it->xe()<<" "<<it->ye()<<"\n";
			std::cout<<"Distance to wall is "<<dist<<"\n";
			if(distTravelled<(dist-errorMargin)) continue;
			else if(distTravelled<dist) distTravelled=dist;
			double fx=A*(coeff)+m, fy=B*(coeff)+n;
			double r=(distTravelled*distTravelled-dist*dist);
			r=sqrt(r);
			double dx=r*B/sqcoeff, dy=-r*A/sqcoeff;
			//std::cout<<"foot is "<<fx<<" "<<fy<<std::endl;
			//std::cout<<"dx is "<<dx<<" "<<dy<<std::endl;
			int sign=1;
			for(int k=0;k<2;k++,sign*=-1)
			{
				double x=fx+(sign*dx),y=fy+(sign*dy);
				std::cout<<"Trying be at ("<<x<<","<<y<<" "<<sign<<")\n";
				if(x<(std::min(it->xs(),it->xe())-errorMargin)||x>(std::max(it->xs(),it->xe())+errorMargin)) continue;
				if(y<(std::min(it->ys(),it->ye())-errorMargin)||y>(std::max(it->ys(),it->ye())+errorMargin)) continue;
				cPoint start=it->s(),end=it->e();
				bool cw=startLocation->clockwise(&start, &end);
				Wall add=*it;
				if(!cw) add.swap();
				double angle=fabs(startLocation->three_points_angle(new cPoint(x,y), &end));
				double actAngle=fabs(location->theta());
				std::cout<<"angles are "<<angle<<" and "<<actAngle<<std::endl;
				if(fabs(angle-actAngle)>angleError) continue;
				std::cout<<"Could be at ("<<x<<","<<y<<") on wall "<<add.xs()<<" "<<add.ys()<<" "<<add.xe()<<" "<<add.ye()<<"\n";
				potentialLoc.pb(mp(cPoint(x,y),add));
				if(r==0) break;
			}
		}
	}
	bool atCorner(double frontDist,int mode)
	{
		if(pmode==5) frontDist=-16;
		if(frontDist==100||mode==5) frontDist=0;
		pmode=mode;
		cPoint* at=location->point();
		int vs=viscPoints.size();
		double angle=at->three_points_angle(&viscPoints[vs-1],&viscPoints[vs-2]);
		angle=fabs(angle);
		if(angle<0.15) return false;
		double distTravelled=((*at)-viscPoints[vs-1]).abs()+robFLength+frontDist;
		if(vs!=2) distTravelled+=robBLength;
		int pl=potentialLoc.size();
		std::cout<<"FRONT DIST "<<frontDist<<std::endl;
		std::cout<<"ANGLE "<<angle<<"DISTANCE "<<distTravelled<<std::endl;
		std::vector<std::pair<cPoint,Wall> > newLoc;
		REP(i,pl)
		{
			double wallDist=(potentialLoc[i].second.e()-potentialLoc[i].first).abs();
			//if(vs==2) wallDist+=robBLength;
			std::cout<<"Checking on wall"<<potentialLoc[i].second.xs()<<"  "<<potentialLoc[i].second.ys()<<" to "<<potentialLoc[i].second.xe()<<"  "<<potentialLoc[i].second.ye()<<std::endl;
			std::cout<<"Wall dist"<<wallDist<<std::endl;
			if(wallDist-errorMargin>distTravelled||wallDist+errorMargin<distTravelled) continue;
			tr(outerWall)
			{
				if((*it)==potentialLoc[i].second) continue;
				if(!it->pointLies(potentialLoc[i].second.e())) continue;
				std::cout<<"wall"<<it->s().x()<<" "<<it->s().y()<<" "<<it->e().x()<<" "<<it->e().y()<<std::endl;
				cPoint start=potentialLoc[i].second.e(),end=it->e();
				bool cw=potentialLoc[i].second.s().clockwise(&start,&end );
				std::cout<<"clockwise? is "<<cw<<std::endl;
				if(it->s()==potentialLoc[i].second.e())
				{
					newLoc.pb(mp(potentialLoc[i].second.e(),Wall(potentialLoc[i].second.e(),it->e())));
				}
				else if(it->e()==potentialLoc[i].second.e())
				{
					newLoc.pb(mp(potentialLoc[i].second.e(),Wall(potentialLoc[i].second.e(),it->s())));
				}
				else if(cw)
				{
					newLoc.pb(mp(potentialLoc[i].second.e(),Wall(potentialLoc[i].second.e(),it->e())));
				}
				else
				{
					newLoc.pb(mp(potentialLoc[i].second.e(),Wall(potentialLoc[i].second.e(),it->s())));
				}
			}
		}
		potentialLoc=newLoc;
		viscPoints.pb(*at);
		debug();
		if(potentialLoc.size()==1) 
		{
			std::cout<<"I'm at "<<potentialLoc[0].second.xs()<<" "<<potentialLoc[0].second.ys()<<std::endl;
			return true;
		}
		return false;
	}
	void debug()
	{
		int pl=potentialLoc.size();
		REP(i,pl)
		{
			std::cout<<"Maybe on wall"<<potentialLoc[i].second.xs()<<"  "<<potentialLoc[i].second.ys()<<" to "<<potentialLoc[i].second.xe()<<"  "<<potentialLoc[i].second.ye()<<std::endl;
		}
	}
};
