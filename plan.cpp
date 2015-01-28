#include <iostream>
#include <vector>
#include <random> 
#include <chrono>
#include "data.cpp"
#include "mapdealer.cpp"

#define TR(i,it) for(typeof(i.begin()) it=i.begin(); it!=i.end(); it++) 
#define pb push_back

const float range_wall = 9;
class cNode {
	cPoint* pt;
	cNode* _parent;
public:
	cNode(cPoint _pt, cNode* parent) 
	{
		pt = &_pt;
		_parent = parent;
	}
	float distance(cPoint other)
	{
		return pt->distance(&other);
	}
	cNode* parent()
	{
		return _parent;
	}
	cPoint point()
	{
		return *pt;
	}
};
class Rect {
	cPoint* p1;
	cPoint* p2;
	cPoint* p3;
	cPoint* p4;
public:
	Rect( Wall wall, float range) 
	{
		float x0 = 24.0*wall.xs();
		float y0 = 24.0*wall.ys();
		float x1 = 24.0*wall.xe();
		float y1 = 24.0*wall.ye();
		float length = wall.length()*24.0;
		p1 =new cPoint(x0 - (x1-x0)*range/length + (y1-y0)*range/length, y0 - (y1-y0)*range/length + (x0-x1)*range/length);
		p2 =new cPoint(x0 - (x1-x0)*range/length - (y1-y0)*range/length, y0 - (y1-y0)*range/length - (x0-x1)*range/length);
		p3 =new cPoint(x1 + (x1-x0)*range/length + (y1-y0)*range/length, y1 + (y1-y0)*range/length + (x0-x1)*range/length);
		p4 =new cPoint(x1 + (x1-x0)*range/length - (y1-y0)*range/length, y1 + (y1-y0)*range/length - (x0-x1)*range/length);
	}
	bool line_intersect(cPoint pa, cPoint pb, cPoint pc, cPoint pd) //x1,x2 a line; x3,x4 a line
	{
		float x1 = pa.x(); float y1 = pa.y(); float x2 = pb.x(); float y2 = pb.y();
		float x3 = pc.x(); float y3 = pc.y(); float x4 = pd.x(); float y4 = pd.y();
		float m1 = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / ((y4-y3) * (x2-x1) - (x4-x3) * (y2-y1));
		float m2 = ((x3-x1) * (y4-y3) - (x4-x3) * (y3-y1)) / ((y4-y3) * (x2-x1) - (y2-y1) * (x4-x3));
		return ((m1<1) && (m1>0) && (m2<1) && (m2>0)); 
	}
	bool intersect(cPoint ps, cPoint pe) 
	{
		return (line_intersect(ps,pe,*p1,*p3) || line_intersect(ps,pe,*p2,*p1) || line_intersect(ps,pe,*p3,*p4) || line_intersect(ps,pe,*p2,*p4));
	}
};
class Plan {
	cPoint* start;
	cPoint* target;
	std::vector<Rect> expwall;
	cNode* last;
	unsigned seed;
	std::default_random_engine *generator;
  	std::uniform_real_distribution<double> xgen;
  	std::uniform_real_distribution<double> ygen;
	float xmin; float xmax; float ymin; float ymax;

	void planner() 
	{
		std::vector<cNode> tree;
		cNode st(*start,NULL);
		tree.pb(st);
		bool end_plan = false;
		TR(expwall,rct) {end_plan = !(rct->intersect(*start,*target));}
		if(end_plan) 
		{
			last = &st;
			return ;
		}
		while(!end_plan)
		{
			float new_x = xgen(generator);
			float new_y = ygen(generator);
			cPoint new_point(new_x,new_y);
			std::cout<<"getting new point: "<<std::endl;
			std::cout<<new_x<<" , "<<new_y<<std::endl;
			float nearestdis = 10000;
			cNode* nearest;
			bool connectable = false;
			TR(tree,nd) 
			{
				bool reachable = false;
				TR(expwall,rct) reachable = !(rct->intersect((*nd).point(),new_point));
				connectable = connectable || reachable;
				if(((*nd).distance(new_point)<nearestdis)&&reachable)
				{
					std::cout<<"I am reachable"<<std::endl;
					nearestdis = (*nd).distance(new_point);
					nearest = &(*nd);
				}
			}
			if (connectable) 
			{
				TR(expwall,rct) end_plan = !(rct->intersect(new_point,*target));
				cNode new_node(new_point,nearest); 
				std::cout<<"my nearest point is: "<<nearest->point().x()<<" , "<<nearest->point().y()<<std::endl;
				tree.pb(new_node);
				if(end_plan) 
				{
					last = &new_node;
					return ;
				}
			}
		}
	}
public:
	Plan(std::vector<Wall> allwall) 
	{
		xmin = allwall[0].xe()*24;
		xmax = allwall[0].xe()*24;
		ymin = allwall[0].ye()*24;
		ymax = allwall[0].ye()*24;
		tr(allwall) 
		{
			Rect rect(*it, range_wall);
			expwall.pb(rect);
			if((*it).xe()*24<xmin) xmin = (*it).xe()*24;
			if((*it).xs()*24<xmin) xmin = (*it).xs()*24;
			if((*it).ye()*24<ymin) ymin = (*it).ye()*24;
			if((*it).ys()*24<ymin) ymin = (*it).ys()*24;
			if((*it).xe()*24>xmax) xmax = (*it).xe()*24;
			if((*it).xs()*24>xmax) xmax = (*it).xs()*24;
			if((*it).ye()*24>ymax) ymax = (*it).ye()*24;
			if((*it).ys()*24>ymax) ymax = (*it).ys()*24;
		}
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		generator = std::default_random_engine(seed);
		xgen = std::uniform_real_distribution<double>(xmin,xmax);
		ygen = std::uniform_real_distribution<double>(ymin,ymax);
	}
	// set the starting point
	void setStart(cPoint _start) 
	{
		start = &_start;
	}
	// set the target point void setTarget(cPoint _target) 
	void setTarget(cPoint _target)
	{
		target = &_target;
	}
	//return a whole group a point that connect start to end 
	std::vector<cPoint> plan() 
	{
		planner();
		std::vector<cPoint> myplan;
		myplan.pb(*target);
		cNode current = *last;
		while(current.parent() != NULL) 
		{
			myplan.pb(current.point());
			current = *current.parent();
		}
		myplan.pb(*start);
		std::reverse(myplan.begin(),myplan.end());
		return myplan;
	}
};
