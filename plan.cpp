#include <iostream>
#include <vector>
#include <time.h>
#include "mapdealer.cpp"
#include <cassert>
#include "motion.cpp"

#define TR(i,it) for(typeof(i.begin()) it=i.begin(); it!=i.end(); it++) 
#define pb push_back

const float range_wall = 8;
const float dis_threshold = 9;
class cNode {
	cNode* _parent;
public:
	cPoint* pt;
	cNode(){}
	cNode(cPoint _pt, cNode* parent) 
	{
		pt = new cPoint(_pt.x(),_pt.y());
		_parent = parent;
	}
	/*
	cNode(const cNode& other)
	{
		pt=other.pt;
		_parent=other.parent();
	}*/
	float distance(cPoint other)
	{
		return pt->distance(&other);
	}
	cNode* parent()
	{
	//	if(_parent!=NULL) std::cout<<_parent->point().x()<<" PARENT "<<_parent->point().y()<<std::endl;
		return _parent;
	}
	cPoint point()
	{
		//if(pt==NULL) std::cout<<"I AM NULL"<<std::endl;
		assert(pt!=NULL);
		return *pt;
	}
};
class Rect {
	cPoint* p1;
	cPoint* p2;
	cPoint* p3;
	cPoint* p4;
	float x0; float y0; float x1; float y1;
public:
	Rect( Wall wall, float range) 
	{
		x0 = wall.xs();
		y0 = wall.ys();
		x1 = wall.xe();
		y1 = wall.ye();
		float length = wall.length();
		p1 =new cPoint(x0 - (x1-x0)*range/length + (y1-y0)*range/length, y0 - (y1-y0)*range/length + (x0-x1)*range/length);
		p2 =new cPoint(x0 - (x1-x0)*range/length - (y1-y0)*range/length, y0 - (y1-y0)*range/length - (x0-x1)*range/length);
		p3 =new cPoint(x1 + (x1-x0)*range/length + (y1-y0)*range/length, y1 + (y1-y0)*range/length + (x0-x1)*range/length);
		p4 =new cPoint(x1 + (x1-x0)*range/length - (y1-y0)*range/length, y1 + (y1-y0)*range/length - (x0-x1)*range/length);
		//std::cout<<"this wall:"<<x0<<","<<y0<<" & "<<x1<<","<<y1<<std::endl;
		//std::cout<<"up left:"<<p1->x()<<","<<p1->y()<<" ,down left:"<<p2->x()<<","<<p2->y()<<std::endl; 
		//std::cout<<"up right:"<<p3->x()<<","<<p3->y()<<" ,down right:"<<p4->x()<<","<<p4->y()<<std::endl;
	}
	bool line_intersect(cPoint pa, cPoint pb, cPoint pc, cPoint pd) //x1,x2 a line; x3,x4 a line
	{
		float x1 = pa.x(); float y1 = pa.y(); float x2 = pb.x(); float y2 = pb.y();
		float x3 = pc.x(); float y3 = pc.y(); float x4 = pd.x(); float y4 = pd.y();
		float m1 = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / ((y4-y3) * (x2-x1) - (x4-x3) * (y2-y1));
		float m2 = ((x3-x1) * (y4-y3) - (x4-x3) * (y3-y1)) / ((y4-y3) * (x2-x1) - (y2-y1) * (x4-x3));
		//std::cout<<"m1:"<<m1<<" ,m2:"<<m2<<std::endl;
		return ((m1<1) && (m1>0) && (m2<1) && (m2>0)); 
	}
	bool intersect(cPoint ps, cPoint pe) 
	{
	//	std::cout<<std::endl;
		//std::cout<<"these point:"<<ps.x()<<","<<ps.y()<<" & "<<pe.x()<<","<<pe.y()<<std::endl;
		//std::cout<<"the wall:"<<x0<<","<<y0<<" & "<<x1<<","<<y1<<std::endl;
		return (line_intersect(ps,pe,*p1,*p3) || line_intersect(ps,pe,*p2,*p1) || line_intersect(ps,pe,*p3,*p4) || line_intersect(ps,pe,*p2,*p4));
	}
};
class Plan {
	cPoint* start;
	cPoint* target;
	Motor* left;
	Motor* right;
	Location* current;
	float start_angle;
	int count;
	Motion* motion;
	Odometry* odo;
	std::vector<Rect> expwall;
	cNode last;
	std::vector<cNode*> tree;
	float xmin; float xmax; float ymin; float ymax;
	bool initialized;
	std::vector<cPoint> this_plan;

	float random(float min, float max) 
	{
		std::srand (clock());
		rand();
		rand();
   		float rand_num = ( rand()) / (float) RAND_MAX;
   		rand_num = rand_num*(max-min)+min;
		return rand_num;
	}
	float small_angle(float pass_angle) {
		int round = floor(pass_angle/6.283);
		if(fabs(pass_angle-round*6.283)>=fabs(pass_angle-round*6.283-6.283)) return pass_angle-round*6.283-6.283;
		else return pass_angle-round*6.283;
	}
	std::vector<cPoint> resize(std::vector<cPoint> v) {
		std::vector<cPoint> resize_point = v;
		if (v.size()<3) return v;
		int size;
		bool end_resize = false;
		while(!end_resize)
		{
			end_resize = true;
			std::vector<cPoint> after_resize;
			after_resize.pb(resize_point[0]);
			size = resize_point.size();
			for(int i =0;i<(size-2);i++) {
				bool connectable = true;
				TR(expwall,rct) {connectable = (!(rct->intersect(resize_point[i],resize_point[i+2])))&&connectable;}
				if (!connectable) after_resize.pb(resize_point[i+1]);
				end_resize = end_resize && (!connectable);
			}
			after_resize.pb(resize_point[size-1]);
			resize_point = after_resize;
		}
		return resize_point;
	}

public:
	Plan(std::vector<Wall> allwall, Motor* _left, Motor* _right, Location* location) 
	{
		xmin = allwall[0].xe();
		xmax = allwall[0].xe();
		ymin = allwall[0].ye();
		ymax = allwall[0].ye();
		left = _left;
		right = _right;
		current = location;
		odo = new Odometry(left,right,location);
		motion = new Motion(left,right,odo,location);
		initialized = false;
		tr(allwall) 
		{
			Rect rect(*it, range_wall);
			expwall.pb(rect);
			if((*it).xe()<xmin) xmin = (*it).xe();
			if((*it).xs()<xmin) xmin = (*it).xs();
			if((*it).ye()<ymin) ymin = (*it).ye();
			if((*it).ys()<ymin) ymin = (*it).ys();
			if((*it).xe()>xmax) xmax = (*it).xe();
			if((*it).xs()>xmax) xmax = (*it).xs();
			if((*it).ye()>ymax) ymax = (*it).ye();
			if((*it).ys()>ymax) ymax = (*it).ys();
		}
	}
	void setStart() 
	{
		start_angle = current->theta();
		start = new cPoint(current->x(),current->y());
	}
	// set the starting point
	void setStart(Location* _start) 
	{
		start_angle = _start->theta();
		start = new cPoint(_start->x(),_start->y());
		odo->set(_start);
	}
	// set the target point void setTarget(cPoint _target) 
	void setTarget(cPoint _target)
	{
		target = new cPoint(_target.x(),_target.y());
	}
	std::vector<cPoint> plan() 
	{
		cNode* st= new cNode(*start,NULL);
		tree.pb(st);
		bool end_plan = true;
		//std::cout<<"start:"<<start->x()<<","<<start->y()<<" & "<<"target:"<<target->x()<<","<<target->y()<<std::endl;
		TR(expwall,rct) {end_plan = (!(rct->intersect(*start,*target)))&&end_plan;}
		if(end_plan) 
		{
			//std::cout<<"I find my way out directly"<<std::endl;
			last = *st;
		}
		else {
			while(!end_plan)
			{
				float new_x = random(xmin,xmax);
				float new_y = random(ymin,ymax);
				bool point_not_too_close = true;
				cPoint new_point(new_x,new_y);
				//std::cout<<std::endl;
				//std::cout<<"getting new point: "<<std::endl;
				//std::cout<<new_x<<" , "<<new_y<<std::endl;
				TR(tree,node) {
					point_not_too_close = point_not_too_close&&(!((*node)->distance(new_point)<dis_threshold));
				}
				if(!point_not_too_close) {
					//std::cout<<"this point is too close to other point"<<std::endl;
					continue;
				}
				float nearestdis = 10000;
				cNode* nearest = NULL;

				bool connectable = false;
				TR(tree,nd) 
				{
					bool reachable = true;
					TR(expwall,rct) {reachable = (!(rct->intersect((*nd)->point(),new_point)))&&reachable;}
					connectable = connectable || reachable;
					if(((*nd)->distance(new_point)<nearestdis)&&reachable)
					{
						//std::cout<<"I am reachable"<<std::endl;
						nearestdis = (*nd)->distance(new_point);
						//std::cout<<"NEARESDT "<<nd->point().x()<<" "<<nd->point().y()<<std::endl;
						nearest = (*nd);
						//std::cout<<nearest<<std::endl;
					}
				}
				if (connectable) 
				{
					assert(nearest!=NULL);
					bool end_plan2 = true;
					TR(expwall,rct) {
						end_plan2 = (!(rct->intersect(new_point,*target)))&&end_plan2;
						//std::cout<<(*target).x()<<(*target).y()<<std::endl;
					}
					cNode* new_node= new cNode(new_point,nearest); 
					//std::cout<<"my nearest point is: "<<nearest->point().x()<<" , "<<nearest->point().y()<<std::endl;
					tree.pb(new_node);
					if(end_plan2) 
					{
						last = *new_node;
						//std::cout<<"last setting "<<last.point().x()<<" "<<last.point().y()<<std::endl;
						break;
					}
				}
			}
		}
		//std::cout<<"last setting "<<last.parent()->point().x()<<" "<<last.parent()->point().y()<<std::endl;
		std::vector<cPoint> return_points;
		return_points.pb(*target);
		while(last.parent()!=NULL) {
			//std::cout<<last.point().x()<<" , "<<last.point().y()<<std::endl;
			return_points.pb(last.point());
			last = *(last.parent());
			//std::cout<<last->point().x()<<" , "<<last->point().y()<<std::endl;
		}
		return_points.pb(*start);
		std::reverse(return_points.begin(),return_points.end());
		return resize(return_points);
	}

	int run(int state) {
		if (state == 0) {  //start
			if(!initialized) {
				this_plan = plan();
				for(int i=0; i<this_plan.size();i++) {
					std::cout<<"point"<<i+1<<": "<<this_plan[i].x()<<" , "<<this_plan[i].y()<<std::endl;
				}
				count = 0;
				float angle = small_angle(this_plan[count].angle(this_plan[count+1])-start_angle);
				std::cout<<"I am initialized state 0, turn: "<<angle<<std::endl;
				motion->rotate(angle);
				initialized = true;
				return 0;
			}
			else {
				if (!motion->run()) return 0;
				else{
					initialized = false;
					return 2;
				}
			}
		}
		if (state == 1) {
			if(!initialized) {
				float angle = small_angle(this_plan[count].angle(this_plan[count+1])-odo->getAngle());
				motion->rotate(angle);
				std::cout<<"I am initialized state 1, turn: "<<angle<<std::endl;
				initialized = true;
				return 1;
			}
			else {
				if (!motion->run()) return 1;
				else{
					initialized = false;
					return 2;
				}
			}
		}
		if (state == 2) {
			if(!initialized) {
				float distance = this_plan[count].distance(this_plan[count+1]);
				motion->straight(distance);
				std::cout<<"I am initialized state 2, forward: "<<distance<<std::endl;
				initialized = true;
				return 2;
			}
			else {
				if(!motion->run()) return 2;
				else{
					initialized = false;
					count++;
					if (count>=(this_plan.size()-1)) return 3;
					else return 1;
				}

			}
		}
		
	}
	
};
