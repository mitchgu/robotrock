#include <iostream>
#include <vector>
#include "shortIR.cpp"
#include "odometry.cpp"
#include <math.h>
const float slp = 0.5;
const float forward_speed = 1.2;
const float forward_side_dis_threshold = 12;
const float forward_front_dis_threshold = 10;
const float block_size = 3.5;
const float camera_center =24; //24 inch away from the center of the robot

class Planningwalk {
	Motor* left;
	Motor* right;
	IR* irf;
	IR* irlf;
	IR* irlb;
	IR* irr;
	Odometry* odo;
	Motion* motion;
	mraa::Gpio* irb;
	Location* start;
	Location* current;
	Planner* map;
	bool initialized;
	bool r_init; bool l_init; int f_cnt; Location* l_start; Location* r_start; int wall_side; //wallside: 0:left; 1:right; 2:front
	bool end_turn;


public:
	Planningwalk (Motor* _left,Motor* _right,IR* _irf,IR* _irlf,IR* _irlb,IR* _irr,mraa::Gpio* _irb, Location* _start) {
		left = _left; right = _right;
		irf = _irf; irlf = _irlf; irlb = _irlb; irr = _irr; irb = _irb;
		start = _start; current = _start;
		odo = new Odometry(_left, _right, _start->x(),_start->y(),_start->theta());
		motion = new Motion(_left,_right,odo,_start);
		initialized = false;
		map = new Planner();
	}
	int run(int channel){
		if (channel == 1) {
			if (!initialized) {
				forward_setup();
				return 1;
			}
			else {
				if(!forward_next()){
					forward_run();
					return 1;
				}
				else {
					channel_stop();
					wall_dealer();
					return 2;
				}
			}
		}
		if (channel == 2) {
			if (!initialized) {
				turn_setup();
				return 2;
			}
			else {
				if(!turn_next()){
					turn_run();
					return 2;
				}
				else {
					channel_stop();
					return 1;
				}
			}
		}
	}
	void channel_stop() {
		left->stop(); right->stop();
		initialized = false;
		sleep (slp);
	}
	void forward_setup() {
		left->forward();
		right->forward();
		left->setSpeed(forward_speed);
		right->setSpeed(forward_speed);
		r_init = false; l_init = false; f_cnt = 0; 
		initialized = true;
	}
	void forward_run() {
		left->run();
		right->run();
		current = odo->run();
		map->update_grid(current);
		std::cout<<"x position: "<<current->x()<<" y position: "<<current->y()<<" theta: "<<current->theta()<<std::endl;
	}
	void wall_dealer() {
		map->whole_update();
		current = odo->run();
		float dis;
		if (wall_side==0) {
			dis = irlf->getDistance();
			std::cout<<"wall in left: "<<dis<<std::endl;
			map->is_wall(current->move(dis,0));
		}
		else{
			if (wall_side==1) {
				dis = irr->getDistance();
				std::cout<<"wall in right: "<<dis<<std::endl;
				map->is_wall(current->move(dis,1.57));
			}
			else{
				if (wall_side==2) {
					dis = irf->getDistance();
					std::cout<<"wall in front: "<<dis<<std::endl;
					map->is_wall(current->move(dis,-1.57));
				}
			}
		}
	}
	void turn_setup() {
		float angle = map->plan();
		motion->rotate(angle);
		initialized = true;
		end_turn = false;
	}
	void turn_run() {
		end_turn = motion->run();
	}
	bool turn_next() {
		return end_turn;
	}
	bool forward_next () {
		float dlf = irlf->getDistance();
		float dr = irr->getDistance();
		float df = irf->getDistance();
		std::cout<<"dlf: "<< dlf<<" dr: "<<dr<<" df: "<<df<<std::endl;
		if (!((dlf>forward_side_dis_threshold) && (dr>forward_side_dis_threshold) && (df>forward_front_dis_threshold) && (!irb->read()))) {
			if (df<=forward_front_dis_threshold) {   //front wall dealer
				if (f_cnt<0) f_cnt++;
				else {
					wall_side = 2;
					return true;
				}
			}
			else f_cnt = 0;

			if (dlf<=forward_side_dis_threshold) {   //left wall dealer
				if (!l_init) {
					l_start = current;
					l_init = true;
				}
				else {
					if ((current->distance(l_start))>block_size) {
						wall_side = 0;
						return true;
					}
				}
			}
			else l_init = false;

			if (dr<=forward_side_dis_threshold) {  //right wall dealer
				if (!r_init) {
					r_start = current;
					r_init = true;
				}
				else {
					if ((current->distance(r_start))>block_size) {
						wall_side = 1;
						return true;
					}
				}
			}
			else r_init = false;
		}
		else {
			r_init = false;
			l_init = false;
			f_cnt = 0;
		}
		return false;
	}
};

class Planner {
	vector<vector<Grid>> map;
	int grid_i; int grid_j; int t_grid_i; int t_grid_j;

	float total_weight(float angle,int start_i,int start_j){
		vector<Grid> temp;
		temp.push_back(map[start_i][start_j]);
		int point=0;
		float total = 0;
		int now_i; int now_j;
		bool keep_run = true;
		int mypush = 1;
		if(angle<180) {
			if(angle<90) {
				for(;keep_run;) {
					int size = temp.size();
					point = size-mypush;
					mypush = 0;
					for(int k = point; k<size; k++) {
						total = total+temp[k].w()/temp[0].dis(temp[k]);
						now_i = temp[k].i(); now_j = temp[k].j();
						if ((now_j+1)<21 && (now_i-1>=0)) {
							if (temp[0].inside(map[now_i-1][now_j],angle)) {
								if (!map[now_i-1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = _point;v<size;v++) {
										if(temp[v].equalTo(now_i-1,now_j)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j]);
									}
								}
							}
							if (temp[0].inside(map[now_i-1][now_j+1],angle)){
								if (!map[now_i-1][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i-1,now_j+1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j+1]);
									}
								}
							}
							if (temp[0].inside(map[now_i][now_j+1],angle)) {
								if (!map[now_i][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i,now_j+1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i][now_j+1]);
									}
								}
							}
						}
						else keep_run = false;
					}
				}
			}
			else{
				for(;keep_run;) {
					int size = temp.size();
					point = size-mypush;
					mypush = 0;
					for(int k = point; k<size; k++) {
						total = total+temp[k].w()/temp[0].dis(temp[k]);
						now_i = temp[k].i(); now_j = temp[k].j();
						if ((now_j+1)<21 && (now_i+1>=0)) {
							if (temp[0].inside(map[now_i+1][now_j],angle)) {
								if (!map[now_i+1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = _point;v<size;v++) {
										if(temp[v].equalTo(now_i+1,now_j)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i+1][now_j]);
									}
								}
							}
							if (temp[0].inside(map[now_i+1][now_j+1],angle)){
								if (!map[now_i+1][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i+1,now_j+1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i+1][now_j+1]);
									}
								}
							}
							if (temp[0].inside(map[now_i][now_j+1],angle)) {
								if (!map[now_i][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i,now_j+1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i][now_j+1]);
									}
								}
							}
						}
						else keep_run = false;
					}
				}
			}
		}
		else{
			if(angle<270) {
				for(;keep_run;) {
					int size = temp.size();
					point = size-mypush;
					mypush = 0;
					for(int k = point; k<size; k++) {
						total = total+temp[k].w()/temp[0].dis(temp[k]);
						now_i = temp[k].i(); now_j = temp[k].j();
						if ((now_j-1)<21 && (now_i+1>=0)) {
							if (temp[0].inside(map[now_i+1][now_j],angle)) {
								if (!map[now_i+1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = _point;v<size;v++) {
										if(temp[v].equalTo(now_i+1,now_j)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i+1][now_j]);
									}
								}
							}
							if (temp[0].inside(map[now_i+1][now_j-1],angle)){
								if (!map[now_i+1][now_j-1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i+1,now_j-1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i+1][now_j-1]);
									}
								}
							}
							if (temp[0].inside(map[now_i][now_j-1],angle)) {
								if (!map[now_i][now_j-1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i,now_j-1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i][now_j-1]);
									}
								}
							}
						}
						else keep_run = false;
					}
				}
			}
			else{
				for(;keep_run;) {
					int size = temp.size();
					point = size-mypush;
					mypush = 0;
					for(int k = point; k<size; k++) {
						total = total+temp[k].w()/temp[0].dis(temp[k]);
						now_i = temp[k].i(); now_j = temp[k].j();
						if ((now_j-1)<21 && (now_i-1>=0)) {
							if (temp[0].inside(map[now_i-1][now_j],angle)) {
								if (!map[now_i-1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = _point;v<size;v++) {
										if(temp[v].equalTo(now_i-1,now_j)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j]);
									}
								}
							}
							if (temp[0].inside(map[now_i-1][now_j-1],angle)){
								if (!map[now_i-1][now_j-1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i-1,now_j-1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j-1]);
									}
								}
							}
							if (temp[0].inside(map[now_i][now_j-1],angle)) {
								if (!map[now_i][now_j-1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<size;v++) {
										if(temp[v].equalTo(now_i,now_j-1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i][now_j-1]);
									}
								}
							}
						}
						else keep_run = false;
					}
				}
			}
		}
		return total;
	}
public:
	Planner() {
		for (int i=0; i<21;i++) {
			vector<Grid> vm;
			for (int j=0; j<21; j++) {
				Grid g((j-11)*12,(11-i)*12,i,j);
				vm.push_back(g);      //i is row, j is column
			}
			map.push_back(vm);
		}
		grid_i = 0;
		grid_j = 0;
		t_grid_i = 0;
		t_grid_j = 0;
	}
	float plan(Location* location) { //return the angle that robot should turn
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		float x = location->x();
		float y = location->y();
		int this_i; int this_j;
		if (x>0) this_j = ((int) ((x/12)+0.5))+11;
		else this_j = ((int) ((x/12)-0.5))+11;
		if (y>0) this_i = 11-((y/12)+0.5));
		else this_i = 11-((y/12)-0.5));
		vector<float> v;
		for(int i=0; i<12;i++) {
			v.push_back(total_weight(i*30,this_i,this_j));
		}
		float largest = v[0];
		int largest_pin = 0;
		for(int i=1;i<12;i++) {
			if(v[i]>largest) {
				largest = v[i];
				largest_pin = i;
			}
		}
		return ((3.14*largest_pin/6)-location->theta());
	}
	void whole_update() {
		for (int i=0; i<21;i++) {
			for (int j=0; j<21; j++) {
				map[i][j].more_passible();
			}
		}
	}
	void is_wall(*Location location) {   //this location is a wall block
		int update_i;
		int update_j;
		float x = update->x();
		float y = update->y();
		if (x>0) update_j = ((int) ((x/12)+0.5))+11;
		else update_j = ((int) ((x/12)-0.5))+11;
		if (y>0) update_i = 11-((y/12)+0.5));
		else update_i = 11-((y/12)-0.5));
		map[update_i][update_j].is_not_passible();
	}
	void update_grid(*Location location) {
		int this_i;
		int this_j;
		float _x = location->x();
		float _y = location->y();
		if (_x>0) this_j = ((int) ((_x/12)+0.5))+11;
		else this_j = ((int) ((_x/12)-0.5))+11;
		if (_y>0) this_i = 11-((_y/12)+0.5));
		else this_i = 11-((_y/12)-0.5));
		if ((this_i != t_grid_i)||(this_j != t_grid_j)) {
			map[this_i][this_j].is_passible();
			t_grid_i = this_i;
			t_grid_j = this_j;
		}
		Location* update = location->move_forward(camera_center);
		int update_i;
		int update_j;
		float x = update->x();
		float y = update->y();
		if (x>0) update_j = ((int) ((x/12)+0.5))+11;
		else update_j = ((int) ((x/12)-0.5))+11;
		if (y>0) update_i = 11-((y/12)+0.5));
		else update_i = 11-((y/12)-0.5));
		if ((update_i != grid_i)||(update_j != grid_j)) {
			map[update_i][update_j].update();
			grid_i = update_i; grid_j = update_j;
		}
	}
};

class Grid {
	float x_center;
	float y_center;
	int myi;
	int myj;
	int weight;
	float passible;
public:
	Grid(float _x_center, float _y_center, int _i; int _j) {
		passible = 0;
		x_center = _x_center;
		y_center = _y_center;
		weight = 5;
		myi = _i;
		myj = _j;
	}
	int i() {return myi;}
	int j() {return myj;}
	float x() {return x_center;}
	float y() {return y_center;}
	int w() {return weight;}
	void more_passible() {
		passible = passible*0.8;
	}
	void is_passible() {
		passible = passible/3;
	}
	void update() {
		weight--;
	}
	void is_not_passible() {
		if ((passible+0.8)>1) passible = 1;
		else passible = passible + 0.8;
	}
	bool Passible() {
		return (passible>0.6);
	}
	float dis(Grid other){
		return sqrt(pow((other.x()-x()),2.0)+pow((other.y()-y()),2.0))+2;
	}
	bool equalTo(int i_,int j_){
		return ((i()==i_)&&(j()==j_));
	}
	bool inside(Grid other, float angle) {
		
	}
};