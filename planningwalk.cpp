#include <iostream>
#include <vector>
#include "shortIR.cpp"
#include "motion.cpp"
#include <math.h>
const float slp = 0.5;
const float forward_speed = 1.2;
const float forward_side_dis_threshold = 12;
const float forward_front_dis_threshold = 10;
const float block_size = 3.5;
const float camera_center =24; //24 inch away from the center of the robot
class Grid {
	float x_center;
	float y_center;
	int myi;
	int myj;
	int weight;
	float passible;
public:
	Grid(float _x_center, float _y_center, int _i, int _j) {
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
		if ((passible+0.8)>=1) passible = 1;
		else passible = passible + 0.8;
	}
	bool Passible() {
		return (passible<0.6);
	}
	float dis(Grid other){
		return sqrt(pow((other.x()-x()),2.0)+pow((other.y()-y()),2.0))+2;
	}
	bool equalTo(int i_,int j_){
		return ((i()==i_)&&(j()==j_));
	}
	bool inside(Grid other, float angle) {  //this angle is in degree
		if (angle == 0){
			return (other.i()<=i())&&(other.j() ==j());
		}
		else {
			if (angle == 90) {
				return (other.i()==i())&&(other.j()>=j());
			}
			else {
				if (angle == 180) {
					return (other.j()==j())&&(other.i()>=i());
				}
				else {
					if (angle == 270){
						return (other.j()<=j())&&(other.i()==i());
					}
				}
			}
		}
		if (angle<90) {
			if ((other.j()<j())||(other.i()>i())) return false;
			else {
				bool i1 = ((other.j()-j()-0.5)/tan(angle*3.14/180))>(i()-other.i()-0.5);
				bool i2 = ((other.j()-j()-0.5)/tan(angle*3.14/180))<(i()-other.i()+0.5);
				bool i3 = ((i()-other.i()-0.5)*tan(angle*3.14/180))>(other.j()-j()-0.5);
				bool i4 = ((i()-other.i()-0.5)*tan(angle*3.14/180))<(other.j()-j()+0.5);
				return((i1&&i2)||(i3&&i4));
			}
		}
		else {
			if (angle<180) {
				if ((other.j()<j())||(other.i()<i())) return false;
				else {
					bool i1 = ((other.j()-j()-0.5)/tan(3.14-(angle*3.14/180)))>(other.i()-i()-0.5);
					bool i2 = ((other.j()-j()-0.5)/tan(3.14-(angle*3.14/180)))<(other.i()-i()+0.5);
					bool i3 = ((other.i()-i()-0.5)*tan(3.14-(angle*3.14/180)))>(other.j()-j()-0.5);
					bool i4 = ((other.i()-i()-0.5)*tan(3.14-(angle*3.14/180)))<(other.j()-j()+0.5);
					return((i1&&i2)||(i3&&i4));
				}
			}
			else {
				if(angle<270) {
					if ((other.j()>j())||(other.i()<i())) return false;
					else {
						bool i1 = ((j()-other.j()-0.5)/tan((angle*3.14/180)-3.14))>(other.i()-i()-0.5);
						bool i2 = ((j()-other.j()-0.5)/tan((angle*3.14/180)-3.14))<(other.i()-i()+0.5);
						bool i3 = ((other.i()-i()-0.5)*tan((angle*3.14/180)-3.14))>(j()-other.j()-0.5);
						bool i4 = ((other.i()-i()-0.5)*tan((angle*3.14/180)-3.14))<(j()-other.j()+0.5);
						return((i1&&i2)||(i3&&i4));
					}
				}
				else{
					if ((other.j()>j())||(other.i()>i())) return false;
					else {
						bool i1 = ((j()-other.j()-0.5)/tan(6.28-(angle*3.14/180)))>(i()-other.i()-0.5);
						bool i2 = ((j()-other.j()-0.5)/tan(6.28-(angle*3.14/180)))<(i()-other.i()+0.5);
						bool i3 = ((i()-other.i()-0.5)*tan(6.28-(angle*3.14/180)))>(j()-other.j()-0.5);
						bool i4 = ((i()-other.i()-0.5)*tan(6.28-(angle*3.14/180)))<(j()-other.j()+0.5);
						return((i1&&i2)||(i3&&i4));
					}
				}
			}
		}
	}
};
class Planner {
	std::vector<std::vector<Grid> > map;
	int grid_i; int grid_j; int t_grid_i; int t_grid_j;

	float total_weight(float angle,int start_i,int start_j){
		std::vector<Grid> temp;
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
						std::cout<<"now_i: "<<now_i<<",now_j: "<<now_j<<std::endl;
						if ((now_j+1)<21 && (now_i-1>=0)) {
							if (temp[0].inside(map[now_i-1][now_j],angle)) {
								//std::cout<<"inside?"<<std::endl;
								if (!map[now_i-1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
										if(temp[v].equalTo(now_i-1,now_j)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j]);
									}
								}
							}
							if (temp[0].inside(map[now_i-1][now_j+1],angle)){
								//std::cout<<"inside?"<<std::endl;
								if (!map[now_i-1][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
										if(temp[v].equalTo(now_i-1,now_j+1)) already_in = true;
									}
									if (!already_in) {
										mypush++;
										temp.push_back(map[now_i-1][now_j+1]);
									}
								}
							}
							if (temp[0].inside(map[now_i][now_j+1],angle)) {
								//std::cout<<"inside?"<<std::endl;
								if (!map[now_i][now_j+1].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
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
						std::cout<<"now_i: "<<now_i<<",now_j: "<<now_j<<std::endl;
						if ((now_j+1)<21 && (now_i+1<21)) {
							if (temp[0].inside(map[now_i+1][now_j],angle)) {
								if (!map[now_i+1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
						std::cout<<"now_i: "<<now_i<<",now_j: "<<now_j<<std::endl;
						if ((now_j-1)>=0 && (now_i+1<21)) {
							if (temp[0].inside(map[now_i+1][now_j],angle)) {
								if (!map[now_i+1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
						std::cout<<"now_i: "<<now_i<<",now_j: "<<now_j<<std::endl;
						if ((now_j-1)>=0 && (now_i-1>=0)) {
							if (temp[0].inside(map[now_i-1][now_j],angle)) {
								if (!map[now_i-1][now_j].Passible()) keep_run = false;
								else {
									bool already_in = false;
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
									for(int v = point;v<temp.size();v++) {
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
		std::cout<<"did I return?"<<std::endl;
		return total;
	}
public:
	Planner() {
		for (int i=0; i<21;i++) {
			std::vector<Grid> vm;
			for (int j=0; j<21; j++) {
				Grid g((j-11)*12,(11-i)*12,i,j);
				vm.push_back(g);      //i is row, j is column
			}
			map.push_back(vm);
		}
		grid_i = -1;
		grid_j = -1;
		t_grid_i = -1;
		t_grid_j = -1;
	}
	float plan(Location* location) { //return the angle that robot should turn
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		std::cout<<"planning"<<std::endl;
		float x = location->x();
		float y = location->y();
		int this_i; int this_j;
		if (x>0) this_j = ((int) ((x/12)+0.5))+10;
		else this_j = ((int) ((x/12)-0.5))+10;
		if (y>0) this_i = 10-(int)((y/12)+0.5);
		else this_i = 10-(int)((y/12)-0.5);
		std::vector<float> v;
		for(int i=0; i<12;i++) {
			float ttw = total_weight(i*30,this_i,this_j);
			std::cout<<"degree: "<<i*30<<"  ,total weight:  "<< ttw<<std::endl;
			v.push_back(ttw);
		}
		float largest = v[0];
		int largest_pin = 0;
		for(int i=1;i<12;i++) {
			if(v[i]>largest) {
				largest = v[i];
				largest_pin = i;
			}
		}
		std::cout<<" I plan to go in: "<<largest*30<<std::endl;
		float real_angle = location->theta() - (int)(location->theta()/6.28);
		return ((3.14*largest_pin/6)-real_angle);
	}
	void whole_update() {
		for (int i=0; i<21;i++) {
			for (int j=0; j<21; j++) {
				map[i][j].more_passible();
			}
		}
	}
	void is_wall(Location* location,float dis, float angle) {   //this location is a wall block
		std::cout<<"run through is_wall"<<std::endl;
		int this_i;
		int this_j;
		float x = location->x();
		float y = location->y();
		if (x>0) this_j = ((int) ((x/12)+0.5))+10;
		else this_j = ((int) ((x/12)-0.5))+10;
		if (y>0) this_i = 10-(int)((y/12)+0.5);
		else this_i = 10-(int)((y/12)-0.5);
		Location* update = location->move(dis,angle);
		int update_i;
		int update_j;
		float _x = update->x();
		float _y = update->y();
		if (_x>0) update_j = ((int) ((_x/12)+0.5))+10;
		else update_j = ((int) ((_x/12)-0.5))+10;
		if (_y>0) update_i = 10-(int)((_y/12)+0.5);
		else update_i = 10-(int)((_y/12)-0.5);
		int return_i;
		int return_j;
		if ((update_i == this_i)&&(update_j == this_j)) {
			float final_angle = location->theta()+angle;
			float test_angle = final_angle-6.28*floor(final_angle/6.28);
			if (test_angle<0.78||test_angle>=5.5) {return_i = this_i-1; return_j = this_j;}
			if ((test_angle>=0.78)&&(test_angle<2.35)) {return_i = this_i; return_j = this_j+1;}
			if ((test_angle>=2.35)&&(test_angle<3.92)) {return_i = this_i+1; return_j = this_j+1;}
			if ((test_angle>=3.92)&&(test_angle<5.5)) {return_i = this_i; return_j = this_j-1;}
		}
		else {
			return_i = update_i; return_j = update_j;
		}
		std::cout<<"walli:  "<<return_i<<",  wallj:  "<<return_j<<std::endl;
		map[return_i][return_j].is_not_passible();
	}
	void update_grid(Location* location) {
		int this_i;
		int this_j;
		float _x = location->x();
		float _y = location->y();
		if (_x>0) this_j = ((int) ((_x/12)+0.5))+10;
		else this_j = ((int) ((_x/12)-0.5))+10;
		if (_y>0) this_i = 10-(int)((_y/12)+0.5);
		else this_i = 10-(int)((_y/12)-0.5);
		if ((this_i != t_grid_i)||(this_j != t_grid_j)) {
			map[this_i][this_j].is_passible();
			t_grid_i = this_i;
			std::cout<<"I am at: "<<this_i<<" "<<this_j<<std::endl;
			t_grid_j = this_j;
		}
		Location* update = location->move_forward(camera_center);
		int update_i;
		int update_j;
		float x = update->x();
		float y = update->y();
		if (x>0) update_j = ((int) ((x/12)+0.5))+10;
		else update_j = ((int) ((x/12)-0.5))+10;
		if (y>0) update_i = 10-(int)((y/12)+0.5);
		else update_i = 10-(int)((y/12)-0.5);
		if ((update_i != grid_i)||(update_j != grid_j)) {
			map[update_i][update_j].update();
			std::cout<<"I update: "<<update_i<<" "<<update_j<<std::endl;
			grid_i = update_i; grid_j = update_j;
		}
	}
};


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
		odo = new Odometry(_left, _right, current);
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
		odo->run();
		map->update_grid(current);
		std::cout<<"x position: "<<current->x()<<" y position: "<<current->y()<<" theta: "<<current->theta()<<std::endl;
	}
	void wall_dealer() {
		map->whole_update();
		odo->run();
		float dis;
		if (wall_side==0) {
			dis = irlf->getDistance();
			std::cout<<"wall in left: "<<dis<<std::endl;
			map->is_wall(current,dis,-1.57);
		}
		else{
			if (wall_side==1) {
				dis = irr->getDistance();
				std::cout<<"wall in right: "<<dis<<std::endl;
				map->is_wall(current,dis,1.57);
			}
			else{
				if (wall_side==2) {
					dis = irf->getDistance();
					std::cout<<"wall in front: "<<dis<<std::endl;
					map->is_wall(current,dis,0);
					std::cout<<"I updated is wall"<<std::endl;
				}
			}
		}
	}
	void turn_setup() {
		float angle = map->plan(current);
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
			if (dr<=forward_side_dis_threshold) {
				if(!r_init) {
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


