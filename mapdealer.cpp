#include <iostream>
#include <fstream>
#include <string>
#include <deque>  
#include <algorithm>
#include "data.cpp"
#include <assert.h> 

class Mapdealer {
	std::deque<Point*>* point_dq; 
	std::deque<Wall*>* wall_dq;
	Point* start;
	bool point_is_in_dq(Point* pt) {
		for (std::deque<Point*>::iterator it = point_dq->begin(); it!=point_dq->end(); ++it) {
			if ((*it)->equalTo(pt)) return true;
		}
		return false;
	}
	int thing_to_combine(Wall* wall) {
		int i = 0;
		for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) {
			if (wall->can_combine(*it)) return i;
			i++;
		}
		return -1;
	}

public:
	Mapdealer(std::string str) {
		point_dq = new std::deque<Point*>();
		wall_dq = new std::deque<Wall*>();
		std::string line;
		int comma=1;
		int xs; int ys; int xe; int ye;
		int cnt = 0;
  		std::string sub;
		std::ifstream myfile (str);
  		if (myfile.is_open()) {
    		while ( getline( myfile, line ) ) {
    			if ((line[0] == 'W')|| line[0] =='P') {
        			for (int i=2; i<line.length(); i++) {
        				if (line[i] == ',') {
            				sub = line.substr(comma+1,i-comma);
            				if (cnt==0) {
            					xs = 24*atoi( sub.c_str() );
            				}
            				if (cnt==1) {
            					ys = 24*atoi( sub.c_str() );
            				}
            				if (cnt==2) {
            					xe = 24*atoi( sub.c_str() );
            				}
            				cnt++;
            				comma = i;
          				}
        			}
        			sub = line.substr(comma+1,line.length()-comma);
        			ye = 24*atoi( sub.c_str() );
        			cnt=0; comma = 1;
                    std::cout<<"wall: "<<xs<<" "<<ys<<" "<<xe<<" "<<ye<<std::endl;
        			Wall* wall = new Wall(xs,ys,xe,ye);
        			Point* starts = new Point(xs,ys);
        			Point* ends = new Point(xe,ye);
        			bool startin = point_is_in_dq(starts);
        			bool endin = point_is_in_dq(ends);
        			if (!startin) point_dq->push_back(starts);
        			if (!endin) point_dq->push_back(ends);
        			if (startin || endin) {
        				int i = thing_to_combine(wall);
                        std::cout<<"can combine:  "<<i<<std::endl;
        				if (i != -1) {
        					wall_dq->at(i) = wall_dq->at(i)->combine(wall);
                            std::cout<<"after combined: "<<wall_dq->at(i)->xs()<<" "<<wall_dq->at(i)->ys()<<" "<<wall_dq->at(i)->xe()<<" "<<wall_dq->at(i)->ye()<<std::endl;
        					int j=0; 
        					for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) {
        						if (j!=i){
									if (wall_dq->at(i)->can_combine(*it)) {
										wall_dq->at(i) = wall_dq->at(i)->combine(wall_dq->at(j));
										wall_dq->erase(wall_dq->begin()+j);
                                        std::cout<<"can combine j:  "<<j<<std::endl;
                                        std::cout<<"after combined: "<<wall_dq->at(i)->xs()<<" "<<wall_dq->at(i)->ys()<<" "<<wall_dq->at(i)->xe()<<" "<<wall_dq->at(i)->ye()<<std::endl;
                                        break;
									}
								}
                                j++;
							}
        				}
        				else {
        					wall_dq->push_back(wall);
        				}
        			}
        			else wall_dq->push_back(wall);
      			}
      			if (line[0] =='L') {
      				for (int i=2; i<line.length(); i++) {
        				if (line[i] == ',') {
            				sub = line.substr(comma+1,i-comma);
            				xs = 24*atoi( sub.c_str() );
            				comma = i;
            			}
            		}
            		sub = line.substr(comma+1,line.length()-comma);
        			ys = 24*atoi( sub.c_str() );
        			cnt=0; comma = 1;
        			start = new Point(xs,ys);
      			}
    		}
    		myfile.close();
  		}
	}
    std::deque<Point*>* stack_of_point() {
        return point_dq;
    }
    std::deque<Wall*>* stack_of_wall() {
        return wall_dq;
    }
};
int main() {
    Mapdealer* map = new Mapdealer("example.txt");
    std::deque<Point*>* point_dq = map->stack_of_point();
    std::deque<Wall*>* wall_dq = map->stack_of_wall();
    for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) {
        std::cout<<"wall: "<<(*it)->xs()<<" "<<(*it)->ys()<<" "<<(*it)->xe()<<" "<<(*it)->ye()<<" ,and angle: "<<(*it)->angle()<<std::endl;
    }
    return 0;
}
