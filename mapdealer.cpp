#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <deque>  
#include <algorithm>
#include "data.cpp"
#include <assert.h> 

#define tr(i) for(typeof(i.begin()) it=i.begin(); it!=i.end(); it++) 

class Mapdealer {
	std::deque<cPoint*>* point_dq; 
	std::deque<Wall*>* wall_dq;
	cPoint* start;
	bool point_is_in_dq(cPoint* pt) 
	{
		for (std::deque<cPoint*>::iterator it = point_dq->begin(); it!=point_dq->end(); ++it) 
		{
			if ((*it)->equalTo(pt)) return true;
		}
		return false;
	}
	int thing_to_combine(Wall* wall) {
		int i = 0;
		for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) 
		{
			if (wall->can_combine(*it)) return i;
			i++;
		}
		return -1;
	}
public:
	Mapdealer(char* str) {
		point_dq = new std::deque<cPoint*>();
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
        			cPoint* starts = new cPoint(xs,ys);
        			cPoint* ends = new cPoint(xe,ye);
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
					start = new cPoint(xs,ys);
				}
			}
			myfile.close();
			std::cout<<"closing file!"<<std::endl;
		}
	}
	std::vector<Wall> getPolygon() {
		std::vector<Wall> out;

		sort(point_dq->begin(),point_dq->end());

		cPoint beg=**(point_dq->begin()),cur=beg;
		do
		{
			std::cout<<cur.x()<<" "<<cur.y()<<std::endl;
			//if(cur.x()==120&&cur.y()==120) return out;
			tr((*wall_dq))
			{
				if((*it)->s()==cur)
				{
					//std::cout<<"wall: "<<((*it)->xs())<<" "<<((*it)->xs())<<" "<<((*it)->xs())<<" "<<((*it)->xs())<<std::endl;
					cur=(*it)->e();
					out.push_back(**it);
					wall_dq->erase(it);
					break;
				}
				else if((*it)->e()==cur)
				{
					//std::cout<<"wall: "<<((*it)->xs())<<" "<<((*it)->xs())<<" "<<((*it)->xs())<<" "<<((*it)->xs())<<std::endl;
		//std::cout<<"wall: "<<((*it)->xs())<<std::endl;
					cur=(*it)->s();
					(*it)->swap();
					out.push_back(**it);
					wall_dq->erase(it);
					break;
				}
			}
		} while(cur!=beg);

		return out;
	}
	std::deque<cPoint*>* stack_of_point() {
		return point_dq;
	}
	std::vector<Wall> allWalls() {
		std::vector<Wall> out;
		tr((*wall_dq)) out.push_back(**it);
		return out;
	}
	std::deque<Wall*>* stack_of_wall() {
		return wall_dq;
	}
	cPoint* getStart(){
		return start;
	}
};
/*
int main() {
	Mapdealer* map = new Mapdealer("example2.txt");
	std::vector<Wall> test=map->getPolygon();
	//std::deque<Wall*>::iterator it;
	tr(test)std::cout<<"wall: "<<((it)->xs())<<" "<<((it)->ys())<<" "<<((it)->xe())<<" "<<((it)->ye())<<std::endl;
	std::deque<cPoint*>* point_dq = map->stack_of_point();
	std::deque<Wall*>* wall_dq = map->stack_of_wall();
	for (std::deque<Wall*>::iterator it = wall_dq->begin(); it!=wall_dq->end(); ++it) {
		std::cout<<"wall: "<<(*it)->xs()<<" "<<(*it)->ys()<<" "<<(*it)->xe()<<" "<<(*it)->ye()<<std::endl;
	}
	return 0;
}
*/
