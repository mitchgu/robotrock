#include <math.h>
#include "motor.cpp"
#include "data.cpp"
#include <sys/time.h>

class Odometry {
	static const float diameter = 4; //in
	static const float whealdistance = 10; //in
	Location* location;
	Motor* left;
	Motor* right;
	struct timeval tv;
	bool init;
public:
	Odometry (Motor* l, Motor* r,float xval, float yval, float thetaval)
	{
		left = l;
		right = r;
		location = new Location(xval, yval, thetaval);
	}
/*
reset odometry
*/

	void set(Location* _location) 
	{
		location=_location;
		gettimeofday(&tv,NULL);
	}
/*
run pointer=&object;
*/
	Location* run()
	{
		if(init) 
		{
			float lrps = left->rps();
			float rrps = right->rps();
			float speed = (lrps+rrps)* M_PI * diameter/2; //ins
			float rotationalspeed = M_PI *(lrps-rrps)*diameter/whealdistance; //rps
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			ms -= (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			int msi = (int)ms;
			float msf = (float)msi;
			float distance = msf*speed/1000;
			float angledelta = msf*rotationalspeed/1000;
			float x = location->x()+ sin(location->theta())*distance;
			float y = location->y()+ cos(location->theta())*distance;
			float angle = location->theta()+angledelta;
			location->set(x,y,angle);
		}
		else 
		{
			init=true;
			gettimeofday(&tv, NULL);
		}
		return location;
	}
};
