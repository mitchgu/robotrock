#include <math.h>
#include "motor.cpp"
#include <sys/time.h>

class Odometry {
	static const float diameter = 3.85; //in
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
		init = false;
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
		gettimeofday(&tv, NULL);
		if(init) 
		{
			float lrps = left->rps();
			float rrps = right->rps();
			float speed = (lrps+rrps)* M_PI * diameter/2; //ins
			float rotationalspeed = M_PI *(lrps-rrps)*diameter/whealdistance; //rps
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			float msf = (float)(msl-ms);
			float distance = msf*speed/1000;
			float angledelta = msf*rotationalspeed/1000;
			float x = location->x()+ (sin(location->theta()))*distance;
			float y = location->y()+ (cos(location->theta()))*distance;
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
