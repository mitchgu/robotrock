#include <math.h>
#include "motor.cpp"
#include <sys/time.h>
#include "gyro.cpp"


const float diameter = 3.85; //in
const float whealdistance = 8.875; //in
class Odometry {
	Gyroscope* gyr;
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
		gyr = new Gyroscope(10);
		gyr->reset(thetaval);
		init = false;
		location = new Location(xval, yval, thetaval);
	}
/*
reset odometry
*/
	void set(Location* _location) 
	{
		location=_location;
		gyr->reset(_location->theta());
		gettimeofday(&tv,NULL);
	}
/*
run pointer=&object;
*/
	float getAngle(){
		return location->theta();
	}
	Location* run()
	{
		gettimeofday(&tv, NULL);
		if(init) 
		{
			float lrps = left->rps();
			float rrps = right->rps();
			float speed = (lrps+rrps)* M_PI * diameter/2; //ins
			float angle = gyr->run();
			unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			gettimeofday(&tv, NULL);
			unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
			float msf = (float)(msl-ms);
			float distance = msf*speed/1000;
			float x = location->x()+ (sin(angle))*distance;
			float y = location->y()+ (cos(angle))*distance;
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
