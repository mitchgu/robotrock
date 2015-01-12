#include <math.h>
class Location {
	float _x;
	float _y;
	float _theta;
public:
	Location(float xval, float yval, float thetaval) 
	{
		_x = xval;
		_y = yval;
		_theta = thetaval;
	}
	void set(float xval, float yval, float thetaval) 
	{
		_x = xval;
		_y = yval;
		_theta = thetaval;
	}
	float x()
	{
		return _x;
	}
	float y()
	{
		return _y;
	}
	float theta()
	{
		return _theta;
	}
	float distance(Location* other)
	{
		return sqrt(pow((other->x()-x()),2.0)+pow((other->y()-y()),2.0));
	}

};