#include <math.h>
#include <assert.h> 

class Point {
	int _x;
	int _y;
public:
	Point(int x,int y) {
		_x = x;
		_y = y;
	}
	int x() {
		return _x;
	}
	int y() {
		return _y;
	}
	bool equalTo(Point* other) {
		return ((x() == other->x())&&(y() == other->y()));
	}
};

class Wall {
	int _xs;
	int _ys;
	int _xe;
	int _ye;
	float wall_angle;
	float wall_length;
public:
	Wall (int xs, int ys, int xe, int ye) {
		_xs = xs;
		_ys = ys;
		_xe = xe;
		_ye = ye;
		wall_length = sqrt(pow((_xs-_xe),2.0)+pow((_ys-_ye),2.0));
		wall_angle = atan(((float)(_xe-_xs))/(_ye-_ys));
	}
	float length() {
		return wall_length;
	}
	float angle() {
		return wall_angle;
	}
	int xs() {
		return _xs;
	}
	int xe() {
		return _xe;
	}
	int ys() {
		return _ys;
	}
	int ye() {
		return _ye;
	}
	Wall* combine(Wall* other) {
		//assert (other->angle()==angle());
		Wall* return_wall;
		if (((other->xs()==xs())&&(other->ys()==ys()))) {
			return_wall = new Wall(other->xe(),other->ye(),xe(),ye());
		}
		if (((other->xs()==xe())&&(other->ys()==ye()))) {
			return_wall = new Wall(other->xe(),other->ye(),xs(),ys());
		}
		if (((other->xe()==xe())&&(other->ye()==ye()))) {
			return_wall = new Wall(other->xs(),other->ys(),xs(),ys());
		}
		if (((other->xe()==xs())&&(other->ye()==ys()))) {
			return_wall = new Wall(other->xs(),other->ys(),xe(),ye());
		}
		return return_wall;
	}
	bool can_combine(Wall* other) {
		if (((other->xs()==xs())&&(other->ys()==ys())) || ((other->xe()==xe())&&(other->ye()==ye())) || ((other->xs()==xe())&&(other->ys()==ye())) || ((other->xe()==xs())&&(other->ye()==ys()))) {
			if ((other->ys() == other->ye())&& (ys()==ye())) {
				return true;
			}
			else {
				return ((((float)(other->xs()-other->xe()))/(other->ys()-other->ye())) == (((float)(xs()-xe()))/(ys()-ye())));
			}
		}
		else {return false;}
	}
};
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
	Location* move(float dis,float inputtheta) {
		Location* returnlocation;
		float returntheta = theta() + inputtheta;
		float returnx = x()+dis*sin(returntheta);
		float returny = y()+dis*cos(returntheta);
		returnlocation->set(returnx, returny, returntheta);
		return returnlocation;
	}
	float distance(Location* other)
	{
		return sqrt(pow((other->x()-x()),2.0)+pow((other->y()-y()),2.0));
	}

};
