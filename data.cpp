#include <math.h>
#include <iostream>
#include <algorithm>
#include <assert.h> 

class cPoint {
	float _x;
	float _y;
public:
	cPoint(float x,float y) {
		_x = x;
		_y = y;
	}
	float x() {
		return _x;
	}
	float y() {
		return _y;
	}
	bool operator<(cPoint other) {
			if(_x==other._x) return _y<other._y;
			return _x<other._x;
	}
	bool operator==(cPoint other) {
		return ((x() == other.x())&&(y() == other.y()));
	}
	bool operator!=(cPoint other) {
		return ((x() != other.x())||(y() != other.y()));
	}
	bool equalTo(cPoint* other) {
		return ((x() == other->x())&&(y() == other->y()));
	}
	float distance(cPoint* other){
		return sqrt(pow((other->x()-x()),2.0)+pow((other->y()-y()),2.0));
	}
	float three_points_angle(cPoint* p1, cPoint* p2) {
		float theta1 = atan2((p1->y()-y()),(p1->x()-x()));
		float theta2 = atan2((p2->y()-p1->y()),(p2->x()-p1->x()));
		return (theta1-theta2);
	}
	cPoint* four_points_crossing(cPoint* p1, cPoint* p2, cPoint* p3) {
		float m1 = (x()-p1->x())/(y()-p1->y());
		float m2 = (p2->x()-p3->x())/(p2->y()-p3->y());
		float return_x = (m1*m2*(p3->y()-y())+m2*x()-m1*p3->x())/(m2-m1);
		float return_y = (x()-p3->x()+m2*p3->y()-m1*y())/(m2-m1);
		cPoint* return_point = new cPoint(return_x,return_y);
		return return_point;
	}
	cPoint operator-(cPoint other) {
		cPoint out(x()-other.x(),y()-other.y());
	}
	float abs()
	{
		return sqrt(_x*_x+_y*_y);
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
	cPoint s() {
		return cPoint(_xs,_ys);
	}
	cPoint e() {
		return cPoint(_xe,_ye);
	}
	void swap()
	{
		std::swap(_xs,_xe);
		std::swap(_ys,_ye);
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
	Location(Location* other)
	{
		_x = other->x();
		_y = other->y();
		_theta = other->theta();
	}
	void incAngle(float inc)
	{
		_theta+=inc;
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
		returnlocation = new Location(returnx, returny, returntheta);
		return returnlocation;
	}
	Location* move_forward(float dis) {
		Location* return_location;
		float return_theta = theta();
		float return_x = x() + dis*sin(return_theta);
		float return_y = y() + dis*cos(return_theta);
		return_location = new Location(return_x,return_y,return_theta);
		return return_location;
	}
	cPoint* point() {
		cPoint* pt = new cPoint(x(),y());
		return pt;
	}
	float distance(Location* other)
	{
		return sqrt(pow((other->x()-x()),2.0)+pow((other->y()-y()),2.0));
	}

};
