#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>
#include <math.h>


class Gyroscope 
{
	mraa::Gpio *chipSelect;
	mraa::Spi* spi;
	char rxBuf[2];
	char writeBuf[4];
	float total;
	struct timeval tv;
	bool init;
public:
//set chipselect pin
	Gyroscope(int gyrp)
	{
		chipSelect=new mraa::Gpio(gyrp);    //setting pin
		chipSelect->dir(mraa::DIR_OUT);
		chipSelect->write(1);
	 	spi = new mraa::Spi(0);
		spi->bitPerWord(32);
		unsigned int sensorRead = 0x20000000;
		writeBuf[0] = sensorRead & 0xff;
		writeBuf[1] = (sensorRead >> 8) & 0xff;
		writeBuf[2] = (sensorRead >> 16) & 0xff;
		writeBuf[3] = (sensorRead >> 24) & 0xff;
		total=0, init=false;
	}
/*
set the initial angle 
*/
	void set(float angle)
	{
		total = angle;
		gettimeofday(&tv, NULL);
	}
/*
return believed angle
*/
	float run()
	{
		chipSelect->write(0);
		char* recv = spi->write(writeBuf, 4);
		chipSelect->write(1);
		if (recv != NULL)
		{
			unsigned int recvVal = ((uint8_t) recv[3] & 0xFF);
			recvVal = (recvVal << 8) | ((uint8_t)recv[2] & 0xFF);
			recvVal = (recvVal << 8) | ((uint8_t)recv[1] & 0xFF);
			recvVal = (recvVal << 8) | ((uint8_t)recv[0] & 0xFF);
			//printf("Received: 0x%.8x, ", recvVal);
			// Sensor reading
			short reading = (recvVal >> 10) & 0xffff;
			if (init) {
				unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
					(unsigned long long)(tv.tv_usec) / 1000;
				gettimeofday(&tv, NULL);
				ms -= (unsigned long long)(tv.tv_sec)*1000 +
					(unsigned long long)(tv.tv_usec) / 1000;
				int msi = (int)ms;
				float msf = (float)msi;
				float rf = (float)reading;
				total += -0.001 * msf * (rf / 80.0);
				//printf("Total: %f, Reading: %f, Time: %f\n", total, rf, -msf);
			}
			else {
				init=true;
				gettimeofday(&tv, NULL);
			}
		}
		return total * M_PI/180;
	}
};



class Location {
	float x;
	float y;
	float theta;
	Location(float xval, float yval, float thetaval) 
	{
		x = xval;
		y = yval;
		theta = thetaval;
	}
	void set(float xval, float yval, float thetaval) 
	{
		x = xval;
		y = yval;
		theta = thetaval;
	}
	float x()
	{
		return x;
	}
	float y()
	{
		return y;
	}
	float theta()
	{
		return theta;
	}
	float distance(Location* other)
	{
		return sqrt(pow((other->x()-x()),2.0)+pow((other->y()-y()),2.0))
	}

}



class Odometry {
	const float diameter = 4; //in
	const float whealdistance = 10; //in
	Location* location;
	Motor* left;
	Motor* right;
	struct timeval tv;
	bool init;
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
			location.set(x,y,angle);
		}
		else 
		{
			init=true;
			gettimeofday(&tv, NULL);
		}
		return location;
	}
}
/*
class State {
	Location* location;
	Location* target;
	Odometry* odo;
	Gyroscope* gyr;
	Motor* left;
	Motor* right;
	IR* ir1,ir2,ir3,ir4;
	bool map [300][300];
public:
	State(int ldirp, int rdirp, int lpwmp, int rpwmp,int rhall, int lhall, int IR1p, int IR2p, int IR3p, int IR4p, bool* setmap)
	{
		left = new Motor(ldirp,lpwmp,lhall,0);
		right = new Motor(rdirp,rpwmp,rhall,1);
		ir1 = new IR(IR1p);
		ir2 = new IR(IR2p);
		ir3 = new IR(IR3p);
		ir4 = new IR(IR4p);
		setmap = map;;
	}

	void reset (Location _location) {
		odo->set(_location);
		gyr->set(_location->theta());
	}

	void setTarget (Location _target) {
		target = _target;
	}

	void track () {

	}

	bool planwork () {

	}


	Location[] plan() {

	}

	void approach () {
		Location[] Myplan = plan();
		while ()


	}
}
*/
class Motor
{
	mraa::Gpio* dir;
	mraa::Pwm* pwm;
	mraa::Gpio* hall;
	bool side; //0 -left or 1-right
	float speed;
public:
	Motor(int dpin, int ppin, int hpin, bool _side)
	{
		dir=new mraa::Gpio(dpin);
		pwm=new mraa::Pwm(ppin);
		hall=new mraa::Gpio(hpin);
		pwm->enable(true);
		dir->dir(mraa::DIR_OUT);
		hall->dir(mraa::DIR_IN);
		side=_side;
		std::cout<<"set at "<<side<<std::endl;
		speed=0;
	}	
	void forward()
	{
		if(side) dir->write(1);
		else dir->write(0);
	}
	void backward()
	{
		if(side) dir->write(0);
		else dir->write(1);
	}
	void setSpeed(float set)
	{
		speed=set;
		pwm->write(speed);
	}
	float getSpeed() { return speed; }

	/*
	get the real speed
	*/
	float rps()
	{
		struct timeval tv;
		int reading = hall->read();
		while(reading == hall->read())
		{
			usleep(10);
		}
		gettimeofday(&tv,NULL);
		unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		int count==0;
		reading = hall->read();
		while(count<=10) 
		{
			if (reading != hall->read())
			{
				count++;
				reading = hall->read();
			}
			usleep(10);
		}
		gettimeofday(&tv,NULL);
		ms -= (unsigned long long)(tv.tv_sec)*1000 +
						(unsigned long long)(tv.tv_usec) / 1000;
		float msf = (float)ms;
		return (1/((msf/10000)*2))/1920;
	}
};

int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
}

int main()
{
	signal(SIGINT, sig_handler);
	Motor left(12,5,4,false),right(8,3,2,true);
	Gyroscope gyr(10);

	const float target=0, K=0.003, base=0.05;

	left.forward();
	right.forward();

	left.setSpeed(base);
	right.setSpeed(base);

	
	while(running) 
	{
		/*
		float angle=gyr.run();
		float diff=(angle-target)*K;
		std::cout<<angle<<" "<<diff<<std::endl;
		left.setSpeed(base);
		right.setSpeed(base);
		if(diff>0) left.setSpeed(base+diff);
		else right.setSpeed(base-diff);
		sleep(0.01);
		*/
		left.setSpeed(base);
		float realspeed = left.rps();
		std::cout<<realspeed<<" "<<diff<<std::endl;
	}

	left.setSpeed(0);
	right.setSpeed(0);
    	sleep(1);
    	return 0;
}
