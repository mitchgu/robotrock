#include <math.h>
#include "motor.cpp"
#include <sys/time.h>


const float diameter = 3.85; //in
const float whealdistance = 8.875; //in
const int gpin=10;
class Odometry {
	Location* location;
	Motor* left;
	Motor* right;
	mraa::Gpio *chipSelect;
	mraa::Spi* spi;
	char rxBuf[2];
	char writeBuf[4];
	struct timeval tv;
	bool init;
	double timeDiff()
	{
		unsigned long long ms = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec) / 1000;
		gettimeofday(&tv, NULL);
		unsigned long long msl = (unsigned long long)(tv.tv_sec)*1000 + (unsigned long long)(tv.tv_usec) / 1000;
		double msf = (double)(msl-ms);
		return msf;
	}
public:
	Odometry (Motor* l, Motor* r,Location* _location)
	{
		chipSelect=new mraa::Gpio(10);
		chipSelect->dir(mraa::DIR_OUT);
		chipSelect->write(1);
	 	spi = new mraa::Spi(0);
		spi->bitPerWord(32);
		unsigned int sensorRead = 0x20000000;
		writeBuf[0] = sensorRead & 0xff;
		writeBuf[1] = (sensorRead >> 8) & 0xff;
		writeBuf[2] = (sensorRead >> 16) & 0xff;
		writeBuf[3] = (sensorRead >> 24) & 0xff;
		init=false;
		left = l; right = r;
		location=_location;
	}
	Odometry (Motor* l, Motor* r,float xval, float yval, float thetaval)
	{
		chipSelect=new mraa::Gpio(10);
		chipSelect->dir(mraa::DIR_OUT);
		chipSelect->write(1);
	 	spi = new mraa::Spi(0);
		spi->bitPerWord(32);
		unsigned int sensorRead = 0x20000000;
		writeBuf[0] = sensorRead & 0xff;
		writeBuf[1] = (sensorRead >> 8) & 0xff;
		writeBuf[2] = (sensorRead >> 16) & 0xff;
		writeBuf[3] = (sensorRead >> 24) & 0xff;
		init=false;
		left = l; right = r;
		location=new Location(xval,yval,thetaval);
	}
/*
reset odometry
*/
	void set(Location* _location) 
	{
		location=_location;
		gettimeofday(&tv,NULL);
		init=true;
	}
/*
run pointer=&object;
*/
	double getAngle()
	{
		return location->theta();
	}
	void run()
	{
		chipSelect->write(0);
		char* recv = spi->write(writeBuf, 4);
		chipSelect->write(1);
		if(init) 
		{
			double msf=timeDiff();
			std::cout<<"Time diff "<<msf<<std::endl;
			//angle code
			double angle;
			if (recv != NULL)
			{
				unsigned int recvVal = ((uint8_t) recv[3] & 0xFF);
				recvVal = (recvVal << 8) | ((uint8_t)recv[2] & 0xFF);
				recvVal = (recvVal << 8) | ((uint8_t)recv[1] & 0xFF);
				recvVal = (recvVal << 8) | ((uint8_t)recv[0] & 0xFF);
				short reading = (recvVal >> 10) & 0xffff;
				angle=location->theta()+ (0.001 * msf * (reading / 80.0)*3.14/180);
			}
			else {
				printf("No recv\n");
			}

			//odo
			double lrps = left->srps(); float rrps = right->srps();
			double speed = (lrps+rrps)* M_PI * diameter/2; //ins
			double distance = msf*speed/1000;
			double x = location->x()+ (sin(angle))*distance;
			double y = location->y()+ (cos(angle))*distance;
			location->set(x,y,angle);
			std::cout<<"Position "<<location->x()<<" "<<location->y()<<" "<<location->theta()*180/3.14<<std::endl;
		}
		else 
		{
			init=true;
			gettimeofday(&tv, NULL);
			std::cout<<"INIT Position "<<location->x()<<" "<<location->y()<<" "<<location->theta()<<std::endl;
		}
	}
};
