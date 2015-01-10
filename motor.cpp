#include <cassert>
#include <sys/time.h>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include <mraa.hpp>
#include <signal.h>

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
	Gyroscope()
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
		total=0, init=false;
	}
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
				printf("Total: %f, Reading: %f, Time: %f\n", total, rf, -msf);
			}
			else {
				init=true;
				gettimeofday(&tv, NULL);
			}
		}
		return total;
	}
};
class Motor
{
	mraa::Gpio* dir;
	mraa::Pwm* pwm;
	bool side; //0 -left or 1-right
	float speed;
public:
	Motor(int dpin, int ppin, bool _side)
	{
		dir=new mraa::Gpio(dpin);
		pwm=new mraa::Pwm(ppin);
		pwm->enable(true);
		dir->dir(mraa::DIR_OUT);
		side=_side;
		std::cout<<"set at "<<side<<std::endl;
		speed=0;
	}	
	void forward()
	{
		if(side) dir->write(1);
		else dir->write(1);
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
	Motor left(8,3,false),right(12,5,true);
	Gyroscope gyr;

	const float target=0, K=0.003, base=0.1;

	left.forward();
	right.forward();

	left.setSpeed(base);
	right.setSpeed(base);

	
	while(running) 
	{
		float angle=gyr.run();
		float diff=(angle-target)*K;
		std::cout<<angle<<" "<<diff<<std::endl;
		left.setSpeed(base);
		right.setSpeed(base);
		if(diff>0) left.setSpeed(base+diff);
		else right.setSpeed(base-diff);
		sleep(0.01);
	}

	left.setSpeed(0);
	right.setSpeed(0);
    	sleep(1);
    	return 0;
}
