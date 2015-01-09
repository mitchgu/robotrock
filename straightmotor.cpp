#include <cassert>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>
#include "mraa.hpp"
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>


#define MS 1000

#define GYRO_DATA_OKAY_MASK 0x0C000000
#define GYRO_DATA_OKAY 0x04000000


int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM and spi nicely\n");
        running = 0;
    }
}
/*
========================
gyro fixing code
*/
double spi()
{
  mraa::Gpio *chipSelect = new mraa::Gpio(10);
  chipSelect->dir(mraa::DIR_OUT);
  chipSelect->write(1);
  mraa::Spi* spi = new mraa::Spi(0);
  spi->bitPerWord(32);
  char rxBuf[2];
  char writeBuf[4];
  unsigned int sensorRead = 0x20000000;
  writeBuf[0] = sensorRead & 0xff;
  writeBuf[1] = (sensorRead >> 8) & 0xff;
  writeBuf[2] = (sensorRead >> 16) & 0xff;
  writeBuf[3] = (sensorRead >> 24) & 0xff;
  float total = 0;
  struct timeval tv;
  int init = 0;
  chipSelect->write(0);
  char* recv = spi->write(writeBuf, 4);
  chipSelect->write(1);
//    printf("%x %x %x %x\r\n", recv[0], recv[1], recv[2], recv[3]);
  if (recv != NULL) {
    unsigned int recvVal = ((uint8_t) recv[3] & 0xFF);
    recvVal = (recvVal << 8) | ((uint8_t)recv[2] & 0xFF);
    recvVal = (recvVal << 8) | ((uint8_t)recv[1] & 0xFF);
    recvVal = (recvVal << 8) | ((uint8_t)recv[0] & 0xFF);
    //printf("Received: 0x%.8x, ", recvVal);
    // Sensor reading
    short reading = (recvVal >> 10) & 0xffff;
    if (init) 
    {
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
		init = 1;
		gettimeofday(&tv, NULL);
    }
  }
  else 
  {
    //printf("No recv\n");
  }
  return total;
}

/*
========================
motor running code below
*/

/*
========================
set the pin first, dir, pwm are already well set before using this 
function
set the motor turning clockwise in speed (round/s)
not done yet*/
void clockwise(mraa::Gpio& dir, mraa::Pwm& pwm, double speed ) 
{
	dir.write(1);
	if (speed== 0.0) 
	{
		pwm.write(speed);
	}

}
/*
========================
moving forward for time and all the other pin are well set before using 
this function
time (s)
and then stop
*/
void forward(mraa::Gpio& vdir, mraa::Pwm& vpwm, mraa::Gpio& jdir, 
mraa::Pwm& jpwm, double time) 
{
	double speed = 0.05;
	vdir.write(1);
	jdir.write(0);
	vpwm.write(speed);
	vpwm.write(speed);
	usleep(time*1000);
	vpwm.write(0);
	vpwm.write(0);

}

/*
========================
Jingyi run with speed
*/
void jingyi(mraa::Gpio& jdir, mraa::Pwm& jpwm, double speed) 
{
	int velocity = speed;
	if (speed >=0)
	{
		jdir.write(1);
	}
	else 
	{
		jdir.write(0);
		velocity = -velocity;
	}
	jpwm.write(velocity);

}

/*
========================
Victoria run with speed
*/
void victoria(mraa::Gpio& vdir, mraa::Pwm& vpwm, double speed)
{
	int velocity = speed;
	if (speed >=0)
	{
		vdir.write(0);
	}
	else 
	{
		vdir.write(1);
		velocity = -velocity;
	}
	vpwm.write(velocity);
}

/*
========================
keep going staight without turning (even though you turn it by hand)
*/
void feedbackstraight(mraa::Gpio& vdir, mraa::Pwm& vpwm, mraa::Gpio& 
jdir, mraa::Pwm& jpwm, double time) 
{

}
int main()
{
	signal(SIGINT, sig_handler);
	mraa::Gpio vdir = mraa::Gpio(8);
	mraa::Gpio jdir = mraa::Gpio(12);
	mraa::Pwm jpwm = mraa::Pwm(5);
	mraa::Pwm vpwm = mraa::Pwm(3);
	double jspeed = 0.2;
	double vspeed = 0.2;
	double K = 0.001;
	while(running) 
	{
		double angle = spi();
		if (angle>=0)
		{
			vspeed = vspeed-angle*K;
			jspeed = jspeed+angle*K;
		}
		victoria(vdir,vpwm,vspeed);
		jingyi(jdir,jpwm,jspeed);
		sleep(0.001);
	}
    jpwm.write(0);
    vpwm.write(0);
    sleep(0.001);
    jpwm.write(0);
    vpwm.write(0);
    sleep(1);
    return 0;
}
