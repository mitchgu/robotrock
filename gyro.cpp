#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include "mraa.hpp"
#define MS 1000
#define GYRO_DATA_OKAY_MASK 0x0C000000
#define GYRO_DATA_OKAY 0x04000000

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
	Gyroscope(int gpin)
	{
		chipSelect=new mraa::Gpio(gpin);
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
	void reset()
	{
		total=0;
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
			printf("Received: 0x%.8x, ", recvVal);
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
				init = 1;
				gettimeofday(&tv, NULL);
			}
		}
		else {
			printf("No recv\n");
		}
		return total*3.14/180;
	}

};
