#include "mraa.hpp"
#include <cassert>
#include <iostream>
#include <vector>
const double upper=0.491, lower=0.471;

#define SHIELD_I2C_ADDR 0x40

std::vector<uint8_t> sregisters = {
  6,   // output 0
  10,  // output 1
  14,  // output 2
  18,  // output 3
  22,  // output 4
  26,  // output 5
  30,  // output 6
  34,  // output 7
  38,  // output 8
  42,  // output 9
  46,  // output 10
  50,  // output 11
  54,  // output 12
  58,  // output 13
  62,  // output 14
  66   // output 15
};
class Servo
{
	int pin;
	bool forward;
	mraa::I2c* i2c;
	void writePWM(int index, double duty) 
	{
		assert(0.0 <= duty && duty <= 1.0);
		assert(0 <= index && index < 16);
		double on = 4096.0 * duty;
		uint16_t onRounded = (uint16_t) on;
		uint16_t offRounded = 4096 - onRounded;

		uint8_t writeBuf[2];

		// ON_L
		writeBuf[0] = sregisters[index];
		writeBuf[1] = onRounded & 0xff;
		i2c->address(SHIELD_I2C_ADDR);
		i2c->write(writeBuf, 2);

		// ON_H
		writeBuf[0] = sregisters[index] + 1;
		writeBuf[1] = (onRounded >> 8) & 0xff;
		i2c->address(SHIELD_I2C_ADDR);
		i2c->write(writeBuf, 2);

		// OFF_L
		writeBuf[0] = sregisters[index] + 2;
		writeBuf[1] = offRounded & 0xff;
		i2c->address(SHIELD_I2C_ADDR);
		i2c->write(writeBuf, 2);

		// OFF_H
		writeBuf[0] = sregisters[index] + 3;
		writeBuf[1] = (offRounded >> 8) & 0xff;
		i2c->address(SHIELD_I2C_ADDR);
		i2c->write(writeBuf, 2);
	}
	public:
	Servo(int _pin, bool dir=true)
	{
		forward=dir;
		pin=_pin;
		i2c=new mraa::I2c(6);
	}
	void write(double val)
	{
		if(!forward) val=2-val;
		double out=(lower+(upper-lower)*val);
		//std::cout<<"out "<<out<<std::endl;
		writePWM(pin,out);
	}
	void release()
	{
		writePWM(pin,0);
	}
};
