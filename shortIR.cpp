#include <cassert>
#include <iostream>

/*
read the instant value of the IR sensor
input: 
mraa::Aio sesorpin
*/
double instantRead(mraa::Aio sensor)
{
	double result = sensor.read();
	return result;
}

/*
read 10 IR sensor value shown in 1ms
input:
mraa::Aio sensorpin
*/
double averageRead(mraa::Aio sensor)
{
	int count = 10;
	double sum = 0;
	while (count > 0)
	{
		sum = sum + sensor.read();
		count--;
		usleep(100);
	}
	return sum/10.0;
}
double get distance()
{}

int main()
{
	mraa::Aio sensor = mraa::Aio(0);
	std::cout<< "voltage:"<< instantRead(sensor)<<std::endl;
}