#include <cassert>
#include <iostream>
#include <algorithm>

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
	int count = 0;
	double sum = 0;
	int[] smaple = int[10];
	while (count < 10)
	{
		sum = sum + sensor.read();
		sample[count] = sensor.read();
		count++;
		usleep(100);
	}
	sort(sample,sample+10);
	return (sample[4]+sample[5])/2;
}
double get distance()
{}

int main()
{
	mraa::Aio sensor = mraa::Aio(0);
	std::cout<< "voltage:"<< instantRead(sensor)<<std::endl;
}