#include <cassert>
#include <iostream>
#include <mraa.hpp>
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
	int* sample = new int[10];
	while (count < 10)
	{
		sample[count] = sensor.read();
		count++;
		usleep(100);
	}
	std::sort(sample,sample+10);
	return (sample[4]+sample[5])/2;
}
double getDistance(mraa::Aio sensor)
{
	double voltage = averageRead(sensor)/1000;
	double distance[29] = {1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10,
	11,12,13,14,15,16,17,18,19,20};
	double svoltage[29] = {3.112,2.718,2.191,1.777,1.555,1.413,1.215,1.118,1.007,0.927,0.839,0.77,
	0.72,0.674,0.638,0.591,0.554,0.517,0.492,0.445,0.414,0.372,0.347,0.329,0.312,0.309,0.294,0.27,0.259};
	if (voltage >= 3.0)
	{
		std::cout<<"error!! too close"<<std::endl;
		return 0.0;
	}
	if (voltage <= 0.260)
	{
		std::cout<<"more that 20 in away :D"<< std::endl;
		return 100.0;
	}
	int count = 0;
	for(; svoltage[count]>=voltage; count++)
	{}
	return (distance[count-1]*(voltage-svoltage[count])+distance[count]*(svoltage[count-1]-voltage))/(svoltage[count-1]-svoltage[count]);
}

int main()
{
	mraa::Aio sensor = mraa::Aio(0);
	for(int i=0; i<10;i++)
	{
		double distance = getDistance(sensor);
		std::cout<< "distance:"<< distance <<std::endl;
		sleep(2);
	}
}
