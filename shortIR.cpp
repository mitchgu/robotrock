#include <cassert>
#include <iostream>
#include <mraa.hpp>
#include <algorithm>

class IR {
	mraa::Aio* _IR;
	public:
	IR (int IRp) 
	{
		_IR = new mraa::Aio(IRp);
	}
	/*
	   read the instant value of the IR sensor
input: 
mraa::Aio sesorpin
	 */
	float instantRead()
	{
		float result = _IR->read();
		return result*5/1024;
	}

	/*
	   read 20 IR sensor value shown in 2ms
input:
mraa::Aio sensorpin
	 */
	float averageRead()
	{	
		int count = 0;
		int* sample = new int[20];
		while (count < 20)
		{
			sample[count] = _IR->read();
			count++;
			usleep(100);
		}
		std::sort(sample,sample+20);
		return ((sample[9]+sample[10])/2)*5.0/1024;
	}
	/*
	   get the Distance from the wall (in)
	 */

	float getDistance()
	{
		float voltage = averageRead();
		float distance[29] = {1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10,
			11,12,13,14,15,16,17,18,19,20};
		float svoltage[29] = {3.112,2.718,2.191,1.777,1.555,1.413,1.215,1.118,1.007,0.927,0.839,0.77,
			0.72,0.674,0.638,0.591,0.554,0.517,0.492,0.445,0.414,0.372,0.347,0.329,0.312,0.309,0.294,0.27,0.259};
		if (voltage >= 2.5)
		{
			//std::cout<<"error!! too close"<<std::endl;
			return 0.0;
		}
		if (voltage <= 0.280)
		{
			//std::cout<<"more that 10 in away :D"<< std::endl;
			return 20.0;
		}
		int count = 0;
		for(; svoltage[count]>=voltage; count++)
		{}
		return (distance[count-1]*(voltage-svoltage[count])+distance[count]*(svoltage[count-1]-voltage))/(svoltage[count-1]-svoltage[count]);
	}
};

/*
test IR code*/ 
