#include <iostream>
#include <algorithm>

#include "mraa.hpp"

using namespace std;

#define START 0
#define SEARCH 1
#define SCH_KNOCK 2
#define SCH_PICK 3
#define SCAN 4
#define SCA_KNOCK 5
#define SCA_PICK 6
#define GO_DROP 7
#define DROP 8

int state=START;

void run() 
{
	switch(state)
	{
		case START:
			//initialzie everything
			state=SEARCH;
		break;

		case SCH_KNOCK:
		break;

		case SEARCH:

		break;

		case SCAN:
		break;

		case SCA_KNOCK:
		break;

		case SCA_PICK:
		break;

		case GO_DROP:
		break;
		
		case DROP:
		break;
	}
}

int main()
{
	while(true) run();
	return 0;
}
