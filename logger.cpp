#include <iostream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <string>

using namespace std;

//Current date and time, YYYY-MM-DD.HH:mm:ss
const string currentDateTime() 
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

long long getmsofday()
{
   struct timeval tv;
   gettimeofday(&tv, 0);
   return (long long)tv.tv_sec*1000 + tv.tv_usec/1000;
}

class Logger {
	long long time_start;
	string text_file;
public:
	Logger ()
	{
		time_start = getmsofday();
		text_file = currentDateTime() + ".txt";
		ofstream logging;
		logging.open(text_file.c_str());
		logging.close();
	}

	void log (string name, string val)
	{
		ofstream logging;
		logging.open(text_file.c_str(), ios_base::app);
		long long time_current = getmsofday();
		logging << time_current - time_start << " " << name << ": " << val << endl;
		cout << time_current - time_start << " " << name << ": " << val << endl;
		logging.close();
	}

} ;

int main () {
	Logger test_log;
	test_log.log("IR1", "hello");
	test_log.log("IR2", "hello again");
	return 0;
}