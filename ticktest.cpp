#include <iostream>
#include "motion.cpp"
#include "servo.cpp"

int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
};

int main()
{
	signal(SIGINT, sig_handler);
	Motor* base = new Motor(10,11,6,false); //base motor -counterclockwise when forward
	Motor* lift = new Motor(8,9,6,false); //lift motor
	Motor* left = new Motor(0,2,4,false);
	Motor* right = new Motor(4,6,2,true);
	Odometry* odo = new Odometry(left, right, 0, 0, 0);
	Location* location = new Location(0.0,0.0,0.0);

	Motion* motion = new Motion(left,right,odo,location); 

	//motion->straight(12);
	//while(running&&!motion->run() ) usleep(10000);
	left->stop();
	right->stop();
	sleep(1);

	Servo claw(15);

	claw.write(0.7);

	lift->forward();
	lift->setSpeed(5);
	usleep(1700000);
	lift->stop();
	sleep(1);

	claw.write(0.15);
	sleep(1);

	lift->backward();
	lift->setSpeed(5);
	usleep(1700000);
	lift->stop();
	sleep(1);

	base->forward();
	base->turnAngle(90,3);

	lift->forward();
	lift->setSpeed(5);
	usleep(100000);
	lift->stop();

	claw.write(0.7);
	sleep(1);

	lift->backward();
	lift->setSpeed(5);
	usleep(100000);
	lift->stop();

	claw.release();
	sleep(1);
	return 0;
}
