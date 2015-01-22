#include <iostream>
#include "shortIR.cpp"
#include "motion.cpp"
int running=1;

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = 0;
    }
};
int main() {
	signal(SIGINT, sig_handler);
	IR ir(3);
	float distance = ir.getDistance();
	std::cout<<"distance: "<<distance<<std::endl;
   	return 0;
}
