#include <iostream>
#include "shortIR.cpp"
int main() {
	for ( int i = 0;i<10;i++) {
		IR ir(0);
		float distance = ir.getDistance();
		std::cout<<"distance: "<<distance<<std::endl;
		sleep(1);
	}
   	return 0;
}
