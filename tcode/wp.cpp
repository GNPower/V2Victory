#include <wiringPi.h>

int main(void){
	wiringPiSetup();
	pinMode (18, OUTPUT);

	while(1){
		digitalWrite(18, HIGH); delay(500);
		digitalWrite(18, LOW); delay(500);
	
	}
	return 0;

}
