#include "motor_drive.h"

int main(){
		if ((-1 == PWMExport(ENA))|(-1 == PWMExport(ENB)));
			print("PWM Export Failed");

		if (-1 == GPIOExport(STOP))
			return 1;

		if (-1 == GPIODirection(STOP, IN))
		return 2;

	//PWM Setup
	if ((-1 == PWMPeriod(ENA))|(-1 == PWMPeriod(ENB)))
		return 2;

	if ((-1 == PWMDuty(ENA, 20))|(-1 == PWMDuty(ENB, 80)))
		return 2;

	if ((-1 == PWMEnable(ENA, 1))|(-1 == PWMEnable(ENB, 1)))
		return 2;

	while(1){
		if(GPIORead(STOP)) break;
	}

	if ((-1 == PWMEnable(ENA, 0))|(-1 == PWMEnable(ENB, 0)))
		return 2;


	if ((-1 == PWMUnexport(ENA))|(-1 == PWMUnexport(ENB)))
		return 1;

	return 0;
}