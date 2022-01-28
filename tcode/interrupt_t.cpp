#include "motor_drive.h"

int main(){
	if (-1 == GPIOExport(STOP))
		return 1;


	if (-1 == GPIODirection(STOP, IN))
		return 2;

	if (-1 == GPIOEdge(STOP))
		return 2;

	//while(1){


	//}


	if (-1 == GPIOUnexport(STOP))
		return 1;

}