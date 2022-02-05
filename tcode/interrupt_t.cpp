#include "motor_drive.h"
#include <poll.h>

int main(){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];
	int count = 0;

	if (-1 == GPIOExport(LENCODER))
		return 1;


	if (-1 == GPIODirection(LENCODER, IN))
		return 2;

	if (-1 == GPIOEdge(LENCODER, RISING))
		return 2;


	sprintf(str, "/sys/class/gpio/gpio%d/value", LENCODER);
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", LENCODER);
		return -1;
	}
	while(1){
		pfd.fd = fd;
		pfd.events = POLLPRI;

		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		poll(&pfd, 1, -1);


		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		count ++;
		printf("COUNT: %d \n", count);

		if (count == 20)
			break;
	}


	if (-1 == GPIOUnexport(LENCODER))
		return 1;

	printf("No problems today \n");
	return 0;

}