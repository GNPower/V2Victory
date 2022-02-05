#include "motor_drive.h"
#include <poll.h>

int main(){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];
	int count = 0;

	if (-1 == GPIOExport(STOP))
		return 1;


	if (-1 == GPIODirection(STOP, IN))
		return 2;

	if (-1 == GPIOEdge(STOP, RISING))
		return 2;


	sprintf(str, "/sys/class/gpio/gpio%d/value", STOP);
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", STOP);
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

		if (count = 10)
			break;
	}


	if (-1 == GPIOUnexport(STOP))
		return 1;

	printf("No problems today \n");
	return 0;

}