#include "motor_drive.h"
#include "localize.h"
#include <poll.h>
#include <pthread.h>

void* counter(void*){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];
	int count = 0;


	sprintf(str, "/sys/class/gpio/gpio%d/value", LENCODER);
	
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", LENCODER);
		return NULL;
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

}


int main(){
	pthread_t tid;


	if (-1 == GPIOExport(STOP))
		return 1;

	if (-1 == GPIODirection(STOP, IN))
	return 2;


	if (-1 == GPIOExport(LENCODER))
		return 1;


	if (-1 == GPIODirection(LENCODER, IN))
		return 2;

	if (-1 == GPIOEdge(LENCODER, RISING))
		return 2;


	pthread_create(&tid, NULL, counter, NULL);

	while(1){
		if(GPIORead(STOP)) break;
	}


	pthread_join(tid, NULL);

	if (-1 == GPIOUnexport(LENCODER))
		return 1;

	printf("No problems today \n");
	return 0;

}