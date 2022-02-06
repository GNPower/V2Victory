#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include<sys/ipc.h>
#include<sys/shm.h>

#include "motor_drive.h"
#include "localize.h"
#include <poll.h>
#include <pthread.h>
#include <signal.h>

#define BUF_SIZE 1024
#define SHM_KEY 0x1234

volatile int l_encoder;
volatile int r_encoder;

void* l_encode(void*){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];


	sprintf(str, "/sys/class/gpio/gpio%d/value", LENCODER);
	
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", LENCODER);
		
	}

	while(1){


		pfd.fd = fd;
		pfd.events = POLLPRI;

		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		poll(&pfd, 1, -1);


		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		l_encoder ++;
		

	}
	return NULL;

}

void* r_encode(void*){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];


	sprintf(str, "/sys/class/gpio/gpio%d/value", RENCODER);
	
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", RENCODER);
		
	}

	while(1){


		pfd.fd = fd;
		pfd.events = POLLPRI;

		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		poll(&pfd, 1, -1);


		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		r_encoder ++;
		

	}
	return NULL;

}


int main(){
	pthread_t left_tid, right_tid;


	if (-1 == GPIOExport(STOP))
		return 1;

	if (-1 == GPIODirection(STOP, IN))
	return 2;


	if (-1 == GPIOExport(LENCODER))
		return 1;


	if (-1 == GPIODirection(LENCODER, IN))
		return 2;

	if (-1 == GPIOEdge(LENCODER, FALLING))
		return 2;



	if (-1 == GPIOExport(RENCODER))
		return 1;


	if (-1 == GPIODirection(RENCODER, IN))
		return 2;

	if (-1 == GPIOEdge(RENCODER, FALLING))
		return 2;


	pthread_create(&left_tid, NULL, l_encode, NULL);
	pthread_create(&right_tid, NULL, r_encode, NULL);

	while(1){
		printf("LEFT: %d \t RIGHT: %d \n", l_encoder, r_encoder);
		if(GPIORead(STOP)) break;
		usleep(40000);
	}

	if (-1 == GPIOUnexport(LENCODER))
		return 1;
	if (-1 == GPIOUnexport(STOP))
		return 1;

	printf("No problems today \n");


	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);


	return 0;

}