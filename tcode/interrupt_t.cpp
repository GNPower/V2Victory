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
#include "encode.h"

#include <poll.h>
#include <pthread.h>
#include <signal.h>

#define BUF_SIZE 1024
#define SHM_KEY 0x1234


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


	pthread_create(&left_tid, NULL, poll_l_encoder, NULL);
	pthread_create(&right_tid, NULL, poll_r_encoder, NULL);

	while(1){
		printf("LEFT: %d \t RIGHT: %d \n", l_encoder, r_encoder);
		if(GPIORead(STOP)) break;
		usleep(40000);
	}

	if (-1 == GPIOUnexport(LENCODER))
		return 1;
	if (-1 == GPIOUnexport(RENCODER))
		return 1;
	if (-1 == GPIOUnexport(STOP))
		return 1;

	printf("No problems today \n");


	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);


	return 0;

}