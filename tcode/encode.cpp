#include "encode.h"


volatile global int l_encoder;
volatile global int r_encoder;


void* poll_l_encoder(void*){
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


void* poll_r_encoder(void*){
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
	return NULL;}
