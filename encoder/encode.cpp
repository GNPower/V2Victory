#include "encode.h"


volatile int l_encoder;
volatile int r_encoder;

void init_encoders(pthread_t* l_encoder, pthread_t* r_encoder){
	pthread_create(l_encoder, NULL, poll_l_encoder, NULL);
	pthread_create(r_encoder, NULL, poll_r_encoder, NULL);
}

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



int get_encoder_value(int encoder){
	int value;
	if (encoder == LENCODER) {
		value = l_encoder;
		l_encoder = 0;
		return value;}
	else if (encoder == RENCODER) {
		value = r_encoder;
		r_encoder = 0;
		return value;}
	else return -1;
	}


void get_x_distance_traveled(Vehicle_Data *ego, float* distance){
	int l_encoder, r_encoder, encoder_average;
	float abs_traveled, x_traveled;

	l_encoder = get_encoder_value(LENCODER);
	r_encoder = get_encoder_value(RENCODER);
	encoder_average = (l_encoder+r_encoder)/2;
	float cycles = encoder_average/20.0;

	float circumference = 2.0*RADIUS*PI;
	abs_traveled = circumference*cycles;
	x_traveled = abs_traveled*cos(ego->heading);

	*distance = x_traveled;
}

void get_y_distance_traveled(Vehicle_Data *ego, float* distance){
	int l_encoder, r_encoder, encoder_average;
	float abs_traveled, y_traveled;

	l_encoder = get_encoder_value(LENCODER);
	r_encoder = get_encoder_value(RENCODER);
	encoder_average = (l_encoder+r_encoder)/2;
	float cycles = encoder_average/20.0;

	float circumference = 2.0*RADIUS*PI;
	abs_traveled = circumference*cycles;
	y_traveled = abs_traveled*sin(ego->heading);

	*distance = y_traveled;
}
