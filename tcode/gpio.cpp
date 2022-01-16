#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define POUT 4
#define PIN 24
#define PIN2 23
#define PPWM 5

#define VALUE_MAX 30
#define DIRECTION_MAX 35

static int GPIOExport(int pin){
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (-1 == fd){
		fprintf(stderr, "Failed to open export for writing \n");
		printf("ERROR: %d \n", errno);
		return -1;
	}


	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);

	return 0;
}


static int GPIOUnexport(int pin){
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;

	int fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd){
		fprintf(stderr, "Failed to open unexport for writing \n");
		printf("ERROR: %d \n", errno);
		return -1;
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return 0;
}


static int GPIODirection(int pin, int dir){
	static const char s_directions_str[] = "in\0out";
	
	char path[DIRECTION_MAX];
	int fd;

	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open direction for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/direction \n", pin);
		return -1;
	}

	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2: 3)){
		fprintf(stderr, "Failed to set direction");
		printf("ERROR: %d \n", errno);
	}


	close(fd);
	return 0;

}



static int GPIOWrite(int pin, int value){
	static const char s_values_str[] = "01";

	char path[VALUE_MAX];
	int fd;
	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);

		
	if (-1 == fd){
		fprintf(stderr, "Failed to open gpio for writing \n");
		printf("ERROR: %d \n", errno);
		return -1;
	}

	if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)){
		fprintf(stderr, "Failed to write GPIO \n");
		printf("ERROR: %d \n", errno);
		return -1;
	}

	if (close(fd)== -1)
		printf("Close failed \n");
	return 0;

}


static int GPIORead(int pin){

	char path[VALUE_MAX];
	char value_str[3];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);

		
	if (-1 == fd){
		fprintf(stderr, "Failed to open gpio for reading \n");
		printf("ERROR: %d \n", errno);
		return -1;
	}

	if (-1 == read(fd, value_str, 3)){
		fprintf(stderr, "Failed to read gpio\n");
		printf("ERROR: %d \n", errno);
		return -1;
	}
	close(fd);
	return (atoi(value_str));
}




int PWM(int pin, int duty){
        
	if (-1 == GPIOWrite(PPWM, 1))
		return 3;

	usleep(duty*100);


	if (-1 == GPIOWrite(PPWM, 0))
		return 3;

	usleep((200-duty)*100);
	return 0;
}


int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	bool toggle = 1;
	int input1, input2;
	int duty = 13;
	//Enable GPIO
	if (-1 == GPIOExport(POUT))
		return 1;

	if (-1 == GPIOExport(PIN))
		return 1;


	if (-1 == GPIOExport(PIN2))
		return 1;

	if (-1 == GPIOExport(PPWM))
		return 1;

	//Set Direction
	if (-1 == GPIODirection(POUT, OUT))
		return 2;

	if (-1 == GPIODirection(PIN, IN))
		return 2;
	

	if (-1 == GPIODirection(PIN2, IN))
		return 2;
	
	if (-1 == GPIODirection(PPWM, OUT))
		return 2;
	
	//Main loop
	while(1){
	if (-1 == GPIOWrite(POUT, toggle))
		return 3;

	input1 = GPIORead(PIN);
	input2 = GPIORead(PIN2);
	printf("%d in GPIO %d and %d on GPIO %d, DUTY: %d\n", input1, PIN, input2, PIN2, duty);
	
	if ((duty > 1) && (input1 == 1))
		duty--;
	if ((duty<25) && (input2 == 1))
		duty++;
	
	PWM(PPWM, duty);
	
	//usleep(50*1000);
	//toggle = !toggle;
	
	}

	//Disable GPIO
	if (-1 == GPIOUnexport(POUT))
		return 1;

	if (-1 == GPIOUnexport(PIN))
		return 1;
	
	
	if (-1 == GPIOUnexport(PIN2))
		return 1;
	
	if (-1 == GPIOUnexport(PPWM))
		return 1;
	
	
	return 0;
}

