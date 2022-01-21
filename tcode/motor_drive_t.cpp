//Motor Driver for Rapberry Pi - LM298N
//Sam Baker 01/2022
//
//
//Accepts command line arg int from 10-25 to set speed


#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define ENA 0      //GPIO PIN 18 - PWM Controls Speed Right -- Currently disabled and both motors are linked on ENB 
#define ENB 1	   //PWM Controls Speed 
#define RFORWARD 5             
#define RBACKWARD 6         
#define LBACKWARD 13    
#define LFORWARD 19     

#define STOP 26  
#define PWM_PERIOD 20000000

#define VALUE_MAX 30
#define PATH_MAX 46

#define BUFFER_MAX 3
#define BUFFER_MAX2 10

static int GPIOExport(int pin){
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	printf("Exporting %d \n", pin);
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


static int PWMExport(int pin){
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	printf("Exporting %d \n", pin);
	fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
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

static int PWMPeriod(int pin){
	int fd;
	int period_ns = PWM_PERIOD;
	char path[PATH_MAX];

	char buffer[BUFFER_MAX2];
	ssize_t bytes_written;

	snprintf(path, PATH_MAX, "/sys/class/pwm/pwmchip0/pwm%d/period", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open period for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/pwm/pwmchip0/pwm%d/period \n", pin);
		return -1;
		}

	bytes_written = snprintf(buffer, BUFFER_MAX2, "%d", period_ns);

	if (-1 == write(fd, buffer, bytes_written)){
		fprintf(stderr, "Failed to set period for %d \n", pin);
		printf("ERROR: %d \n", errno);
	}

	close(fd);
	return 0;
	}


static int PWMDuty(int pin, int duty){
	int fd;
	int period_ns = PWM_PERIOD;
	char path[PATH_MAX];

	char buffer[BUFFER_MAX2];
	ssize_t bytes_written;

	int duty_time = PWM_PERIOD*duty/100;
	//printf("Duty Time:%d", duty_time);

	snprintf(path, PATH_MAX, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open duty for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/pwm/pwmchip0/pwm%d/duty_cycle \n", pin);
		return -1;
		}

	bytes_written = snprintf(buffer, BUFFER_MAX2, "%d", duty_time);

	if (-1 == write(fd, buffer, bytes_written)){
		fprintf(stderr, "Failed to set duty for %d at %d \n", pin, duty);
		printf("ERROR: %d \n", errno);
	}

	close(fd);
	return 0;
	}


static int PWMEnable(int pin, int enable){
	int fd;
	char path[PATH_MAX];

	char buffer[1];
	ssize_t bytes_written;


	snprintf(path, PATH_MAX, "/sys/class/pwm/pwmchip0/pwm%d/enable", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open duty for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/pwm/pwmchip0/pwm%d/enable \n", pin);
		return -1;
		}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", 1);

	if (-1 == write(fd, buffer, bytes_written)){
		fprintf(stderr, "Failed to set enable for %d \n", pin);
		printf("ERROR: %d \n", errno);
	}

	close(fd);
	return 0;
	}


static int GPIOUnexport(int pin){
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;

	int fd;
	printf("Unexporting %d \n", pin);
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


static int PWMUnexport(int pin){
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;

	int fd;
	printf("Unexporting %d \n", pin);
	fd = open("/sys/class/pwm/pwmchip0/unexport", O_WRONLY);
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
	
	char path[PATH_MAX];
	int fd;

	snprintf(path, PATH_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open direction for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/direction \n", pin);
		return -1;
	}

	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2: 3)){
		fprintf(stderr, "Failed to set direction for %d \n", pin);
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


int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	int duty = atoi(argv[1]);
	int time_ms = atoi(argv[2]);
	int stop;
	char* direct = argv[3];

	//Enable GPIOs
	if ((-1 == PWMExport(ENA))|
		(-1 == PWMExport(ENB))|
		(-1 == GPIOExport(LFORWARD))|
		(-1 == GPIOExport(LBACKWARD))|
		(-1 == GPIOExport(RFORWARD))|
		(-1 == GPIOExport(RBACKWARD)))
		return 1;


	if (-1 == GPIOExport(STOP))
		return 1;


	//Set Direction
	if (/*(-1 == GPIODirection(ENA, OUT))|
		(-1 == GPIODirection(ENB, OUT))|*/
		(-1 == GPIODirection(LFORWARD, OUT))|
		(-1 == GPIODirection(LBACKWARD, OUT))|
		(-1 == GPIODirection(RFORWARD, OUT))|
		(-1 == GPIODirection(RBACKWARD, OUT)))
		return 2;

	if (-1 == GPIODirection(STOP, IN))
		return 2;


	//PWM Setup
	if ((-1 == PWMPeriod(ENA))|
		(-1 == PWMPeriod(ENB)))
		return 2;

	if ((-1 == PWMDuty(ENA, duty))|
		(-1 == PWMDuty(ENB, duty)))
		return 2;

	if ((-1 == PWMEnable(ENA, 1))|
		(-1 == PWMEnable(ENB, 1)))
		return 2;


	//Main loop
	while(time_ms > 0){
		
		//Choose moter direction
		if (!strcmp(direct, "FORWARD")){
			if ((-1 == GPIOWrite(LFORWARD, 1))|
				(-1 == GPIOWrite(LBACKWARD, 0))|
				(-1 == GPIOWrite(RBACKWARD, 0))|
				(-1 == GPIOWrite(RFORWARD, 1)))
				return 3;
			//else
				//printf("Moving Forward \n");
				
		}
		else if (!strcmp(direct, "BACKWARD")){
			if ((-1 == GPIOWrite(LFORWARD, 0))|
				(-1 == GPIOWrite(LBACKWARD, 1))|
				(-1 == GPIOWrite(RBACKWARD, 1))|
				(-1 == GPIOWrite(RFORWARD, 0)))
				return 3;
			//else
				//printf("Moving Backward \n");
		}
		else if (!strcmp(direct, "RIGHT")){
			if ((-1 == GPIOWrite(LFORWARD, 1))|
				(-1 == GPIOWrite(LBACKWARD, 0))|
				(-1 == GPIOWrite(RBACKWARD, 1))|
				(-1 == GPIOWrite(RFORWARD, 0)))
				return 3;
			//else
				//printf("Turn Right \n");
		}
		else if (!strcmp(direct, "LEFT")){
			if ((-1 == GPIOWrite(LFORWARD, 0))|
				(-1 == GPIOWrite(LBACKWARD, 1))|
				(-1 == GPIOWrite(RBACKWARD, 0))|
				(-1 == GPIOWrite(RFORWARD, 1)))
				return 3;
			//else
				//printf("Turn Left \n");
		}
		else{
			printf("Problem with instructions\n Please include command line args for: \n int speed, int time and str direct {FORWARD, BACKWARD, RIGHT, LEFT}");
		}

	if(GPIORead(STOP)) break;

	//speedSet(duty);
	time_ms = time_ms - 20;
	usleep(20*1000);
	//toggle = !toggle;
	
	}

	//Disable GPIO
	if ((-1 == PWMEnable(ENA, 0))|
		(-1 == PWMEnable(ENB, 0)))


	if ((-1 == PWMUnexport(ENA))|
		(-1 == PWMUnexport(ENB))|
		(-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 1;
	
	
	return 0;
}
