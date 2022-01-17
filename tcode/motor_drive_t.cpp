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

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define ENA 23     //PWM Controls Speed Right 
#define ENB 24	   //PWM Controls Speed 
#define RFORWARD 5      
#define RBACKWARD 6      
#define LBACKWARD 13    
#define LFORWARD 19     

#define STOP 26  

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




int speedSet(int right_duty /*, int left_duty*/){
    //int l_count = left_duty;
    int r_count = right_duty;
    //bool l_high = 1;
    bool r_high = 1;

	if (-1 == GPIOWrite(ENB, 1))
			return 3;

    for (int i = 0; i < 200; i++){
    	//l_count--;
    	r_count--;

    	usleep(100);

    	/*
    	if ((l_count < 0)&(l_high)){
			if (-1 == GPIOWrite(ENA, 0))
				return 3;    
			l_high = 0;		
    		}
		*/

    	if ((r_count < 0)&(r_high)){
			if (-1 == GPIOWrite(ENB, 0))
				return 3;
			r_high = 0;    		
    		}
    	}


    return 0;
 	}


int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	int duty = atoi(argv[1]);
	int time_ms = atoi(argv[2]);
	int stop;
	char* direct = argv[3];

	//Enable GPIOs
	if ((-1 == GPIOExport(ENA))|
		(-1 == GPIOExport(ENB))|
		(-1 == GPIOExport(LFORWARD))|
		(-1 == GPIOExport(LBACKWARD))|
		(-1 == GPIOExport(RFORWARD))|
		(-1 == GPIOExport(RBACKWARD)))
		return 1;


	if (-1 == GPIOExport(STOP))
		return 1;


	//Set Direction
	if ((-1 == GPIODirection(ENA, OUT))|
		(-1 == GPIODirection(ENB, OUT))|
		(-1 == GPIODirection(LFORWARD, OUT))|
		(-1 == GPIODirection(LBACKWARD, OUT))|
		(-1 == GPIODirection(RFORWARD, OUT))|
		(-1 == GPIODirection(RBACKWARD, OUT)))
		return 2;

	if (-1 == GPIODirection(STOP, IN))
		return 2;

	
	//Main loop
	while(time_ms > 0){
		
		//Choose moter direction
		switch(direct){
			case "FORWARD":
				if ((-1 == GPIOWrite(LFORWARD, 1))|
					(-1 == GPIOWrite(LBACKWARD, 0))|
					(-1 == GPIOWrite(RBACKWARD, 0))|
					(-1 == GPIOWrite(RFORWARD, 1)))
					return 3;
				break;

			case "BACKWARD":
				if ((-1 == GPIOWrite(LFORWARD, 0))|
					(-1 == GPIOWrite(LBACKWARD, 1))|
					(-1 == GPIOWrite(RBACKWARD, 1))|
					(-1 == GPIOWrite(RFORWARD, 0)))
					return 3;			
				break;


			case "RIGHT":
				if ((-1 == GPIOWrite(LFORWARD, 1))|
					(-1 == GPIOWrite(LBACKWARD, 0))|
					(-1 == GPIOWrite(RBACKWARD, 1))|
					(-1 == GPIOWrite(RFORWARD, 0)))
					return 3;			
				break;

			case "LEFT":
				if ((-1 == GPIOWrite(LFORWARD, 0))|
					(-1 == GPIOWrite(LBACKWARD, 1))|
					(-1 == GPIOWrite(RBACKWARD, 0))|
					(-1 == GPIOWrite(RFORWARD, 1)))
					return 3;			
				break;

			default:
				printf("Problem with instructions\n Please include command line args for: \n int speed, int time and str direct {FORWARD, BACKWARD, RIGHT, LEFT}");
				break;
		}

	if(GPIORead(STOP)) break;

	speedSet(duty);
	time_ms = time_ms - 20;
	//usleep(50*1000);
	//toggle = !toggle;
	
	}

	//Disable GPIO
	if ((-1 == GPIOUnexport(ENA))|
		(-1 == GPIOUnexport(ENB))|
		(-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 1;
	
	
	return 0;
}
