#include "motor_drive.h"

int GPIO_init(int duty_a, int duty_b){
	if ((-1 == setup_gpio(LFORWARD, OUT))|
		(-1 == setup_gpio(LBACKWARD, OUT))|
		(-1 == setup_gpio(RFORWARD, OUT))|
		(-1 == setup_gpio(STOP, IN))|
		(-1 == setup_gpio(LENCODER, IN))|
		(-1 == setup_gpio(RENCODER, IN))|
		(-1 == setup_gpio(RBACKWARD, OUT)))
		return 2;

	if (-1 == GPIOEdge(LENCODER, FALLING))
		return 2;
	if (-1 == GPIOEdge(RENCODER, FALLING))
		return 2;

	//PWM Setup
	if ((-1 == setup_pwm(ENA, duty_a))|
		(-1 == setup_pwm(ENB, duty_b)))
		return 2;

	return 1;
}


int GPIO_Close(){
	if ((-1 == PWMEnable(ENA, 0))|(-1 == PWMEnable(ENB, 0)))
		return 2;

	if ((-1 == PWMUnexport(ENA))|
		(-1 == PWMUnexport(ENB))|
		(-1 == GPIOUnexport(STOP))|
		(-1 == GPIOUnexport(LENCODER))|
		(-1 == GPIOUnexport(RENCODER))|
		(-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 2;

	return 1;
} 


int GPIOExport(int pin){
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


int PWMExport(int pin){
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


int GPIOUnexport(int pin){
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


int PWMUnexport(int pin){
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

int PWMPeriod(int pin){
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


int PWMDuty(int pin, int duty){
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


int PWMEnable(int pin, int enable){
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

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", enable);

	if (-1 == write(fd, buffer, bytes_written)){
		fprintf(stderr, "Failed to set enable for %d \n", pin);
		printf("ERROR: %d \n", errno);
	}

	close(fd);
	return 0;
	}


int GPIODirection(int pin, int dir){
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


int GPIOEdge(int pin, int edge_sel){
	static const char rising[] = "rising";
	static const char falling[] = "falling";
	
	char path[PATH_MAX];
	int fd;

	snprintf(path, PATH_MAX, "/sys/class/gpio/gpio%d/edge", pin);
	fd = open(path, O_WRONLY);

	if (-1 == fd){
		fprintf(stderr, "Failed to open edge for writing \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/edge \n", pin);
		return -1;
	}

	if (-1 == write(fd, ((edge_sel == RISING) ? rising : falling) , sizeof(((edge_sel == RISING) ? rising : falling)))){
		fprintf(stderr, "Failed to set edge for %d \n", pin);
		printf("ERROR: %d \n", errno);
	}


	close(fd);
	return 0;

	}


int GPIOWrite(int pin, int value){
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


int GPIORead(int pin){

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


int set_forward(){
	if ((-1 == GPIOWrite(LFORWARD, 1))|
		(-1 == GPIOWrite(LBACKWARD, 0))|
		(-1 == GPIOWrite(RBACKWARD, 0))|
		(-1 == GPIOWrite(RFORWARD, 1)))
		return -1;
	else
		return 0;
	}


int set_backward(){
	if ((-1 == GPIOWrite(LFORWARD, 0))|
		(-1 == GPIOWrite(LBACKWARD, 1))|
		(-1 == GPIOWrite(RBACKWARD, 1))|
		(-1 == GPIOWrite(RFORWARD, 0)))
		return -1;
	else
		return 0;
	}


int set_left(){
	if ((-1 == GPIOWrite(LFORWARD, 1))|
		(-1 == GPIOWrite(LBACKWARD, 0))|
		(-1 == GPIOWrite(RBACKWARD, 1))|
		(-1 == GPIOWrite(RFORWARD, 0)))
		return -1;
	else
		return 0;
	}


int set_right(){
	if ((-1 == GPIOWrite(LFORWARD, 0))|
		(-1 == GPIOWrite(LBACKWARD, 1))|
		(-1 == GPIOWrite(RBACKWARD, 0))|
		(-1 == GPIOWrite(RFORWARD, 1)))
		return -1;
	else
		return 0;
	}


int setup_gpio(int pin, int direction){	
	if (GPIOExport(pin) == -1)
		return -1;
	if (GPIODirection(pin, direction) == -1)
		return -1;
	return 0;
	}


int setup_pwm(int pin, int duty){
	if (PWMExport(pin) == -1)
		return -1;
	if (PWMPeriod(pin) == -1)
		return -1;
	if (PWMDuty(pin, duty) == -1)
		return -1;
	if (PWMEnable(pin, 1) == -1)
		return -1;
	return 0;
	}