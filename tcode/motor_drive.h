//Motor Driver Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#ifndef MDRIVE
#define MDRIVE

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define RISING 0
#define FALLING 1

#define ENA 0      //GPIO PIN 18 - PWM Controls Speed Right 
#define ENB 1	   //GPIO PIN 19 - PWM Controls Speed 
#define RFORWARD 5             
#define RBACKWARD 6         
#define LBACKWARD 13    
#define LFORWARD 12    

#define STOP 26  

#define PWM_PERIOD 20000000

#define VALUE_MAX 30
#define PATH_MAX 46

#define BUFFER_MAX 3
#define BUFFER_MAX2 10


int GPIOExport(int pin);
int PWMExport(int pin);
int GPIOUnexport(int pin);
int PWMUnexport(int pin);

int PWMPeriod(int pin);
int PWMDuty(int pin, int duty);
int PWMEnable(int pin, int enable);

int GPIODirection(int pin, int dir);
int GPIOEdge(int pin, int edge_sel);
int GPIOWrite(int pin, int value);
int GPIORead(int pin);

int set_forward();
int set_backward();
int set_left();
int set_right();

#endif 
