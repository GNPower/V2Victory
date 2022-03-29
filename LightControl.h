#include <iostream>
#include <wiringPi.h>

// GPIO Definition
#define GND 26
#define GREEN 27
#define YELLOW 28
#define RED 29

#define FLASH_RED 0
#define SOLID_RED 10
#define SOLID_YELLOW 20
#define SOLID_GREEN 30
#define SOLID_GREEN_STRAIGHT 31
#define PROTECTED_LEFT 32
#define PROTECTED_RIGHT 33
#define PEDESTRIAN_GREEN 100
#define CYCLIST_GREEN 110

struct {
    // Required setup
    int setup() {
        int returnVar = 0;

        wiringPiSetup();
        pinMode(GREEN,OUTPUT);
        pinMode(YELLOW,OUTPUT);
        pinMode(RED,OUTPUT);

        return returnVar;
    }

    // GPIO returns to safe state
    int close() {
        int returnVar = 0;

        digitalWrite(GREEN,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(RED,LOW);

        pinMode(GREEN,INPUT);
        pinMode(YELLOW,INPUT);
        pinMode(RED,INPUT);

        return returnVar;
    }

    int change(toState) {
        int returnVar = 0;

        if (toState == SOLID_RED) {
            digitalWrite(RED,HIGH);
            digitalWrite(YELLOW,LOW);
            digitalWrite(GREEN,LOW);

        } else if (toState == SOLID_YELLOW) {
            digitalWrite(RED,LOW);
            digitalWrite(YELLOW,HIGH);
            digitalWrite(GREEN,LOW);

        } else if (toState == SOLID_GREEN) {
            digitalWrite(RED,LOW);
            digitalWrite(YELLOW,LOW);
            digitalWrite(GREEN,HIGH);
            
        } else {
            // Nothing
        }

        return returnVar;
    }
} LightControl;
