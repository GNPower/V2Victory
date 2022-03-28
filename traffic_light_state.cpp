// Traffic Light State Machine v 0.6.5

#include <iostream>
#include <unistd.h>
#include <string>
#include <ctime>
#include "wiringPi.h"
#include "lightControl.h"

#define FLASH_RED 0
#define SOLID_RED 10
#define SOLID_YELLOW 20
#define SOLID_GREEN 30
#define SOLID_GREEN_STRAIGHT 31
#define PROTECTED_LEFT 32
#define PROTECTED_RIGHT 33
#define PEDESTRIAN_GREEN 100
#define CYCLIST_GREEN 110

class RoadSegment{

    int id;
    int priority;
    int type;
    int speedLimit;

    public:
    RoadSegment(int setId, int setPriority, int setType, int setSpeedLimit) {
        id = setId;
        priority = setPriority;
        type = setType;
        speedLimit = setSpeedLimit;
    }
};

class Intersection{

    public:
    int roadCount = 4; // default 4 roadsegments
    int pedestrainCountdown = 20; // default 20s
    int length[4] = {20, 20, 20, 20}; // default 20m
    int directions[4] = {0, 90, 180, 270}; // default perpendicular
    int state[4] = {FLASH_RED, FLASH_RED, FLASH_RED, FLASH_RED};
    double duration = 20.0;

    public:
    Intersection(int setRoadCount) {
        roadCount = setRoadCount;
    }

    int changeState() {
        if ((state[0] == FLASH_RED && state[1] == FLASH_RED) && (state[2] == FLASH_RED && state[3] == FLASH_RED)) {
            state[0] = SOLID_RED;
            state[1] = SOLID_RED;
            state[2] = SOLID_RED;
            state[3] = SOLID_RED;

            duration = 5;

        } else if ((state[0] == SOLID_RED && state[1] == SOLID_RED) && (state[2] == SOLID_RED && state[3] == SOLID_RED)) {
            state[0] = SOLID_GREEN;
            state[1] = SOLID_RED;
            state[2] = SOLID_GREEN;
            state[3] = SOLID_RED;

            duration = 30;

        } else if ((state[0] == SOLID_GREEN && state[1] == SOLID_RED) && (state[2] == SOLID_GREEN && state[3] == SOLID_RED)) {
            state[0] = SOLID_YELLOW;
            state[1] = SOLID_RED;
            state[2] = SOLID_YELLOW;
            state[3] = SOLID_RED;

            duration = 2.5;

        } else if ((state[0] == SOLID_YELLOW && state[1] == SOLID_RED) && (state[2] == SOLID_YELLOW && state[3] == SOLID_RED)) {
            state[0] = SOLID_RED;
            state[1] = SOLID_GREEN;
            state[2] = SOLID_RED;
            state[3] = SOLID_GREEN;

            duration = 30;

        } else if ((state[0] == SOLID_RED && state[1] == SOLID_GREEN) && (state[2] == SOLID_RED && state[3] == SOLID_GREEN)) {
            state[0] = SOLID_RED;
            state[1] = SOLID_YELLOW;
            state[2] = SOLID_RED;
            state[3] = SOLID_YELLOW;

            duration = 2.5;

        } else if ((state[0] == SOLID_RED && state[1] == SOLID_YELLOW) && (state[2] == SOLID_RED && state[3] == SOLID_YELLOW)) {
            state[0] = SOLID_RED;
            state[1] = SOLID_RED;
            state[2] = SOLID_RED;
            state[3] = SOLID_RED;

            duration = 1;

        } else {
            state[0] = FLASH_RED;
            state[1] = FLASH_RED;
            state[2] = FLASH_RED;
            state[3] = FLASH_RED;

            duration = 600;
        }

        return 0;
    }
    
};



int main() {
    lightControlSetup();

    std::clock_t start;
    double timeCtr = 0;
    bool inTimer = true;
    Intersection intersection(4);

    while (true) {
        inTimer = true;
        timeCtr = 0;
        start = std::clock();

        while(inTimer) {
            std::cout<< "Time:" << timeCtr << " " << intersection.state[0] << " " << intersection.state[1] << " " << intersection.state[2] << " " << intersection.state[3] <<std::endl;
            usleep(1000000);
            timeCtr = timeCtr + 1; //(std::clock() - start)/(double) CLOCKS_PER_SEC;
            if (timeCtr > intersection.duration) {
                intersection.changeState();
                inTimer = false;
                lightControlChange(State[0]);
            }
        }
    }

    lightControlFree();

    exit(0);
}

