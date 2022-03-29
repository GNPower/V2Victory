// LightLogic v0.7

#include <iostream>
#include <unistd.h>
#include <string>
//#include <ctime>
#include <chrono>
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

// This is a dummy class simulates the data from GIS databases
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

    private:
    // Identifier
    int k_id;

    // Dimensional and locational constants
    int k_entry_directions[4] = {0, 90, 180, 270}; // default perpendicular
    int k_entry_group[4];
    double k_intersection_length[4];
    int k_intersection_type;
    double k_intersection_position[2];
    
    // Signaling parameter constants
    int p_state_cycle_0[3] = {SOLID_RED, SOLID_RED, SOLID_GREEN, SOLID_YELLOW}; 
    int p_state_cycle_1[3] = {SOLID_GREEN, SOLID_YELLOW, SOLID_RED, SOLID_RED};

    int p_pedestrain_countdown = 20;
    double p_max_time_to_switch[4] = {20, 5, 20, 5};

    // Monitored variables


    // Controlled variables
    int c_intersection_state[4];
    int c_intersection_next_state[4];
    double c_intersection_time_to_switch;
    double c_intersection_pedestrain_countdown;
    unsigned int c_intersection_next_vehicle_id[4];

    // Internal variables
    int roadCount = 4;
    int lightGroupCount = 2;
    int cycleCount = 4;
    int lightGroup[4] = {0,1,0,1};

    int currentCycle = 0;
    time_t updateTimeStamp;

    public:
    Intersection(int setRoadCount) {
        //
    }

    void initState() {  // Creates a 5 seconds initializing period with all lights in red
        currentCycle = -1;
        c_intersection_time_to_switch = 5;
        c_intersection_pedestrain_countdown = 0.0;

        for (int ctr=0; ctr<roadCount; ctr++) {
            c_intersection_state[ctr] = SOLID_RED;

            if (lightGroup[ctr] == 0) {
                c_intersection_next_state[ctr] = p_state_cycle_0[currentCycle + 1];
            } else if (lightGroup[ctr] == 1) {
                c_intersection_next_state[ctr] = p_state_cycle_1[currentCycle + 1];
            }
        }

        updateTimeStamp = time();
    }

    void changeState() {
        c_intersection_time_to_switch = p_max_time_to_switch[currentCycle];
        c_intersection_pedestrain_countdown = p_pedestrain_countdown;

        for (int ctr=0; ctr<roadCount; ctr++) {
            c_intersection_state[ctr] = c_intersection_next_state[ctr];

            if (lightGroup[ctr] == 0) {
                c_intersection_next_state[ctr] = p_state_cycle_0[currentCycle + 1];
            } else if (lightGroup[ctr] == 1) {
                c_intersection_next_state[ctr] = p_state_cycle_1[currentCycle + 1];
            }
        }

        currentCycle++;
        if (currentCycle >= cycleCount) {
            currentCycle = 0;
        }
    }

    void run() {
        while (true) {

            time_t current = time();
            duration = current - updateTimeStamp;

            c_intersection_time_to_switch = p_max_time_to_switch[currentCycle] - duration;

            if (c_intersection_time_to_switch <= 0) {
                self.changeState();
            }
        }
    }

    int getIntersectionState(int num) {
        int returnVar = 0;

        if (num < roadCount) {
            returnVar = c_intersection_state[num];
        } else {
            returnVar = -1;
        }
        return returnVar;
    }

    int getIntersectionNextState(int num) {
        int returnVar = 0;

        if (num < roadCount) {
            returnVar = c_intersection_next_state[num];
        } else {
            returnVar = -1;
        }
        return returnVar;
    }

    double getIntersectionTime(int num) {
        double returnVar = 0;

        if (num < roadCount) {
            returnVar = c_intersection_time_to_switch[num];
        } else {
            returnVar = -1;
        }
        return returnVar;
    }

    double getIntersectionPedTime(int num) {
        double returnVar = 0;

        if (num < roadCount) {
            returnVar = c_intersection_pedestrain_countdown[num]
        } else {
            returnVar = -1;
        }
        return returnVar;
    }

    
};

Intersection intersection(4);

int intersectionSetup() {

    intersection.initState();

    LightControl.setup();
}

int intersectionController() {
    time_t 


}


int main() {


    while (true) {
        inTimer = true;
        timeCtr = 0;

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

