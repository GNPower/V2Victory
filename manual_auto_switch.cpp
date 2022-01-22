#include <iostream>
#include <cstdio>

// Temporary testing
#define DUMMY_BRAKE_REQ 0
#define DUMMY_ACCEL_REQ 0
#define DUMMY_SYSTEM_REQ 100
// Temporary testing

using namespace std;

int driver_accel_req(){
    // driver_accel_req returns an INTEGER between -100 to 100. -100 is full brake, 100 is full throttle and 0 is neither throttle nor braking.
    int brakeReq = 0;
    int accelReq = 0;
    int finalReq = 0;

    // This is the hardware hiding section
    // brakeReq = mapping(readPstnSensor(PinA));   // To be implemented when we have the hardware. Map sensor to a percentage.
    // accelReq = mapping(readPstnSensor(PinB));   // To be implemented when we have the hardware. Map sensor to a percentage.

    // Temporary testing
    brakeReq = DUMMY_BRAKE_REQ;
    accelReq = DUMMY_ACCEL_REQ;
    // Temporary testing

    // Priority encoder: brake request takes precendence over acceleration request. IE: If there is both a brake request and an acceleration request, take the brake request.
    if(brakeReq > 0){
        finalReq = -brakeReq;   // Return brake request as a -ve number.
    }else if(accelReq > 0){
        finalReq = accelReq;    // Return acceleration request as a +ve number.
    }else{
        finalReq = 0;           // Set request to 0 if there is no brake or throttle request.
    }

    return finalReq;
}

int get_accel_req(int system_req){
    int driver_req = driver_accel_req();
    int finalReq = 0;

    // Priority encoder: brake request takes precendence over acceleration request. IE: If there is both a brake request and an acceleration request, take the brake request.
    if(driver_req < 0){
        // Evaluate brake request if driver is requesting braking
        if(driver_req < system_req){
            finalReq = driver_req;
        }else{
            finalReq = system_req;
        }
    }else if(driver_req > 0){
        // Evaluate acceleration request if the driver is not requesting braking and is requesting acceleration
        if(driver_req > system_req){
            finalReq = driver_req;
        }else{
            finalReq = system_req;
        }
    }else{
        // Evaluate request if the driver is not requesting braking or acceleration
        finalReq = system_req;
    }

    return finalReq;
}

int main(){
    // Dummy function.
    // Not be used after final integration.

    // int system_req = get_system_accel_req();
    // get_accel_req(system_req);

    // Temporary testing
    int system_req = DUMMY_SYSTEM_REQ;
    int final_accel_req = get_accel_req(system_req);
    printf("Final_accel_req = %d\n", final_accel_req);
    // Temporary testing

    return 0;
}