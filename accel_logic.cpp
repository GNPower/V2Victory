#include <iostream>

using namespace std;

#define TIMESTEP 0.1 //0.01s = 10ms
#define MAX_ACCEL 4.91 //4.91m/s&^2
#define MIN_YELLOW_TIME 3 //3s
#define INTERSECTION_SIZE 10 //10m
#define OPTIMAL_SPEED 10 //10m/s

double get_accel_request(double new_timestamp, double vehicle_speed, int cur_intersection_state, int next_intersection_state, double time_to_switch, double intersection_distance) {
    //Use static variables to have way to estimate at each time step incase no data received from intersection
    static double last_timestamp = -1;
    static int last_cur_state = 0;
    static int last_next_state = 1;
    static double last_time_to_switch = 0;
    static double last_intersection_distance = 0;

    if (new_timestamp != last_timestamp) {
        //New data received
        last_timestamp = new_timestamp;
        last_cur_state = cur_intersection_state;
        last_next_state = next_intersection_state;
        last_time_to_switch = time_to_switch;
        last_intersection_distance = intersection_distance;
    }
    else {
        //No new data, update old data
        last_intersection_distance = last_intersection_distance - vehicle_speed*TIMESTEP;
        if ((last_time_to_switch - TIMESTEP) > 0) {
            last_time_to_switch -= TIMESTEP;
            //intersection states are same
        }
        else {
            last_time_to_switch = MIN_YELLOW_TIME; //idk what value here would be
            last_cur_state = (last_cur_state + 1)%2;
            last_next_state = (last_next_state + 1)%2;
        }
    }
    //now actual logic (basic version)
    double full_distance = last_intersection_distance + INTERSECTION_SIZE; //this is the full distance to clear the intersection
    double full_time = last_time_to_switch;
    if (last_cur_state == 1) {//Green, need to add yellow time to logic
        full_time += MIN_YELLOW_TIME; //add in yellow light time when determinining issue
    }

    double time_exit_intersection = full_distance/vehicle_speed; //time needed at current speed to exit intersection
    double time_enter_intersection = last_intersection_distance/vehicle_speed; //time needed at current speed to enter intersection

    double req_accel;
    /*Options:
        - if Green/Yellow: need to see if car will exit intersection before switch to Red
            - if it will make it, keep constant
            - if it wont, decelerate
        - if Red, need to see if car will enter intersection before switch to Green
            - if it will make it, decelerate
            - if it wont, keep constant
    */
   //currently assuming no cars
   //use vf^2 = vi^2 + 2*a*d where vf = 0 for decel commands
   //need to add something to check if car is going below driver's set speed
    if (last_cur_state == 3) {//Red Light
        if (time_to_switch > time_enter_intersection) {
            //need to decelerate since will enter intersection
            req_accel = -vehicle_speed*vehicle_speed/(2*last_intersection_distance);
        }
        else {
            req_accel = 0; //no need to decelerate since will not reach intersection by time it turns green
        }
    }
    else { //Green or Yellow Light
        if (time_to_switch > time_exit_intersection) {
            req_accel = 0; //no need to decelerate since will exit intersection before it turns red/changes light
        }
        else {
            //need to decelerate since won't make intersection
            req_accel = -vehicle_speed*vehicle_speed/(2*last_intersection_distance);
        }
    }

    return req_accel;
}


int main() {

    //Vehicle Data
    double vehicle_speed = 10; //m/s

    //Intersection Data
    int cur_intersection_state = 1; //Green
    int next_intersection_state = 2; //Yellow
    double time_to_switch = 5; //seconds
    double intersection_distance = 50; //meters
    
    double new_timestamp = 0;
    double req_accel;

    for (int i = 0 ; i < 50 ; i++) {
        req_accel = get_accel_request(new_timestamp, vehicle_speed, cur_intersection_state, next_intersection_state, time_to_switch, intersection_distance);
        printf("Time=%f\tDistance=%f\tSpeed=%f\tAccel=%f\n", time_to_switch, intersection_distance, vehicle_speed, req_accel);

        //Update variables
        new_timestamp += TIMESTEP;
        time_to_switch -= TIMESTEP;
        intersection_distance = intersection_distance - vehicle_speed*TIMESTEP; //this order might be backwards between updating distance vs updating speed
        vehicle_speed = vehicle_speed + TIMESTEP*req_accel;
    }
}