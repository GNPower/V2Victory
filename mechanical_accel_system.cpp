#include <cstdio>
#include <math.h>

#define MAS_SCALE 100

double convert_percent_to_dutyCycle(double percent){
    return (int) percent/MAS_SCALE;
}

int main(){
    return 0;
}