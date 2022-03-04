#ifndef CAR_H
#define CAR_H

#include "vector.h"


typedef int CarID;
typedef Vector Location;


class Car {
public:
    CarID id;
    Location location;
    Car(CarID id, Location location);
};

#endif
