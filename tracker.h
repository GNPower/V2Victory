#ifndef TRACKER_H
#define TRACKER_H

#include <vector>

#include "car.h"


typedef std::vector<Car>::const_iterator CarIterator;


class CarTracker {
    std::vector<Car> cars;
public:
    // TODO: May want a different way to update car (e.g., just updating
    // certain fields)
    void update(Car car);
    /*
     * Returns nullptr if the car is not found
     */
    const Car * get_car(CarID id);
    CarIterator begin();
    CarIterator end();
};

#endif
