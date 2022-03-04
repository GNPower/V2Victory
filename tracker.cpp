#include "tracker.h"


void CarTracker::update(Car car) {
    size_t i = 0, number_of_cars = cars.size();
    for (; i < number_of_cars; i++) {
        if (cars[i].id == car.id) {
            cars[i] = car;
            break;
        }
    }
    if (i >= number_of_cars) {
        cars.push_back(car);
    }
}


const Car * CarTracker::get_car(CarID id) {
    for (size_t i = 0; i < cars.size(); i++) {
        if (cars[i].id == id) {
            return &cars[i];
        }
    }
    return nullptr;
}


CarIterator CarTracker::begin() {
    return cars.cbegin();
}


CarIterator CarTracker::end() {
    return cars.cend();
}
