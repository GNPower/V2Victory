#include <cmath>

#include "vector.h"


Vector::Vector(Scalar x, Scalar y):
    x(x),
    y(y)
{}


Scalar Vector::distance(Vector v1, Vector v2) {
    Scalar dx = v2.x - v1.x;
    Scalar dy = v2.y - v1.y;
    return std::hypot(dx, dy);
}
