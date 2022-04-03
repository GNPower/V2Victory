#include <cmath>
#include <ostream>

#include "vector.hpp"


Vector::Vector(Scalar x, Scalar y):
    x(x),
    y(y)
{}


Scalar Vector::distance(const Vector v1, const Vector v2) {
    Scalar dx = v2.x - v1.x;
    Scalar dy = v2.y - v1.y;
    return std::hypot(dx, dy);
}


Vector Vector::from_polar_form(Scalar magnitude, Scalar angle) {
    Scalar x = magnitude * std::cos(angle);
    Scalar y = magnitude * std::sin(angle);
    return Vector (x, y);
}


Vector operator+(const Vector v1, const Vector v2) {
    return Vector (v1.x+v2.x, v1.y+v2.y);
}


Vector operator-(const Vector v1, const Vector v2) {
    return Vector (v1.x-v2.x, v1.y-v2.y);
}


std::ostream &operator<<(std::ostream &out, const Vector v) {
    return out << "Vector(" << v.x << ", " << v.y << ")";
}
