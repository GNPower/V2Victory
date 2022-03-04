#ifndef VECTOR_H
#define VECTOR_H

#include <ostream>


typedef float Scalar;


class Vector {
public:
    Scalar x;
    Scalar y;
    Vector(Scalar x, Scalar y);
    static Scalar distance(const Vector v1, const Vector v2);
};


Vector operator+(const Vector v1, const Vector v2);
Vector operator-(const Vector v1, const Vector v2);
std::ostream &operator<<(std::ostream &out, const Vector v);

#endif
