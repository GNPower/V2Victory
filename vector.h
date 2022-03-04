#ifndef VECTOR_H
#define VECTOR_H

typedef float Scalar;


class Vector {
public:
    Scalar x;
    Scalar y;
    Vector(Scalar x, Scalar y);
    static Scalar distance(Vector v1, Vector v2);
};

#endif
