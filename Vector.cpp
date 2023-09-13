
#include "Vector.hpp"

#include <cmath>

Vector::Vector() {
    xComponent = 0;
    yComponent = 0;
}
Vector::Vector(int xComponentIn, int yComponentIn) {
    xComponent = xComponentIn;
    yComponent = yComponentIn;
}
int Vector::getLength() {
    return sqrt(pow(xComponent, 2) + pow(yComponent, 2));
}
