
#include "Vector.hpp"

#include <cmath>

Vector::Vector() {
    xComponent = 0;
    yComponent = 0;
}
Vector::Vector(float xComponentIn, float yComponentIn) {
    xComponent = xComponentIn;
    yComponent = yComponentIn;
}
float Vector::getLength() {
    return sqrt(pow(xComponent, 2) + pow(yComponent, 2));
}
Vector Vector::getUnitVector() {
    float length = getLength();
    if (length == 0){
        return (Vector(0, 0));
    }
    return (Vector(xComponent/length, yComponent/length));
}
