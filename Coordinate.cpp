
#include "Coordinate.hpp"

#include <cmath>

Coordinate::Coordinate () {
    x = 0;
    y = 0;
}
Coordinate::Coordinate (float xIn, float yIn) {
    x = xIn;
    y = yIn;
}

float Coordinate::getDistanceFrom(Coordinate other) {
    return sqrt(pow(x-other.x, 2) + pow(y-other.y, 2)); 
}