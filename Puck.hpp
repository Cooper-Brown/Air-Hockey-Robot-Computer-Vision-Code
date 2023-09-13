#ifndef PUCK_CLASS
#define PUCK_CLASS

#include "Coordinate.hpp"
#include "Vector.hpp"

class Puck {
    public:
        Coordinate center;
        Vector velocity;
        Puck();

};

#endif