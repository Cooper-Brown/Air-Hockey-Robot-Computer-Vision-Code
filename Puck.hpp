#ifndef PUCK_CLASS
#define PUCK_CLASS

#include "Coordinate.hpp"
#include "Vector.hpp"

class Puck {
    public:
        Coordinate center, lastKnownCenter;
        Vector velocity;
        bool puckLost;
        Puck();

};

#endif