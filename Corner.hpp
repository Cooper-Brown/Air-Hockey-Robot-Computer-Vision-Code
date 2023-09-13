#ifndef CORNER_CLASS
#define CORNER_CLASS

#include "Coordinate.hpp"

class Corner {
    public:
        Coordinate center;
        float radius;
        Corner();
        Corner(Coordinate centerIn, float radiusIn);
};

#endif