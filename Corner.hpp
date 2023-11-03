#ifndef CORNER_CLASS
#define CORNER_CLASS

#include "Coordinate.hpp"

class Corner {
    public:
        Coordinate center;
        float radius;
        float startAngle;
        float endAngle;
        Corner();
        Corner(Coordinate centerIn, float radiusIn, float startAngleIn, float endAngleIn);
};

#endif