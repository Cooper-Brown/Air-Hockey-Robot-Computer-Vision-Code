#ifndef LINE_CLASS
#define LINE_CLASS

#include "Coordinate.hpp"

class Line {
    public:
        Coordinate p1;
        Coordinate p2;
        Line();
        Line(Coordinate p1In, Coordinate p2In);
        bool contains(Coordinate coordinateIn);
        Coordinate computeCenterCoordinate();
};

#endif