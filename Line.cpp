#include "Line.hpp"

#include <cmath>
#include <iostream>

Line::Line() {
    p1 = Coordinate();
    p2 = Coordinate();
}
Line::Line(Coordinate p1In, Coordinate p2In) {
    p1 = p1In;
    p2 = p2In;
}

bool Line::contains(Coordinate coordinateIn) {
    float zeroEquivalenceTolerance = 0.01;
    float rise = p2.y - p1.y;
    float run = p2.x - p1.x;
    if (fabs(run) < zeroEquivalenceTolerance) {
        std::cout << "Horizontal " << std::endl;
        return (
            ((p1.y < coordinateIn.y) && (coordinateIn.y < p2.y)) || 
            ((p2.y < coordinateIn.y) && (coordinateIn.y < p1.y))
        );
    }
    float gradient = rise / run;
    float intercept = p1.y - gradient * p1.x;
    float expectedY = gradient * coordinateIn.x + intercept;
    return (fabs(expectedY - coordinateIn.y) < zeroEquivalenceTolerance);
}

Coordinate Line::computeCenterCoordinate() {
    Coordinate center = Coordinate(
        (p1.x + p2.x)/2.0,
        (p1.y + p2.y)/2.0
    );
    return center;
}
 