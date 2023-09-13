#include "Line.hpp"

Line::Line() {
    p1 = Coordinate();
    p2 = Coordinate();
}
Line::Line(Coordinate p1In, Coordinate p2In) {
    p1 = p1In;
    p2 = p2In;
}
