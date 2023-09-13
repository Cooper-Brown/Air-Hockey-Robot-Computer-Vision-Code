#include "Puck.hpp"

Puck::Puck() {
    center = Coordinate();
    lastKnownCenter = Coordinate();
    velocity = Vector();
    puckLost = true;
}