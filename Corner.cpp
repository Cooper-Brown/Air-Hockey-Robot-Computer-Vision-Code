
#include "Corner.hpp"

Corner::Corner() {
    center = Coordinate();
    radius = 0;
}
Corner::Corner(Coordinate centerIn, float radiusIn) {
    center = centerIn;
    radius = radiusIn;
}
