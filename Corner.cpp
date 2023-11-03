
#include "Corner.hpp"

Corner::Corner() {
    center = Coordinate();
    radius = 0;
    startAngle = 0;
    endAngle = 0;
}
Corner::Corner(Coordinate centerIn, float radiusIn, float startAngleIn, float endAngleIn) {
    center = centerIn;
    radius = radiusIn;
    startAngle = startAngleIn;
    endAngle = endAngleIn;
}
