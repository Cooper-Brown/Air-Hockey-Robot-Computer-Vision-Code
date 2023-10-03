#ifndef COORDINATE_CLASS
#define COORDINATE_CLASS

class Coordinate {
    public:
        float x;
        float y;
        Coordinate();
        Coordinate (float xIn, float yIn);
        float getDistanceFrom(Coordinate other);
};

#endif