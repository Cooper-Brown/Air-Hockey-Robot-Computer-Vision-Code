#ifndef PUCK_CLASS
#define PUCK_CLASS

#include "Coordinate.hpp"
#include "Vector.hpp"
#include <opencv2/opencv.hpp>

class Puck {
    public:
        Coordinate center, lastKnownCenter;
        Vector velocity;
        bool puckLost, noPrevPosition, stationary;
        unsigned int lastUpdateTime;
        int puckLostCounter;
        float radius;
        Puck();
        void update(cv::Vec3f positionalData);
        void draw(cv::Mat imageToDrawOn);
        void registerLostPuck();
};

#endif