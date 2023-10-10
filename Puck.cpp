#include "Puck.hpp"
#include "MatDrawFunctions.hpp"

Puck::Puck() {
    center = Coordinate();
    lastKnownCenter = Coordinate();
    velocity = Vector();
    radius = 0;
    puckLostCounter = 0;
    puckLost = true;
    lastUpdateTime = 0;
    noPrevPosition = true;
    stationary = true;
}

void Puck::update(cv::Vec3f positionalData) {
    unsigned int ticksAtUpdateTime = GetTickCount();
    float newX = positionalData[0];
    float newY = positionalData[1];
    radius = positionalData[2];
    //std::cout << "Position " << newX << " " << newY << std::endl;
    if (noPrevPosition) {
        lastKnownCenter.x = newX;
        lastKnownCenter.y = newY;
        lastUpdateTime = ticksAtUpdateTime;
        noPrevPosition = false;
    }
    center.x = newX;
    center.y = newY;
    
    unsigned int timeElapsed = ticksAtUpdateTime - lastUpdateTime;
    if (timeElapsed == 0){
        velocity.xComponent = 0;
        velocity.yComponent = 0;
    }
    else {
        velocity.xComponent = (center.x - lastKnownCenter.x) / (timeElapsed/1000.0);
        velocity.yComponent = (center.y - lastKnownCenter.y) / (timeElapsed/1000.0);
    }

    stationary = ((velocity.xComponent < 0.5) && (velocity.yComponent < 0.5)); // was 3

    lastKnownCenter = center;
    lastUpdateTime = ticksAtUpdateTime;
    puckLost = false;
    puckLostCounter = 0;
    return;
}

void Puck::registerLostPuck() {
    if (++puckLostCounter < 3){
        return;
    }
    puckLost = true;
}

void Puck::draw(cv::Mat imageToDrawOn){
    std::vector<cv::Vec3f> detectedGreenCircles(1);
    detectedGreenCircles[0] = cv::Vec3f(center.x, center.y, radius);
    drawDetectedCircles(imageToDrawOn, detectedGreenCircles);
    
    Coordinate unitVectorEndpoint(center.x + velocity.xComponent/10.0, center.y + velocity.yComponent/10.0);
    Line velocityRepresentation(center, unitVectorEndpoint);
    drawVelocityLine(imageToDrawOn, velocityRepresentation);
    
}