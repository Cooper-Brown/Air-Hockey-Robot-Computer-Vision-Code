#include "Puck.hpp"
#include "MatDrawFunctions.hpp"

Puck::Puck() {
    center = Coordinate();
    lastKnownCenter = Coordinate();
    velocity = Vector();
    radius = 0;
    puckLost = true;
    lastUpdateTime = 0;
    noPrevPosition = true;
}

void Puck::update(cv::Vec3f positionalData) {
    unsigned int ticksAtUpdateTime = GetTickCount();
    float newX = positionalData[0];
    float newY = positionalData[1];
    radius = positionalData[2];
    // std::cout << "Position " << newX << " " << newY << std::endl;
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
    else{
        velocity.xComponent = (center.x - lastKnownCenter.x) / (timeElapsed/1000.0);
        velocity.yComponent = (center.y - lastKnownCenter.y) / (timeElapsed/1000.0);
    }

    if ((velocity.xComponent != 0) || (velocity.yComponent != 0)) {
        //std::cout << velocity.xComponent << " " << velocity.yComponent << std::endl;
    }

    lastKnownCenter = center;
    lastUpdateTime = ticksAtUpdateTime;
    puckLost = false;
    return;
}

void Puck::registerLostPuck() {
    puckLost = true;
}

void Puck::draw(cv::Mat imageToDrawOn){
    std::vector<cv::Vec3f> detectedGreenCircles(1);
    detectedGreenCircles[0] = cv::Vec3f(center.x, center.y, radius);
    drawDetectedCircles(imageToDrawOn, detectedGreenCircles);

    Vector puckUnitVelocity = velocity.getUnitVector();
    Coordinate unitVectorEndpoint(center.x + 100.0*puckUnitVelocity.xComponent, center.y + 100.0*puckUnitVelocity.yComponent);
    Line velocityRepresentation(center, unitVectorEndpoint);
    drawVelocityLine(imageToDrawOn, velocityRepresentation);
    
}