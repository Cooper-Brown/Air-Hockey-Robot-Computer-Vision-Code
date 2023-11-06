#include "Puck.hpp"
#include "MatDrawFunctions.hpp"

Puck::Puck() {
    center = Coordinate();
    velocity = Vector();
    averageVelocity = Vector();
    radius = 0;
    puckLostCounter = 0;
    puckLost = true;
    lastUpdateTime = 0;
    noPrevPosition = true;
    stationary = true;
}

void Puck::update(cv::Vec3f positionalData) {
    float puckMovingThreshold = 2;
    
    unsigned int ticksAtUpdateTime = GetTickCount();
    float newX = positionalData[0];
    float newY = positionalData[1];
    radius = positionalData[2];
    //std::cout << "Position " << newX << " " << newY << std::endl;

    if (noPrevPosition) {
        center.x = newX;
        center.y = newY;
        lastUpdateTime = ticksAtUpdateTime;
        noPrevPosition = false;
    }
    
    unsigned int timeElapsed = ticksAtUpdateTime - lastUpdateTime;
    Vector newVelocity = Vector();
    if (timeElapsed == 0){
        newVelocity.xComponent = 0;
        newVelocity.yComponent = 0;
    }
    else {
        newVelocity.xComponent = (newX - center.x) / (timeElapsed/1000.0);
        newVelocity.yComponent = (newY - center.y) / (timeElapsed/1000.0);
    }

    bool lastVelocityUnderThreshold = ((fabs(velocity.xComponent) < puckMovingThreshold) && (fabs(velocity.yComponent) < puckMovingThreshold));
    bool newVelocityUnderThreshold = ((fabs(newVelocity.xComponent) < puckMovingThreshold) && (fabs(newVelocity.yComponent) < puckMovingThreshold)); // was 3
    stationary = lastVelocityUnderThreshold && newVelocityUnderThreshold;

    if (stationary || (signOf(velocity.xComponent) != signOf(newVelocity.xComponent)) || (signOf(velocity.yComponent) != signOf(newVelocity.yComponent))){
        averageVelocity = newVelocity;
    }
    else{
        averageVelocity.xComponent = (averageVelocity.xComponent + newVelocity.xComponent) / 2.0;
        averageVelocity.yComponent = (averageVelocity.yComponent + newVelocity.yComponent) / 2.0;
    }

    center.x = newX;
    center.y = newY;
    velocity = newVelocity;
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