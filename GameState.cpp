
#include <unistd.h>
#include <string>

#include "GameState.hpp"
#include "MatDrawFunctions.hpp"

GameState::GameState(cv::Size rescaledSize) {
    float AHT_x = rescaledSize.width-40;
    float AHT_y = rescaledSize.height-80;
    float AHT_r = 40;
    float AHT_xOffset = (rescaledSize.width - AHT_x)/2;
    float AHT_yOffset = (rescaledSize.height - AHT_y)/2 + 5;
    pixelSpaceTable = AirHockeyTable(AHT_x, AHT_y, AHT_r, AHT_xOffset, AHT_yOffset);
    greenPuck = Puck();
}

void GameState::registerLostPuck() {
    greenPuck.registerLostPuck();
}

void GameState::updatePuckPosition(cv::Vec3f positionalData) {
    greenPuck.update(positionalData);
    
}

bool resetTrackingAverage = true;

#define STATE_STANDBY 0
#define STATE_DEFEND 1
int robotEntityState = STATE_STANDBY;

void GameState::updateLogic(cv::Mat imageToDrawOn) {
    computeFirstOrderPuckReflection(imageToDrawOn);
    if (resetTrackingAverage){
        firstOrderReflection.averagePosition = firstOrderReflection.mostRecentReflectionPosition;
        resetTrackingAverage = false;
    }
    else {
        firstOrderReflection.averagePosition.x = (firstOrderReflection.averagePosition.x + firstOrderReflection.mostRecentReflectionPosition.x) / 2;
        firstOrderReflection.averagePosition.y = (firstOrderReflection.averagePosition.y + firstOrderReflection.mostRecentReflectionPosition.y) / 2;
        if (firstOrderReflection.averagePosition.getDistanceFrom(firstOrderReflection.mostRecentReflectionPosition) > 100){
            resetTrackingAverage = true;
        }
    }

    if ((!greenPuck.stationary) && (firstOrderReflection.reflectedSurface == "playerWinGoalLine")) {
        robotEntityState = STATE_DEFEND;
        std::cout << "Defending" << std::endl;        
    }

    if (robotEntityState == STATE_DEFEND){
        circle(imageToDrawOn, cv::Point(firstOrderReflection.averagePosition.x, firstOrderReflection.averagePosition.y), 5, cv::Scalar(0, 0, 255), 2);
        if (greenPuck.puckLost){
            robotEntityState = STATE_STANDBY;
        }
    }
    else if (robotEntityState == STATE_STANDBY) {
        circle(imageToDrawOn, cv::Point(firstOrderReflection.averagePosition.x, firstOrderReflection.averagePosition.y), 5, cv::Scalar(0, 255, 0), 2);
    }
}

void GameState::computeFirstOrderPuckReflection(cv::Mat imageToDrawOn) {
    Vector puckVelocityUnitVector = greenPuck.velocity;//.getUnitVector(); UNIT VECTORS DEVIATE SIGNIFICANTLY FROM THE REGULAR VELOCITY FOR SOME REASON
    Line puckTravelProjection(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    
    // We need to compute a list of all the reflections that could happen with the straight lines in the table.
    Vector reflectedVector;
    bool drawPoint = !greenPuck.stationary;
    if (getLineIntersection2(pixelSpaceTable.playerWinGoalLine, puckTravelProjection, &(firstOrderReflection.mostRecentReflectionPosition))) { // puckTravelProjection
        firstOrderReflection.reflectedSurface.assign("playerWinGoalLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.leftLine, puckTravelProjection, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("leftLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.rightLine, puckTravelProjection, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("rightLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.topLine, puckTravelProjection, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("topLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.bottomLine, puckTravelProjection, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("bottomLine");
    }
    else {
        drawPoint = false;
    }
    if (drawPoint){
        drawBorderLine(imageToDrawOn, puckTravelProjection);
        circle(imageToDrawOn, cv::Point(firstOrderReflection.mostRecentReflectionPosition.x, firstOrderReflection.mostRecentReflectionPosition.y), 5, cv::Scalar(255, 0, 0), 2);
    }

} 