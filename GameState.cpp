
#include <unistd.h>
#include <string>

#include "GameState.hpp"
#include "MatDrawFunctions.hpp"

GameState::GameState(cv::Size rescaledSize, StmCommunicator* stmCommsIn) {
    stmComms = stmCommsIn;
    float AHT_x = rescaledSize.width-40;
    float AHT_y = rescaledSize.height-80;
    float AHT_r = 40;
    float AHT_xOffset = (rescaledSize.width - AHT_x)/2;
    float AHT_yOffset = (rescaledSize.height - AHT_y)/2 + 5;
    pixelSpaceTable = AirHockeyTable(AHT_x, AHT_y, AHT_r, AHT_xOffset, AHT_yOffset);
    greenPuck = Puck();
    gameState = STATE_HARD_DEFENCE;
    resetTrackingAverage = true;
}

int GameState::translatePixelSpaceToRobotSpace(Coordinate pixelSpaceCoordinate, Coordinate* robotSpaceCoordinate){
    float leftExtremity = pixelSpaceTable.leftLine.p1.x;
    float rightExtremity = pixelSpaceTable.rightLine.p1.x;
    float topExtremity = pixelSpaceTable.bottomLine.p1.y;
    float bottomExtremity = pixelSpaceTable.robotBoundaryTopLine.p1.y;

    if (
        (pixelSpaceCoordinate.x > rightExtremity)  ||
        (pixelSpaceCoordinate.x < leftExtremity)   ||
        (pixelSpaceCoordinate.y > topExtremity)    ||
        (pixelSpaceCoordinate.y < bottomExtremity)
    ){
        return -1;
    }

    float pixelSpaceRatio_X = pixelSpaceCoordinate.x / (rightExtremity - leftExtremity);
    float pixelSpaceRatio_Y = pixelSpaceCoordinate.y / (topExtremity - bottomExtremity);
    robotSpaceCoordinate->x = TABLE_X_BOUNDARY_MIN + ((TABLE_X_BOUNDARY_MAX - TABLE_X_BOUNDARY_MIN)*pixelSpaceRatio_X);
    robotSpaceCoordinate->y = TABLE_Y_BOUNDARY_MIN + ((TABLE_Y_BOUNDARY_MAX - TABLE_Y_BOUNDARY_MIN)*pixelSpaceRatio_Y);
    return 0;
}

void GameState::registerLostPuck() {
    greenPuck.registerLostPuck();
}

void GameState::updatePuckPosition(cv::Vec3f positionalData) {
    greenPuck.update(positionalData);
}

void GameState::updateLogic(cv::Mat imageToDrawOn) {
    switch (gameState) {
        case STATE_STANDBY:
            standbyProcedure(imageToDrawOn);
            break;
        case STATE_ATTACK:
            attackProcedure(imageToDrawOn);
            break;
        case STATE_DEFEND:
            defendProcedure(imageToDrawOn);
            break;
        case STATE_HARD_DEFENCE:
            hardDefendProcedure(imageToDrawOn);
            break;
        default:
            break;
    }
    /*
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
    // (!greenPuck.stationary) && 
    if ((firstOrderReflection.reflectedSurface == "playerWinGoalLine")) {
        gameState = STATE_DEFEND;
        std::cout << "Defending " << defendCount++ << std::endl;        
    }

    if (gameState == STATE_DEFEND){
        circle(imageToDrawOn, cv::Point(firstOrderReflection.averagePosition.x, firstOrderReflection.averagePosition.y), 5, cv::Scalar(0, 0, 255), 2);
        if (greenPuck.puckLost){
            gameState = STATE_STANDBY;
        }
    }
    else if (gameState == STATE_STANDBY) {
        circle(imageToDrawOn, cv::Point(firstOrderReflection.averagePosition.x, firstOrderReflection.averagePosition.y), 5, cv::Scalar(0, 255, 0), 2);
    }
    */
}

bool hardDefendFirstIteration = true;
void GameState::hardDefendProcedure(cv::Mat imageToDrawOn){
    computeFirstOrderPuckReflection(imageToDrawOn);
    if (hardDefendFirstIteration){
        stmComms->setCoordinate(Coordinate((TABLE_X_BOUNDARY_MAX-TABLE_X_BOUNDARY_MIN)/2.0, TABLE_Y_BOUNDARY_MIN));
        hardDefendFirstIteration = false;
        return;
    }

}

void GameState::standbyProcedure(cv::Mat imageToDrawOn){
    // If the puck is lost, stay in standby mode
    if (greenPuck.puckLost){
        return;
    }

    // Stay in standby until the puck is stationary 
    if (!greenPuck.stationary){
        std::cout << "Puck Not Stationary" << std::endl;
        return;
    }

    // Determine if the puck is in the area of influence.
    if (pixelSpaceTable.checkCoordinateInRobotArea(greenPuck.center)){
        std::cout << "In Area of Influence" << std::endl;
        //gameState = STATE_ATTACK;
    }
}

// TEMPORARY DEBUGGING VARIABLES
int defendCount = 0;
void GameState::defendProcedure(cv::Mat imageToDrawOn){

}

bool attackCommandSent = false;
unsigned int moveTwoWaitTime = 5000;
unsigned int moveTwoTimerStartTime = 0;
void GameState::attackProcedure(cv::Mat imageToDrawOn){
    // Ensure conditions for the attack are still met
    if (!pixelSpaceTable.checkCoordinateInRobotArea(greenPuck.center)){
        std::cout << "Leaving Defend State" << std::endl;
        gameState = STATE_STANDBY;
        return;
    }
    
    // Come up with a plan
    Line targetTrajectoryDirect = Line(greenPuck.center, pixelSpaceTable.robotWinGoalLine.computeCenterCoordinate());
    drawBorderLine(imageToDrawOn, targetTrajectoryDirect);

    if (!attackCommandSent){
        if (moveTwoTimerStartTime == 0){
            moveTwoTimerStartTime = GetTickCount();
            if (stmComms->connectionEstablished)
                stmComms->setCoordinate(Coordinate((TABLE_X_BOUNDARY_MAX-TABLE_X_BOUNDARY_MIN)/2.0, 5000));
            std::cout << "Position 1 set" << std::endl;
            
        }
        if (GetTickCount() - moveTwoTimerStartTime > moveTwoWaitTime)
        {
            if (stmComms->connectionEstablished)
                stmComms->setCoordinate(Coordinate((TABLE_X_BOUNDARY_MAX-TABLE_X_BOUNDARY_MIN)/2.0, TABLE_Y_BOUNDARY_MAX-5000));
            std::cout << "Position 2 set" << std::endl;
            attackCommandSent = true;
        }
    }
}

// Will update the firstOrderPuckReflection class variable according to the most recent data.
void GameState::computeFirstOrderPuckReflection(cv::Mat imageToDrawOn) {
    // Project the puck's instantaneous velocity
    Vector puckVelocityUnitVector = greenPuck.velocity;//.getUnitVector(); UNIT VECTORS DEVIATE SIGNIFICANTLY FROM THE REGULAR VELOCITY FOR SOME REASON
    Line puckTravelProjection(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );

    puckTravelProjection = pixelSpaceTable.testLine;
    
    // Check the intersections for all boundaries
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
    //else if (getCornerIntersection(pixelSpaceTable.testLine, pixelSpaceTable.bottomRightCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
    //    std::cout << "BR Intersection" << std::endl;
    //}
    //else if (getCornerIntersection(pixelSpaceTable.testLine, pixelSpaceTable.topRightCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
    //    std::cout << "TR Intersection" << std::endl;
    //}
    else if (getCornerIntersection(puckTravelProjection, pixelSpaceTable.bottomLeftCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
        std::cout << "BL Intersection " << firstOrderReflection.mostRecentReflectionPosition.x << " " << firstOrderReflection.mostRecentReflectionPosition.y << std::endl;
    }
    //else if (getCornerIntersection(pixelSpaceTable.testLine, pixelSpaceTable.topLeftCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
    //    std::cout << "TL Intersection" << std::endl;
    //}
    else {
        drawPoint = false;
    }
    if (true){ // drawPoint
        drawBorderLine(imageToDrawOn, puckTravelProjection);
        circle(imageToDrawOn, cv::Point(firstOrderReflection.mostRecentReflectionPosition.x, firstOrderReflection.mostRecentReflectionPosition.y), 5, cv::Scalar(255, 0, 0), 2);
        circle(imageToDrawOn, cv::Point(41,67), 5, cv::Scalar(255, 0, 0), 2);

    }

} 