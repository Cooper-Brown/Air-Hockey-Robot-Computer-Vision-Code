
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
    hardDefendFirstIteration = true;
}

int GameState::translatePixelSpaceToRobotSpace(Coordinate pixelSpaceCoordinate, Coordinate* robotSpaceCoordinate){
    float leftExtremity = pixelSpaceTable.robotBoundaryLeftLine.p1.x;
    float rightExtremity = pixelSpaceTable.robotBoundaryRightLine.p1.x;
    float topExtremity = pixelSpaceTable.robotBoundaryTopLine.p1.y;
    float bottomExtremity = pixelSpaceTable.robotBoundaryBottomLine.p1.y;

    if (
        (pixelSpaceCoordinate.x > rightExtremity)  ||
        (pixelSpaceCoordinate.x < leftExtremity)   ||
        (pixelSpaceCoordinate.y > topExtremity)    ||
        (pixelSpaceCoordinate.y < bottomExtremity)
    ){
        return -1;
    }

    
    float pixelSpaceRatio_X = (pixelSpaceCoordinate.x - leftExtremity) / (rightExtremity - leftExtremity);
    float pixelSpaceRatio_Y = (pixelSpaceCoordinate.y - bottomExtremity) / (topExtremity - bottomExtremity);
    // Coordinate flip happens here
    robotSpaceCoordinate->x = TABLE_X_BOUNDARY_MIN + ((TABLE_X_BOUNDARY_MAX - TABLE_X_BOUNDARY_MIN)*pixelSpaceRatio_Y);
    robotSpaceCoordinate->y = TABLE_Y_BOUNDARY_MIN + ((TABLE_Y_BOUNDARY_MAX - TABLE_Y_BOUNDARY_MIN)*pixelSpaceRatio_X);
    return 0;
}

void GameState::registerLostPuck() {
    greenPuck.registerLostPuck();
}

void GameState::updatePuckPosition(cv::Vec3f positionalData) {
    greenPuck.update(positionalData);
    //std::cout << positionalData[2] << std::endl; // puck radius is 12 pixels
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
    bool transmissionSent = stmComms->processPendingTransmission();
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

void GameState::hardDefendProcedure(cv::Mat imageToDrawOn){
    
    computeFirstOrderPuckReflection(imageToDrawOn);
    computeSecondOrderPuckReflection(imageToDrawOn);
    
    // when game first starts, put puck in center of goal.
    if (hardDefendFirstIteration){
        stmComms->setCoordinate(Coordinate(TABLE_X_BOUNDARY_MIN + (TABLE_X_BOUNDARY_MAX-TABLE_X_BOUNDARY_MIN)/2.0, TABLE_Y_BOUNDARY_MIN));
        hardDefendFirstIteration = false;
        return;
    }

    if (greenPuck.puckLost){
        return;
    }
    
    // if puck will go into goal, figure out where mallet needs to be to block it.
    Reflection reflectionToUse;
    bool defendActivate = false;
    if (firstOrderReflection.reflectedSurface == "playerWinGoalLine"){
        reflectionToUse = firstOrderReflection;
        defendActivate = true;
    }
    else if (secondOrderReflection.reflectedSurface == "playerWinGoalLine"){
        if (!(firstOrderReflection.reflectedSurface == "rightLine") && !(firstOrderReflection.reflectedSurface == "robotWinGoalLine")){
            reflectionToUse = secondOrderReflection;
            defendActivate = true;
        }
    }
    if (defendActivate) {
        // Used for plotting only
        Coordinate goalEntryCoordinate = Coordinate(
            pixelSpaceTable.playerWinGoalLine.p1.x,
            reflectionToUse.mostRecentReflectionPosition.y
        );
        //Line goalTrajectory = Line(goalEntryCoordinate, reflectionToUse.incomingTrajectory);
        

        Coordinate newPositionPixelSpace = Coordinate();
        getLineIntersection2(pixelSpaceTable.robotBoundaryLeftLine, reflectionToUse.incomingTrajectory, &newPositionPixelSpace);
        
        //std::cout << pixelSpaceTable.robotBoundaryLeftLine.p1.x << std::endl;
        //std::cout << "Circle at X:" << newPositionPixelSpace.x << " Y:" << newPositionPixelSpace.y << std::endl;
        circle(imageToDrawOn, cv::Point(goalEntryCoordinate.x, goalEntryCoordinate.y), 10, cv::Scalar(0, 0, 255), 2);

        Coordinate newPositionRobotSpace = Coordinate();
        if (translatePixelSpaceToRobotSpace(newPositionPixelSpace, &newPositionRobotSpace) < 0){
            return;
        }
        stmComms->setCoordinate(newPositionRobotSpace);
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
    Vector puckVelocityUnitVector = greenPuck.averageVelocity;
    Line puckTravelProjectionCenter(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionBottom(
        Coordinate(greenPuck.center.x, greenPuck.center.y-PUCK_RADIUS_PIXELS), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionTop(
        Coordinate(greenPuck.center.x, greenPuck.center.y+PUCK_RADIUS_PIXELS), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionLeft(
        Coordinate(greenPuck.center.x-PUCK_RADIUS_PIXELS, greenPuck.center.y), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionRight(
        Coordinate(greenPuck.center.x+PUCK_RADIUS_PIXELS, greenPuck.center.y), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );

    firstOrderReflection.incomingVector = puckVelocityUnitVector;
    firstOrderReflection.incomingTrajectory = puckTravelProjectionCenter;
    // Used for debugging trajectories
    //puckTravelProjection = pixelSpaceTable.testLine;
    
    // Check the intersections for all boundaries
    Vector reflectedVector;
    bool drawPoint = !greenPuck.stationary;
    if (getLineIntersection2(pixelSpaceTable.playerWinGoalLine, puckTravelProjectionLeft, &(firstOrderReflection.mostRecentReflectionPosition))) { // puckTravelProjection
        firstOrderReflection.reflectedSurface.assign("playerWinGoalLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.leftLine, puckTravelProjectionLeft, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("leftLine");
        firstOrderReflection.mostRecentReflectionPosition.x += 12;
        firstOrderReflection.reflectedVector = greenPuck.velocity;
        firstOrderReflection.reflectedVector.xComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.rightLine, puckTravelProjectionRight, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("rightLine");
        firstOrderReflection.mostRecentReflectionPosition.x -= 12;
        firstOrderReflection.reflectedVector = greenPuck.velocity;
        firstOrderReflection.reflectedVector.xComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.topLine, puckTravelProjectionTop, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("topLine");
        firstOrderReflection.mostRecentReflectionPosition.y -= 12;
        firstOrderReflection.reflectedVector = greenPuck.velocity;
        firstOrderReflection.reflectedVector.yComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.bottomLine, puckTravelProjectionBottom, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("bottomLine");
        firstOrderReflection.mostRecentReflectionPosition.y += 12;
        firstOrderReflection.reflectedVector = greenPuck.velocity;
        firstOrderReflection.reflectedVector.yComponent *= -1;
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.bottomRightCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("bottomRightCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.topRightCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("topRightCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.bottomLeftCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("bottomLeftCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.topLeftCorner, &(firstOrderReflection.mostRecentReflectionPosition))) {
        firstOrderReflection.reflectedSurface.assign("topLeftCorner");
    }
    else {
        drawPoint = false;
    }
    if (drawPoint){

        firstOrderReflection.reflectedTrajectory = Line(
            firstOrderReflection.mostRecentReflectionPosition,
            Coordinate(
                (firstOrderReflection.mostRecentReflectionPosition.x + 1000*firstOrderReflection.reflectedVector.xComponent),
                (firstOrderReflection.mostRecentReflectionPosition.y + 1000*firstOrderReflection.reflectedVector.yComponent)
            )
        );

        drawBorderLine(imageToDrawOn, puckTravelProjectionCenter);
        drawBorderLine(imageToDrawOn, firstOrderReflection.reflectedTrajectory);

        //drawBorderLine(imageToDrawOn, puckTravelProjectionTop);
        //drawBorderLine(imageToDrawOn, puckTravelProjectionBottom);
        //drawBorderLine(imageToDrawOn, puckTravelProjectionLeft);
        //drawBorderLine(imageToDrawOn, puckTravelProjectionRight);
        circle(imageToDrawOn, cv::Point(firstOrderReflection.mostRecentReflectionPosition.x, firstOrderReflection.mostRecentReflectionPosition.y), 5, cv::Scalar(255, 0, 0), 2);
    }
} 

void GameState::computeSecondOrderPuckReflection(cv::Mat imageToDrawOn) {
    // Project the puck's instantaneous velocity
    Vector puckVelocityUnitVector = firstOrderReflection.reflectedVector;//.getUnitVector(); UNIT VECTORS DEVIATE SIGNIFICANTLY FROM THE REGULAR VELOCITY FOR SOME REASON
    Line puckTravelProjectionCenter(
        firstOrderReflection.mostRecentReflectionPosition, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionBottom(
        Coordinate(firstOrderReflection.mostRecentReflectionPosition.x, firstOrderReflection.mostRecentReflectionPosition.y-PUCK_RADIUS_PIXELS), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionTop(
        Coordinate(firstOrderReflection.mostRecentReflectionPosition.x, firstOrderReflection.mostRecentReflectionPosition.y+PUCK_RADIUS_PIXELS), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionLeft(
        Coordinate(firstOrderReflection.mostRecentReflectionPosition.x-PUCK_RADIUS_PIXELS, firstOrderReflection.mostRecentReflectionPosition.y), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    Line puckTravelProjectionRight(
        Coordinate(firstOrderReflection.mostRecentReflectionPosition.x+PUCK_RADIUS_PIXELS, firstOrderReflection.mostRecentReflectionPosition.y), 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    secondOrderReflection.incomingVector = puckVelocityUnitVector;
    secondOrderReflection.incomingTrajectory = puckTravelProjectionCenter;
    Vector reflectedVector;
    bool drawPoint = !greenPuck.stationary;
    if (getLineIntersection2(pixelSpaceTable.playerWinGoalLine, puckTravelProjectionLeft, &(secondOrderReflection.mostRecentReflectionPosition))) { // puckTravelProjection
        secondOrderReflection.reflectedSurface.assign("playerWinGoalLine");
    }
    else if (getLineIntersection2(pixelSpaceTable.leftLine, puckTravelProjectionLeft, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("leftLine");
        secondOrderReflection.mostRecentReflectionPosition.x += 12;
        secondOrderReflection.reflectedVector = greenPuck.velocity;
        secondOrderReflection.reflectedVector.xComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.rightLine, puckTravelProjectionRight, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("rightLine");
        secondOrderReflection.mostRecentReflectionPosition.x -= 12;
        secondOrderReflection.reflectedVector = greenPuck.velocity;
        secondOrderReflection.reflectedVector.xComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.topLine, puckTravelProjectionTop, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("topLine");
        secondOrderReflection.mostRecentReflectionPosition.y -= 12;
        secondOrderReflection.reflectedVector = greenPuck.velocity;
        secondOrderReflection.reflectedVector.yComponent *= -1;
    }
    else if (getLineIntersection2(pixelSpaceTable.bottomLine, puckTravelProjectionBottom, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("bottomLine");
        secondOrderReflection.mostRecentReflectionPosition.y += 12;
        secondOrderReflection.reflectedVector = greenPuck.velocity;
        secondOrderReflection.reflectedVector.yComponent *= -1;
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.bottomRightCorner, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("bottomRightCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.topRightCorner, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("topRightCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.bottomLeftCorner, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("bottomLeftCorner");
    }
    else if (getCornerIntersection(puckTravelProjectionCenter, pixelSpaceTable.topLeftCorner, &(secondOrderReflection.mostRecentReflectionPosition))) {
        secondOrderReflection.reflectedSurface.assign("topLeftCorner");
    }
    else {
        drawPoint = false;
    }
    if (drawPoint){

        secondOrderReflection.reflectedTrajectory = Line(
            secondOrderReflection.mostRecentReflectionPosition,
            Coordinate(
                (secondOrderReflection.mostRecentReflectionPosition.x + 1000*secondOrderReflection.reflectedVector.xComponent),
                (secondOrderReflection.mostRecentReflectionPosition.y + 1000*secondOrderReflection.reflectedVector.yComponent)
            )
        );
        
        circle(imageToDrawOn, cv::Point(secondOrderReflection.mostRecentReflectionPosition.x, secondOrderReflection.mostRecentReflectionPosition.y), 5, cv::Scalar(0, 255, 0), 2);
    }
    return;
}
