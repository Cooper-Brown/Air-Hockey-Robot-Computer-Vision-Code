
#include <unistd.h>

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

void GameState::updateLogic(cv::Mat imageToDrawOn) {
    computeFirstOrderPuckReflection(imageToDrawOn);
}

void GameState::computeFirstOrderPuckReflection(cv::Mat imageToDrawOn) {
    Vector puckVelocityUnitVector = greenPuck.velocity;//.getUnitVector(); UNIT VECTORS DEVIATE SIGNIFICANTLY FROM THE REGULAR VELOCITY FOR SOME REASON
    Line puckTravelProjection(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );

    drawBorderLine(imageToDrawOn, puckTravelProjection);
    
    // We need to compute a list of all the reflections that could happen with the straight lines in the table.
    Coordinate intersectionPoint;
    Vector reflectedVector;
    bool drawPoint = true;
    if (getLineIntersection2(pixelSpaceTable.playerWinGoalLine, puckTravelProjection, &intersectionPoint)) { // puckTravelProjection
        
    }
    else if (getLineIntersection2(pixelSpaceTable.leftLine, puckTravelProjection, &intersectionPoint)) {
        
    }
    else if (getLineIntersection2(pixelSpaceTable.rightLine, puckTravelProjection, &intersectionPoint)) {
        
    }
    else if (getLineIntersection2(pixelSpaceTable.topLine, puckTravelProjection, &intersectionPoint)) {
        
    }
    else if (getLineIntersection2(pixelSpaceTable.bottomLine, puckTravelProjection, &intersectionPoint)) {
        
    }
    else {
        drawPoint = false;
    }
    if (drawPoint){
        circle(imageToDrawOn, cv::Point(intersectionPoint.x, intersectionPoint.y), 5, cv::Scalar(255, 0, 0), 2);
    }

} 