
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

void GameState::updatePuckPosition(cv::Vec3f positionalData, cv::Mat imageToDrawOn) {
    greenPuck.update(positionalData);
    computeFirstOrderPuckReflection(imageToDrawOn);
}

bool sleepNext = false;

void GameState::computeFirstOrderPuckReflection(cv::Mat imageToDrawOn) {
    Vector puckVelocityUnitVector = greenPuck.velocity;//.getUnitVector(); UNIT VECTORS DEVIATE SIGNIFICANTLY FROM THE REGULAR VELOCITY FOR SOME REASON
    Line puckTravelProjection(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );

    drawBorderLine(imageToDrawOn, puckTravelProjection);
    
    // We need to compute a list of all the reflections that could happen with the straight lines in the table.
    std::list<Coordinate> intersectionPoints;

    Coordinate intersectionFunctionOut;
    if (getLineIntersection2(pixelSpaceTable.playerWinGoalLine, puckTravelProjection, &intersectionFunctionOut)) { // puckTravelProjection
        intersectionPoints.push_back(intersectionFunctionOut);
    }

    if (sleepNext){
        //sleep(1);
        sleepNext = false;
    }
    if (intersectionPoints.size() != 0){
        circle(imageToDrawOn, cv::Point(intersectionPoints.front().x, intersectionPoints.front().y), 5, cv::Scalar(255, 0, 0), 2);
        sleepNext = true;
    }

}