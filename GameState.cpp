#include <opencv2/opencv.hpp>

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
    computeFirstOrderPuckReflection();
}

void GameState::computeFirstOrderPuckReflection() {
    Vector puckVelocityUnitVector = greenPuck.velocity.getUnitVector();
    Line puckTravelProjection(
        greenPuck.center, 
        Coordinate(puckVelocityUnitVector.xComponent*1000, puckVelocityUnitVector.yComponent*1000)
    );
    
    // We need to compute a list of all the reflections that could happen with the straight lines in the table.
    std::list<Coordinate> intersectionPoints;

    Coordinate* intersectionFunctionOut;
    intersectionFunctionOut = new Coordinate();
    if (getLineIntersection(pixelSpaceTable.leftLine, puckTravelProjection, intersectionFunctionOut)) {
        intersectionPoints.push_back(*intersectionFunctionOut);
    }

    if (intersectionPoints.size() != 0){
        std::cout << "Will intersect" << std::endl;
    }

}