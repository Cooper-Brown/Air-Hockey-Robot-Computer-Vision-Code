#include "AirHockeyTable.hpp"

#include "MatDrawFunctions.hpp"

AirHockeyTable::AirHockeyTable() {
    cv::Size rescaledSize(640, 360);
    float AHT_x = rescaledSize.width-40;
    float AHT_y = rescaledSize.height-80;
    float AHT_r = 40;
    float AHT_xOffset = (rescaledSize.width - AHT_x)/2;
    float AHT_yOffset = (rescaledSize.height - AHT_y)/2 + 5;
    AirHockeyTable(AHT_x, AHT_y, AHT_r, AHT_xOffset, AHT_yOffset);
}

AirHockeyTable::AirHockeyTable(float xIn, float yIn, float rIn, float xOffsetIn, float yOffsetIn) {
    x = xIn;
    y = yIn;
    r = rIn;
    xOffset = xOffsetIn;
    yOffset = yOffsetIn;

    topLine = Line(Coordinate(xOffset + r, y + yOffset), Coordinate(xOffset + x - r, y + yOffset));
    bottomLine = Line(Coordinate(xOffset + r, yOffset), Coordinate(xOffset + x - r, yOffset));
    leftLine = Line(Coordinate(xOffset, yOffset + r), Coordinate(xOffset, yOffset+y-r));
    rightLine = Line(Coordinate(xOffset+x, yOffset + r), Coordinate(xOffset+x, yOffset+y-r));
    topLeftCorner = Corner(Coordinate(xOffset + r, yOffset + y - r), r, 90,180);
    topRightCorner = Corner(Coordinate(xOffset + x - r, yOffset + y - r), r, 0,90);
    bottomRightCorner = Corner(Coordinate(xOffset + x - r, yOffset + r), r, 270, 360);
    bottomLeftCorner = Corner(Coordinate(xOffset + r, yOffset + r), r, 180, 270);

    playerWinGoalLine = Line(
        Coordinate(leftLine.p1.x, leftLine.p1.y + GOAL_LENGTH_REDUCTION_AMOUNT), 
        Coordinate(leftLine.p2.x, leftLine.p2.y - GOAL_LENGTH_REDUCTION_AMOUNT)
    );
    robotWinGoalLine = Line(
        Coordinate(rightLine.p1.x, rightLine.p1.y + GOAL_LENGTH_REDUCTION_AMOUNT), 
        Coordinate(rightLine.p2.x, rightLine.p2.y - GOAL_LENGTH_REDUCTION_AMOUNT)
    );

    robotBoundaryTopLine = Line(
        Coordinate(topLine.p1.x - 20, topLine.p1.y - 30), // 1st num could be -20 for more accurate
        Coordinate(topLine.p2.x - 270, topLine.p2.y - 30)
    );
    robotBoundaryBottomLine = Line(
        Coordinate(bottomLine.p1.x - 20, bottomLine.p1.y + 25),
        Coordinate(bottomLine.p2.x - 270, bottomLine.p2.y + 25)
    );
    robotBoundaryLeftLine = Line(
        Coordinate(leftLine.p1.x + 20, leftLine.p1.y - 15),
        Coordinate(leftLine.p2.x + 20, leftLine.p2.y + 10)
    );
    robotBoundaryRightLine = Line(
        Coordinate(rightLine.p1.x - r - 270, rightLine.p1.y - 15),
        Coordinate(rightLine.p2.x - r - 270, rightLine.p2.y + 10)
    );

    testLine = Line(Coordinate(640/2.0, 360/2.0), Coordinate(0, 50));

}

void AirHockeyTable::draw(cv::Mat imageToDrawOn) {
    drawBorderLine(imageToDrawOn, topLine);
    drawBorderLine(imageToDrawOn, bottomLine);
    drawBorderLine(imageToDrawOn, leftLine);
    drawBorderLine(imageToDrawOn, rightLine);

    drawCorner(imageToDrawOn, topLeftCorner);
    drawCorner(imageToDrawOn, topRightCorner);
    drawCorner(imageToDrawOn, bottomLeftCorner);
    drawCorner(imageToDrawOn, bottomRightCorner);

    drawGoalLine(imageToDrawOn, playerWinGoalLine);
    drawGoalLine(imageToDrawOn, robotWinGoalLine);
    

    drawBorderLine(imageToDrawOn, robotBoundaryTopLine);
    drawBorderLine(imageToDrawOn, robotBoundaryBottomLine);
    drawBorderLine(imageToDrawOn, robotBoundaryLeftLine);
    drawBorderLine(imageToDrawOn, robotBoundaryRightLine);

    drawBorderLine(imageToDrawOn, testLine);
}

bool AirHockeyTable::checkCoordinateInRobotArea(Coordinate coordinateIn){
    return (
        (coordinateIn.x > robotBoundaryLeftLine.p1.x) && (coordinateIn.x < robotBoundaryRightLine.p1.x) &&
        (coordinateIn.y > robotBoundaryBottomLine.p1.y) && (coordinateIn.y < robotBoundaryTopLine.p1.y)
    );
}
