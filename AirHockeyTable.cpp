#include "AirHockeyTable.hpp"

#include "MatDrawFunctions.hpp"

AirHockeyTable::AirHockeyTable() {
    x = 0; // DEFAULT VALUES
    //AirHockeyTable();
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
    topLeftCorner = Corner(Coordinate(xOffset + r, yOffset + y - r), r);
    topRightCorner = Corner(Coordinate(xOffset + x - r, yOffset + y - r), r);
    bottomRightCorner = Corner(Coordinate(xOffset + x - r, yOffset + r), r);
    bottomLeftCorner = Corner(Coordinate(xOffset + r, yOffset + r), r);
}

void AirHockeyTable::draw(cv::Mat imageToDrawOn) {
    drawBorderLine(imageToDrawOn, topLine);
    drawBorderLine(imageToDrawOn, bottomLine);
    drawBorderLine(imageToDrawOn, leftLine);
    drawBorderLine(imageToDrawOn, rightLine);
}
