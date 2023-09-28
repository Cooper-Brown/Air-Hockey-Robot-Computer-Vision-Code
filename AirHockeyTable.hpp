#ifndef AIR_HOCKEY_TABLE_CLASS
#define AIR_HOCKEY_TABLE_CLASS

#include <opencv2/opencv.hpp>
#include "Line.hpp"
#include "Corner.hpp"

#define GOAL_LENGTH_REDUCTION_AMOUNT 50

class AirHockeyTable {
    public:
        float x, y, r, xOffset, yOffset;
        Line topLine, bottomLine, leftLine, rightLine;
        Line playerWinGoalLine, robotWinGoalLine;
        Line robotBoundaryTopLine, robotBoundaryBottomLine, robotBoundaryLeftLine, robotBoundaryRightLine;
        Line testLine;
        Corner topLeftCorner, topRightCorner, bottomRightCorner, bottomLeftCorner;
        AirHockeyTable();
        AirHockeyTable(float xIn, float yIn, float rIn, float xOffsetIn, float yOffsetIn);
        void draw(cv::Mat imageToDrawOn);
        
};

#endif