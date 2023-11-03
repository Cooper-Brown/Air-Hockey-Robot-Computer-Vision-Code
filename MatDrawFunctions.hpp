#ifndef MAT_DRAW_FUNCTIONS
#define MAT_DRAW_FUNCTIONS

#include <opencv2/opencv.hpp>
#include "Line.hpp"
#include "Corner.hpp"
#include <memory>
#include <string>
#include <stdexcept>

void drawBorderLine(cv::Mat image, Line line);
void drawCorner(cv::Mat image, Corner corner);
void drawGoalLine(cv::Mat image, Line line);
void drawVelocityLine(cv::Mat image, Line line);
void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles);

unsigned int GetTickCount();

//bool getLineIntersection(Line line1, Line line2, Coordinate* intersectionPoint);
bool getLineIntersection2(Line line1, Line line2, Coordinate* intersectionPoint);
bool getCornerIntersection(Line line1, Corner corner1, Coordinate* intersectionPoint);

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args );

#endif
