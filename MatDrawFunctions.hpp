#ifndef MAT_DRAW_FUNCTIONS
#define MAT_DRAW_FUNCTIONS

#include <opencv2/opencv.hpp>
#include "Line.hpp"
#include <memory>
#include <string>
#include <stdexcept>

void drawBorderLine(cv::Mat image, Line line);
void drawGoalLine(cv::Mat image, Line line);
void drawVelocityLine(cv::Mat image, Line line);
void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles);

unsigned int GetTickCount();

bool getLineIntersection(Line line1, Line line2, Coordinate* intersectionPoint);

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args );

#endif
