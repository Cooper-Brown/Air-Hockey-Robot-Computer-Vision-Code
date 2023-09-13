#ifndef MAT_DRAW_FUNCTIONS
#define MAT_DRAW_FUNCTIONS

#include <opencv2/opencv.hpp>
#include "Line.hpp"

void drawBorderLine(cv::Mat image, Line line);
void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles);

#endif
