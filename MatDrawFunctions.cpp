
#include "MatDrawFunctions.hpp"
#include <sys/time.h>

#define COLOR_PURPLE cv::Scalar(255,0,255)
#define COLOR_BLUE cv::Scalar(255,0,0)

void drawBorderLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_PURPLE, thickness, cv::LINE_4);
}

void drawVelocityLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_BLUE, thickness, cv::LINE_8);
    std::cout << line.p1.x << ":" << line.p1.y << " " << line.p2.x << ":" << line.p1.y << std::endl;
}

void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles){
    for (const cv::Vec3f& circleInstance : circles) {
        cv::Point center(cvRound(circleInstance[0]), cvRound(circleInstance[1]));
        int radius = cvRound(circleInstance[2]);
        circle(image, center, radius, cv::Scalar(0, 0, 255), 2); // You can adjust the color and thickness as needed.
    }
}

unsigned int GetTickCount()
{
        struct timeval tv;
        if(gettimeofday(&tv, NULL) != 0)
                return 0;
        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}