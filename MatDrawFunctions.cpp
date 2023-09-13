
#include "MatDrawFunctions.hpp"

#define COLOR_PURPLE cv::Scalar(255,0,255)

void drawBorderLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_PURPLE, thickness, cv::LINE_4);
}

void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles){
    for (const cv::Vec3f& circleInstance : circles) {
        cv::Point center(cvRound(circleInstance[0]), cvRound(circleInstance[1]));
        int radius = cvRound(circleInstance[2]);
        circle(image, center, radius, cv::Scalar(0, 0, 255), 2); // You can adjust the color and thickness as needed.
    }
}
