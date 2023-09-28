
#include "MatDrawFunctions.hpp"
#include <sys/time.h>

#define COLOR_PURPLE cv::Scalar(255,0,255)
#define COLOR_BLUE cv::Scalar(255,0,0)
#define COLOR_RED cv::Scalar(0,0,255)

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
// FUNCTION TAKEN FROM https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool getLineIntersection(Line line1, Line line2, Coordinate* intersectionPoint)
{
    float p0_x = line1.p1.x;    float p0_y = line1.p1.y;
    float p1_x = line1.p2.x;    float p1_y = line1.p2.y;
    float p2_x = line2.p1.x;    float p2_y = line2.p1.x;
    float p3_x = line2.p2.x;    float p3_y = line2.p2.y;

    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        if (intersectionPoint != NULL) {
            intersectionPoint->x = p0_x + (t * s1_x);
            intersectionPoint->y = p0_y + (t * s1_y);
        }
            
        return true;
    }

    return false; // No collision
}

bool getLineIntersection2(Line line1, Line line2, Coordinate* intersectionPoint){
    float x1 = line1.p1.x;         float y1 = line1.p1.y;
    float x2 = line1.p2.x;         float y2 = line1.p2.y;
    float x3 = line2.p1.x;         float y3 = line2.p1.y;
    float x4 = line2.p2.x;         float y4 = line2.p2.y;

    float denominator = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    // Check if the lines are parallel (denominator is close to 0)
    if (denominator == 0) {
        return false;
    }
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

    // Check if the intersection point is within the line segments
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        intersectionPoint->x = x1 + t * (x2 - x1);
        intersectionPoint->y = y1 + t * (y2 - y1);
        return true;
    }

    return false;
}

void drawBorderLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_PURPLE, thickness, cv::LINE_4);
}

void drawGoalLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_RED, thickness, cv::LINE_4);
}

void drawVelocityLine(cv::Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_BLUE, thickness, cv::LINE_8);
    //std::cout << line.p1.x << ":" << line.p1.y << " " << line.p2.x << ":" << line.p1.y << std::endl;
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

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
