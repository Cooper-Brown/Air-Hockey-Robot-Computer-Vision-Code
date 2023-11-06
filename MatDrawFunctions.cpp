
#include "MatDrawFunctions.hpp"
#include <sys/time.h>
#include <cmath>

#define COLOR_PURPLE cv::Scalar(255,0,255)
#define COLOR_BLUE cv::Scalar(255,0,0)
#define COLOR_RED cv::Scalar(0,0,255)

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
// FUNCTION TAKEN FROM https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
/*
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
*/

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

bool pointInQuadrant(Coordinate coordinate, Corner corner){
    Coordinate extremity1 = Coordinate(
        (corner.center.x + corner.radius * cos((M_PI/180.0)*corner.startAngle)),
        (corner.center.y + corner.radius * sin((M_PI/180.0)*corner.startAngle))
    );
    Coordinate extremity2 = Coordinate(
        (corner.center.x + corner.radius * cos((M_PI/180.0)*corner.endAngle)),
        (corner.center.y + corner.radius * sin((M_PI/180.0)*corner.endAngle))
    );

    float largeX = extremity2.x;
    float smallX = extremity1.x;
    float largeY = extremity2.y;
    float smallY = extremity1.y;
    if (extremity1.x > extremity2.x){
        largeX = extremity1.x;
        smallX = extremity2.x;
    }
    if (extremity1.y > extremity2.y){
        largeY = extremity1.y;
        smallY = extremity2.y;
    }

    //std::cout << smallX << " " << coordinate.x << " " << largeX << std::endl;
    //std::cout << smallY << " " << coordinate.y << " " << largeY << std::endl;

    if ((coordinate.x >= smallX) && (coordinate.x <= largeX) &&
        (coordinate.y >= smallY) && (coordinate.y <= largeY) )
    {
        return true;
    }
    return false;
}

bool getCornerIntersection(Line line1, Corner corner1, Coordinate* intersectionPoint){
    float x1 = line1.p1.x;         float y1 = line1.p1.y;
    float x2 = line1.p2.x;         float y2 = line1.p2.y;
    float px = x1;                 float py = y1;
    float rx = x2 - x1;            float ry = y2 - y1;

    float circleEquationXOffset = corner1.center.x;
    float circleEquationYOffset = corner1.center.y;

    //std::cout << "Circle Center x:" << circleEquationXOffset << " y:" << circleEquationYOffset << " R:" << corner1.radius << std::endl;
    //std::cout << "P:(" << px << " , " << py << ") R:(" << rx << " , " << ry << ")" << std::endl;

    float a = pow(rx,2) + pow(ry,2);
    float b = 2 * rx * (px - circleEquationXOffset) + 2 * ry * (py - circleEquationYOffset);
    float c = pow(px - circleEquationXOffset,2) + pow(py - circleEquationYOffset,2) - pow(corner1.radius-PUCK_RADIUS_PIXELS,2);
    
    float quadraticRoot = pow(b,2) - 4*a*c;
    if ((a == 0) || (quadraticRoot < 0)){
        return false;
    }
    
    float t1 = (-b + sqrt(quadraticRoot))/(2*a);
    float t2 = (-b - sqrt(quadraticRoot))/(2*a);

    //std::cout << "t1=" << t1 << " t2=" << t2 << std::endl;

    bool intersectionPoint1Exists = (t1 >= 0) && (t1 <= 1);
    bool intersectionPoint2Exists = (t2 >= 0) && (t2 <= 1);

    Coordinate intersectionPoint1;
    bool intersectionPoint1InQuadrant = false;
    if (intersectionPoint1Exists){
        intersectionPoint1 = Coordinate((px + t1 * rx), (py + t1 * ry));
        //std::cout << "Intersection 1 " << intersectionPoint1.x << " " << intersectionPoint1.y << std::endl;
        intersectionPoint1InQuadrant = pointInQuadrant(intersectionPoint1, corner1);
        //std::cout << "Point in quadrant:" << intersectionPoint1InQuadrant << std::endl;
    }

    Coordinate intersectionPoint2;
    bool intersectionPoint2InQuadrant = false;
    if (intersectionPoint2Exists){
        intersectionPoint2 = Coordinate((px + t2 * rx), (py + t2 * ry));
        //std::cout << "Intersection 2 " << intersectionPoint2.x << " " << intersectionPoint2.y << std::endl;
        intersectionPoint2InQuadrant = pointInQuadrant(intersectionPoint2, corner1);
        //std::cout << "Point in quadrant:" << intersectionPoint1InQuadrant << std::endl;
    }

    if (intersectionPoint1InQuadrant && intersectionPoint2InQuadrant){
        std::cout << "Corner-trajectory intersection at two points not handled. Prediction shearing will occur..." << std::endl;
    }
    else if (intersectionPoint1InQuadrant) {
        intersectionPoint->x = intersectionPoint1.x;
        intersectionPoint->y = intersectionPoint1.y;
        return true;
    }
    else if (intersectionPoint2InQuadrant){
        intersectionPoint->x = intersectionPoint2.x;
        intersectionPoint->y = intersectionPoint2.y;
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

void drawCorner(cv::Mat image, Corner corner){
    cv::ellipse(
        image, 
        cv::Point(corner.center.x,corner.center.y), 
        cv::Size(corner.radius, corner.radius), 
        0, 
        corner.startAngle,
        corner.endAngle,
        COLOR_PURPLE,
        2,
        cv::LINE_8
    );
}

void drawDetectedCircles(cv::Mat image, std::vector<cv::Vec3f> circles){
    for (const cv::Vec3f& circleInstance : circles) {
        cv::Point center(cvRound(circleInstance[0]), cvRound(circleInstance[1]));
        int radius = cvRound(circleInstance[2]);
        circle(image, center, radius, cv::Scalar(0, 0, 255), 2); // You can adjust the color and thickness as needed.
    }
}

unsigned int GetTickCount(){
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

int signOf(int number){
    if (number > 0){
        return 1;
    }
    else if (number == 0){
        return 0;
    }
    else{
        return -1;
    }
}
