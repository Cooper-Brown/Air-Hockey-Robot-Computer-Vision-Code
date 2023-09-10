
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/cudaarithm.hpp>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

/*
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Types.h>
#include <vpi/Image.h>
#include <vpi/algo/Undistort.h>
*/

#include <iostream>
#include <vector>
#include <sys/time.h>
#include <cstdlib>
#include "v4l2_helper.h"
#include <unistd.h>

using namespace std;
using namespace cv;

#define ENVIRONMENT_WIDTH 1760
#define ENVIROMENT_HEIGHT 990
#define TABLE_WIDTH 1620
#define TABLE_HEIGHT 750
#define TABLE_CURVE_RADIUS 120
#define PUCK_RADIUS 26
#define MALLET_RADIUS 36

#define COLOR_PURPLE cv::Scalar(255,0,255)

class Coordinate {
    public:
        float x;
        float y;
        Coordinate () {
            x = 0;
            y = 0;
        }
        Coordinate (float xIn, float yIn) {
            x = xIn;
            y = yIn;
        }
};

class Line {
    public:
        Coordinate p1;
        Coordinate p2;
        Line () {
            p1 = Coordinate();
            p2 = Coordinate();
        }
        Line (Coordinate p1In, Coordinate p2In) {
            p1 = p1In;
            p2 = p2In;
        }
};

class Corner {
    public:
        Coordinate center;
        float radius;
        Corner() {
            center = Coordinate();
            radius = 0;
        }
        Corner(Coordinate centerIn, float radiusIn) {
            center = centerIn;
            radius = radiusIn;
        }
};

class AirHockeyTable {
    public:
        float x, y, r, xOffset, yOffset;
        Line topLine, bottomLine, leftLine, rightLine;
        Corner topLeftCorner, topRightCorner, bottomRightCorner, bottomLeftCorner;
        AirHockeyTable() {
            x = 0; // DEFAULT VALUES
            //AirHockeyTable();
        }
        AirHockeyTable(float xIn, float yIn, float rIn, float xOffsetIn, float yOffsetIn) {
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
        
};

class Vector {
    public:
        int xComponent, yComponent;
        Vector() {
            xComponent = 0;
            yComponent = 0;
        }
        Vector(int xComponentIn, int yComponentIn) {
            xComponent = xComponentIn;
            yComponent = yComponentIn;
        }
        int getLength() {
            return sqrt(pow(xComponent, 2) + pow(yComponent, 2));
        }
};

class Puck {
    public:
        Coordinate center;
        Vector velocity;

};

unsigned int GetTickCount()
{
        struct timeval tv;
        if(gettimeofday(&tv, NULL) != 0)
                return 0;
 
        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void drawDetectedCircles(Mat image, vector<Vec3f> circles){
    for (const Vec3f& circleInstance : circles) {
        Point center(cvRound(circleInstance[0]), cvRound(circleInstance[1]));
        int radius = cvRound(circleInstance[2]);
        circle(image, center, radius, Scalar(0, 0, 255), 2); // You can adjust the color and thickness as needed.
    }
}

void drawBorderLine(Mat image, Line line) {
    cv::Point p1(line.p1.x, line.p1.y);
    cv::Point p2(line.p2.x, line.p2.y);
    int thickness = 2;
    cv::line(image, p1, p2, COLOR_PURPLE, thickness, cv::LINE_4);
}

int main() {
    unsigned int width = 1280;
    unsigned int height = 720;
	const char* videodev = "/dev/video0";
	unsigned char* ptr_cam_frame;
	int bytes_used;
    unsigned int start, end;

    cv::cuda::setDevice(0);
    //cv::cuda::checkCudaErrors(cudaGetLastError());

    namedWindow("TRAHT_Vision");
    
    if (helper_init_cam(videodev, width, height, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) {
        cout << "Error: Failed to initialise camera.\n";
        return 0;
    }
    
    // Get the parameters to undistort the images from file
    Mat cameraMatrix, distCoeffs;
    FileStorage fs2("./calibration_params_360p.yml", FileStorage::READ);
    fs2["camera_matrix"] >> cameraMatrix;
    fs2["distortion_coefficients"] >> distCoeffs;
    fs2.release();

    // Create a Gaussian filter kernel
    cv::Size kernelSize(5, 5);  // Adjust kernel size as needed
    double sigmaX = 2.0;        // Adjust sigmaX as needed
    double sigmaY = 2.0;        // Adjust sigmaY as needed
    cv::Ptr<cv::cuda::Filter> gaussianFilter = cv::cuda::createGaussianFilter(CV_8UC3, CV_8UC3, kernelSize, sigmaX, sigmaY);
    
    // The image will be rescaled to this resolution
    cv::Size rescaledSize(640, 360);

    // Parameters for the circle detector
    cv::Ptr<cv::cuda::HoughCirclesDetector> houghCircleDetectorI = cv::cuda::createHoughCirclesDetector(1.75, 10, 200, 40, 1, 20);

    // Instantiate all of the temporary variables we need, so they aren't in the loop.
    cv::Mat cpuFrame = cv::Mat(height, width, CV_8UC2);
    cv::Mat undistortedImage = cv::Mat(rescaledSize.width, rescaledSize.height, CV_8UC3);
    cv::Mat cpuFrameBlurred = cv::Mat(rescaledSize.width, rescaledSize.height, CV_8UC3);
    cv::Mat cpuFrameBGR = cv::Mat(height, width, CV_8UC3);
    cv::Mat cpuFrameConverted;
    cv::Mat cpuFrameResized = cv::Mat(rescaledSize.width, rescaledSize.height, CV_8UC3);;
    cv::cuda::GpuMat gpuFrameDownscaled = cv::cuda::GpuMat(rescaledSize.width, rescaledSize.height, CV_8UC1);
    cv::cuda::GpuMat gpuFrameBlurred = cv::cuda::GpuMat(rescaledSize.width, rescaledSize.height, CV_8UC1);
    cv::cuda::GpuMat gpuFrame;
    cv::cuda::GpuMat gpuFrame_channels[3];
    cv::cuda::GpuMat redChannelGPU;
    cv::cuda::GpuMat greenChannelGPU;

    cv::cuda::GpuMat detectedGreenCirclesGPU;
    cv::cuda::GpuMat detectedRedCirclesGPU;
    std::vector<cv::Vec3f> detectedGreenCircles;
    std::vector<cv::Vec3f> detectedRedCircles;
    cv::Mat detectedGreenCircles2;
    cv::Mat detectedRedCircles2;
    cout << "Finished configuration, starting loop..." << endl;

    while (1) {
        start = GetTickCount();

        // on escape key, exit the 
        if(waitKey(1) == 27) break;

        // Get the camera frame
        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0) {
            cout << "Error: Could not get an image frame.\n";
            return 0;
        }
        cpuFrame.data = ptr_cam_frame;
        if(cpuFrame.empty()) {
            cout << "Img load failed" << endl;
            return 0;
        }

        // Run Image Processing
        cv::cvtColor(cpuFrame, cpuFrameBGR, cv::COLOR_YUV2BGR_UYVY);
        cv::resize(cpuFrameBGR, cpuFrameResized, rescaledSize, INTER_LINEAR);
        cv::undistort(cpuFrameResized, undistortedImage, cameraMatrix, distCoeffs);
        
        gpuFrame = cv::cuda::GpuMat(undistortedImage);
        //cv::cuda::resize(gpuFrame, gpuFrameDownscaled, rescaledSize, INTER_LINEAR);
        // Apply Gaussian blur using the filter function
        gaussianFilter->apply(gpuFrame, gpuFrameBlurred);
        cv::cuda::split(gpuFrameBlurred, gpuFrame_channels);
        greenChannelGPU = gpuFrame_channels[1];
        redChannelGPU = gpuFrame_channels[2];
        houghCircleDetectorI->detect(greenChannelGPU, detectedGreenCirclesGPU);
        houghCircleDetectorI->detect(redChannelGPU, detectedRedCirclesGPU);

        // ERROR GETS THROWN IN THE NEXT TWO LINES IF NO CIRCLES CAN BE FOUND
        if (!detectedGreenCirclesGPU.empty()) {
            detectedGreenCirclesGPU.download(detectedGreenCircles);
            drawDetectedCircles(undistortedImage, detectedGreenCircles);
        }
        if (!detectedRedCirclesGPU.empty()) {
            detectedRedCirclesGPU.download(detectedRedCircles);
            drawDetectedCircles(undistortedImage, detectedRedCircles);
        }
        

        // TESTING
        /*
        Mat resultImage = undistortedImage.clone();
        // END TESTING

        Line l1(Coordinate(10, 10), Coordinate(500, 10));
        drawBorderLine(undistortedImage, l1);
        
        // TESTING LINE DETECTION FOR TABLE BOUNDARIES
        // Convert the image to grayscale
        Mat grayImage;
        cvtColor(resultImage, grayImage, COLOR_BGR2GRAY);

        // Apply Gaussian blur to reduce noise (optional)
        GaussianBlur(grayImage, grayImage, Size(5, 5), 0);

        int thresholdValue = 200;  // Adjust this value as needed
        threshold(grayImage, grayImage, thresholdValue, 255, THRESH_TOZERO);

        // Perform edge detection using the Canny edge detector
        Mat edges;
        Canny(grayImage, edges, 150, 250, 3); // works ok with 200, 200

        // Perform the Hough Line Transform to detect lines in the image
        //std::vector<Vec2f> lines;
        std::vector<Vec4i> lines;
        //HoughLines(edges, lines, 1, CV_PI / 180, 150);
        HoughLinesP(edges, lines, 1, CV_PI / 180, 180);

        // Draw detected lines on a copy of the original image
        for (size_t i = 0; i < lines.size(); i++) {
            
            //float rho = lines[i][0];
            //float theta = lines[i][1];
            //Point pt1, pt2;
            //double a = cos(theta), b = sin(theta);
            //double x0 = a * rho, y0 = b * rho;
            //pt1.x = cvRound(x0 + 1000 * (-b));
            //pt1.y = cvRound(y0 + 1000 * (a));
            //pt2.x = cvRound(x0 - 1000 * (-b));
            //pt2.y = cvRound(y0 - 1000 * (a));
            //line(resultImage, pt1, pt2, Scalar(255, 0, 0), 2, LINE_AA);
            
            line(resultImage, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 2, LINE_AA);
        }
        */

        /*
        Mat cpuFrame_BGR;
        cvtColor(cpuFrame, cpuFrame_BGR, COLOR_YUV2BGR_UYVY);
        Mat undistortedImage;
        undistort(cpuFrame_BGR, undistortedImage, cameraMatrix, distCoeffs);
        Mat cpuFrameR;
        Size rescaledSize(1024, 576);
        resize(undistortedImage, cpuFrameR, rescaledSize);
        Mat cpuFrame_Blurred;
        GaussianBlur(cpuFrameR, cpuFrame_Blurred, Size(5, 5), 0);
        Mat cpuFrameR_channels[3];
        split(cpuFrame_Blurred, cpuFrameR_channels);
        Mat greenChannel = cpuFrameR_channels[1];
        Mat redChannel = cpuFrameR_channels[2];
        Mat greenChannelHoughReady;
        greenChannel.convertTo(greenChannelHoughReady, CV_8U);
        Mat redChannelHoughReady;
        redChannel.convertTo(redChannelHoughReady, CV_8U);
        vector<Vec3f> detectedGreenCircles;
        HoughCircles(greenChannelHoughReady, detectedGreenCircles, cv::HOUGH_GRADIENT, 1.75, 10, 200, 40, 1, 20);
        vector<Vec3f> detectedRedCircles;
        HoughCircles(redChannelHoughReady, detectedRedCircles, cv::HOUGH_GRADIENT, 1.75, 10, 200, 40, 1, 20);
        //Mat cpuFrame_greyscale;
        //cvtColor(greenChannel, cpuFrame_greyscale, COLOR_GRAY2BGR);
        drawDetectedCircles(cpuFrame_Blurred, detectedGreenCircles);
        drawDetectedCircles(cpuFrame_Blurred, detectedRedCircles);
        */
        
        imshow("TRAHT_Vision", undistortedImage);

        helper_release_cam_frame();

        end = GetTickCount();
		cout << 1000/(end-start) << "fps" << endl;
    }

    if (helper_deinit_cam() < 0)
	{
		return EXIT_FAILURE;
	}

    return 1;
}
