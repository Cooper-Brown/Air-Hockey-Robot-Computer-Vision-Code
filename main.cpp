
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

#include "Coordinate.hpp"
#include "Line.hpp"
#include "Vector.hpp"
#include "Corner.hpp"
#include "Puck.hpp"
#include "AirHockeyTable.hpp"
#include "MatDrawFunctions.hpp"

#define ENVIRONMENT_WIDTH 1760
#define ENVIROMENT_HEIGHT 990
#define TABLE_WIDTH 1620
#define TABLE_HEIGHT 750
#define TABLE_CURVE_RADIUS 120
#define PUCK_RADIUS 26
#define MALLET_RADIUS 36

unsigned int GetTickCount()
{
        struct timeval tv;
        if(gettimeofday(&tv, NULL) != 0)
                return 0;
 
        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
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

    cv::namedWindow("TRAHT_Vision");
    
    if (helper_init_cam(videodev, width, height, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) {
        std::cout << "Error: Failed to initialise camera.\n";
        return 0;
    }
    
    // Get the parameters to undistort the images from file
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs2("./calibration_params_360p.yml", cv::FileStorage::READ);
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
    std::cout << "Finished configuration, starting loop..." << std::endl;

    float AHT_x = rescaledSize.width-40;
    float AHT_y = rescaledSize.height-80;
    float AHT_r = 40;
    float AHT_xOffset = (rescaledSize.width - AHT_x)/2;
    float AHT_yOffset = (rescaledSize.height - AHT_y)/2 + 5;
    AirHockeyTable pixelSpaceTable(AHT_x, AHT_y, AHT_r, AHT_xOffset, AHT_yOffset);

    while (1) {
        start = GetTickCount();

        // on escape key, exit the 
        if(cv::waitKey(1) == 27) break;

        // Get the camera frame
        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0) {
            std::cout << "Error: Could not get an image frame.\n";
            return 0;
        }
        cpuFrame.data = ptr_cam_frame;
        if(cpuFrame.empty()) {
            std::cout << "Img load failed" << std::endl;
            return 0;
        }

        // Run Image Processing
        cv::cvtColor(cpuFrame, cpuFrameBGR, cv::COLOR_YUV2BGR_UYVY);
        cv::resize(cpuFrameBGR, cpuFrameResized, rescaledSize, cv::INTER_LINEAR);
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

        pixelSpaceTable.draw(undistortedImage);
        
        imshow("TRAHT_Vision", undistortedImage);

        helper_release_cam_frame();

        end = GetTickCount();
		std::cout << 1000/(end-start) << "fps" << std::endl;
    }

    if (helper_deinit_cam() < 0)
	{
		return EXIT_FAILURE;
	}

    return 1;
}
