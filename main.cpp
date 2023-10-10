
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

#include "Line.hpp"
#include "Coordinate.hpp"
#include "Vector.hpp"
#include "Corner.hpp"
#include "Puck.hpp"
#include "AirHockeyTable.hpp"
#include "MatDrawFunctions.hpp"
#include "StmCommunicator.hpp"
#include "GameState.hpp"

// A BUNCH OF REAL SPACE PARAMETERS FOR THE SYSTEM
/*
#define ENVIRONMENT_WIDTH 1760
#define ENVIROMENT_HEIGHT 990
#define TABLE_WIDTH 1620
#define TABLE_HEIGHT 750
#define TABLE_CURVE_RADIUS 120
#define PUCK_RADIUS 26
#define MALLET_RADIUS 36
*/

#define CAM_INPUT_WIDTH 1280
#define CAM_INPUT_HEIGHT 720
#define CAM_INPUT_DEVICE "/dev/video0"

#define CAM_RESCALED_WIDTH 640
#define CAM_RESCALED_HEIGHT 360

#define TABLE_X_BOUNDARY_MIN 2000
#define TABLE_X_BOUNDARY_MAX 15100
#define TABLE_Y_BOUNDARY_MIN 2000
#define TABLE_Y_BOUNDARY_MAX 17300

//#define CONNECT_TO_BOARD

int main() {
    std::cout.flush();
    StmCommunicator stmComms = StmCommunicator();
    bool connected = false;
    #ifdef CONNECT_TO_BOARD
    connected = stmComms.connect();
    if (!connected){
        std::cout << "ERROR: Connection could not be established. Signals will not be sent to board" << std::endl;
    }
    #endif

    // Used to access image through V4L2 helper library
	unsigned char* ptr_cam_frame;
	int bytes_used;

    // Used to calculate FPS statistics
    unsigned int start, end;

    // Set the GPU to use
    cv::cuda::setDevice(0);

    // Set up a window to display a live feed
    cv::namedWindow("TRAHT_Vision");

    // INITIALISE THE V4L2 HELPER LIBRARY
    if (helper_init_cam(CAM_INPUT_DEVICE, CAM_INPUT_WIDTH, CAM_INPUT_HEIGHT, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) {
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

    // Set up the circle detector. Currently set up for green circle detection
    float dp = 1.75;
    float minDist = 10;
    int cannyThreshold = 300;
    int votesThreshold = 30;
    int minRadius = 8;
    int maxRadius = 14;
    int maxQuantity = 1;
    cv::Ptr<cv::cuda::HoughCirclesDetector> houghCircleDetectorGreen = cv::cuda::createHoughCirclesDetector(
        dp, minDist, cannyThreshold, votesThreshold, minRadius, maxRadius, maxQuantity
    );
    cv::Ptr<cv::cuda::HoughCirclesDetector> houghCircleDetectorRed = cv::cuda::createHoughCirclesDetector(
        dp, minDist, cannyThreshold, votesThreshold, minRadius, maxRadius
    );

    // The image will be rescaled to this resolution
    cv::Size rescaledSize(CAM_RESCALED_WIDTH, CAM_RESCALED_HEIGHT);

    GameState gameStateInstance(rescaledSize);

    // Instantiate all of the temporary variables we need.
    cv::Mat cpuFrame = cv::Mat(CAM_INPUT_HEIGHT, CAM_INPUT_WIDTH, CV_8UC2);
    cv::Mat cpuFrameBGR = cv::Mat(CAM_INPUT_HEIGHT, CAM_INPUT_WIDTH, CV_8UC3);
    cv::Mat cpuFrameResized = cv::Mat(rescaledSize.width, rescaledSize.height, CV_8UC3);
    cv::Mat undistortedImage = cv::Mat(rescaledSize.width, rescaledSize.height, CV_8UC3);

    cv::cuda::GpuMat gpuFrame = cv::cuda::GpuMat(rescaledSize.width, rescaledSize.height, CV_8UC3);
    cv::cuda::GpuMat gpuFrame_channels[3];
    cv::cuda::GpuMat detectedGreenCirclesGPU;
    cv::cuda::GpuMat detectedRedCirclesGPU;

    std::vector<cv::Vec3f> detectedGreenCircles;
    std::vector<cv::Vec3f> detectedRedCircles;

    // BGR
    // A lot of green
    // little red
    // little blue
    cv::Scalar lowerBound(0, 50, 0);   // Lower bound for bright green (B, G, R)
    cv::Scalar upperBound(220, 255, 220); // Upper bound for bright green (B, G, R)
    cv::Mat gpuMask;

    std::cout << "Finished configuration, hit space to start game..." << std::endl;

    if (connected){
        while(!(cv::waitKey(0) == 32)) sleep(0.01); // wait forever on space key (From ascii table)
        stmComms.enableBoard();
    }

    std::cout << "Game Started" << std::endl;

    sleep(3);

    Coordinate dummyTargetCoordinate(7500, 7500);
    if (connected) {
        stmComms.setCoordinate(dummyTargetCoordinate);
    }

    sleep(3);
    
    dummyTargetCoordinate = Coordinate(3000, 3000);
    if (connected) {
        stmComms.setCoordinate(dummyTargetCoordinate);
    }

    while (1) {
        // Used for getting FPS counter
        start = GetTickCount();

        // on escape key in window, exit the program
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

        // Run Image Processing to get base processable image
        cv::cvtColor(cpuFrame, cpuFrameBGR, cv::COLOR_YUV2BGR_UYVY);
        cv::resize(cpuFrameBGR, cpuFrameResized, rescaledSize, cv::INTER_LINEAR);
        cv::undistort(cpuFrameResized, undistortedImage, cameraMatrix, distCoeffs);

        gpuFrame = cv::cuda::GpuMat(undistortedImage);
        
        // Find Circles
        if (1) {
            gaussianFilter->apply(gpuFrame, gpuFrame);
            //cv::inRange(undistortedImage, lowerBound, upperBound, gpuMask);
            //gpuFrame.setTo(cv::Scalar(0, 0, 0), cv::cuda::GpuMat(gpuMask));
            //gpuFrame.download(undistortedImage);
            cv::cuda::split(gpuFrame, gpuFrame_channels);
            houghCircleDetectorGreen->detect(gpuFrame_channels[1], detectedGreenCirclesGPU);
            houghCircleDetectorRed->detect(gpuFrame_channels[2], detectedRedCirclesGPU);
        }


        // Draw Circles
        // We only want to download the circles GPU_MAT if circles are detected, otherwise an error will be thrown. 
        if (detectedGreenCirclesGPU.empty()) { 
            gameStateInstance.registerLostPuck();
        }
        else {
            detectedGreenCirclesGPU.download(detectedGreenCircles);
            gameStateInstance.updatePuckPosition(detectedGreenCircles[0]);
            gameStateInstance.greenPuck.draw(undistortedImage);
        }

        if (!detectedRedCirclesGPU.empty()) {
            detectedRedCirclesGPU.download(detectedRedCircles);
            //drawDetectedCircles(undistortedImage, detectedRedCircles);
        }

        gameStateInstance.updateLogic(undistortedImage);

        // Draw the hockey table outline to the display.
        gameStateInstance.pixelSpaceTable.draw(undistortedImage);

        // DEBUGGING
        //circle(undistortedImage, cv::Point(10, 10), 5, cv::Scalar(255, 255, 0), 2);
        
        // Update display
        imshow("TRAHT_Vision", undistortedImage);

        helper_release_cam_frame();

        // FPS calculations
        end = GetTickCount();
		//std::cout << 1000/(end-start) << "fps" << std::endl;
    }

    stmComms.disconnect();

    // When the program exits, uninitialise the V4L2 helper library.
    if (helper_deinit_cam() < 0)
	{
		return EXIT_FAILURE;
	}

    return 1;
}
