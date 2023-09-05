#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/time.h>
#include <cstdlib>
#include "v4l2_helper.h"
#include <unistd.h>

using namespace std;
using namespace cv;

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

int main() {
    unsigned int width = 1280;
    unsigned int height = 720;
	const char* videodev = "/dev/video0";
	unsigned char* ptr_cam_frame;
	int bytes_used;
    unsigned int start, end, fps = 0;

    namedWindow("TRAHT_Vision");
    
    if (helper_init_cam(videodev, width, height, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) {
        cout << "Error: Failed to initialise camera.\n";
        return 0;
    }
    
    Mat cpuFrame = Mat(height, width, CV_8UC2);

    Mat cameraMatrix, distCoeffs;
    FileStorage fs2("./calibration_params.yml", FileStorage::READ);
    fs2["camera_matrix"] >> cameraMatrix;
    fs2["distortion_coefficients"] >> distCoeffs;
    fs2.release();
    
    start = GetTickCount();
    while (1) {
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
        
        imshow("TRAHT_Vision", cpuFrame_Blurred);

        helper_release_cam_frame();
        
        fps++;
		end = GetTickCount();
		if ((end - start) >= 1000) {
			cout << "fps = " << fps << endl;
			fps = 0;
			start = end;
		}
    }

    if (helper_deinit_cam() < 0)
	{
		return EXIT_FAILURE;
	}

    return 1;
}
