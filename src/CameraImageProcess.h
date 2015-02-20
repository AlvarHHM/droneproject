#ifndef CAMERAIMAGEPROCESS_H
#define CAMERAIMAGEPROCESS_H
#pragma once
#include <iostream>

// ROS includes.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ImageProcess.h"
#include <sys/time.h>
#include <boost/thread.hpp>

using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;

class StateData;

// CameraImageProcess class that processes images from the drone.
class CameraImageProcess: public ImageProcess {
public:
	// Default constructor for camera image processing.
	CameraImageProcess(Flight* flight, bool record, string& importModel);

	// Destructor for the CameraImageProcess class.
	~CameraImageProcess(void);

	// Converts the image from ROS Image to OpenCV type Mat.
	void ProcessImage(const sensor_msgs::ImageConstPtr& msg);

private:
	// Gets ROS image from the drone's camera feed.
	void GetImage(const sensor_msgs::ImageConstPtr& msg);

	// Initialises the video capture to file.
	void InitialiseVideoCapture(void);

	// Calculates the FPS of the video feed and image processing techniques.
	void CalculateFps(void);

	// Text to display to the window, holds the FPS counter.
	string textToDisplay;

	// Where to display the counter on the screen.
	cv::Point textCoords;

	// Counts the current FPS of the video feed.
	int fps;

	// Last and current times for FPS counter.
	time_t last, current;

	// Current image from drone.
	Mat currentImage;

	// States whether or not to record the video feed.
	bool recordVideo;
};

#endif
