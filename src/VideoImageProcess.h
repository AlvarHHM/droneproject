#ifndef VIDEOIMAGEPROCESS_H
#define VIDEOIMAGEPROCESS_H

#include <iostream>
#include "ImageProcess.h"
#include <boost/thread.hpp>

// OpenCV includes.
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class VideoImageProcess: public ImageProcess {
public:
	// VideoImageProcess class constructor.
	VideoImageProcess(Flight* flight, string videoName, string& importModel);

	// VideoImageProcess class destructor.
	~VideoImageProcess(void);

	// Processes the video feed.
	void ProcessVideo(void);

private:
	// Video capture for reading from video file.
	VideoCapture* capture;

	// Sets the bounding boxes for the test videos made.
	void SetBoundingBox(string videoName);
};

#endif
