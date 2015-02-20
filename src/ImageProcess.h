#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#pragma once

#include <iostream>

// ROS includes.
#include <ros/ros.h>

// TLD Includes.
#include "tld/TLD.h"

// OpenCV includes.
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include "NavigationData.h"
#include "Flight.h"
#include "State.h"

class StateData;
class TLD;

using namespace cv;
using namespace std;
using namespace ros;
using namespace tld;

// ImageProcess base class.
class ImageProcess {
public:
	// Constructor for the ImageProcess class.
	ImageProcess(Flight* flight, string& importModel, bool cameraOnly);

	// Destructor for the ImageProcess class.
	~ImageProcess(void);

	// Name of the window.
	static const char WINDOWNAME[];

	// TLD Predator image tracking algorithm.
	tld::TLD* tld;

	// Sets the current state data.
	void CurrentStateData(StateData* stateData);

	// Gets the current state data.
	StateData* CurrentStateData(void);

	// Processes the keyboard input.
	void ProcessKeyInput(int input);

	// Processes the current information to send flight commands to the drone.
	Flight* flight;

	// State data for the current state.
	StateData* currentStateData;

	// Current particle filter state.
	State* currentState;

	// Default number of particles.
	static const uint NUM_PARTICLES;

	// Frame from camera.
	Mat frame;

	// Writes the drone's feed to file.
	VideoWriter* writer;

	// Specifies whether the video writer has been initialized.
	bool writerInitialized;

	// Specifies whether TLD has a model from file.
	bool modelImported;

};

#endif
