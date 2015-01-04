#include <iostream>

// ROS includes.
#include <ros/ros.h>

// OpenCV includes.
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include "ImageProcess.h"
#include "VideoImageProcess.h"
#include "CameraImageProcess.h"
#include "NavigationData.h"


// Particle Filter algorithm includes.
#include "Selector.h"
#include "State.h"
#include <unistd.h> // For getopt

using namespace cv;
using namespace std;
using namespace ros;

class DroneProject
{
	// Node Handle.
	NodeHandle node;

	// Image transport varible.
	image_transport::ImageTransport imageTransport;

	// Subscriber to the video feed from the drone.
	image_transport::Subscriber imageSub;

	// Subscriber for drone data..
	ros::Subscriber droneData;

	// Processes the navigation data from the drone.
	NavigationData* navigationData;

	// Uses the camera from the drone to do processing.
	CameraImageProcess* cameraImageProcess;

	// Uses a video input to do processing.
	VideoImageProcess* videoImageProcess;

public:
	// Default constructor, creates the image subscriptions/advertisements.
	DroneProject(string videoName, bool record, string& importModel)
	: imageTransport(node)
	{
		if (videoName != "")
		{
			this->videoImageProcess = new VideoImageProcess(node, videoName, importModel);
			this->videoImageProcess->ProcessVideo();
		}
		else
		{
			/*
			// Initialize the navigation data.
			this->navigationData = new NavigationData();

			// Get data from drone.
			droneData = node.subscribe("ardrone/navdata", 1, &NavigationData::ProcessNavigationData, this->navigationData);
			 */
			this->cameraImageProcess = new CameraImageProcess(node, record, importModel);
			imageSub = imageTransport.subscribe("ardrone/front/image_raw", 1, &CameraImageProcess::ProcessImage, this->cameraImageProcess);
		}
	}

	~DroneProject()
	{
		delete this->navigationData;
		delete this->cameraImageProcess;
		delete this->videoImageProcess;
	}
};

void parseCommandLine(int argc, char** argv, string& videoName, bool& record, string& importModel)
{
	int command = 0;
	extern int opterr;
	opterr = 0;

	while ((command = getopt(argc, argv, ":rhf:m:v")) != -1)
	{
		switch(command)
		{
			case 'r':
				record = true;
				break;
			case 'm':
				// Read model from file.
				importModel = optarg;
				break;
			case 'v':
				// Read video from file.
				videoName = argv[optind];
				break;
			default:
				cerr << "Invalid command; Commands are: " << endl;
				cerr << "\t-r records the video feed." << endl;
				cerr << "\t [input file] states what video to read from file." << endl;
				exit(1);
		}
	}
}

int main(int argc, char * argv[])
{
	// Initialize ROS.
	init(argc, argv, "Drone_Tracking");

	string videoName = "";
	bool record = false;
	string importModel = "";
	parseCommandLine(argc, argv, videoName, record, importModel);

	//Image process from video.
	DroneProject droneVideo(videoName, record, importModel);

	ros::spin();
	return 0;
}
