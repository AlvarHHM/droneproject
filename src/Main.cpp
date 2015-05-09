#include <iostream>

// ROS includes.
#include <ros/ros.h>

// OpenCV includes.
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <ardrone_autonomy/CamSelect.h>

#include "ImageProcess.h"
#include "VideoImageProcess.h"
#include "CameraImageProcess.h"
#include "NavigationData.h"
#include "Flight.h"
#include "ObstacleTestFlight.h"
#include "BottomTrackFlight.h"
#include "FrontTrackFlight.h"
#include "WanderFlight.h"

// Particle Filter algorithm includes.
#include "Selector.h"
#include "State.h"
#include <unistd.h> // For getopt
#include "WanderFlight.h"

using namespace cv;
using namespace std;
using namespace ros;

class DroneProject {
    // Node Handle.
    NodeHandle node;

    // Image transport varible.
    image_transport::ImageTransport imageTransport;

    // Subscriber to the video feed from the drone.
    image_transport::Subscriber imageSub;

    // Subscriber for drone data..
    ros::Subscriber droneData;

    // Processes the navigation data from the drone.
    NavigationData *navigationData;

    // Uses the camera from the drone to do processing.
    CameraImageProcess *cameraImageProcess;

    // Uses a video input to do processing.
    VideoImageProcess *videoImageProcess;

    Flight *flight;

public:

    enum FlightMode {
        FRONT = 'f', BOTTOM = 'b', OBSTACLE_TEST = 'o', WANDER = 'W'
    };

    // Default constructor, creates the image subscriptions/advertisements.
    DroneProject(FlightMode mode, string videoName, bool record,
                 string &importModel) :
            imageTransport(node) {
        ros::ServiceClient sc
                = node.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");
        ardrone_autonomy::CamSelect camsrv;
        switch (mode) {
            case FRONT:
                ROS_INFO("front");
                camsrv.request.channel = 0;
                this->flight = new FrontTrackFlight(this->node);
                break;
            case BOTTOM:
                ROS_INFO("bottom");
                camsrv.request.channel = 1;
                this->flight = new BottomTrackFlight(this->node);
                break;
            case OBSTACLE_TEST:
                ROS_INFO("Obstacle Test");
                camsrv.request.channel = 0;
                this->flight = new ObstacleTestFlight(this->node);
                break;
            case WANDER:
                ROS_INFO("Wander");
                // Initialize the navigation data.
                this->navigationData = new NavigationData();
                // Get data from drone.
                droneData = node.subscribe("ardrone/navdata", 1, &NavigationData::ProcessNavigationData,
                                           this->navigationData);
                camsrv.request.channel = 0;
                this->flight = new WanderFlight(this->node, this->navigationData);
                break;
            default:
                camsrv.request.channel = 0;
                this->flight = new FrontTrackFlight(this->node);
                break;
        }
        sc.call(camsrv);

        if (videoName != "") {
            this->videoImageProcess = new VideoImageProcess(&*flight, videoName,
                                                            importModel);
            this->videoImageProcess->ProcessVideo();
        } else {

            this->cameraImageProcess = new CameraImageProcess(flight, record,
                                                              importModel);
            imageSub = imageTransport.subscribe("ardrone/image_raw", 1,
                                                &CameraImageProcess::ProcessImage,
                                                this->cameraImageProcess);
        }
    }

    ~DroneProject() {
        delete this->navigationData;
        delete this->cameraImageProcess;
        delete this->videoImageProcess;
        delete this->flight;
    }
};

void parseCommandLine(int argc, char **argv, DroneProject::FlightMode &mode,
                      string &videoName, bool &record, string &importModel) {
    int command = 0;
    extern int opterr;
    opterr = 0;

    while ((command = getopt(argc, argv, ":bfworhf:m:v")) != -1) {
        cout << command;
        switch (command) {
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
            case 'f':
                mode = DroneProject::FRONT;
                break;
            case 'b':
                mode = DroneProject::BOTTOM;
                break;
            case 'o':
                mode = DroneProject::OBSTACLE_TEST;
                break;
            case 'w':
                mode = DroneProject::WANDER;
                break;
            default:
                cerr << "Invalid command; Commands are: " << endl;
                cerr << "\t-r records the video feed." << endl;
                cerr << "\t [input file] states what video to read from file."
                << endl;
                exit(1);
        }
    }
}

int main(int argc, char *argv[]) {
    // Initialize ROS.
    init(argc, argv, "Drone_Tracking");

    string videoName = "";
    bool record = false;
    string importModel = "";
    DroneProject::FlightMode angle = DroneProject::FRONT;
    parseCommandLine(argc, argv, angle, videoName, record, importModel);

    //Image process from video.
    DroneProject droneVideo(angle, videoName, record, importModel);

    ros::spin();
    return 0;
}
