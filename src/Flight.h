#ifndef FLIGHT_H
#define FLIGHT_H
#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "StateData.h"
#include "Pid.h"
#include <iostream>
#include <fstream>
#include <ostream>

using namespace cv;
using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

class Pid;
class StateData;

class Flight {
public:
	// Flight class constructor, takes in the node.
	Flight(NodeHandle& node);

	// Destructor for the Flight class.
	~Flight(void);

	// Process image data to convert into flight commands.
	virtual void ProcessFlight(StateData& stateData) = 0;

	// Emergency stops the drone's flight.
	void EmergencyStop(void);

	// Starts or stops the target-track-flight loop.
	void FlightAllowed(bool allowed);

	// Resets the flight.
	void Reset(void);

	// Sends the land command to the drone.
	void Land(void);

	// Sends the take off command to the drone.
	void TakeOff(void);

	// Sets the initial bounding box.
	virtual void InitialBoundingBox(Rect* boundingBox) = 0;

	// Returns if flight is allowed or not.
	bool FlightStatus(void);

	// Sets the angular Z for turning the drone left and right.
	void AngularZ(double z);

	// Gets the angular Z, used for turning the drone left and right.
	double AngularZ(void);

	// Sets the linear X.
	void LinearX(double x);

	// Gets the linear X.
	double LinearX(void);

	// Sets the linear Y.
	void LinearY(double y);

	// Gets the linear Y.
	double LinearY(void);

	// Sets the linear Z.
	void LinearZ(double z);

	// Gets the linear Z.
	double LinearZ(void);

	// Sends the hover command to the drone.
	void Hover(void);

	// Sends the spinOnce and loop rate sleep to the drone.
	void SpinAndSleep(void);

	// Sends the flight command to the drone.
	void SendFlightCommand(void);

	double prop;

	// Gets the original bounding box.
	Rect* OriginalBoundingBox(void);

	// The original bounding box for the tracked object.
	Rect* originalBoundingBox;

	// Middle value for X.
	int middleX;

	// Middle value for Y.
	int middleY;

//protected:

	// States whether it is the first flight command, if it is then PID cannot be done.
	bool firstCommand;

	// Calculates the movement value for the Y-axis.
	Pid* PidY;

	// Calculates the movement value for the Z-axis.
	Pid* PidZ;

	// Calculates the movement value for the X-axis.
	Pid* PidX;

	// Calculates the movement value for angular z.
	Pid* PidAngularZ;



	// Node handler.
	NodeHandle nodeHandle;

	// Take off command message.
	Publisher takeOff;

	// Land command message.
	Publisher land;

	// Reset command message.
	Publisher reset;

	// Twist publisher for commands.
	Publisher twist;

	// Empty drone message, used for commands such as land and takeoff.
	Empty emptyMessage;

	// Hover command.
	Twist hover;

	// Flight command to be sent to the drone.
	Twist flightCommand;

	// The current bounding box for the tracked object.
	Rect* currentBoundingBox;

	// Bounding box that takes into account the deadband for the higher end.
	Rect* higherBoundingBox;

	// Bounding box that takes into account the deadband for the lower end.
	Rect* lowerBoundingBox;

	// Loop rate for ROS.
	Rate* loopRate;

	// Last and current times for PID controller.
	timeval currentTime;

	double lastTime, currentMilliseconds, timeDifference;

	// Whether or not flight tracking is allowed.
	bool flightAllowed;

	// Sets the different axis linear values.
	void SetHoverValues(double x, double y, double z, double angularZ);

protected:

	void drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
									  cv::Scalar &color, int thickness, int lineType);
};

#endif
