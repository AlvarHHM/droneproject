#include "Flight.h"

using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;

// Flight class constructor, takes in the node.
Flight::Flight(NodeHandle& node) {
	ROS_INFO("Initialising flight.");
	this->nodeHandle = node;

	// Disable flying.
	// TODO: Change this when flying the real drone.
	this->flightAllowed = false;
	this->firstCommand = true;

	// Set the commands for take off, landing and resetting.
	this->takeOff = this->nodeHandle.advertise<Empty>("/ardrone/takeoff", 1);
	this->land = this->nodeHandle.advertise<Empty>("/ardrone/land", 1);
	this->reset = this->nodeHandle.advertise<Empty>("/ardrone/reset", 1);
	this->twist = this->nodeHandle.advertise<Twist>("/cmd_vel", 1);

	// Set the loopRate;
	this->loopRate = new Rate(50);

	// Init middle X and Y.
	this->middleX = 0;
	this->middleY = 0;

	// Set hover coordinates.
	this->hover.linear.x = 0.0;
	this->hover.linear.y = 0.0;
	this->hover.linear.z = 0.0;
	this->hover.angular.x = 0.0;
	this->hover.angular.y = 0.0;
	this->hover.angular.z = 0.0;

	// Set the initial flight commands.
	this->flightCommand.linear.x = 0.0;
	this->flightCommand.linear.y = 0.0;
	this->flightCommand.linear.z = 0.0;
	this->flightCommand.angular.x = 0.0;
	this->flightCommand.angular.y = 0.0;
	this->flightCommand.angular.z = 0.0;

	gettimeofday(&this->currentTime, NULL);
	this->lastTime = 0;
	this->currentMilliseconds = 0;
	this->timeDifference = 0;
	this->prop = 10 / 2.2;
	// 25
}

// Destructor for the Flight class.
Flight::~Flight(void) {
	// Destroy flight class.

	delete this->loopRate;
	delete this->currentBoundingBox;
	delete this->originalBoundingBox;
	delete this->lowerBoundingBox;
	delete this->higherBoundingBox;
}

// Converts an int to string.
template<typename T>
string IntToString(T number) {
	ostringstream stream;
	stream << number;
	return stream.str();
}

// Sends the flight command to the drone.
void Flight::SendFlightCommand(void) {
	// Send the calculated movement message to the drone.
	if(this->LinearX() + this->LinearY() + this->LinearZ() + this->AngularZ() != 0.0) {
		ROS_INFO("LinearX: %.3f, LinearY: %.3f AngularZ: %.3f", this->LinearX(),
				this->LinearY(), this->AngularZ());
	}
	this->twist.publish(this->flightCommand);

	this->SpinAndSleep();
}

// Emergency stops the drone's flight.
void Flight::EmergencyStop(void) {
	this->flightAllowed = false;
	this->reset.publish(this->emptyMessage);
	ROS_INFO("Stopped");
}

// Starts the target-track-flight loop.
void Flight::FlightAllowed(bool allowed) {
	this->flightAllowed = allowed;

	if (!this->flightAllowed) {
		// If flight is not allowed, send the hover command to the drone.
		this->Hover();
		this->SpinAndSleep();
	}
}

// Sends the hover command to the drone.
void Flight::Hover(void) {
	this->twist.publish(this->hover);
}

// Sends the spinOnce and loop rate sleep to the drone.
void Flight::SpinAndSleep(void) {
	spinOnce();
	this - loopRate->sleep();
}

// Returns if flight is allowed or not.
bool Flight::FlightStatus(void) {
	return this->flightAllowed;
}

// Resets the flight.
void Flight::Reset(void) {
	// TODO: Reset particle filtering.
	this->EmergencyStop();
}

// Sends the land command to the drone.
void Flight::Land(void) {
	this->Hover();
	this->land.publish(this->emptyMessage);

	ROS_INFO("Landing");

	this->SpinAndSleep();
}

// Sends the take off command to the drone.
void Flight::TakeOff(void) {
	this->takeOff.publish(this->emptyMessage);
	this->Hover();

	ROS_INFO("Taking Off");

	this->SpinAndSleep();
}

// Sets the 3 different axis values.
void Flight::SetHoverValues(double x, double y, double z, double angularZ) {
	this->LinearX(x);
	this->LinearY(y);
	this->LinearZ(z);
	this->AngularZ(angularZ);
}

// Sets the angular Z for turning the drone left and right.
void Flight::AngularZ(double z) {
	this->flightCommand.angular.z = z;
}

// Gets the angular Z, used for turning the drone left and right.
double Flight::AngularZ(void) {
	return this->flightCommand.angular.z;
}

// Sets the linear X.
void Flight::LinearX(double x) {
	this->flightCommand.linear.x = x;
}

// Gets the linear X.
double Flight::LinearX(void) {
	return this->flightCommand.linear.x;
}

// Sets the linear Y.
void Flight::LinearY(double y) {
	this->flightCommand.linear.y = y;
}

// Gets the linear Y.
double Flight::LinearY(void) {
	return this->flightCommand.linear.y;
}

// Sets the linear Z.
void Flight::LinearZ(double z) {
	this->flightCommand.linear.z = z;
}

// Gets the linear Z.
double Flight::LinearZ(void) {
	return this->flightCommand.linear.z;
}

Rect* Flight::OriginalBoundingBox(void) {
	return this->originalBoundingBox;
}

void Flight::drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
								  cv::Scalar &color, int thickness, int lineType) {
	const double PI = 3.1415926;
	Point arrow;
	double angle = atan2((double) (pStart.y - pEnd.y), (double) (pStart.x - pEnd.x));
	line(img, pStart, pEnd, color, thickness, lineType);
	arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
	arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
	line(img, pEnd, arrow, color, thickness, lineType);
	arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
	arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
	line(img, pEnd, arrow, color, thickness, lineType);
}
