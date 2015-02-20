#include "NavigationData.h"

using namespace std;
using namespace cv;

// NavigationData class constructor, holds the navigational data from the drone.
NavigationData::NavigationData(void) {

	this->StatusWindow = "Status Window";
	namedWindow(this->StatusWindow, WINDOW_AUTOSIZE);

	// TODO: Make this a relative path.
	this->statusBackground =
			imread(
					"/home/ardrone/ros_workspace/sandbox/drone_project/src/background.jpg",
					CV_LOAD_IMAGE_COLOR);
	this->statusFrame = statusBackground.clone();

	if (!this->statusFrame.data) {
		ROS_INFO("Couldn't find background.");
	}

	this->batteryLevel = 0;
	this->state = 0;
	this->altitude = 0;
	this->temperature = 0;
	this->windSpeed = 0;
	this->windAngle = 0;
	this->rotationX = 0;
	this->rotationY = 0;
	this->rotationZ = 0;
	this->velocityX = 0;
	this->velocityY = 0;
	this->velocityZ = 0;
	this->accelerationX = 0;
	this->accelerationY = 0;
	this->accelerationZ = 0;

	// Set the initial and current FPS to 0.
	this->fps = 0;
	this->currentFps = 0;

	// Set the beginning last and current time.
	time(&this->last);
	time(&this->current);
}

// NavigationData class destructor.
NavigationData::~NavigationData(void) {
	destroyWindow(this->StatusWindow);
}

// Process the navigation data from the drone.
void NavigationData::ProcessNavigationData(
		const ardrone_autonomy::Navdata& ardroneData) {
	// Set drone data in the NavigationData class.
	this->BatteryLevel(ardroneData.batteryPercent);
	this->State(ardroneData.state);
	this->Altitude(ardroneData.altd);
	this->Temperature(ardroneData.temp);
	this->WindSpeed(ardroneData.wind_speed);
	this->WindAngle(ardroneData.wind_angle);
	this->RotationX(ardroneData.rotX);
	this->RotationY(ardroneData.rotY);
	this->RotationZ(ardroneData.rotZ);
	this->VelocityX(ardroneData.vx);
	this->VelocityY(ardroneData.vy);
	this->VelocityZ(ardroneData.vz);
	this->AccelerationX(ardroneData.ax);
	this->AccelerationY(ardroneData.ay);
	this->AccelerationZ(ardroneData.az);

	fps++;

	if (difftime(this->current, this->last) > 1.0) {
		// A second has elapsed, set the current fps.
		this->currentFps = this->fps;

		// Reset the fps counter.
		this->fps = 0;

		// Set the last timer to the current.
		this->last = this->current;
	}

	// Update the current time.
	time(&this->current);

	// Display drone status text to the window.
	this->DisplayDroneStatus();

	this->ShowWindow();
}

// Shows the window.
void NavigationData::ShowWindow() {
	imshow(this->StatusWindow, this->statusFrame);
	waitKey(3);
	this->statusFrame.release();
}

// Processes the drone's current status to to displayed the status window.
void NavigationData::DisplayDroneStatus(void) {
	// Reset the background image.
	this->statusFrame = statusBackground.clone();

	// Write text to the frame.
	this->DisplayText(this->BatteryLevel(), "Battery Level", 1);
	this->DisplayText(this->State(),
			"State: " + this->getStateText(this->State()), 2);
	this->DisplayText(this->Altitude(), "Altitude", 3);
	this->DisplayText(this->Temperature(), "Temperature", 4);
	this->DisplayText(this->WindSpeed(), "Wind Speed", 5);
	this->DisplayText(this->WindAngle(), "Wind Angle", 6);
	this->DisplayText(this->RotationX(), "Rotation X", 8);
	this->DisplayText(this->RotationY(), "Rotation Y", 9);
	this->DisplayText(this->RotationZ(), "Rotation Z", 10);
	this->DisplayText(this->VelocityX(), "Velocity X", 12);
	this->DisplayText(this->VelocityY(), "Velocity  Y", 13);
	this->DisplayText(this->VelocityZ(), "Velocity Z", 14);
	this->DisplayText(this->AccelerationX(), "Acceleration X", 16);
	this->DisplayText(this->AccelerationY(), "Acceleration Y", 17);
	this->DisplayText(this->AccelerationZ(), "Acceleration Z", 18);
	this->DisplayText(this->currentFps, "FPS", 19);
}

// Overload of the DisplayText to convert int to float.
void NavigationData::DisplayText(int value, string description, int x) {
	this->DisplayText(static_cast<float>(value), description, x);
}

// Displays the text to the status window.
void NavigationData::DisplayText(float value, string description, int x) {
	cv::Point textCoords(20, 15 * x);
	string textToDisplay = description + ": " + this->NumberToString(value);
	cv::putText(this->statusFrame, textToDisplay, textCoords,
			FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1, 8);
}

string NavigationData::NumberToString(float num) {
	ostringstream buff;
	//buff << fixed << setprecision(2) << num;
	buff << num;
	return buff.str();
}

// Sets the drone's battery level.
void NavigationData::BatteryLevel(float batteryLevel) {
	this->batteryLevel = batteryLevel;
}

// Gets the drone's battery level.
float NavigationData::BatteryLevel(void) {
	return this->batteryLevel;
}

// Sets the state of the drone.
void NavigationData::State(int state) {
	this->state = state;
}

// Gets the string of the state number.
std::string NavigationData::getStateText(int state) {
	switch (state) {
	case 0:
		return "Unknown";
		break;
	case 1:
		return "Initiated";
		break;
	case 2:
		return "Landed";
		break;
	case 3:
	case 7:
		return "Flying";
		break;
	case 4:
		return "Hovering";
		break;
	case 5:
		return "Test";
		break;
	case 6:
		return "Taking Off";
		break;
	case 9:
		return "Looping";
		break;
	default:
		return "Unknown State";
		break;
	}

	return "Unknown";
}

// Gets the state of the drone.
int NavigationData::State(void) {
	return this->state;
}

// Sets the altitude of the drone.
void NavigationData::Altitude(int altitude) {
	this->altitude = altitude;
}

// Gets the altitude of the drone.
int NavigationData::Altitude(void) {
	return this->altitude;
}

// Sets the temperature of the drone.
void NavigationData::Temperature(int temperature) {
	this->temperature = temperature;
}

// Gets the temperature of the drone.
int NavigationData::Temperature(void) {
	return this->temperature;
}

// Sets the estimated wind speed about the drone.
void NavigationData::WindSpeed(float windSpeed) {
	this->windSpeed = windSpeed;
}

// Gets the estimated wind speed about the drone.
float NavigationData::WindSpeed(void) {
	return this->windSpeed;
}

// Sets the estimated wing angle about the drone.
void NavigationData::WindAngle(float wingAngle) {
	this->windAngle = windAngle;
}

// Gets the estimated wing angle about the drone.
float NavigationData::WindAngle(void) {
	return this->windAngle;
}

// Sets the x-axis rotation of the drone.
void NavigationData::RotationX(float rotationX) {
	this->rotationX = rotationX;
}

// Gets the x-axis rotation of the drone.
float NavigationData::RotationX(void) {
	return this->rotationX;
}

// Sets the y-axis rotation of the drone.
void NavigationData::RotationY(float rotationY) {
	this->rotationY = rotationY;
}

// Gets the y-axis rotation of the drone.
float NavigationData::RotationY(void) {
	return this->rotationY;
}

// Sets the z-axis rotation of the drone.
void NavigationData::RotationZ(float rotationZ) {
	this->rotationZ = rotationZ;
}

// Gets the z-axis rotation of the drone.
float NavigationData::RotationZ(void) {
	return this->rotationZ;
}

// Sets the x-axis velocity of the drone.
void NavigationData::VelocityX(float velocityX) {
	this->velocityX = velocityX;
}

// Gets the x-axis velocity of the drone.
float NavigationData::VelocityX(void) {
	return this->velocityX;
}

// Sets the y-axis velocity of the drone.
void NavigationData::VelocityY(float velocityY) {
	this->velocityY = velocityY;
}

// Gets the y-axis velocity of the drone.
float NavigationData::VelocityY(void) {
	return this->velocityY;
}

// Sets the z-axis velocity of the drone.
void NavigationData::VelocityZ(float velocityZ) {
	this->velocityZ = velocityZ;
}

// Gets the z-axis velocity of the drone.
float NavigationData::VelocityZ(void) {
	return this->velocityZ;
}

// Sets the x-axis acceleration of the drone.
void NavigationData::AccelerationX(float accelerationX) {
	this->accelerationX = accelerationX;
}

// Gets the x-axis acceleration of the drone.
float NavigationData::AccelerationX(void) {
	return this->accelerationX;
}

// Sets the y-axis acceleration of the drone.
void NavigationData::AccelerationY(float accelerationY) {
	this->accelerationY = accelerationY;
}

// Gets the y-axis acceleration of the drone.
float NavigationData::AccelerationY(void) {
	return this->accelerationY;
}

// Sets the z-axis acceleration of the drone.
void NavigationData::AccelerationZ(float accelerationZ) {
	this->accelerationZ = accelerationZ;
}

// Gets the z-axis acceleration of the drone.
float NavigationData::AccelerationZ(void) {
	return this->accelerationZ;
}
