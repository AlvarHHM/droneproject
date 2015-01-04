#ifndef NAVIGATIONDATA_H
#define NAVIGATIONDATA_H
#pragma once

#include <iostream>
#include <ros/console.h>
#include "ardrone_autonomy/Navdata.h"
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;

class NavigationData
{
	public:
		// NavigationData class constructor, holds the navigational data from the drone.
		NavigationData(void);

		// NavigationData class destructor.
		~NavigationData(void);

		// Process the navigation data from the drone.
		void ProcessNavigationData(const ardrone_autonomy::Navdata& ardroneData);

		// Shows the window.
		void ShowWindow(void);

		// Set the battery level.
		void BatteryLevel(float batteryLevel);

		// Gets the battery level.
		float BatteryLevel();

		// Sets the state of the drone.
		void State(int state);

		// Gets the state of the drone.
		int State();

		// Gets the string of the state number.
		std::string getStateText(int state);

		// Sets the altitude of the drone.
		void Altitude(int altitude);

		// Gets the altitude of the drone.
		int Altitude();

		// Sets the temperature of the drone.
		void Temperature(int temperature);

		// Gets the temperature of the drone.
		int Temperature();

		// Sets the estimated wind speed about the drone.
		void WindSpeed(float windSpeed);

		// Gets the estimated wind speed about the drone.
		float WindSpeed();

		// Sets the estimated wing angle about the drone.
		void WindAngle(float wingAngle);

		// Gets the estimated wing angle about the drone.
		float WindAngle();

		// Sets the x-axis rotation of the drone.
		void RotationX(float rotationX);

		// Gets the x-axis rotation of the drone.
		float RotationX();

		// Sets the y-axis rotation of the drone.
		void RotationY(float rotationY);

		// Gets the y-axis rotation of the drone.
		float RotationY();

		// Sets the z-axis rotation of the drone.
		void RotationZ(float rotationZ);

		// Gets the z-axis rotation of the drone.
		float RotationZ();

		// Sets the x-axis velocity of the drone.
		void VelocityX(float velocityX);

		// Gets the x-axis velocity of the drone.
		float VelocityX();

		// Sets the y-axis velocity of the drone.
		void VelocityY(float velocityY);

		// Gets the y-axis velocity of the drone.
		float VelocityY();

		// Sets the z-axis velocity of the drone.
		void VelocityZ(float velocityZ);

		// Gets the z-axis velocity of the drone.
		float VelocityZ();

		// Sets the x-axis acceleration of the drone.
		void AccelerationX(float accelerationX);

		// Gets the x-axis acceleration of the drone.
		float AccelerationX();

		// Sets the y-axis acceleration of the drone.
		void AccelerationY(float accelerationY);

		// Gets the y-axis acceleration of the drone.
		float AccelerationY();

		// Sets the z-axis acceleration of the drone.
		void AccelerationZ(float accelerationZ);

		// Gets the z-axis acceleration of the drone.
		float AccelerationZ();

		// Overload of the DisplayText to convert int to float.
		void DisplayText(int value, string description, int x);

		// Displays the text to the status window.
		void DisplayText(float value, string description, int x);

		string StatusWindow;

	private:

		// Counts the current FPS of the video feed.
		int fps;

		// Is the current FPS of the video feed.
		int currentFps;

		// Last and current times for FPS counter.
		time_t last, current;

		// Frame that holds the background image for the status frame.
		Mat statusBackground;

		// Frame that holds the status information.
		Mat statusFrame;

		// Battery level for drone.
		float batteryLevel;

		// The state of the drone:
		// 0: Unknown * 1: Init'ed * 2: Landed * 3,7: Flying *
		// 4: Hovering * 5: Test (?) * 6: Taking off * 8: Landing * 9: Looping (?).
		int state;

		// Estimated altitude of the drone, in mm.
		int altitude;

		// Temperature sensed.
		int temperature;

		// Estimated wind speed.
		float windSpeed;

		// Estimated wind angle.
		float windAngle;

		// Left/right tilt in degrees.
		float rotationX;

		// Forward/backward tilt in degrees.
		float rotationY;

		// Orientation in degrees.
		float rotationZ;

		// Linear velocity in mm/s of the x axis.
		float velocityX;

		// Linear velocity in mm/s of the y axis.
		float velocityY;

		// Linear velocity in mm/s of the z axis.
		float velocityZ;

		// Linear acceleration in g of the x axis.
		float accelerationX;

		// Linear acceleration in g of the y axis.
		float accelerationY;

		// Linear acceleration in g of the z axis.
		float accelerationZ;

		// Processes the drone's current status to to displayed the status window.
		void DisplayDroneStatus(void);

		// Converts float to string.
		string NumberToString(float num);
};

#endif
