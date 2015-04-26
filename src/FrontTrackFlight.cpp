/*
 * FrontTrackFlight.cpp
 *
 *  Created on: 20 Feb 2015
 *      Author: ardrone
 */

#include "FrontTrackFlight.h"

FrontTrackFlight::FrontTrackFlight(NodeHandle& node) :
		Flight(node) {
	// TODO Auto-generated constructor stub

}

FrontTrackFlight::~FrontTrackFlight() {
	// TODO Auto-generated destructor stub
}

void FrontTrackFlight::InitialBoundingBox(Rect* boundingBox) {
	// Drone camera height: 360px.
	// Drone camera width: 640px.
	// TODO: Look into implementing 720p video stream.

	// Get the middle X and Y coordinatates for the original bounding box.
	/* The (0,0) coordinate is the top left of the image. Increasing the x or y value
	 * moves the point right or down respectively.
	 * Thus the middle point for the original bounding box is its most top left coordinate
	 * and not the centre of the video feed.
	 * This can be calculated as:
	 * (height or width / 2) minus (bounding box height or width / 2).
	 */

	this->middleX = (640 / 2) - (boundingBox->width / 2);
	this->middleY = (360 / 2) - (boundingBox->height / 2);

	this->originalBoundingBox = boundingBox;
	this->currentBoundingBox = boundingBox;

	// TODO: Calibrate these last few numbers.

	// Calculate the "middle point" for calculating error, which will be the bounding box height + width.
	// TODO: Change.
	this->PidX = new Pid(
			this->originalBoundingBox->width
					+ this->originalBoundingBox->height, 15 / 2.2, 0.005,
			0.00125);

	// X-axis on the 2D image represents the Y-axis in the 3D plane.
	// Initialise the PID Controller for the Y-axis.
	this->PidY = new Pid(this->middleX, 0, 0, 0);

	// Y-axis on the 2D image represents the Z-axis in the 3D plane.
	// Initialise the PID controller for the Z-axis.
	this->PidZ = new Pid(this->middleY, 0, 0, 0);

	// Initialise the angular Z PID, used for calculating left and right turn.
	this->PidAngularZ = new Pid(this->middleX, 29 / 2.2, 0.02, 0.005);
	// Ku = 29.

	// Pu = ~1.25
}

// Process image data to convert into flight commands.
void FrontTrackFlight::ProcessFlight(StateData& stateData) {
	/* Remember
	 * X-axis: Forward and backwards. -> is a combination of the height and width of the bounding box.
	 * Y-axis: Left and right. -> Is the X-axis on the bounding box.
	 * Z-axis: Up and down -> is the Y-axis on the bounding box.
	 */
	// Only process commands if flight is allowed.
	if (this->flightAllowed) {
		// Get the current estimated state's confidence.
		int confident = (stateData.tld->currConf >= 0.5) ? 1 : 0;

		// Get current estimated state's bounding box.
		this->currentBoundingBox = stateData.tld->currBB;

		if (!confident) {
			// TODO: Move with a prediction.

			// Send the hover command, confidence is too low to make a prediction as to where to go.
			this->SetHoverValues(0, 0, 0, 0);

			//ROS_INFO("**** Target Lost; Hovering ****");
		} else {
			/*
			 * When the size of the bounding box changes, so does the middle position of the X and Y axis.
			 * Thus the need to recalculate the middle position on each cycle.
			 */

			int offsetY = 320 - (this->currentBoundingBox->width / 2);

			// Update the current timer.
			gettimeofday(&this->currentTime, NULL);

			// Get the current milliseconds.
			this->currentMilliseconds = (this->currentTime.tv_sec * 1000)
					+ (this->currentTime.tv_usec / 1000);

			// Calculate the time difference.
			this->timeDifference = (this->currentMilliseconds - this->lastTime);

			// Update the last timer.
			this->lastTime = this->currentMilliseconds;

			// PID control cannot be calculated with less than 1 error point, so if this isn't the first command.
			if (!this->firstCommand) {
				// Minus values turn left, opposite for turning right.

				double angular = this->PidAngularZ->ProcessPid(
						this->currentBoundingBox->x, this->timeDifference,
						offsetY) * -1.0;

				//cout << "Angular Z: " << angular << endl;
				this->AngularZ(angular);

				/*
				 double lineary = this->PidY->ProcessPid(this->currentBoundingBox->x, this->timeDifference, offsetY, prop);
				 cout << "Linear Y: " << lineary << endl;
				 this->LinearY(lineary);
				 /*
				 //this->LinearZ(this->PidZ->ProcessPid(this->currentBoundingBox->y, this->timeDifference, offsetZ) * -1.0);

				 /*
				 * The distance from the drone to the object can be shown as how big the bounding box is in comparison
				 * to either the original bounding box given or one specified by the user for a known object.
				 */
				double linearX = this->PidX->ProcessPid(
						this->currentBoundingBox->height
								+ this->currentBoundingBox->width,
						this->timeDifference);
				//cout << "Movement: " << linearX << endl;
				this->LinearX(linearX);
			} else {
				// If this is the first command, skip PID processing.
				this->firstCommand = false;
				return;
			}
		}

		// Send the command to the drone.
		this->SendFlightCommand();
	}
	// TODO: If the object is too low, don't go any lower, move back to get a better view -> active sensing!

	// TODO: Take into account the lag between video feed in, flight command out and it actually done to make the flight more responsive.
}
