/*
 * BottomTrackFlight.h
 *
 *  Created on: 19 Feb 2015
 *      Author: ardrone
 */

#ifndef DRONEPROJECT_BOTTOMTRACKFLIGHT_H_
#define DRONEPROJECT_BOTTOMTRACKFLIGHT_H_

#include "Flight.h"

class BottomTrackFlight: public Flight {
public:
	BottomTrackFlight(NodeHandle& node);
	virtual ~BottomTrackFlight();

	// Process image data to convert into flight commands.
	void ProcessFlight(StateData& stateData);

	// Sets the initial bounding box.
	void InitialBoundingBox(Rect* boundingBox);

	void drawFlightDirection(StateData &stateData);
};

#endif /* DRONEPROJECT_BOTTOMTRACKFLIGHT_H_ */
