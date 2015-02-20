/*
 * FrontTrackFlight.h
 *
 *  Created on: 20 Feb 2015
 *      Author: ardrone
 */

#ifndef DRONEPROJECT_SRC_FRONTTRACKFLIGHT_H_
#define DRONEPROJECT_SRC_FRONTTRACKFLIGHT_H_

#include "Flight.h"

class FrontTrackFlight: public Flight {
public:
	FrontTrackFlight(NodeHandle& node);
	virtual ~FrontTrackFlight();

	// Process image data to convert into flight commands.
	void ProcessFlight(StateData& stateData);

	// Sets the initial bounding box.
	void InitialBoundingBox(Rect* boundingBox);
};

#endif /* DRONEPROJECT_SRC_FRONTTRACKFLIGHT_H_ */
