#ifndef STATEDATA_H
#define STATEDATA_H

#include <opencv/cv.hpp>
#include "Selector.h"
#include "Flight.h"
#include "ObstacleAvoid.h"

// TLD Includes.
#include "tld/TLD.h"



using namespace cv;
using namespace std;

class Flight;
class TLD;

class StateData {
public:
	// StateData class constructor.
	StateData(tld::TLD& tld, const char* windowName, Flight& flight, ObstacleAvoid& obstacleAvoid,
			bool& importedModel, bool cameraOnly);

	// StateData class destructor.
	~StateData();

	// TLD object tracking data.
	tld::TLD* tld;

	ObstacleAvoid* obstacleAvoid;

	// Gets the image retrieved from the drone's camera.
	Mat& Image(void);

	// Sets the image retrieved from the drone's camera.
	void Image(Mat image);

	// Gets the greyscale Mat image.
	Mat LastGray(void);

	// Sets the greyscale Mat image.
	void LastGray(Mat lastGray);

	// Gets the Selector.
	Selector* BoxSelector(void);

	// Gets the selection of the bounding box.
	Rect& Selection(void);

	// Sets the selection of the bounding box.
	void Selection(Rect selection);

	// Current image from drone.
	Mat image;

	// Image's gray scaled.
	Mat lastGray;

	// Flight class.
	Flight* flight;

	// States whether a model has been imported.
	bool modelImported;

	//States whether it is a camera feed or video feed.
	bool cameraOnly;

	Mat displayImg;

private:
	// Selector for bounding box.
	Selector* selector;

	// Selection from user selection.
	Rect selection;

};

#endif
