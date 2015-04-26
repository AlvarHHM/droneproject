#include "StateData.h"
#include <opencv/cv.hpp>

using namespace cv;
using namespace std;

// StateData class constructor.
StateData::StateData(tld::TLD& tld, const char* windowName, Flight& flight, ObstacleDetect & obstacleAvoid,
		bool& importedModel, bool cameraOnly) {
	this->selector = new Selector(windowName, flight);
	this->flight = &flight;
	this->tld = &tld;
	this->obstacleDetect = &obstacleAvoid;
	this->modelImported = importedModel;
	this->cameraOnly = cameraOnly;
}
;

// StateData class destructor.
StateData::~StateData(void) {

}

// Gets the image retrieved from the drone's camera.
Mat& StateData::Image(void) {
	return this->image;
}

// Sets the image retrieved from the drone's camera.
void StateData::Image(Mat image) {
	image.copyTo(this->image);
}

// Gets the greyscale Mat image.
Mat StateData::LastGray(void) {
	return this->lastGray;
}

// Sets the greyscale Mat image.
void StateData::LastGray(Mat lastGray) {
	image.copyTo(this->lastGray);
}

// Gets the Selector.
Selector* StateData::BoxSelector(void) {
	return this->selector;
}

// Gets the selection of the bounding box.
Rect& StateData::Selection(void) {
	return this->selection;
}

// Sets the selection of the bounding box.
void StateData::Selection(Rect selection) {
	this->selection = selection;
}
