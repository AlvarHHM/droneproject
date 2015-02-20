#include "Selector.h"
#include "Flight.h"

using namespace cv;
using namespace std;

// Constructor for the Selector class.
Selector::Selector(const char* window, Flight& flight) {
	this->selecting = false;
	this->selectionValid = false;
	this->selection = Rect(0, 0, 0, 0);
	this->flight = &flight;
	cv::setMouseCallback(window, mouse_callback, this);
}

// Mouse callback method to get around the static call of the cvMouseCallBack.
void Selector::doMouseCallback(int event, int x, int y, int flags) {
	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		// User has started their selection, initialise the rectangle.
		this->selectionValid = false;
		this->selecting = true;
		this->selection = Rect(0, 0, 0, 0);
		this->originPoint.x = x;
		this->originPoint.y = y;
		break;
	case CV_EVENT_LBUTTONUP:
		// User has selected their selection.
		this->selectionValid = true;
		this->selecting = false;

		this->flight->InitialBoundingBox(&selection);
		break;
	case CV_EVENT_MOUSEMOVE:
		// Mouse is moving, user is selecting an area.
		if (this->Selecting()) {
			this->selection.x = MIN(x, this->originPoint.x);
			this->selection.y = MIN(y, this->originPoint.y);
			this->selection.width = std::abs(x - this->originPoint.x);
			this->selection.height = std::abs(y - this->originPoint.y);
		}
		break;
	default: {
		// Unknown user input, usually result of a right-click.
		break;
	}
	}
}

// Calls doMouseCallBack, is static to confirm to standards.
void Selector::mouse_callback(int event, int x, int y, int flags, void *data) {
	Selector *self = static_cast<Selector*>(data);
	self->doMouseCallback(event, x, y, flags);
}

// Returns whether the current bounding box is valid.
bool Selector::Valid() {
	return this->selectionValid;
}

// Returns whether the user is currently selecting a bounding box.
bool Selector::Selecting() {
	return this->selecting;
}

// Returns the bounding box.
cv::Rect& Selector::Selection() {
	return this->selection;
}

// Manually sets the bounding box.
void Selector::SetBoundingBox(Rect* boundingBox) {
	this->selection.x = boundingBox->x;
	this->selection.y = boundingBox->y;
	this->selection.width = boundingBox->width;
	this->selection.height = boundingBox->height;
	this->selectionValid = true;
}
