#include "State.h"

using namespace cv;
using namespace std;

// Colour definitions.
const Scalar State::RED = Scalar(0, 0, 255);
const Scalar State::BLUE = Scalar(255, 0, 0);
const Scalar State::GREEN = Scalar(0, 255, 0);
const Scalar State::CYAN = Scalar(255, 255, 0);
const Scalar State::MAGENTA = Scalar(255, 0, 255);
const Scalar State::YELLOW = Scalar(0, 255, 255);
const Scalar State::WHITE = Scalar(255, 255, 255);
const Scalar State::BLACK = Scalar(0, 0, 0);

StateSelecting::StateSelecting(void) :
		State() {
	cout << "**** Selecting State ****" << endl;
}

State* StateSelecting::Do(StateData* stateData) {
	if (stateData->modelImported && !stateData->cameraOnly) {
		return new StateInitializing();
	} else {
		if (stateData->BoxSelector()->Valid()) {
			stateData->Selection() = stateData->BoxSelector()->Selection();

			// Initialize the state with the current state data.
			return new StateInitializing();
		} else {
			// Shows the user what they are currently selecting by inversing the colours.
			Mat roi(stateData->image, stateData->BoxSelector()->Selection());
			bitwise_not(roi, roi);
		}
	}

	return this;
}

StateInitializing::StateInitializing(void) :
		State() {
	cout << "**** Initializing State ****" << endl;
}

// State initializing.
State* StateInitializing::Do(StateData* stateData) {
	if (stateData->BoxSelector()->Selecting()) {
		cout << "**** Selecting State ****" << endl;
		return this;
	}

	// Initialise TLD.
	stateData->tld->detectorCascade->imgWidth = stateData->LastGray().cols;
	stateData->tld->detectorCascade->imgHeight = stateData->LastGray().rows;
	stateData->tld->detectorCascade->imgWidthStep = stateData->LastGray().step;

	if (stateData->modelImported) {
		stateData->tld->selectObject(stateData->lastGray,
				stateData->flight->OriginalBoundingBox());
	} else {
		stateData->tld->selectObject(stateData->lastGray,
				&stateData->Selection());
	}

	// Call state tracking with current state data.
	return new StateTracking();

}

StateTracking::StateTracking(void) :
		State() {
	cout << "**** Tracking State ****" << endl;
}

// State tracking.
State* StateTracking::Do(StateData* stateData) {
	if (stateData->BoxSelector()->Selecting()) {
		cout << "**** Selecting State ****" << endl;
		return new StateSelecting();
	}
	stateData->tld->processImage(stateData->Image());

	int confident = (stateData->tld->currConf >= 0.5) ? 1 : 0;

	if (stateData->tld->currBB != NULL) {
		Scalar rectangleColor = (confident) ? this->BLUE : this->YELLOW;
		rectangle(stateData->displayImg, stateData->tld->currBB->tl(),
				stateData->tld->currBB->br(), rectangleColor, 8, 8, 0);
	}
	return this;
}

StartState::StartState() :
		State() {
	cout << "**** Start State ****" << endl;
}

State* StartState::Do(StateData* stateData) {
	if (stateData->BoxSelector()->Selecting() || stateData->modelImported) {
		return new StateSelecting();
	} else {
		return this;
	}
}
