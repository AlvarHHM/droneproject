#ifndef STATE_H
#define STATE_H

#include <opencv/cv.hpp>
#include <boost/thread.hpp>
#include "StateData.h"

using namespace cv;

class StartState;
class StateSelecting;
class StateInitializing;
class StateTracking;

class StateData;
class TLD;
class Flight;

// TLD Includes.
#include "tld/TLD.h"

class State {
public:
	State(void) {
	}
	;
	virtual State* Do(StateData* stateData) {
	}
	;

	// Colour definitions.
	static const Scalar RED;
	static const Scalar BLUE;
	static const Scalar GREEN;
	static const Scalar CYAN;
	static const Scalar MAGENTA;
	static const Scalar YELLOW;
	static const Scalar WHITE;
	static const Scalar BLACK;
};

class StartState: public State {
public:
	StartState(void);
	~StartState(void);
	State* Do(StateData* stateData);
};

class StateSelecting: public State {
public:
	StateSelecting(void);
	~StateSelecting(void);
	State* Do(StateData* stateData);
};

class StateInitializing: public State {
public:
	StateInitializing(void);
	~StateInitializing(void);
	State* Do(StateData* stateData);
};

class StateTracking: public State {
public:
	StateTracking(void);
	~StateTracking(void);
	State* Do(StateData* stateData);
};

#endif
