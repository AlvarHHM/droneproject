#include "Pid.h"

using namespace std;

// Default constructor that takes in an offset, a constant proportional, a constant integral and a constant derivative.
Pid::Pid(int offset, double kp, double ki, double kd) {
	// Initialize values.
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->offset = offset;
	this->tp = 0;
	this->error = 0;
	this->integral = 0;
	this->lastError = 0;
	this->derivative = 0;
	this->calculatedMovement = 0;
	this->dt = 0;
}

// Default destructor.
Pid::~Pid(void) {
	// Do nothing.
}

// Does the PID calculations from the value input.
double Pid::ProcessPid(float currentValue, double time) {
	//cout << "Calculating PID" << endl;

	this->error = this->offset - currentValue;
	//cout << "Current Value: " << currentValue << endl;
	//cout << "Error: " << this->error << endl;

	this->integral = (this->integral * 0.66) + (this->error * time);
	//cout << "integral: " << this->integral << endl;

	this->derivative = (this->error - this->lastError) / time;

	this->calculatedMovement = ((this->kp * this->error)
			+ (this->ki * this->integral) + (this->kd * this->derivative));

	this->lastError = this->error;

	// TODO: Normalising.
	// Min = -5000
	// Max = +5000

	this->calculatedMovement = this->calculatedMovement / 5000;

	/*
	 * The minimum value that can be sent to the drone is -1.0
	 * The maximum value that can be sent is +1.0
	 */
	//cout << "Calculated movement: " << this->calculatedMovement << endl;
	if (this->calculatedMovement >= 1.0) {
		return 1.0;
	} else if (this->calculatedMovement <= -1.0) {
		return -1.0;
	} else {
		return this->calculatedMovement;
	}

}

// Allows re-setting of the offset.
double Pid::ProcessPid(float currentValue, double time, int offset) {
	this->offset = offset;
	return this->ProcessPid(currentValue, time);
}

// Overload of the ProcessPid method that takes in an int.
double Pid::ProcessPid(int currentValue, double time, int offset) {
	this->offset = offset;
	return this->ProcessPid((float) currentValue, time);
}

// Overload of the ProcessPid method that takes in a prop.
double Pid::ProcessPid(int currentValue, double time, double pidChange) {
	this->kp = pidChange;
	this->ProcessPid(currentValue, time);
}

// Process the PID, change the value of the proportional.
double Pid::ProcessPid(float currentValue, double time, int offset,
		double pidChange) {
	this->kp = pidChange;
	this->ProcessPid(currentValue, time, offset);
}

