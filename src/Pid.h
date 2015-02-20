#ifndef PID_H
#define PID_H

#include <iostream>

class Pid {
public:
	// Default constructor that takes in an offset, a constant proportional, a constant integral and a constant derivative.
	Pid(int offset, double kp, double ki, double kd);

	// Default destructor.
	~Pid(void);

	// Does the PID calculations from the value input.
	double ProcessPid(float currentValue, double time);

	// Allows re-setting of the offset.
	double ProcessPid(float currentValue, double time, int offset);

	// Overload of the ProcessPid method that takes in an int.
	double ProcessPid(int currentValue, double time, int offset);

	// Overload of the ProcessPid method that takes in a prop.
	double ProcessPid(int currentValue, double time, double pidChange);

	// Process the PID, change the value of the proportional.
	double ProcessPid(float currentValue, double time, int offset,
			double pidChange);

private:
	// Delta time.
	double dt;

	// Constant for the proportional controller.
	double kp;

	// Constant for the integral.
	double ki;

	// Constant for the derivative.
	double kd;

	// How much is necessary to subtract to get the error value.
	double offset;

	// Target power level.
	double tp;

	// Sum of the error across time.
	double integral;

	// The last error value.
	double lastError;

	// Change in the error between two consecutive points.
	double derivative;

	// Calculated error.
	double error;

	// Calculated movement from the PID controller.
	double calculatedMovement;
};

#endif
