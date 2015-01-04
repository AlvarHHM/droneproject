#ifndef SELECTOR_H
#define SELECTOR_H

#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <cmath>
#include <iostream>

class Flight;

class Selector
{
public:
	// Constructor for the Selector class.
   Selector(const char* window, Flight& flight);

   // Returns whether the current bounding box is valid.
   bool Valid();

   // Returns whether the user is currently selecting a bounding box.
   bool Selecting();

   // Returns the bounding box.
   cv::Rect& Selection();

   // Manually sets the bounding box.
   void SetBoundingBox(cv::Rect* boundingBox);

private:

   // Calls doMouseCallBack, is static to confirm to standards.
   static void mouse_callback(int event, int x, int y, int flags, void *data);

   // Mouse callback when a mouse event is triggered.
   void doMouseCallback(int event, int x, int y, int flags);

   // Whether or not the selection is valid/complete.
   bool selectionValid;

   // User is currently selecting.
   bool selecting;

   // Selection created by user.
   cv::Rect selection;

   // Originating point for the bounding box.
   cv::Point originPoint;

   // Reference to the flight class.
   Flight* flight;
};

#endif
