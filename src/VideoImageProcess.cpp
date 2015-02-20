#include "VideoImageProcess.h"

using namespace std;
using namespace cv;

// VideoImageProcess class constructor.
VideoImageProcess::VideoImageProcess(Flight* flight, string videoName,
		string& importModel) :
		ImageProcess(flight, importModel, false) {
	// Set the current state to start state.
	this->currentState = new StateSelecting();

	// Set the bounding box.
	this->SetBoundingBox(videoName);

	// Open the video from file.
	this->capture = new VideoCapture(videoName);
	this->capture->set(CV_CAP_PROP_FPS, 30);

	if (!this->capture->isOpened()) {
		cerr << "Couldn't open file" << endl;
		exit(2);
	}
}

// VideoImageProcess class destructor.
VideoImageProcess::~VideoImageProcess(void) {

}

// Sets the bounding boxes for the test videos made.
void VideoImageProcess::SetBoundingBox(string videoName) {
	Rect* boundingBox;
	if (videoName == "back-and-forth.avi") {
		boundingBox = new Rect(145, 120, 110, 50);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "back-forth-ground.avi") {
		boundingBox = new Rect(243, 136, 143, 65);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "in-a-circle.avi") {
		boundingBox = new Rect(204, 157, 135, 65);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "left-and-right.avi") {
		boundingBox = new Rect(238, 150, 132, 65);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "up-and-down.avi") {
		boundingBox = new Rect(230, 171, 120, 55);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "left.avi") {
		boundingBox = new Rect(221, 140, 174, 70);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
		cout << "Set bounding box for video left.avi" << endl;
	} else if (videoName == "right.avi") {
		boundingBox = new Rect(222, 147, 150, 63);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "up.avi") {
		boundingBox = new Rect(220, 177, 140, 65);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "down.avi") {
		boundingBox = new Rect(200, 116, 155, 67);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "slightly-left.avi"
			|| videoName == "slightly-forward.avi"
			|| videoName == "slightly-up.avi") {
		boundingBox = new Rect(248, 150, 121, 54);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "getData.avi") {
		boundingBox = new Rect(206, 215, 124, 56);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "video1.avi") {
		boundingBox = new Rect(1, 1, 1, 1);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else if (videoName == "benchmarks/complex-benchmark.avi") {
		boundingBox = new Rect(285, 150, 104, 38);
		this->CurrentStateData()->BoxSelector()->SetBoundingBox(boundingBox);
	} else {
		boundingBox = new Rect(0, 0, 0, 0);
		cout << "Bounding box not configured yet for video: " << videoName
				<< endl;
	}
	this->flight->InitialBoundingBox(boundingBox);

}

// Processes the video feed.
void VideoImageProcess::ProcessVideo(void) {
	// loop through video frames.
	for (;;) {
		// Get the next image.
		this->capture->read(this->frame);

		// Copy this to the current state data.
		this->CurrentStateData()->Image(this->frame);

		// Convert the image colour.
		cvtColor(this->CurrentStateData()->Image(),
				this->CurrentStateData()->lastGray, CV_BGR2GRAY);

		// Process the current state.
		this->currentState = this->currentState->Do(this->CurrentStateData());

		boost::thread_group threadGroup;

		// Send the current state data to the flight class.
		//threadGroup.add_thread(new boost::thread(&Flight::ProcessFlight, this->flight, *this->CurrentStateData()));

		int keyInput = waitKey(1);

		if (keyInput != -1) {
			threadGroup.add_thread(
					new boost::thread(&ImageProcess::ProcessKeyInput, this,
							keyInput));
		}

		if (keyInput == 1048690 || keyInput == 114) {
			// Pressed the r key.
			ROS_INFO("Stop.");
			break;
			return;
		}

		// Synchronize threads.
		threadGroup.join_all();

		imshow(WINDOWNAME, this->CurrentStateData()->Image());
	}

}
;
