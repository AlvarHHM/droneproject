#include "CameraImageProcess.h"

using namespace ros;
using namespace cv;
using namespace std;
using namespace cv_bridge;

// Default constructor for camera image processing.
CameraImageProcess::CameraImageProcess(Flight *flight, bool record,
        string &importModel) :
        ImageProcess(flight, importModel, true) {
    this->recordVideo = record;

    // Set the initial and current FPS to 0.
    this->fps = 0;

    // Set the default display for the FPS counter.
    this->textToDisplay = "FPS: Initialising";

    // Set where the fps counter will be displayed on the window.
    this->textCoords = {20, 15};

    // Set the beginning last and current time.
    time(&this->last);
    time(&this->current);
}

// Destructor for the CameraImageProcess class.
CameraImageProcess::~CameraImageProcess(void) {

}

// Converts the image from ROS Image to OpenCV type Mat.
void CameraImageProcess::ProcessImage(const sensor_msgs::ImageConstPtr &msg) {

    // Get the current image and flip it to the current state data.
    this->GetImage(msg);

    flip(this->currentImage, this->CurrentStateData()->image, 1);

    // Initialize writer if necessary.
    if (!this->writerInitialized and this->recordVideo) {
        this->InitialiseVideoCapture();
    }

    // Convert the colour.
    cvtColor(this->CurrentStateData()->image,
            this->CurrentStateData()->lastGray, CV_BGR2GRAY);

    // Process the current state.
    this->currentState = this->currentState->Do(this->CurrentStateData());

    boost::thread_group threadGroup;

    // Send the current state data to the flight class.
    threadGroup.add_thread(
            new boost::thread(&Flight::ProcessFlight, this->flight,
                    *this->CurrentStateData()));

    // Calculate the FPS of the video feed and image processing.
    threadGroup.add_thread(
            new boost::thread(&CameraImageProcess::CalculateFps, this));

    // Record the video to file.
    if (this->recordVideo and this->writer->isOpened()) {
        threadGroup.add_thread(
                new boost::thread(&VideoWriter::write, this->writer,
                        this->CurrentStateData()->Image()));
    }

    // Process the keyboard input.
    int keyInput = waitKey(1);
    if (keyInput != -1) {
        threadGroup.add_thread(
                new boost::thread(&ImageProcess::ProcessKeyInput, this,
                        keyInput));
    }

    // Synchronize threads.
    threadGroup.join_all();

    // Display FPS to screen.
    cv::putText(this->CurrentStateData()->image, this->textToDisplay,
            this->textCoords, FONT_HERSHEY_PLAIN, 1.0, Scalar::all(255), 1, 8);

    //Draw flight direction
    Scalar color(0, 0, 255);
    if(this->flight->flightAllowed && this->currentStateData->tld->currConf >= 0.5){

        int bbMidX = this->flight->currentBoundingBox->x + this->flight->currentBoundingBox->width/2;
        int bbMidY = this->flight->currentBoundingBox->y + this->flight->currentBoundingBox->height/2;
        drawArrow(this->CurrentStateData()->image,
                Point(bbMidX,bbMidY),
                Point(bbMidX + (int)(this->flight->LinearY() * -500),
                        bbMidY),
                10, 45, color, 1, 4);
        drawArrow(this->CurrentStateData()->image,
                Point(bbMidX,bbMidY),
                Point(bbMidX,
                        bbMidY + (int)(this->flight->LinearX() * -500)),
                10, 45, color, 1, 4);
    }


    // Show image.
    imshow(WINDOWNAME, this->CurrentStateData()->Image());
//	imshow(WINDOWNAME, this->CurrentStateData()->LastGray());
}

// Initialises the video capture to file.
void CameraImageProcess::InitialiseVideoCapture(void) {
    // TODO: Make this make a name dependent on date/time.
    // Or through the command line.
    string videoName =
            "/home/ardrone/ros_workspace/sandbox/drone_project/bin/video1.avi";

    cout << "Initialising video recording" << endl;
    this->writer = new VideoWriter(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 15,
            this->CurrentStateData()->Image().size(), true);
    if (!this->writer->isOpened()) {
        ROS_INFO("Writer could not be opened");
    }

    this->writerInitialized = true;
}

// Calculates the FPS of the video feed and image processing techniques.
void CameraImageProcess::CalculateFps(void) {
    this->fps++;

    if (difftime(this->current, this->last) > 1.0) {
        ostringstream buff;
        buff << this->fps;

        // Set the current FPS to be displayed to the screen.
        this->textToDisplay = "FPS: " + buff.str();

        // Reset the fps counter.
        this->fps = 0;

        // Set the last timer to the current.
        this->last = this->current;
    }

    // Update the current time.
    time(&this->current);
}

// Gets ROS image from the drone's camera feed.
void CameraImageProcess::GetImage(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // Copy the image from drone.
        CvImagePtr cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        this->currentImage = cv_ptr->image;
        cv_ptr->image.release();
    } catch (cv_bridge::Exception &e) {
        // Catch the error and display.
        ROS_ERROR("cv_bridge exception: %s", e.what());
        Mat mat;
        this->currentImage = mat;
    }
}
