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
    this->CurrentStateData()->image.copyTo(this->CurrentStateData()->displayImg);

    // Process the current state.
    this->currentState = this->currentState->Do(this->CurrentStateData());
//    this->currentStateData->obstacleDetect->processFrame(
//            this->currentStateData->lastGray);
//    if (this->CurrentStateData()->obstacleDetect->hasObstacle){
//        drawKeypoints(this->CurrentStateData()->displayImg, this->CurrentStateData()->obstacleDetect->obstacleCluster,
//                      this->CurrentStateData()->displayImg, Scalar(0, 0, 255),
//                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//        line(this->CurrentStateData()->displayImg, Point(this->CurrentStateData()->obstacleDetect->obstacleX , 0),
//             Point(this->CurrentStateData()->obstacleDetect->obstacleX , 100), Scalar(0, 0, 255), 5);
//        time_t rawtime;
//        time (&rawtime);
//        char buffer[50];
//        sprintf(buffer,"./image_%s.jpg", ctime(&rawtime));
//        imwrite( buffer, this->CurrentStateData()->displayImg );
//    }

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
                        this->CurrentStateData()->displayImg));
    }

    // Process the keyboard input.
    int keyInput = waitKey(1);
    if (keyInput != -1) {
        threadGroup.add_thread(
                new boost::thread(&ImageProcess::ProcessKeyInput, this,
                        keyInput  & 0xff));
    }

    // Synchronize threads.
    threadGroup.join_all();

    // Display FPS to screen.
    cv::putText(this->CurrentStateData()->displayImg, this->textToDisplay,
            this->textCoords, FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1, 8);



    // Show image.
    imshow(WINDOWNAME, this->CurrentStateData()->displayImg);
}

// Initialises the video capture to file.
void CameraImageProcess::InitialiseVideoCapture(void) {
    // TODO: Make this make a name dependent on date/time.
    // Or through the command line.
    string videoName =  "/home/ardrone/video1.avi";
//    time_t rawtime;
//    time (&rawtime);
//    char videoName[50];
//    sprintf(videoName,"~/video_%s.avi", ctime(&rawtime));

    cout << "Initialising video recording" << endl;
//    this->writer = new VideoWriter(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 15,
//            this->CurrentStateData()->displayImg.size(), true);
    this->writer = new VideoWriter(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 15,
                                   cv::Size(640,480), true);
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
