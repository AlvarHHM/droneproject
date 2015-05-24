//
// Created by ardrone on 06/05/15.
//

#include "WanderFlight.h"

WanderFlight::WanderFlight(NodeHandle &node, NavigationData *navData) : Flight(node) {
    this->navData = navData;
    this->PidY = new Pid(0, 4 / 2.2, 0.005, 0);
    this->PidX = new Pid(200, 1 / 2.2, 0.003, 0);
    this->PidZ = new Pid(750, 10 / 2.2, 0, 0.05);
}

void WanderFlight::ProcessFlight(StateData &stateData) {
    // Update the current timer.
    gettimeofday(&this->currentTime, NULL);

    // Get the current milliseconds.
    this->currentMilliseconds = (this->currentTime.tv_sec * 1000)
                                + (this->currentTime.tv_usec / 1000);

    // Calculate the time difference.
    this->timeDifference = (this->currentMilliseconds - this->lastTime);

    // Update the last timer.
    this->lastTime = this->currentMilliseconds;



    if (this->flightAllowed){
        stateData.obstacleDetect->processFrame(
                stateData.lastGray);
        drawKeypoints(stateData.displayImg, stateData.obstacleDetect->queryKP, stateData.displayImg,
                      Scalar(0, 255, 0));
        drawKeypoints(stateData.displayImg, stateData.obstacleDetect->obstacleCluster,
                      stateData.displayImg, Scalar(0, 0, 255),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        line(stateData.displayImg, Point(stateData.obstacleDetect->obstacleX, 0),
             Point(stateData.obstacleDetect->obstacleX, 100), Scalar(0, 0, 255), 5);
        if (stateData.obstacleDetect->hasObstacle) {
            ROS_INFO("Obstacle !");
//            imshow("Obstacle", stateData.displayImg);
            this->SetHoverValues(0, 0, 0, 0);
            this->Hover();
//            this->LinearX(0);
//            this->LinearY(0);
            ServiceClient sc = nodeHandle.serviceClient<ardrone_autonomy::FlightAnim>("ardrone/setflightanimation");
            ardrone_autonomy::FlightAnim animServ;
            animServ.request.type = 7;
            animServ.request.duration = 500;
            sc.call(animServ);
            ros::Duration(1).sleep();
            this->SetHoverValues(0, 0, 0, 0);
            this->Hover();
            stateData.obstacleDetect->reset();
        }else{
            double linearY = this->PidY->ProcessPid(this->navData->VelocityY(), timeDifference);
            double linearX = this->PidX->ProcessPid(this->navData->VelocityX(), timeDifference);
            double linearZ = this->PidZ->ProcessPid(this->navData->Altitude(), timeDifference);
            this->LinearX(linearX);
            this->LinearY(linearY);
//            this->LinearZ(linearZ);
        }
//        double linearY = this->PidY->ProcessPid(this->navData->VelocityY(), timeDifference);
//        double linearX = this->PidX->ProcessPid(this->navData->VelocityX(), timeDifference);
//        double linearZ = this->PidZ->ProcessPid(this->navData->Altitude(), timeDifference);
//        this->LinearX(linearX);
//        this->LinearY(linearY);
//        this->LinearZ(linearZ);
    }else{

        stateData.obstacleDetect->reset();
        this->LinearX(0);
        this->SetHoverValues(0, 0, 0, 0);
        this->Hover();
    }

    this->SendFlightCommand();





}

void WanderFlight::InitialBoundingBox(Rect *boundingBox) {

}
