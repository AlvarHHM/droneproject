//
// Created by ardrone on 03/05/15.
//

#include "ObstacleTestFlight.h"

ObstacleTestFlight::ObstacleTestFlight(NodeHandle &node) : Flight(node){

}

void ObstacleTestFlight::ProcessFlight(StateData &stateData) {
    stateData.obstacleDetect->processFrame(
            stateData.lastGray);
    drawKeypoints(stateData.displayImg, stateData.obstacleDetect->queryKP, stateData.displayImg,
                  Scalar(0, 255, 0));
    drawKeypoints(stateData.displayImg, stateData.obstacleDetect->obstacleCluster,
                  stateData.displayImg, Scalar(0, 0, 255),
                  DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    line(stateData.displayImg, Point(stateData.obstacleDetect->obstacleX , 0),
         Point(stateData.obstacleDetect->obstacleX , 100), Scalar(0, 0, 255), 5);
    if (stateData.obstacleDetect->hasObstacle){
        this->FlightAllowed(false);
        this->Hover();
        ROS_INFO("Obstacle Hover");
    }

    if(this->flightAllowed){
        this->LinearX(0.03);
        this->SendFlightCommand();
    }else{
        this->SetHoverValues(0,0,0,0);
        this->Hover();
    }




}

void ObstacleTestFlight::InitialBoundingBox(Rect *boundingBox) {

}
