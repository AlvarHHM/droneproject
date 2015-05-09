#ifndef DRONEPROJECT_OBSTACLETESTFLIGHT_H
#define DRONEPROJECT_OBSTACLETESTFLIGHT_H

#include "Flight.h"

class ObstacleTestFlight : public Flight{

public:
    ObstacleTestFlight(NodeHandle& node);

    void ProcessFlight(StateData& stateData);


    virtual void InitialBoundingBox(Rect *boundingBox) override;
};


#endif //DRONEPROJECT_OBSTACLETESTFLIGHT_H
