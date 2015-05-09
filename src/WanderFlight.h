#ifndef DRONEPROJECT_WANDERFLIGHT_H
#define DRONEPROJECT_WANDERFLIGHT_H


#include "NavigationData.h"
#include "StateData.h"
#include "ardrone_autonomy/FlightAnim.h"

class WanderFlight : public Flight{
public:
    NavigationData* navData;
    WanderFlight(NodeHandle& node, NavigationData* navData);


    void ProcessFlight(StateData &stateData);

    void InitialBoundingBox(Rect *boundingBox);
};


#endif //DRONEPROJECT_WANDERFLIGHT_H
