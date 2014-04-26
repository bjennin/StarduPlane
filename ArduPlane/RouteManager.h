#ifndef ROUTE_MANAGER_H
#define ROUTE_MANAGER_H

#include "Waypoint.h"

// PID Controller Class Definition
class RouteManager
{
  public:    
    // Constructor
    RouteManager(float route);
    
    // Initialization Routine
    void Initialize(void);
    
    // Step the route manager to determine if a waypoint has been captured
    float Step(float &headingCommand, float &altitudeCommand, float &airspeedCommand);
    
    // Clear the current route
    void ClearRoute(void);
    
    // Set a new route
    void SetRoute(void);
    
 private:
    float mRoute; // The current route to fly
    
};














#endif /* PID_CONTROLLER_H */
