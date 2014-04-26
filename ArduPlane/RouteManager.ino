#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "RouteManager.h"

// Constructor for the RouteManager class
RouteManager::RouteManager(float route)
              : mRoute(route)
{
  Initialize();
}

// Initialize the RouteManager
void RouteManager::Initialize(void)
{
  // I think things like initial state evaluation of the A/C and should be taken care of here
  // There's a chance that we start way off from our initial first waypoint, how do we handle that
  
  return;
}

// Perform a step of the routine to determine waypoint capture and give commands
float RouteManager::Step(float &headingCommand, float &altitudeCommand, float &airspeedCommand)
{
  // First, determine proximity of aircraft to waypoint
  
  // If the proximity criteria are met (radius or line), make sure the heading is +/- 90 degrees from waypoint to next waypoint
  
  // Handle waypoint capture
}

// Clear the current route
void RouteManager::ClearRoute(void)
{
  return;
}
    
// Set a new route
void RouteManager::SetRoute(void)
{
  return;
}
