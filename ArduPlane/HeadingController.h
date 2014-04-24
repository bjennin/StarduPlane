#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include "PID_Controller.h"

// PID Controller Class Definition
class HeadingController: public PidController
{
  public:
    // Constructor
    HeadingController(float PGain, float IGain, float DGain, float maxOutput, float maxIntError, float maxDerError, float maxIntTerm, float maxDerTerm);
    
    // Step the digital controller, pass in the control to update
    float Step(float delta_t, float input_vars);
    
};

#endif /* HEADING_CONTROLLER_H */
