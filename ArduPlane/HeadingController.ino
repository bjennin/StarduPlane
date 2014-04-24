#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "HeadingController.h"

// Constructor for the Heading controller class
HeadingController::HeadingController(float PGain, float IGain, float DGain, float maxOutput, float maxIntError, float maxDerError, float maxIntTerm, float maxDerTerm)
              : PidController(PGain, IGain, DGain, maxOutput, maxIntError, maxDerError, maxIntTerm, maxDerTerm){}

// Perform a digital step in the controller to produce the required output
float HeadingController::Step(float delta_t, float input_var)
{
  // Check delta_t, should be around 20 ms
  if(delta_t < 10)
  {
    delta_t = 20;
  }
  
  // Calculate Current Error, assume input and output in radians (0 - 2*PI)
  float error = mReference - input_var;
  
  if(error >= PI)
  {
    error -= PI;
  }else if (error <= - PI)
  {
    error += 2*PI;
  } 
   
  // Calculate Running Integral of Error
  mErrorInt += ((error + mLastError)*.5)/delta_t;
  
  // Cut Off Maximum Integral Error
  Limit(mErrorInt, mMaxIntError);
  
  // Calculate Derivative of Error
  float derError = (error - mLastError)/delta_t;
  
  // Cut Off Maximum Derivative Error
  Limit(derError, mMaxDerError);
  
  // Calculate All Controller Terms
  float p_term = mPGain*error;  // Proportional Controller Term
  float i_term = mIGain*mErrorInt; // Integral Controller Term
  Limit(i_term, mMaxIntTerm);  // Limit the integral controller
  float d_term = mDGain*derError;  // Derivative Term
  Limit(d_term, mMaxDerTerm);
  
  // Sum the terms
  float output = p_term + i_term + d_term;
  
  // Limit controller output by max control authority
  Limit(output, mMaxOutput);
  
  // Populate Controller Data
  mControllerSummary.p_term = p_term;
  mControllerSummary.i_term = i_term;
  mControllerSummary.d_term = d_term;
  mControllerSummary.error = error;
  mControllerSummary.i_error = mErrorInt;
  mControllerSummary.d_error = derError;
  mControllerSummary.output = output;
  
  // Return Controller Output (Input to Plant)
  return output;
}
