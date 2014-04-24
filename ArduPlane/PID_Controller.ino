#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "PID_Controller.h"


// Constructor for the PID controller class
PidController::PidController(float PGain, float IGain, float DGain, float maxOutput, float maxIntError, float maxDerError, float maxIntTerm, float maxDerTerm)
              : mPGain(PGain),
                mIGain(IGain),
                mDGain(DGain),
                mLastError(0),
                mErrorInt(0),
                mReference(0),
                mMaxIntError(maxIntError),
                mMaxIntTerm(maxIntTerm),
                mMaxDerError(maxDerError),
                mMaxDerTerm(maxDerTerm),
                mMaxOutput(maxOutput)
{
  Initialize(PGain, IGain, DGain);
}

void PidController::Initialize(float PGain, float IGain, float DGain)
{
  // Reset All Errors
  Reset();
  
  // All Local Initializations
  mPGain = PGain;
  mIGain = IGain;
  mDGain = DGain;
  
  return;
}

// Perform a digital step in the controller to produce the required output
float PidController::Step(float delta_t, float input_var)
{
  // Check delta_t, should be around 20 ms
  if(delta_t < 10)
  {
    delta_t = 20;
  }
  
  // Calculate Current Error
  float error = mReference - input_var;
  
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

// Set the reference input
void PidController::SetReference(float reference)
{
  mReference = reference;
}

// Return the controller summary
controller_summary PidController::GetControllerSummary(void)
{
  return mControllerSummary;
}

// Limit the value of a variable based on a maximum
void PidController::Limit(float &variable, float maximum)
{
  // Check if variable exceeds the maximum in either direction
  if(variable > maximum)
  {
    variable = maximum;
  }
  else if(variable < -maximum)
  {
    variable = -maximum;
  }
}

// Reset the member variables to restart controller
void PidController::Reset(void)
{
  mLastError = 0;
  mReference = 0;
  mErrorInt = 0;
}
