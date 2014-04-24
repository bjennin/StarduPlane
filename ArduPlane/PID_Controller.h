#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct controller_summary
{
  float p_term;
  float i_term;
  float d_term;
  float error;
  float i_error;
  float d_error;
  float output;
};

// PID Controller Class Definition
class PidController
{
  public:    
    // Constructor
    PidController(float PGain, float IGain, float DGain, float maxOutput, float maxIntError, float maxDerError, float maxIntTerm, float maxDerTerm);
    
    // Initialization Routine
    void Initialize(float PGain, float IGain, float DGain);
    
    // Step the digital controller, pass in the control to update
    virtual float Step(float delta_t, float input_vars);
    
    // Set the PID reference
    void SetReference(float reference);
    
    // Get some values of the controller terms
    controller_summary GetControllerSummary(void);
    
    // Reset the standing error values
    void Reset(void);
    
 protected:
    float mPGain;       // Proportional Gain of Controller
    float mIGain;       // Integral Gain of Controller
    float mDGain;       // Derivative Gain of Controller
    float mLastError;   // Save the last error term
    float mErrorInt;    // Save the Intgral of the error
    float mReference;   // Commanded Reference Input to Controller
    float mMaxIntError; // Maximum error allowed on integral (prevent windup)
    float mMaxIntTerm;  // Maximum contribution of control by integral term
    float mMaxDerError; // Maximum error allowed on derivative (prevent huge spikes)
    float mMaxDerTerm;  // Maximum contribution of control by derivative term
    float mMaxOutput;   // Limit on control authority
    controller_summary mControllerSummary; // Summary of all controller information
    
    // Helper Function to limit certain Variables
    void Limit(float &variable, float maximum);
    
};














#endif /* PID_CONTROLLER_H */
