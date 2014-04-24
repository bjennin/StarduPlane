
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "PID_Controller.h"

/**** Helper Function Prototypes ****/
static void       Limit(float &variable, float maximum, float minimum);

static int8_t     my_signed_8_bit_variable = 10;  // integer numbers between -128 and 127
static uint8_t    my_unsigned_8_bit_variable = 10;  // positive integer numbers between 0 and 255

static int16_t    my_signed_16_bit_variable = 10;  // integer numbers between −32768 and 32767
static uint16_t   my_unsigned_16_bit_variable = 10;  // positive integer numbers between 0 and 65535

static int32_t    my_signed_32_bit_variable = 10;  // integer numbers between −2147483648 and 2147483647
static uint32_t   my_unsigned_32_bit_variable = 10;  // positive integer numbers between 0 and 4294967295

/**** Control Mode ****/
static uint32_t   controlMode = round(1*MODE_BIT_0 + 2*MODE_BIT_1); // Determine 1 of 4 automatic control modes

/**** Time Variables ****/
static uint32_t   numCalls    = 0;    // Number of times the AUTO loop has been called
static float      delta_t_avg = 0; // Average value of delta_t
static float      delta_t_sum = 0; // Total sum of time since start of AUTO loop

/**** State Variables ****/
static float altitudeCommand  = 50;   // 50 meters is default altitude
static float airspeedCommand  = 14;   // 14 meters / second is default airspeed
static float headingCommand   = 0;    // Go North
static float rollCommand      = 0;    // Keep Level
static float pitchCommand     = .122; // 7 degrees pitch

/**** Outer Loop Command Minimums and Maximums ****/
static float pitchCommandMax = .21;   // 12 degrees pitch
static float pitchCommandMin = -.122; // -7 degrees pitch
static float airspeedCommandMax = 20; // 20 meters / second
static float airspeedCommandMin = 9;  // 9 meters / second

/*************************** Mechanical Limit Variables ***********************************************

Notes about the PWM signal commands, there doesn't seem to be a more appropriate place to put these
comments in anywhere but here at this point. The commanded signal to control the PWM signals going to
each of the servos is produced here in this loop. The command is given in values of percentage, which
are ultimately translated into the PWM duty cycle by the existing StarduPilot code. The chXout variables
given in the "Status" window of the Mission Planner are mapped linearly from the percentages commanded
in the AUTO_FastLoop function.

For each output, the 0% (0 command) is mapped to a corresponding value of 800 on the chXout menu. The
100% (100 command) is mapped to a corresponding value of 2200 on the chXmenu. The output of the menu
is cut of at 900 and 2100 for reasons unknown, but the middle value is 1500 and for each integer increase
in the value of the commanded percentage, a value of 14 is added to the chXout value.

Each control surface has mechanical limits that should not be exceeded by either the RC controller (pilot
input) or the automatic flight system. These mechanical limits are what is defined in this section.

--------------------------------------- Bixler 2 Mechanical Limits ----------------------------------*/
float pitchMax  = 100; // Elevator Down
float pitchMin  = 20;  // Elevator Up
float rollMax   = 100; // Roll Left Aileron
float rollMin   = 0;   // Roll Right Aileron
float rudderMax = 100; // Left Rudder
float rudderMin = 0;   // Right Rudder
float throttleMax = 100; // Throttle Up
float throttleMin = 0;   // Throttle Down

/**** PID Loops ****/
PidController rollController241X(RLL_2_SRV_P, // Proportional Gain
                                 RLL_2_SRV_I, // Integral Gain
                                 RLL_2_SRV_D, // Derivative Gain
                                 25,          // Maximum Controller Output
                                 1,           // Maximum Integral Error
                                 3,           // Maximum Derivative Error
                                 5,           // Maximum Integral Term
                                 5);          // Maximum Derivative Term

PidController pitchController241X(PTCH_2_SRV_P, // Proportional Gain
                                  PTCH_2_SRV_I, // Integral Gain
                                  PTCH_2_SRV_D, // Derivative Gain
                                  20,           // Maximum Controller Output
                                  1,            // Maximum Integral Error
                                  3,            // Maximum Derivative Error
                                  5,            // Maximum Integral Term
                                  5);           // Maximum Derivative Term
                                  
PidController rudderController241X(RUD_2_SRV_P,  // Proportional Gain
                                   RUD_2_SRV_I,  // Integral Gain
                                   RUD_2_SRV_D,  // Derivative Gain
                                   20,           // Maximum Controller Output
                                   1,            // Maximum Integral Error
                                   3,            // Maximum Derivative Error
                                   5,            // Maximum Integral Term
                                   5);           // Maximum Derivative Term                                  

PidController airspeedController241X(SPD_2_SRV_P, // Proportional Gain
                                     SPD_2_SRV_I, // Integral Gain
                                     SPD_2_SRV_D, // Derivative Gain
                                     20,          // Maximum Controller Output
                                     10,          // Maximum Integral Error
                                     3,           // Maximum Derivative Error
                                     5,           // Maximum Integral Term
                                     5);          // Maximum Derivative Term

PidController headingController241X(HEAD_2_SRV_P, // Proportional Gain
                                    HEAD_2_SRV_I, // Integral Gain
                                    HEAD_2_SRV_D, // Derivative Gain
                                    25,           // Maximum Controller Output
                                    1,            // Maximum Integral Error
                                    3,            // Maximum Derivative Error
                                    5,            // Maximum Integral Term
                                    5);           // Maximum Derivative Term
                                    
PidController altitudeController241X(ALT_HOLD_P,  // Proportional Gain
                                     ALT_HOLD_I,  // Integral Gain
                                     ALT_HOLD_D,  // Derivative Gain
                                     25,           // Maximum Controller Output
                                     1,            // Maximum Integral Error
                                     3,            // Maximum Derivative Error
                                     5,            // Maximum Integral Term
                                     5);           // Maximum Derivative Term                                    
                                 
// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void)
{
  // Time between function calls
  float delta_t = (CPU_time_ms - Last_AUTO_stampTime_ms); // Get delta time between AUTO_FastLoop calls  
  
  // Inner Loop Command Signals, set by default to RC pilot commands
  float rollControllerOut = RC_roll;
  float pitchControllerOut = RC_pitch;
  float rudderControllerOut = RC_rudder;
  
  // Outer Loop Command Signals
  float airspeedControllerOut = 0;
  float headingControllerOut  = 0;
  float altitudeControllerOut = 0;
  
  // static struct snapshot mySnapShot = takeASnapshot();
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
    // Reset Average of delta_t
    delta_t_avg = 0;
    delta_t_sum = 0;
    numCalls    = 0;
    
    // Just switched to AUTO, initialize all controller loops
    rollController241X.Initialize(RLL_2_SRV_P, RLL_2_SRV_I, RLL_2_SRV_D);
    pitchController241X.Initialize(PTCH_2_SRV_P, PTCH_2_SRV_I, PTCH_2_SRV_D);
    rudderController241X.Initialize(RUD_2_SRV_P, RUD_2_SRV_I, RUD_2_SRV_D);
    airspeedController241X.Initialize(SPD_2_SRV_P, SPD_2_SRV_I, SPD_2_SRV_D);
    headingController241X.Initialize(HEAD_2_SRV_P, HEAD_2_SRV_I, HEAD_2_SRV_D);
    altitudeController241X.Initialize(ALT_HOLD_P, ALT_HOLD_I, ALT_HOLD_D);
    
    // Save all initial settings
    if(gpsOK == true)
      altitudeCommand = -Z_position_GPS;
    else
      altitudeCommand = -Z_position_Baro;
      
    headingCommand = ground_course;
    airspeedCommand = Air_speed;
    
    // Determine control mode from bits in parameter list
    controlMode = round(1*MODE_BIT_0 + 2*MODE_BIT_1);
  }
  
  // Time Related Tracking
  delta_t_sum += delta_t;
  numCalls    += 1;
  delta_t_avg  = delta_t_sum/numCalls;
  
  // Determine Inner Loop Commands Based on Control Mode
  switch (controlMode)
  {
    // Keep Roll angle controlled based on RC pilot input (should be zero when stick is in center)
    case ROLL_STABILIZE_MODE:
      rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      rollController241X.SetReference(rollCommand);
      rollControllerOut = rollController241X.Step(delta_t, roll);
    break;
    // Keep Roll and Pitch angles controlled based on RC pilot input
    case STABILIZE_MODE:
      // Roll Commands
      rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      rollController241X.SetReference(rollCommand);
      rollControllerOut = rollController241X.Step(delta_t, roll);
      
      // Pitch Commands
      pitchCommand = -(RC_pitch - RC_Pitch_Trim)*0.01*PI/4.0 + (7.0/180.0)*PI;
      pitchController241X.SetReference(pitchCommand);
      pitchControllerOut = -pitchController241X.Step(delta_t, pitch);
      
      // Rudder Commands
      // rudderCommand = 
    break;
    // Maintain Heading, RC pilot commands offset from heading that was saved
    case HEADING_HOLD_MODE:
      // Heading Commands
      headingCommand += headingCommand*0.00174*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz
      headingController241X.SetReference(headingCommand);
      headingControllerOut = headingController241X.Step(delta_t, ground_course);
      
      // Roll Commands
      rollController241X.SetReference(headingControllerOut);
      rollControllerOut = rollController241X.Step(delta_t, roll); 
      
    break;
    // Maintain heading, altitude, and airspeed RC pilot commands offsets from saved initial conditions
    case FBW_MODE:
      // Heading Commands
      headingCommand  += headingCommand*0.00174*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz
      headingController241X.SetReference(headingCommand);
      headingControllerOut = headingController241X.Step(delta_t, ground_course);
      
      // Roll Commands
      rollController241X.SetReference(headingControllerOut);
      rollControllerOut = rollController241X.Step(delta_t, roll);
      
      // Altitude Commands
      float altitude = 50; // Default altitude
      if(gpsOK == true)
        altitude = -Z_position_GPS;
      else
        altitude = -Z_position_Baro;
      
      altitudeCommand += altitudeCommand*0.04*(RC_pitch - RC_Pitch_Trim)/RC_Pitch_Trim; // 2 m/s change rate based on 50 Hz
      altitudeController241X.SetReference(altitudeCommand);
      altitudeControllerOut = headingController241X.Step(delta_t, altitude);
      Limit(altitudeControllerOut, pitchCommandMax, pitchCommandMin);
      
      // Pitch Commands
      pitchController241X.SetReference(altitudeControllerOut);
      pitchControllerOut = -pitchController241X.Step(delta_t, pitch);
      
      airspeedCommand += airspeedCommand*0.02*(RC_throttle - RC_Throttle_Trim)/RC_Throttle_Trim; // Should be trim setting based on pitch command and desired airspeed
      Limit(airspeedCommand, airspeedCommandMax, airspeedCommandMin);
      airspeedController241X.SetReference(airspeedCommand);
      airspeedControllerOut = airspeedController241X.Step(delta_t, Air_speed);
    break;
  }
  
  // Update Roll Servo Command  
  if(controlMode)
  {
    float rollOut    = RC_Roll_Trim + rollControllerOut;
    Limit(rollOut, rollMax, rollMin);
    Roll_servo       = rollOut;
  }
  else
  {
    Roll_servo       = RC_roll;
  }
  
  // Update Pitch Servo Command
  if(controlMode == STABILIZE_MODE || controlMode == FBW_MODE)
  {
    float pitchOut   = RC_Pitch_Trim + pitchControllerOut;
    Limit(pitchOut, pitchMax, pitchMin);
    Pitch_servo      = pitchOut;
  }
  else
  {
    Pitch_servo      = RC_pitch;
  }

  // Update Rudder Servo Command
  if(controlMode == STABILIZE_MODE || controlMode == HEADING_HOLD_MODE || controlMode == FBW_MODE)
  {
    float rudderOut  = RC_Rudder_Trim + rudderControllerOut;
    Limit(rudderOut, rudderMax, rudderMin);
    Rudder_servo     = rudderOut;
  } 
  else
  {
    Rudder_servo     = RC_rudder;
  }
  
  // Update Throttle PWM Command
  if(controlMode == FBW_MODE)
  {
    float throttleOut = RC_Throttle_Trim + airspeedControllerOut;
    Limit(throttleOut, throttleMin, throttleMax);
    Throttle_servo   = throttleOut;
  }
  else
  {
    Throttle_servo   = RC_throttle;
  }

}; /* End AA241X_AUTO_FastLoop */


// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void)
{
  // YOUR CODE HERE
};

// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void)
{
  controller_summary RollControllerSummary = rollController241X.GetControllerSummary();
  controller_summary PitchControllerSummary = pitchController241X.GetControllerSummary();
  
  /*
  // Debug Statements
  hal.console->printf_P(PSTR("\n Roll Value: %f \n"), roll);  
  hal.console->printf_P(PSTR("\n Roll Error: %f \n"), RollControllerSummary.error);
  hal.console->printf_P(PSTR("Roll Integrated Error: %f \n"), RollControllerSummary.i_error);
  hal.console->printf_P(PSTR("Roll Derivative Error: %f \n"), RollControllerSummary.d_error);
  hal.console->printf_P(PSTR("Roll Proportional Term: %f \n"), RollControllerSummary.p_term);
  hal.console->printf_P(PSTR("Roll Integral Term: %f \n"), RollControllerSummary.i_term);
  hal.console->printf_P(PSTR("Roll Derivative Term: %f \n"), RollControllerSummary.d_term);
  hal.console->printf_P(PSTR("Roll Output: %f \n"), RollControllerSummary.output);  

  hal.console->printf_P(PSTR("\n Pitch Value: %f \n"), pitch);
  hal.console->printf_P(PSTR("\n Pitch Command: %f \n"), pitch_command);
  hal.console->printf_P(PSTR("\n Pitch Error: %f \n"), PitchControllerSummary.error);
  hal.console->printf_P(PSTR("Pitch Integrated Error: %f \n"), PitchControllerSummary.i_error);
  hal.console->printf_P(PSTR("Pitch Derivative Error: %f \n"), PitchControllerSummary.d_error);
  hal.console->printf_P(PSTR("Pitch Proportional Term: %f \n"), PitchControllerSummary.p_term);
  hal.console->printf_P(PSTR("Pitch Integral Term: %f \n"), PitchControllerSummary.i_term);
  hal.console->printf_P(PSTR("Pitch Derivative Term: %f \n"), PitchControllerSummary.d_term);
  hal.console->printf_P(PSTR("Pitch Output: %f \n"), PitchControllerSummary.output);
  
  hal.console->printf_P(PSTR("\n RC_roll: %f \n"), RC_roll);
  hal.console->printf_P(PSTR("RC_pitch: %f \n"), RC_pitch);
  hal.console->printf_P(PSTR("RC_rudder: %f \n"), RC_rudder);
  */
  
  hal.console->printf_P(PSTR("\n Avg dT: %f \n"), delta_t_avg);
};

/**** Limit function to not exceed mechanical limits of the servos ****/
static void Limit(float &variable, float maximum, float minimum)
{
  if(variable > maximum)
  {
    variable = maximum;
  }
  else if(variable < minimum)
  {
    variable = minimum;
  }
};



