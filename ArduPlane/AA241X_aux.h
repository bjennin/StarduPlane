#ifndef AA241X_AUX_H
#define AA241X_AUX_H


#define ENERGY_LIMIT   0.450*7.4*3600 // max energy in Joules allowed for the mission.  Above this energy consumption, the camera function stops returning snapshots

// ***** AA241X FLIGHT VARIABLE DEFINITIONS  *****  //
// Euler angles, in radians from -PI to PI . Computed using attitude filter
// compensating for gyro and dead-reckoning drifts using accelerometer,
// magnetometer, GPS and barometer readings.  See AHRS_DCM.cpp if you want to know more
static float roll;
static float pitch;
static float yaw;

// Angular velocity components in body fixed frame,  in rad/s.  Computed using attitude filter
// compensating for gyro and dead-reckoning drifts using accelerometer,
// magnetometer, GPS and barometer readings. See AHRS_DCM.cpp if you want to know more
static float roll_rate;
static float pitch_rate;
static float yaw_rate;

// Linear velocity components in body fixed frame, in m/s
static float u_vel;
static float v_vel;
static float w_vel;

// Linear acceleration in body fixed frame, in m/s^2.
static float accel_x;
static float accel_y;
static float accel_z;

static float X_position;        // North position in meters, with respect to Center of Lake Lag
static float Y_position;        // East position in meters, with respect to Center of Lake Lag
static float Z_position_GPS;    // Down position in meters from sea level, reported by the GPS.  Z_position = -altitude above center of lake lagunita
static float Z_position_Baro;   // Down position in meters from sea level, reported by the Barometer.  Z_position = -altitude above center of lake lagunita

static float X_velocity; // Northwards velocity in m/s
static float Y_velocity; // Eastwards velocity in m/s
static float Z_velocity; // Downwards velocity in m/s.  Z_velocity = dZ_pos/dt = -climb rate

static float ground_speed;  // Speed relative to ground, in m/s
static float ground_course; // Direction of velocity relative to North, in radians

static float Air_speed; // Air relative speed in m/s, measured by pitot tube

static bool gpsOK = false;

static float battery_current;  // Current being consumed by on-board system, in Amps
static float battery_voltage;  // Voltage across battery, in Volts
static float battery_energy_consumed = 0.0;  // Battery energy consumed since last re-boot, in Joules = VoltAmpSec
static float mission_energy_consumed = 0.0;  // Battery energy consumed since start of the mission, in Joules

// Position to which the servos will be udpated, in % of full range (i.e. 0.0 = lower physical limit, 100.0 = upper physical limit)
// IMPORTANT: servos will only be updated when UAV is in AUTO mode
//            and with whatever values ar being set in AA241X_AUTO_FastLoop().
//            They can't be updated from MediumLoop() nor SlowLoop()
static float Roll_servo;     // Servo associated with Channel 1 
static float Pitch_servo;    // Servo associated with Channel 2
static float Throttle_servo; // Servo associated with Channel 3
static float Rudder_servo;   // Servo associated with Channel 4

// Position to which the servos are commanded by the RC Transmitter, in % of full range (i.e. 0.0 = lower physical limit, 100.0 = upper physical limit)
static float RC_roll;  // Command read on Input 1 or APM 2.5, coming from the RC Transmitter, through the receiver
static float RC_pitch;  // Command read on Input 1 or APM 2.5, coming from the RC Transmitter, through the receiver
static float RC_throttle;  // Command read on Input 1 or APM 2.5, coming from the RC Transmitter, through the receiver
static float RC_rudder;  // Command read on Input 1 or APM 2.5, coming from the RC Transmitter, through the receiver

static float RC_Roll_Trim;      // Position of roll stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range. (i.e. 50.0 is halfway, 0.0 all to one side, 100.0 all to the opposite side)
static float RC_Pitch_Trim;     // Position of pitch stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range
static float RC_Throttle_Trim;  // Position of throttle stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range
static float RC_Rudder_Trim;    // Position of rudder stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range

static float CPU_time_ms = 0.0;
static float GPS_time_ms = 0.0;
static float Main_loop_deltaTime_ms; // time in ms that Main loop (i.e. ArduPlane fast + medium + slow loops) took, ideally should be less than 20ms to ensure that the fast loop is executed at 50Hz
static float Last_AUTO_stampTime_ms = 0.0; // holds the last CPU time in ms for which we were in AUTO mode 

static int   personDistributionIndex = 0; // This selects which of the 10 sample distributions you use for your training runs

static float X_person_estimate[3];  // your estimate of the X_position for each of the 3 lost persons, in meters
static float Y_person_estimate[3];  // your estimate of the Y_position for each of the 3 lost persons, in meters

static float CPU_time_mission_ms = 0.0;
static char in_mission = 0;
static float Score = 0.0;

static float WayPointX[3];
static float WayPointY[3];
static float WayPointZ[3];
static float WayPointHeading[3];


// Camera functions 
///////////////////////
// Function takeASnapShot()
//
// Example:
// snapshot mySnapShot = takeASnapshot();
//
// mySnapShot.pictureTaken: range 0 or 1, indicates whether or not the picture was taken.  0 if not enough time between successive pictures, or if no GPS lock, or of energy cap reached, or out of altitude bounds
// mySnapShot.timeOfPicture;
// mySnapShot.personsInPicture: array of length 3 with values 0 or 1, indicating which persons were in the picture. personsInPicture[i] == 1 meanse person i was in the picture
// mySnapShot.centerOfPictureX: X position of the center of the picture, in NED coordinates 
// mySnapShot.centerOfPictureY: Y position of the center of the picture, in NED coordinates
// mySnapShot.diameterOfPicture: indicates the diameter of area covered by the picture 
// mySnapShot.centerOfPersonEstimateX[i]: X position in NED coordinates of the center of the circular area within which the i-th person is located. 
// mySnapShot.centerOfPersonEstimateY[i]: Y position in NED coordinates of the center of the circular area within which the i-th person is located.
// mySnapShot.diameterOfPersonEstimate[i]: diameter of the circular area within which the i-th person is located. 

struct snapshot{
    char pictureTaken;
    unsigned long timeOfPicture_ms;
    char personsInPicture[3];
    
    float centerOfPictureX;
    float centerOfPictureY;
    float diameterOfPicture;
    
    float centerOfPersonEstimateX[3];
    float centerOfPersonEstimateY[3];
    float diameterOfPersonEstimate[3];
};

static struct snapshot takeASnapshot(){
  return takeSnapshot(X_position, Y_position, -Z_position_Baro, gpsOK, mission_energy_consumed);
}

#endif /* AA241X_AUX_H */
