
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_Competition.h"

#define HEAD_BYTE1      0xA3   // Decimal 163
#define HEAD_BYTE2      0x95   // Decimal 149
#define END_BYTE        0xBA   // Decimal 186
#define LOG_AA241X_MSG  0x0B  
#define NED_to_Body(x)  DCM_Matrix.mul_transpose(x)  // Convert vector3f x from NED frame to Body Frame.  Returns a type Vector3f
#define Body_to_NED(x)  DCM_Matrix*(x)  // Convert vector3f x from Body frame to NED Frame.  Returns a type Vector3f

#define PWM_MIN 800
#define PWM_MAX 2200

#define CENTER_LAGUNITA_LAT   374224444  // Latitutde of Center of Lake Lag in degrees * 1e7
#define CENTER_LAGUNITA_LONG -1221760917 // Latitutde of Center of Lake Lag in degrees * 1e7
#define CENTER_LAGUNITA_ALT 40  // altitude above sea level of center of Lake Lag

#define MIN_AIRSPEED 2.0  // airspeed below which the mission will be ended if the mission has started and the altitude is below ALTITUDE_START

#define MAX_ALTITUDE 121.92 // max altitude in meters above Lake Lag allowed for flight.  Above this altitude, the camera function stops returning snapshots 
#define MIN_ALTITUDE_SNAPSHOT 30.48  // min altitude in meters above Lake Lag allowed for flight.
#define ALTITUDE_START 15.24  // min altitude in meters above Lake Lag allowed for flight.
#define MAX_FOV_DIAM 100.0
#define MIN_FOV_DIAM 50.0
#define MAX_ESTIMATE_DIAM 25.0
#define MIN_ESTIMATE_DIAM 10.0

#define WAIT_TIME_MS 3000  // wait time between pictures

#define ALPHA 300.0  // weight for position error term
#define BETA 2000.0  // weight for time of sight term
#define GAMMA 3.0  // min position error used for score

#define Z_POS_CONST 0.7  // constant for the Z_position_Baro low pass filter
#define Z_VEL_CONST 0.7  // constant for the Z_velocity low pass filter

#define GUMSTIX_ACK B1010000


// *****  AA241X AUXILIARY FUNCTIONS - IF BUGS FOUND LET US KNOW ASAP, DO NOT EDIT!   ***** //

// Array with different stranded persons distributions. 
//const float persons_location[][3][2] = { {{57.75,     100.026},   {-115.5 ,  0.0     },  {57.75,     -100.026}}, 
//                                         {{32.9899,   78.7251},   {80.6690, -140.6848},  {-18.2864,  -23.8934}},
//                                         {{70.3668,   22.3351},   {80.0197, -118.3350},  {-115.9529, -73.8221}},
//                      {{-109.7851, 84.6363},   {-0.6705,   19.6712},  {132.0063,  -2.2230 }} };


const int Competition_day = 99;

static int16_t AA241X_roll_servo_PWM = 901;
static int16_t AA241X_pitch_servo_PWM = 901;
static int16_t AA241X_throttle_servo_PWM = 901;
static int16_t AA241X_rudder_servo_PWM = 901;

static int16_t RC_roll_PWM;
static int16_t RC_pitch_PWM;
static int16_t RC_throttle_PWM;
static int16_t RC_rudder_PWM;

static struct Location loc;
static struct Location center_lake_lag;

static char sighted_persons = 0; 
static char sightedPerson[3] = {0, 0, 0};
static float CPU_time_sight_ms = 0.0;

static float CPU_time_start_ms = 0.0;

static float X_person_truth[3];
static float Y_person_truth[3];

// UPDAET SERVO POSITIONS
static void update_servos(void){
  AA241X_roll_servo_PWM     = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Roll_servo/100.0); 
  AA241X_pitch_servo_PWM    = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Pitch_servo/100.0); 
  AA241X_throttle_servo_PWM = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Throttle_servo/100.0); 
  AA241X_rudder_servo_PWM   = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Rudder_servo/100.0); 
  
  channel_roll->radio_out     = constrain_int16(AA241X_roll_servo_PWM,     PWM_MIN, PWM_MAX);
  channel_pitch->radio_out    = constrain_int16(AA241X_pitch_servo_PWM,    PWM_MIN, PWM_MAX);
  channel_throttle->radio_out = constrain_int16(AA241X_throttle_servo_PWM, PWM_MIN, PWM_MAX);
  channel_rudder->radio_out   = constrain_int16(AA241X_rudder_servo_PWM,   PWM_MIN, PWM_MAX);
}

static void update_Last_AUTO_time_and_Main_loop_deltaTime(void){
  Last_AUTO_stampTime_ms = CPU_time_ms;
 // Main_loop_deltaTime_ms = (float)delta_ms_main_loop;
}

static void AA241X_AUX_MediumLoop(void){
  if (control_mode == AUTO)
  {
      AA241X_AUTO_MediumLoop();
  }
}

static void AA241X_AUX_SlowLoop(void){
  if (control_mode == AUTO)
  {
      AA241X_AUTO_SlowLoop();
  }
}


// UPDATE FLIGHT VARIABLES used for AA241X Control Law
static void update_AA241X_flight_variables(void) {
  Vector3f angular_vel;
  Vector3f accelerometer_reading;
  Matrix3f DCM_Matrix;
  Vector3f NED_Velocity;
  Vector3f Body_Velocity;
  Vector3f NED_Gravity;
  Vector3f Body_Gravity;
  
  static float dist;
  static float bearing;
  static float altitude;
  static char allow_start = 1;
 
  static uint16_t last_millis = 0;
  static uint16_t current_millis;
  static float last_CPU_time_ms = 0.0;
  static float delta_time_sec;
  static float battery_energy_consumed_start = 0.0;
  static float last_CPU_time_Z_filter_ms; 
  static float last_Z_position_Baro = 0.0;
  static float last_Z_velocity = 0.0;
  static char Z_filter_flag = 0;
  float Z_position_Baro_raw;
  float Z_velocity_raw;
  
  
  // Update CPU time 
  current_millis = millis();
  if (current_millis < last_millis){
    CPU_time_ms = CPU_time_ms + (65535.0 - (float)last_millis) + (float)current_millis ;
  } else {
    CPU_time_ms = CPU_time_ms + (float)(current_millis - last_millis);
  }
  last_millis = current_millis;
 
  
  // Update total battery energy consumed
  read_battery();
  battery_current = battery.current_amps();  //current_amps1;
  battery_voltage = battery.voltage();   //battery_voltage1;
  delta_time_sec = (CPU_time_ms - last_CPU_time_ms)/1000.0;
  battery_energy_consumed = battery_energy_consumed + battery_current*battery_voltage*delta_time_sec;
  mission_energy_consumed = battery_energy_consumed - battery_energy_consumed_start;
  last_CPU_time_ms = CPU_time_ms;
  
  Main_loop_deltaTime_ms = 0; //(float)delta_ms_fast_loop;
  
  roll  = ahrs.roll;
  pitch = ahrs.pitch;
  yaw   = ahrs.yaw;
  
  angular_vel = ahrs.get_gyro();   
  roll_rate   = angular_vel.x;
  pitch_rate  = angular_vel.y;
  yaw_rate    = angular_vel.z;
  
  // Z_position_Baro and Z_velocity filters
  Z_position_Baro_raw = -1.0*(relative_altitude() + home.alt - CENTER_LAGUNITA_ALT*100)/100.0;  // Z_position in meters from center of lake lag
  if (Z_filter_flag == 0) {
    Z_position_Baro = Z_position_Baro_raw;
    Z_velocity = 0.0;
    Z_filter_flag = 1;
  } else {
    // Z_position_Baro low pass filter
    Z_position_Baro = (1-Z_POS_CONST)*Z_position_Baro_raw + Z_POS_CONST*last_Z_position_Baro;
    // Z_velocity low pass filter
    Z_velocity_raw = 1000.0*(Z_position_Baro - last_Z_position_Baro)/((CPU_time_ms - last_CPU_time_Z_filter_ms));
    Z_velocity      =  (1-Z_VEL_CONST)*Z_velocity_raw     + Z_VEL_CONST*last_Z_velocity;
  }
  last_Z_position_Baro = Z_position_Baro;
  last_Z_velocity = Z_velocity;
  last_CPU_time_Z_filter_ms = CPU_time_ms; 
  
  
  Air_speed = airspeed.get_airspeed();
  
  // Variables derived from a GPS reading
  center_lake_lag.lat = CENTER_LAGUNITA_LAT;
  center_lake_lag.lng = CENTER_LAGUNITA_LONG;
  gpsOK  = ahrs.get_position(loc);
  if (gps.status(0) == AP_GPS::GPS_OK_FIX_3D){
    // Update GPS time
    GPS_time_ms = gps.time_week_ms(0);
    
    bearing = (((float)get_bearing_cd(center_lake_lag, loc))/100.0)*PI/180.0;
    dist    = (float)get_distance(center_lake_lag, loc);
    
    X_position     = cos(bearing)*dist;
    Y_position     = sin(bearing)*dist;
    Z_position_GPS = -((float)(gps.location().alt * 10UL)*1000- CENTER_LAGUNITA_ALT)/100.0;  // altitude in MSL
  
    ground_speed  = ((float) gps.ground_speed())/100.0; 
    ground_course = (((float) gps.ground_course_cd())/100.0)*PI/180;
  
    X_velocity     = cos(ground_course)*ground_speed;
    Y_velocity     = sin(ground_course)*ground_speed;
    NED_Velocity.x = X_velocity;
    NED_Velocity.y = Y_velocity;
    NED_Velocity.z = Z_velocity;
    
    DCM_Matrix.from_euler(roll, pitch, yaw);
    Body_Velocity = NED_to_Body(NED_Velocity);  // see http://gentlenav.googlecode.com/files/EulerAngles.pdf for definition of DCM_Matrix
    u_vel         = Body_Velocity.x;
    v_vel         = Body_Velocity.y;
    w_vel         = Body_Velocity.z; 
  } 
  
  // Compute acceleration
//  accelerometer_reading = imu.get_accel();
//  NED_Gravity.x = 0;
//  NED_Gravity.y = 0;
//  NED_Gravity.z = 9.80665;
//  Body_Gravity  = NED_to_Body(NED_Gravity);
//  accel_x       = accelerometer_reading.x + Body_Gravity.x;
//  accel_y       = accelerometer_reading.y + Body_Gravity.y;
//  accel_z       = accelerometer_reading.z + Body_Gravity.z;
  
  
  // Read RC signals
  RC_roll_PWM      = channel_roll->radio_in;
  RC_pitch_PWM     = channel_pitch->radio_in;
  RC_throttle_PWM  = channel_throttle->radio_in;
  RC_rudder_PWM    = channel_rudder->radio_in;
    
  RC_roll     = 100*((float)(RC_roll_PWM     - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_pitch    = 100*((float)(RC_pitch_PWM    - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_throttle = 100*((float)(RC_throttle_PWM - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_rudder   = 100*((float)(RC_rudder_PWM   - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  
  // Updating the "RC_Trim" value for each RC Channel input
  // see trim_control_surfaces() in radio.pde  
  RC_Roll_Trim     = 100.0*((float)(channel_roll->radio_trim     - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Pitch_Trim    = 100.0*((float)(channel_pitch->radio_trim    - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Throttle_Trim = 100.0*((float)(channel_throttle->radio_trim - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Rudder_Trim   = 100.0*((float)(channel_rudder->radio_trim   - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN)); 
  
  
  
  // Mission Start logic & actions
  altitude = -Z_position_Baro;
  personDistributionIndex = g.aa241x_distribution_index;
  
  if ( ALTITUDE_START < altitude && in_mission == 0 && allow_start == 1){
    if (control_mode == AUTO){ 
      in_mission = 1;
      CPU_time_start_ms = CPU_time_ms;
      battery_energy_consumed_start = battery_energy_consumed;
      sightedPerson[0] = 0;
      sightedPerson[1] = 0;
      sightedPerson[2] = 0;
      sighted_persons = 0;
      CPU_time_sight_ms = 0.0;
      mission_energy_consumed = 0.0;
      // record true positions of persons
      for (int i = 0; i < 3; i++) {
        X_person_truth[i] = persons_location[personDistributionIndex][i][0];
        Y_person_truth[i] = persons_location[personDistributionIndex][i][1];
        X_person_estimate[i] = 1e3;
        Y_person_estimate[i] = 1e3;
      }
    } else { 
      allow_start = 0;
    }
  }
  
  if (  altitude < ALTITUDE_START){
    allow_start = 1;
  }
  
  // Mission End logic
  if (in_mission == 1 && altitude < ALTITUDE_START && Air_speed < MIN_AIRSPEED){
    in_mission = 0;
  }
  if (in_mission == 1 && control_mode != AUTO){
    in_mission = 0;
  }
  if (in_mission == 1 && mission_energy_consumed > ENERGY_LIMIT){
    in_mission = 0;
  }
  
  // determine fisrt time of sighting all three persons
  if (sighted_persons == 0 && sightedPerson[0] == 1 && sightedPerson[1] == 1 && sightedPerson[2] == 1){
      CPU_time_sight_ms = CPU_time_ms - CPU_time_start_ms ;
      sighted_persons = 1;
  }
  
  // calculate mission score while in mission & update time since start of mission
  if (in_mission == 1){
    updateScore(X_person_truth, Y_person_truth);
    CPU_time_mission_ms = CPU_time_ms - CPU_time_start_ms;
  } else {
    CPU_time_mission_ms = 0.0;
    Score = 0.0;
  }
  
}

static void updateScore(float *X_person_truth, float *Y_person_truth){
  float sum = 0.0;
  float err;
  float errX;
  float errY;
  float t_sight = CPU_time_sight_ms/1000.0;
  if (CPU_time_sight_ms == 0.0){
    Score = 0.0;
    return;
  }
  
  if (mission_energy_consumed > ENERGY_LIMIT) return;
  
  for (int i = 0; i < 3; i++){
    errX = (X_person_estimate[i] - X_person_truth[i]);
    errY = (Y_person_estimate[i] - Y_person_truth[i]);
    err = sqrt(errX*errX + errY*errY);
    err = max(GAMMA, err);
    sum = sum + err;
  }  
  
  Score = ALPHA/sum + BETA/t_sight;
}


static void Log_Write_AA241X_AHF(void){
 if (g.aa241x_attitude_log_frequency == 1)
 {
   Log_Write_AA241X_HF();
 }
}

static void Log_Write_AA241X_AMF(void){
 if (g.aa241x_attitude_log_frequency != 1)
 {
   Log_Write_AA241X_HF();
 }
}


// Write an AA241X data packet to memory. Total length : 30 bytes
static void Log_Write_AA241X(void)
{
//    DataFlash.WriteByte(HEAD_BYTE1);
//    DataFlash.WriteByte(HEAD_BYTE2);
//    DataFlash.WriteByte(LOG_AA241X_MSG);
//    
//    DataFlash.WriteLong(((long)CPU_time_ms));
//    DataFlash.WriteLong(((long)GPS_time_ms));
//    
//    DataFlash.WriteLong((int32_t)(100.0*X_position));
//    DataFlash.WriteLong((int32_t)(100.0*Y_position));
//    DataFlash.WriteLong((int32_t)(100.0*Z_position_Baro));
//    DataFlash.WriteLong((int32_t)(100.0*Z_position_GPS));
//    
//    DataFlash.WriteInt((int16_t)(100.0*u_vel));
//    DataFlash.WriteInt((int16_t)(100.0*v_vel));
//    DataFlash.WriteInt((int16_t)(100.0*w_vel));
//    
//    DataFlash.WriteInt((int16_t)(100.0*X_velocity));
//    DataFlash.WriteInt((int16_t)(100.0*Y_velocity));
//    DataFlash.WriteInt((int16_t)(100.0*Z_velocity));
//    DataFlash.WriteInt((int16_t)(100.0*ground_speed));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(ground_course)));
//    
//    DataFlash.WriteInt((int16_t)(100.0*accel_x));
//    DataFlash.WriteInt((int16_t)(100.0*accel_y));
//    DataFlash.WriteInt((int16_t)(100.0*accel_z));
//    
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(roll)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(pitch)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(yaw)));
//    
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(roll_rate)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(pitch_rate)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(yaw_rate)));
//    
//    DataFlash.WriteInt((uint16_t)(100.0*Air_speed));
//    
//    DataFlash.WriteByte(gps->fix);
//    DataFlash.WriteByte(gps->num_sats);
//    DataFlash.WriteLong((long)loc.lat);
//    DataFlash.WriteLong((long)loc.lng);
//    
//    DataFlash.WriteInt((int)(100.0*battery_current));  // in centiAmps
//    DataFlash.WriteByte((byte)(10.0*battery_voltage));  // in deciVolts
//    DataFlash.WriteInt((int)(battery_energy_consumed)); // in Joules
//    
//    DataFlash.WriteInt((uint16_t)APM_RC.OutputCh_current(CH_1)); // PWM to servo on CH_1, integer between 900 and 2100.  
//    DataFlash.WriteInt((uint16_t)APM_RC.OutputCh_current(CH_2)); // PWM to servo on CH_2
//    DataFlash.WriteInt((uint16_t)APM_RC.OutputCh_current(CH_3)); // PWM to servo on CH_3
//    DataFlash.WriteInt((uint16_t)APM_RC.OutputCh_current(CH_4)); // PWM to servo on CH_4
//    
//    DataFlash.WriteInt((uint16_t)AA241X_roll_servo_PWM);     // PWM commanded to servo on CH_1, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)AA241X_pitch_servo_PWM);    // PWM commanded to servo on CH_2, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)AA241X_throttle_servo_PWM); // PWM commanded to servo on CH_3, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)AA241X_rudder_servo_PWM);   // PWM commanded to servo on CH_4, by AUTO_ControlLaw, integer between 800 and 2200
//  
//    DataFlash.WriteInt((uint16_t)RC_roll_PWM);     // PWM commanded to servo on CH_1, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)RC_pitch_PWM);    // PWM commanded to servo on CH_2, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)RC_throttle_PWM); // PWM commanded to servo on CH_3, by AUTO_ControlLaw, integer between 800 and 2200
//    DataFlash.WriteInt((uint16_t)RC_rudder_PWM);   // PWM commanded to servo on CH_4, by AUTO_ControlLaw, integer between 800 and 2200
//   
//    DataFlash.WriteByte(control_mode); 
//    DataFlash.WriteInt((int)(avg_delta_ms_main_loop));  // Time required to execute main looop, should be less than 20ms to ensure the fast loop is called at 50Hz
//    
//    DataFlash.WriteByte(in_mission);
//    DataFlash.WriteInt((int)(mission_energy_consumed)); // in Joules
//    DataFlash.WriteLong(((long)CPU_time_sight_ms));  
//    DataFlash.WriteLong((int32_t)(100.0*(X_person_estimate[0] - X_person_truth[0])));
//    DataFlash.WriteLong((int32_t)(100.0*(Y_person_estimate[0] - Y_person_truth[0])));
//    DataFlash.WriteLong((int32_t)(100.0*(X_person_estimate[1] - X_person_truth[1])));
//    DataFlash.WriteLong((int32_t)(100.0*(Y_person_estimate[1] - Y_person_truth[1])));
//    DataFlash.WriteLong((int32_t)(100.0*(X_person_estimate[2] - X_person_truth[2])));
//    DataFlash.WriteLong((int32_t)(100.0*(Y_person_estimate[2] - Y_person_truth[2])));
//    DataFlash.WriteLong((long)(100.0*Score));
//    DataFlash.WriteByte((byte)Competition_day);
//    DataFlash.WriteByte((byte)personDistributionIndex);
//    
//    DataFlash.WriteByte(END_BYTE);
}


// Read an attitude packet
static void Log_Read_AA241X()
{
  
//    static int flag = 1;
//    if (flag == 1){
//      // Printing Header for AA241X data log
//      Serial.printf_P(PSTR("AA241X, CPU_time_ms, GPS_time_ms, "));
//      Serial.printf_P(PSTR("X_position_cm, Y_poisiton_cm, Z_position_Baro_cm, Z_position_GPS_cm, "));
//      Serial.printf_P(PSTR("u_vel_cm_s, v_vel_cm_s, w_vel_cm_s, "));
//      Serial.printf_P(PSTR("X_velocity_cm_s, Y_velocity_cm_s, Z_velocity_cm_s, GroundSpeed_cm_s, GroundCourse_centiDeg, "));
//      Serial.printf_P(PSTR("accel_x_cm_s^2, accel_y_cm_s^2, accel_z_cm_s^2, " ));
//      Serial.printf_P(PSTR("roll_centiDeg, pitch_centiDeg, yaw_centiDeg, " ));
//      Serial.printf_P(PSTR("roll_rate_centiDeg_s, pitch_rate_centiDeg_s, yaw_rate_centiDeg_s, " ));
//      Serial.printf_P(PSTR("Airspeed_cm_s, " ));
//      Serial.printf_P(PSTR("GPS_Fix, Num_Sats, Latitude, Longitude, " ));
//      Serial.printf_P(PSTR("Battery_current_centiAmps, Battery_voltage_deciVolts, Battery_energy_consumed_Joules, " ));
//      Serial.printf_P(PSTR("CH1_servo_PWM, CH2_servo_PWM, CH3_servo_PWM, CH4_servo_PWM, " ));
//      Serial.printf_P(PSTR("AA241X_roll_servo_PWM, AA241X_pitch_servo_PWM, AA241X_throttle_servo_PWM, AA241X_rudder_servo_PWM, "));
//      Serial.printf_P(PSTR("RC_roll_PWM, RC_pitch_PWM, RC_throttle_PWM, RC_rudder_PWM, "));
//      Serial.printf_P(PSTR("control_mode, "));
//      Serial.printf_P(PSTR("Avg_loop_time_ms, "));
//      Serial.printf_P(PSTR("in_mission, mission_energy_consumed, t_sight, "));
//      Serial.printf_P(PSTR("errX1_cm, errY1_cm, errX2_cm, errY2_cm, errX3_cm, errY3_cm, "));
//      Serial.printf_P(PSTR("Score, "));
//      Serial.printf_P(PSTR("Competition_day, personDistributionIndex "));
//      
//      Serial.printf_P(PSTR("\n"));
//      flag = 0;
//    }
//    Serial.printf_P(PSTR("AA241X, "));
//    
//    byte b[6];
//    int16_t i[26];
//    int32_t l[16];
//    
//    l[0] = DataFlash.ReadLong();  // CPU time
//    l[1] = DataFlash.ReadLong();  // GPS time
//    Serial.printf_P(PSTR("%lu, %lu, "),
//                    (long)l[0], (long)l[1]);
//    
//    l[2] = DataFlash.ReadLong();  // X_position
//    l[3] = DataFlash.ReadLong();  // Y_position
//    l[4] = DataFlash.ReadLong();  // Z_position_Baro
//    l[5] = DataFlash.ReadLong();  // Z_position_GPS
//    
//    i[0] = DataFlash.ReadInt();   // u_vel
//    i[1] = DataFlash.ReadInt();   // v_vel
//    i[2] = DataFlash.ReadInt();   // w_vel
//    
//    i[3] = DataFlash.ReadInt();   // X_velocity
//    i[4] = DataFlash.ReadInt();  // Y_velocity
//    i[5] = DataFlash.ReadInt();  // Z_velocity
//    i[6] = DataFlash.ReadInt();  // ground speed
//    l[6] = DataFlash.ReadLong(); // ground course
//    
//    i[7] = DataFlash.ReadInt();  // accel_x
//    i[8] = DataFlash.ReadInt();  // accel_y
//    i[9] = DataFlash.ReadInt();  // accel_z
//    
//    l[7] = DataFlash.ReadLong(); // roll
//    l[8] = DataFlash.ReadLong(); // pitch
//    l[9] = DataFlash.ReadLong(); // yaw
//    
//    l[10] = DataFlash.ReadLong(); // roll_rate
//    l[11] = DataFlash.ReadLong(); // pitch_rate
//    l[12] = DataFlash.ReadLong(); // yaw_rate
//
//    Serial.printf_P(PSTR("%ld, %ld, %ld, %ld, %d, %d, %d, %d, %d, %d, %d, %ld, %d, %d, %d, %ld, %ld, %ld, %ld, %ld, %ld, "), 
//                            l[2], l[3], l[4], l[5], i[0], i[1], i[2], i[3], i[4], i[5], i[6], l[6], 
//                            i[7], i[8], i[9], l[7], l[8], l[9], l[10], l[11], l[12]);
//
//    i[10] = DataFlash.ReadInt();  // Air speed
//    Serial.printf_P(PSTR("%u, "), i[10]);
//    
//    b[0] = DataFlash.ReadByte(); // GPS fix
//    b[1] = DataFlash.ReadByte(); // GPS num sats
//    l[13] = DataFlash.ReadLong(); // Latitude
//    l[14] = DataFlash.ReadLong(); // Longitude
//    Serial.printf_P(PSTR("%d, %d, %4.7f, %4.7f, "), (int)b[0], (int)b[1], l[13]/t7, l[14]/t7);
//    
//    i[11] = DataFlash.ReadInt();  // Battery current
//    b[2] = DataFlash.ReadByte(); // Battery voltage
//    i[12] = DataFlash.ReadInt();  // Battery energy consumed
//    Serial.printf_P(PSTR("%u, %u, %u, " ), i[11], b[2], i[12]);
//      
//    i[13] = DataFlash.ReadInt();  // Output CH1
//    i[14] = DataFlash.ReadInt();  // Output CH2
//    i[15] = DataFlash.ReadInt();  // Output CH3
//    i[16] = DataFlash.ReadInt();  // Output CH4
//    Serial.printf_P(PSTR("%u, %u, %u, %u, "), i[13], i[14], i[15], i[16]);
//      
//    i[17] = DataFlash.ReadInt();  // AA241X_roll_servo_PWM
//    i[18] = DataFlash.ReadInt();  // AA241X_pitch_servo_PWM
//    i[19] = DataFlash.ReadInt();  // AA241X_throttle_servo_PWM
//    i[20] = DataFlash.ReadInt();  // AA241X_rudder_servo_PWM
//    Serial.printf_P(PSTR("%u, %u, %u, %u, "), i[17], i[18], i[19], i[20]);
//    
//    i[21] = DataFlash.ReadInt();  // RC_roll_PWM
//    i[22] = DataFlash.ReadInt();  // RC_pitch_PWM
//    i[23] = DataFlash.ReadInt();  // RC_throttle_PWM
//    i[24] = DataFlash.ReadInt();  // RC_rudder_PWM
//    Serial.printf_P(PSTR("%u, %u, %u, %u, "), i[21], i[22], i[23], i[24]);
//    
//    b[3] = DataFlash.ReadByte(); // Control mode
//    Serial.printf_P(PSTR("%u, " ), b[3]);
//    
//    i[25] = DataFlash.ReadInt(); // Average loop time, ms
//    Serial.printf_P(PSTR("%d, " ), i[25]);
//    
//    b[3] = DataFlash.ReadByte(); // in_mission flag
//    Serial.printf_P(PSTR("%u, " ), b[3]);
//    
//    i[12] = DataFlash.ReadInt();  // Mission energy consumed, Joules
//    Serial.printf_P(PSTR("%u, " ), i[12]);
//    
//    l[0] = DataFlash.ReadLong();  // CPU time of sight,  ms
//    Serial.printf_P(PSTR("%lu, "), (long)l[0]);
//    
//    l[2] = DataFlash.ReadLong();  // X_position error person 1, cm
//    l[3] = DataFlash.ReadLong();  // Y_position error person 1, cm
//    l[4] = DataFlash.ReadLong();  // X_position error person 2, cm
//    l[5] = DataFlash.ReadLong();  // Y_position error person 2, cm
//    l[6] = DataFlash.ReadLong();  // X_position error person 3, cm
//    l[7] = DataFlash.ReadLong();  // Y_position error person 3, cm
//    Serial.printf_P(PSTR("%ld, %ld, %ld, %ld, %ld, %ld, "), 
//                            l[2], l[3], l[4], l[5], l[6], l[7]);
//    
//    l[2] = DataFlash.ReadLong();  // Mission score
//    Serial.printf_P(PSTR("%4.7f, "), l[2]/100.0);
//    
//    b[0] = DataFlash.ReadByte(); // Competition_day (Number between 1 and 8)
//    b[1] = DataFlash.ReadByte(); // personDistributionIndex, shoudl be 0 if it will count towards the competition
//    Serial.printf_P(PSTR("%d, %d, "), (int)b[0], (int)b[1]);
//    
//    Serial.printf_P(PSTR("\n"));
}

// Camera functions
static struct snapshot takeSnapshot(float X_current, float Y_current, float altitude, bool gpsOK, float mission_energy_consumed){
    uint32_t current_time_ms;
    current_time_ms = millis();
    static uint32_t last_time_ms = 0;
    snapshot mySnap;

    mySnap.pictureTaken = 0;
    mySnap.timeOfPicture_ms = 0;
    mySnap.personsInPicture[0] = 0;
    mySnap.personsInPicture[1] = 0;
    mySnap.personsInPicture[2] = 0;
    mySnap.diameterOfPicture = 0.0;
    mySnap.centerOfPictureX = 0.0;
    mySnap.centerOfPictureY = 0.0;

    if ( gpsOK && (current_time_ms - last_time_ms) >= WAIT_TIME_MS  && MIN_ALTITUDE_SNAPSHOT <= altitude && altitude <= MAX_ALTITUDE && mission_energy_consumed < ENERGY_LIMIT){
        mySnap.pictureTaken = 1;
        mySnap.timeOfPicture_ms = CPU_time_mission_ms;
        mySnap.centerOfPictureX = X_current;
        mySnap.centerOfPictureY = Y_current;
        mySnap.diameterOfPicture = get_diameterOfPicture(altitude);  
        personEstimates(mySnap, X_current, Y_current, altitude);
        
        last_time_ms = current_time_ms;
    }
    return mySnap;
};


static void personEstimates(snapshot &mySnap, float X_current, float Y_current, float altitude){
    float X_person_i;
    float Y_person_i;
    float dX;
    float dY;
    float pict_radius;
    
    pict_radius = 0.5*mySnap.diameterOfPicture;
    
    // check which persons are in the field of view, and get estimate if in FOV
    for (int i = 0; i < 3; i++) {
        X_person_i = persons_location[personDistributionIndex][i][0];
        Y_person_i = persons_location[personDistributionIndex][i][1];
        dX = (X_current - X_person_i);
        dY = (Y_current - Y_person_i);

        if ((dX*dX + dY*dY) <= pict_radius*pict_radius) {
          sightedPerson[i] = 1;
          mySnap.personsInPicture[i] = 1;
          getPersonEstimate(mySnap, X_person_i, Y_person_i, altitude, i);
        }
    }
};

// Random number generator
unsigned long GetUint(unsigned long& u, unsigned long& v);
float GetUniform(unsigned long& u, unsigned long& v);

unsigned long u = 521288629;
unsigned long v = 362436069;

unsigned long GetUint(unsigned long& u, unsigned long& v)
{
    v = 36969*(v & 65535) + (v >> 16);
    u = 18000*(u & 65535) + (u >> 16);
    return (v << 16) + u;
}

float GetUniform(unsigned long& u, unsigned long& v)
{
    // 0 <= u <= 2^32
    unsigned long z = GetUint(u, v);
    // The magic number is 1/(2^32 + 1) and so result is positive and less than 1.
    return z*2.328306435996595e-10;
}

static void getPersonEstimate(snapshot &mySnap, float X_person_i, float Y_person_i, float altitude, int i){
  float diam;
  
  diam = getDiameterOfPersonEstimate(altitude);
  mySnap.diameterOfPersonEstimate[i] = diam;
  
  float r = GetUniform(u,v)*0.5*diam;  // uniform from 0 ~ 0.5*diam
  float theta = GetUniform(u,v)*2*PI;  // uniform from 0 ~ 2*PI
  
  mySnap.centerOfPersonEstimateX[i] = r*cos(theta) + X_person_i;
  mySnap.centerOfPersonEstimateY[i] = r*sin(theta) + Y_person_i;
}

static float get_diameterOfPicture(float altitude){
  return MIN_FOV_DIAM + (altitude-MIN_ALTITUDE_SNAPSHOT)*(MAX_FOV_DIAM - MIN_FOV_DIAM)/(MAX_ALTITUDE - MIN_ALTITUDE_SNAPSHOT);
}

static float getDiameterOfPersonEstimate(float altitude){
  return MIN_ESTIMATE_DIAM + (altitude-MIN_ALTITUDE_SNAPSHOT)*(MAX_ESTIMATE_DIAM - MIN_ESTIMATE_DIAM)/(MAX_ALTITUDE - MIN_ALTITUDE_SNAPSHOT);
}


//// Communications with Gumstix 
//void sendSnapshotDataToGumstix(float X_Estimate, float Y_Estimate, float R_Estimate, uint16_t Estimate_Number, float CPU_mission_time_ms, float mission_energy_consumed) {
//   return;
//}
//
//union Data{
//  byte bytes[4];
//  float f;
//};
//
//
//
//#define PACKET_HEADER   0xAA
//#define PACKET_FOOTER   0x55
//
//#define SEND_TEST_DATA  0x01
//#define POSITION        0x02
//#define GET_NAV         0x03
//#define PICTURE_DATA    0x04
//#define PERSON_ESTIMATE 0x05
//#define MISSION_STATUS  0x06
//#define GET_ESTIMATES   0x07
//
//
//char getNavFromGumstix(void){
//   return getNav(WayPointX,WayPointY,WayPointZ,WayPointHeading);
//}
//
//char sendPosition(float X_current, float Y_current, float Z_current, float yaw){
//  float data[] = {X_current, Y_current, Z_current, yaw};
//  return sendFloatPacket(POSITION, data);
//}
//char sendPositionToGumstix(void){
// return sendPosition(X_position, Y_position, Z_position_Baro, yaw);
//}
//
//char sendMissionStatus(){
//  float data[] = {in_mission, CPU_time_mission_ms, mission_energy_consumed, Score};
//  return sendFloatPacket(MISSION_STATUS, data);
//}
//
//// pictureNumber starts counting from 0
//char sendPictureData(float centerOfPictureX, float centerOfPictureY, float diameterOfPicture, float pictureNumber) {
//  float data[] = {centerOfPictureX, centerOfPictureY, diameterOfPicture, pictureNumber};
//  return sendFloatPacket(PICTURE_DATA, data);
//}
//
//// estimateNumber starts counting from 0
//char sendPersonEstimate(float centerOfPersonEstimateX, float centerOfPersonEstimateY, float diameterOfPersonEstimate, float personIndex, float estimateNumber){
//  float data[] = {centerOfPersonEstimateX, centerOfPersonEstimateY, diameterOfPersonEstimate, personIndex, estimateNumber};
//  return sendFloatPacket(PERSON_ESTIMATE, data);
//}
//
//char getNav(float * WayPointX, float * WayPointY, float * WayPointZ, float * WayPointHeading){
//  if (requestDataFromGumstix(GET_NAV) == -1){
//    return -1;
//  }
//  
//  for (int i = 0; i < 3; i++){
//    WayPointX[i] = readFloat();
//    WayPointY[i] = readFloat();
//    WayPointZ[i] = readFloat();
//    WayPointHeading[i] = readFloat();
//  }
//  
//}
//
//char requestDataFromGumstix(byte request_type){
//  float data[] = {};
//  sendFloatPacket(request_type, data);
//  
//  int count = 0;
//  while (Serial2.available() == 0){
//      delayMicroseconds(100);
//      count = count + 1;
//      if (count == 10){
//        return -1;
//      }
//  }
//  return 1;
//}
//
//char getEstimates(float * X_person_estimate, float * Y_person_estimate){
//  if (requestDataFromGumstix(GET_ESTIMATES) == -1){
//    return -1;
//  }
// 
//  for (int i = 0; i < 3; i++){
//    X_person_estimate[i] = readFloat();
//    Y_person_estimate[i] = readFloat();
//  }
//  return 1;
//}
//
//char sendTestData(void){
//  float data[] = {1.0, 2.0, 1.0, 2.0, 1.0, 2.0};
//  return sendFloatPacket(SEND_TEST_DATA, data);
//}
//
//char sendFloatPacket(byte packet_id, float data[]){
//  byte nFloats = sizeof(data)/4;
//  Serial2.write(PACKET_HEADER);
//  Serial2.write(packet_id);
//  Serial2.write((byte)nFloats);
//  for (int i = 0; i < nFloats; i++){
//  sendFloat(data[i]);
//  }
//  Serial2.write(PACKET_FOOTER);
//  return gotACK();
//}
//
//char sendFloat(float f){
//  byte * p_f = (byte *) &f;
//  Serial2.write(p_f[0]);
//  Serial2.write(p_f[1]);
//  Serial2.write(p_f[2]);
//  Serial2.write(p_f[3]);
//  return 1;
//}
//
//float readFloat(void){
// union Data u;
// u.bytes[0] = Serial2.read();
// u.bytes[1] = Serial2.read();
// u.bytes[2] = Serial2.read();
// u.bytes[3] = Serial2.read();
// return u.f;
//}
//
//char gotACK(){
//  delayMicroseconds(100);
//  char ack = Serial2.read();
//  return ack == GUMSTIX_ACK;
//}
//char readTestData(void){
//  
//    char byte1 = Serial2.read();
//    char byte2 = Serial2.read();
//    if (byte1 == 1 && byte2 == 2){
//      return 1;
//    }
//    return 0;
//}
//
//void toggleLEDA(void) {
//  static char myFlag = 0;
//  if (myFlag == 0){
//    digitalWrite(A_LED_PIN, LED_OFF);
//    myFlag = 1;
//  } else {
//    digitalWrite(A_LED_PIN, LED_ON);
//    myFlag = 0;
//  }
//}
//
//void toggleLEDC(void) {
//  static char myFlag = 0;
//  if (myFlag == 0){
//    digitalWrite(C_LED_PIN, LED_OFF);
//    myFlag = 1;
//  } else {
//    digitalWrite(C_LED_PIN, LED_ON);
//    myFlag = 0;
//  }
//}
//

