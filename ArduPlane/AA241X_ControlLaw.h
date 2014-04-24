#ifndef AA241X_CONTROLLAW_H
#define AA241X_CONTROLLAW_H

// NOTE: the parameter name has to have less than 14 characters! If not you get the error "initialize-string for array chars is too long"
// My parameter 1
#define MODE_SELECT       g.aa241x_1      // Don't change g.aa241x_1 ! Replace my_param_1 with whatever name you want to access the parameter with.  See the example in ControlLaw.ino   
#define AA241X_1_NAME     "AA241_FLT_MODE" // Don't change AA241X_1_NAME ! Replace my_paramName1 with whateve name you want to see on the Mission Planner GCS 
#define AA241X_1_DEFAULT  1.0            // replace 10.0 with whatever default value you wante to this parameter to have, when reloading the code 
// My parameter 2
#define MODE_BIT_1         g.aa241x_2
#define AA241X_2_NAME     "AA241_BIT_1" 
#define AA241X_2_DEFAULT  0 
// My parameter 3
#define RLL_2_SRV_P       g.aa241x_3
#define AA241X_3_NAME     "AA241_Roll_P" 
#define AA241X_3_DEFAULT  5.0 
// My parameter 4
#define RLL_2_SRV_I       g.aa241x_4
#define AA241X_4_NAME     "AA241_Roll_I" 
#define AA241X_4_DEFAULT  15.0 
// My parameter 5
#define RLL_2_SRV_D       g.aa241x_5
#define AA241X_5_NAME     "AA241_Roll_D" 
#define AA241X_5_DEFAULT  100.0 
// My parameter 6
#define PTCH_2_SRV_P      g.aa241x_6
#define AA241X_6_NAME     "AA241_Pitch_P" 
#define AA241X_6_DEFAULT  10.0 
// My parameter 7
#define PTCH_2_SRV_I      g.aa241x_7
#define AA241X_7_NAME     "AA241_Pitch_I" 
#define AA241X_7_DEFAULT  1.0 
// My parameter 8
#define PTCH_2_SRV_D      g.aa241x_8
#define AA241X_8_NAME     "AA241_Pitch_D" 
#define AA241X_8_DEFAULT  1.0 
// My parameter 9
#define SPD_2_SRV_P       g.aa241x_9
#define AA241X_9_NAME     "AA241_Thr_P" 
#define AA241X_9_DEFAULT  1.0 
// My parameter 10
#define SPD_2_SRV_I        g.aa241x_10
#define AA241X_10_NAME     "AA241_Thr_I" 
#define AA241X_10_DEFAULT  0.0 
// My parameter 11
#define SPD_2_SRV_D        g.aa241x_11
#define AA241X_11_NAME     "AA241_Thr_D" 
#define AA241X_11_DEFAULT  0.0 
// My parameter 12
#define HEAD_2_SRV_P       g.aa241x_12
#define AA241X_12_NAME     "AA241_Head_P" 
#define AA241X_12_DEFAULT  1.0 
// My parameter 13
#define HEAD_2_SRV_I       g.aa241x_13
#define AA241X_13_NAME     "AA241_Head_I" 
#define AA241X_13_DEFAULT  0.0 
// My parameter 14
#define HEAD_2_SRV_D       g.aa241x_14
#define AA241X_14_NAME     "AA241_Head_D" 
#define AA241X_14_DEFAULT  0.0 
// My parameter 15
#define RUD_2_SRV_P        g.aa241x_15
#define AA241X_15_NAME     "AA241_Rud_P" 
#define AA241X_15_DEFAULT  1.0 
// My parameter 16
#define RUD_2_SRV_I        g.aa241x_16
#define AA241X_16_NAME     "AA241_Rud_I" 
#define AA241X_16_DEFAULT  0.0 
// My parameter 17
#define RUD_2_SRV_D        g.aa241x_17
#define AA241X_17_NAME     "AA241_Rud_D" 
#define AA241X_17_DEFAULT  0.0 
// My parameter 18
#define ALT_HOLD_P         g.aa241x_18
#define AA241X_18_NAME     "AA241_Alt_P" 
#define AA241X_18_DEFAULT  1.0 
// My parameter 19
#define ALT_HOLD_I         g.aa241x_19
#define AA241X_19_NAME     "AA241_Alt_I" 
#define AA241X_19_DEFAULT  0.0
// My parameter 20
#define ALT_HOLD_D         g.aa241x_20
#define AA241X_20_NAME     "AA241_Alt_D" 
#define AA241X_20_DEFAULT  0.0 

#endif  /* AA241X_CONTROLLAW_H */
