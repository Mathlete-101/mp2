

#ifndef ROVERCONSTANTS_H
#define ROVERCONSTANTS_H

#define SAMPLE_TIME_MS 10

#define BATTERY_VOLTAGE 13.2
#define PI 3.141593

// driving 
#define ENC_CNT_PER_REV     5281
#define WHEEL_RADIUS_M      0.1574 
#define TRACK_WIDTH_M       0.5476
#define WHEEL_UNUSABLE_V    0.47


// digging
#define ENC_CNT_PER_REV_DIG 7156

// dumping 
// TO DO: FIGURE OUT ACTUAL NUMBERS
#define ENC_CNT_PER_REV_DUMP 34471
#define HALF_ROTATION_CNV_BELT (ENC_CNT_PER_REV_DUMP / 2)
#define INC_LOADING_ROTATION_CNV_BELT (ENC_CNT_PER_REV_DUMP / 4) // fit two loads on belt???
#define MAX_LOADS_CNV_BELT 2
#define DUMP_BELT_UNUSABLE_V 0.67  // unused


// pins
#define RIGHT_F_PWM 9
#define RIGHT_F_DIR 8   // spins opposite way, encoders inc with current forward
#define ENCR_A_FR 20
#define ENCR_B_FR 27


#define LEFT_F_PWM 7
#define LEFT_F_DIR 6
#define ENCR_A_FL 2
#define ENCR_B_FL 25

#define RIGHT_B_PWM 5
#define RIGHT_B_DIR 16   // spins opposite way, encoders inc with current forward
#define ENCR_A_BR 3
#define ENCR_B_BR 23

#define LEFT_B_PWM 12
#define LEFT_B_DIR 17
#define ENCR_A_BL 19
#define ENCR_B_BL 29

#define ACT_MTR_PWM 11        
#define ACT_MTR_DIR 15 

#define DB_MTR_PWM 10         
#define DB_MTR_DIR 14 
#define DB_ENC_A 21
#define DB_ENC_B 45

#define CNVB_MTR_PWM 4
#define CNVB_MTR_DIR 30
#define CNVB_ENC_A 18
#define CNVB_ENC_B 47

#define LM_SW_1 49
#define LM_SW_2 48

#endif

