#pragma once

#include "conf.h"

#define COM_REL_VEL_REF           1
#define COM_ABS_VEL_REF           2
#define COM_REL_POS_REF           3
#define COM_ABS_POS_REF           4
#define COM_SET_POS_GAINS         5
#define COM_SET_VEL_GAINS         6
#define COM_SET_U                 13
#define COM_TEST_MOTOR_CONNECTION 18

#define COM_PRINT_LONG_VEL_REPORT  7
#define COM_PRINT_SHORT_VEL_REPORT 8
#define COM_PRINT_LONG_POS_REPORT  9
#define COM_PRINT_SHORT_POS_REPORT 10
#define COM_PRINT_NO_REPORT        11
#define COM_SEND_DATA_FOR_MATLAB   12
#define COM_PRINT_HELP             14
#define COM_GET_POSITION_LONG      15
#define COM_LOG_POSITION           16
#define COM_LOG_CURRENT            17
#define COM_GET_LOG_SIZE_INTEGER   19
#define COM_GET_LOOPRATE_UINT8     20
#define COM_CHANGE_USER_POS_DIR    21
#define COM_RESET_ENCODER_POS      24
#define COM_GET_POS_GAINS          25
#define COM_GET_VEL_GAINS          26

#define COM_SET_LOOPRATE         22
#define COM_SET_PRINTING_PERIOD  23
#define COM_REL_PROF_POS_REF     27
#define COM_ABS_PROF_POS_REF     28
#define COM_SET_DRIVER_STATE     29
#define COM_SET_OPERATING_MODE   30
#define COM_REL_MAX_PROF_POS_REF 31
#define COM_ABS_MAX_PROF_POS_REF 32

#define COM_SET_ESTIMATION_PARAMETERS 33  // M-ID 33 PWM R
//
//

//
//
//#define MANUAL_MODE           0
//#define POSITION_MODE         1
//#define PROFILE_POSITION_MODE 2
//#define VELOCITY_MODE         3
//#define PROFILE_VELOCITY_MODE 4
//#define CURRENT_MODE          5
//#define HOMING_MODE           6
