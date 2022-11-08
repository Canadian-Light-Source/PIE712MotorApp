/*
FILENAME...     PIE712Controller.cpp
 
*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************
 
Version:        $Revision: 1.1 $
Modified By:    $Author: Russ Berg (bergr) $
Last Modified:  $Date: 2018/09/27 13:19:39CST $
HeadURL:        $URL$

Based on drvMotorSim.c

Mark Rivers
December 13, 2009

*/

#ifndef PI_E712_ASYN_DRIVER_INCLUDED_
#define PI_E712_ASYN_DRIVER_INCLUDED_

#include <epicsTime.h>
#include <epicsThread.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include "PIInterface.h"
#include "PIE712_datarecorder_defs.h"

#include "picontrollererrors.h"

#define mmToMicrons(x) x*1000.0 
#define micronsToMMs(x) x*0.001

#define MAX_PIEZO_RANGE 100.0 /* microns */
#define MAX_PIEZO_EGU 50.0 /* microns */
#define MIN_PIEZO_EGU -50.0 /* microns */

#define MAX_MOVE_WITH_FORCED_STATUS 0.5 /* microns */
#define MAX_MOVE_WITH_FORCED_STATUS_VAL -9999 /* just a unique number that must be less than -1*/

/* below and above the voltages below a warning bo record will be set indicating that a warning that
  the stage is nearing the outer limits of its range has occurred */
#define MIN_WARN_VOLTS -10.0
#define MAX_WARN_VOLTS 110.0


#define WVGEN_1_RUNNING 1
#define WVGEN_2_RUNNING 2
#define WVGEN_3_RUNNING 4
#define WVGEN_4_RUNNING 8

#define WVGEN_TBL1_ID  1
#define WVGEN_TBL2_ID  2
#define WVGEN_TBL3_ID  3
#define WVGEN_TBL4_ID  4

#define WVGEN_STRTMODE_DO_NOT_START 	0
#define WVGEN_STRTMODE_IMEDDIATELY	 	1
#define WVGEN_STRTMODE_EXTERNAL_TRIG  2

#define WVGEN_FLAGS_USE_AND_REINIT	64
#define WVGEN_FLAGS_USE							128
#define WVGEN_FLAGS_START_AT_ENDPOS	256




/* this is the number of available data ppoints in E712 */
#define WAVE_MAX_NUM_SAMPLES 262144
#define MAX_NUM_WAVE_TABLES 120

/* Data acquisition modes */
#define MODE_NORMAL 					0
#define MODE_LINE_UNIDIR 			1	
#define MODE_LINE_BIDIR	 			2
#define MODE_POINT 						3
#define MODE_COARSE 					4


/* Force Done move status Flags */
#define FORCE_DONE_NORMAL	0	/* normal status */
#define FORCE_DONE_FORCED 1 /* force it to Done no matter what */
#define FORCE_DONE_INTERNAL_TIMED 2 /* force it to done after an internally timed number of poll iterations */


/* GCS trigger mode param defines, as defined in E-711/712 PZ195E Release 1.3.1 manual */
#define TM_POSITION_DIST 			0
#define TM_ON_TARGET					1
#define TM_MINMAX_THRESHOLD 	2
#define TM_GENERATOR_TRIGGER	3

#define DEFAULT_LOW_LIMIT -10000
#define DEFAULT_HI_LIMIT   10000
#define DEFAULT_HOME       0
#define DEFAULT_START      0

#define MARKER_WIDTH 0.5

#define MAX_NR_AXES 6



#define PI_SUP_POSITION_String		"PI_SUP_POSITION"
#define PI_SUP_TARGET_String			"PI_SUP_TARGET"
#define PI_SUP_SERVO_String				"PI_SUP_SERVO"
#define PI_SUP_LAST_ERR_String		"PI_SUP_LAST_ERR"
#define PI_SUP_PIVOT_X_String			"PI_SUP_PIVOT_X"
#define PI_SUP_PIVOT_Y_String			"PI_SUP_PIVOT_Y"
#define PI_SUP_PIVOT_Z_String			"PI_SUP_PIVOT_Z"
#define PI_SUP_RBPIVOT_X_String		"PI_SUP_RBPIVOT_X"
#define PI_SUP_RBPIVOT_Y_String		"PI_SUP_RBPIVOT_Y"
#define PI_SUP_RBPIVOT_Z_String		"PI_SUP_RBPIVOT_Z"

/* params specific to this driver */
#define P_SetMarkerString     		"PI_SET_MARKER"		/* asynFloat64,    r/w */
#define P_FWString            		"PI_FW_STRING"			/* asynOctet,    r */
#define P_PowerString           	"PI_POWER"					/* asynInt32,   r/w  */
#define P_SysStatusString       	"PI_SYS_STATUS"		/* asynInt32,   r/w  */
#define P_OutputVoltString     		"PI_OUTPUT_VOLT"	/* asynFloat64,    r/w */
                                	
#define P_ModeString           		"PI_MODE"					/* asynInt32,   r/w  */
#define P_MarkerStartString     	"PI_MARKER_START"	/* asynFloat64,    r/w */
#define P_MarkerStopString     		"PI_MARKER_STOP"		/* asynFloat64,    r/w */
#define P_ScanStartString     		"PI_SCAN_START"		/* asynFloat64,    r/w */
#define P_ScanStopString     			"PI_SCAN_STOP"			/* asynFloat64,    r/w */
#define P_SetVeloString     			"PI_SET_VELO"			/* asynFloat64,    r/w */
#define P_SetStartupPosString   	"PI_SET_STARTUP_POS"			/* asynFloat64,    r/w */
#define P_ReInitString   					"PI_RE_INITIALIZE"			/* asynInt32,    r/w */

#define P_AutoZeroString   				"PI_AUTO_ZERO"			/* asynInt32,    r/w */
#define P_SelectEncoderSrcString  "PI_SELECT_ENCODER_SOURCE"			/* asynInt32,    r/w */
#define P_SelectOutputDirString   "PI_SELECT_OUTPUT_DIRECTION"			/* asynInt32,    r/w */
#define P_AdcSensorString     		"PI_ADC_SENSOR"			/* asynParamFloat64,    r */
#define P_CapSensorString     		"PI_CAP_SENSOR"		/* asynFloat64,    r/w */
#define P_ZeroADSensorString   		"PI_ZERO_AD_SENSOR"			/* asynInt32,    r/w */                       
#define P_ForceDoneString   			"PI_FORCE_DONE"			/* asynInt32,    r/w */                                	
#define P_ForceDoneWaitTimeString 	"PI_FORCE_DONE_WAIT_TIME"		/*	how long to wait until driver forces DONE on move  */         	

#define P_TriggerOutputSelString 	"PI_TRIG_OUTPUT_SEL"			/* asynInt32,    r/w */                                	

#define P_DigitalFilterTypeString		"PI_DIGFILT_TYPE"		 /*	1	10	INT	input	Digital Filter Type */ 
#define P_DigitalFilterBWidthString "PI_DIGFILT_BWIDTH"	/*	1	10	FLOAT	input	Digital Filter Bandwidth  */
#define P_DigitalFilterORderString 	"PI_DIGFILT_ORDER"		/*	1	10	INT	input	Digital Filter Order  */
#define P_DigitalFilterParm1String 	"PI_DIGFILT_PARM1"		/*	1	10	FLOAT	input	User Filter Param. 1  */
#define P_DigitalFilterParm2String 	"PI_DIGFILT_PARM2"		/*	1	10	FLOAT	input	User Filter Param. 2  */
#define P_DigitalFilterParm3String 	"PI_DIGFILT_PARM3"		/*	1	10	FLOAT	input	User Filter Param. 3  */
#define P_DigitalFilterParm4String 	"PI_DIGFILT_PARM4"		/*	1	10	FLOAT	input	User Filter Param. 4  */
#define P_DigitalFilterParm5String 	"PI_DIGFILT_PARM5"		/*	1	10	FLOAT	input	User Filter Param. 5  */

#define P_VoltRangeWarningString 		"PI_VOLT_RANGE_WARNING" /* asynInt32,    r/w */                       
#define P_ResetVoltRangeWarningString 		"PI_RESET_VOLT_RANGE_WARNING" /* asynInt32,    r/w */                       
#define P_ServoOffAndCenterString		"PI_SERVO_OFF_AND_CENTER"	/* asynInt32,    r/w */ 

#define P_CapSensorParmBString 	"PI_CAPSENS_B_PARM"		/*	Capsensor to EGU conversion parm for y=mx+B  */
#define P_CapSensorParmMString 	"PI_CAPSENS_M_PARM"		/*	Capsensor to EGU conversion parm for y=Mx+b  */

#define P_PTermString						"PI_P_TERM"  /* asynFloat64,    r/w */
#define P_ITermString						"PI_I_TERM"  /* asynFloat64,    r/w */
#define P_DTermString						"PI_D_TERM"  /* asynFloat64,    r/w */


#define P_SendCommandsString		"PI_SEND_COMMANDS" /* asynOctet, r/w */
#define P_GetIDNString					"PI_GET_IDN" /* asynOctet, r/w */

#define P_CommStatusString			"PI_COMMUNICATION_STATUS" /* asynInt32,    r */ 
#define P_WaveGen1_StatusString		"PI_WAVEGEN_1_STATUS"	/* asynInt32,    r */ 
#define P_WaveGen2_StatusString		"PI_WAVEGEN_2_STATUS"	/* asynInt32,    r */ 
#define P_WaveGen3_StatusString		"PI_WAVEGEN_3_STATUS"	/* asynInt32,    r */ 
#define P_WaveGen4_StatusString		"PI_WAVEGEN_4_STATUS"	/* asynInt32,    r */ 

#define P_WaveTbl1WfString        "PI_WAVETBL_1"             /* asynFloat64Array,  r/o */
#define P_WaveTbl2WfString        "PI_WAVETBL_2"             /* asynFloat64Array,  r/o */
#define P_WaveTbl3WfString        "PI_WAVETBL_3"             /* asynFloat64Array,  r/o */
#define P_WaveTbl4WfString        "PI_WAVETBL_4"             /* asynFloat64Array,  r/o */

#define P_DDLTbl1WfString        "PI_DDLTBL_1"             /* asynFloat64Array,  r/o */
#define P_DDLTbl2WfString        "PI_DDLTBL_2"             /* asynFloat64Array,  r/o */
#define P_DDLTbl3WfString        "PI_DDLTBL_3"             /* asynFloat64Array,  r/o */
#define P_DDLTbl4WfString        "PI_DDLTBL_4"             /* asynFloat64Array,  r/o */

#define P_DDLTblNORDString					"PI_DDLTBL_NORD"             /* asynInt32,  r/o */

#define P_TrigTblWfString					"PI_TRIG_TBL"             /* asynFloat64Array,  r/o */

#define P_GetWaveTbl1String				"PI_GET_WVTBL_1"			/* asynInt32,   r/w  */
#define P_GetWaveTbl2String				"PI_GET_WVTBL_2"			/* asynInt32,   r/w  */
#define P_GetWaveTbl3String				"PI_GET_WVTBL_3"			/* asynInt32,   r/w  */
#define P_GetWaveTbl4String				"PI_GET_WVTBL_4"			/* asynInt32,   r/w  */
	
#define P_GetDDLTbl1String				"PI_GET_DDLTBL_1"			/* asynInt32,   r/w  */
#define P_GetDDLTbl2String				"PI_GET_DDLTBL_2"			/* asynInt32,   r/w  */
#define P_GetDDLTbl3String				"PI_GET_DDLTBL_3"			/* asynInt32,   r/w  */
#define P_GetDDLTbl4String				"PI_GET_DDLTBL_4"			/* asynInt32,   r/w  */

#define P_GetTrigTblString				"PI_GET_TRIG_TBL"			/* asynInt32,   r/w  */

#define P_TrigOutputString				"PI_TRIG_OUTPUT"			/* asynInt32,   r/w  */

#define P_TrigModeWg1String					"PI_TRIG_MODE_WG1"			/* asynInt32,   r  */
#define P_TrigModeWg2String					"PI_TRIG_MODE_WG2"			/* asynInt32,   r  */
#define P_TrigModeWg3String					"PI_TRIG_MODE_WG3"			/* asynInt32,   r  */
#define P_TrigModeWg4String					"PI_TRIG_MODE_WG4"			/* asynInt32,   r  */

#define P_XAxisIdString						"PI_X_AXIS_ID"			/* asynInt32,   r/w  */		
#define P_YAxisIdString						"PI_Y_AXIS_ID"			/* asynInt32,   r/w  */		

#define P_ClrDDLTbl1String				"PI_CLR_DDLTBL_1"			/* asynInt32,   r/w  */
#define P_ClrDDLTbl2String				"PI_CLR_DDLTBL_2"			/* asynInt32,   r/w  */
#define P_ClrDDLTbl3String				"PI_CLR_DDLTBL_3"			/* asynInt32,   r/w  */
#define P_ClrDDLTbl4String				"PI_CLR_DDLTBL_4"			/* asynInt32,   r/w  */

#define P_ClrWavTbl1String				"PI_CLR_WAVTBL_1"			/* asynInt32,   r/w  */  
#define P_ClrWavTbl2String				"PI_CLR_WAVTBL_2"			/* asynInt32,   r/w  */  
#define P_ClrWavTbl3String				"PI_CLR_WAVTBL_3"			/* asynInt32,   r/w  */  
#define P_ClrWavTbl4String				"PI_CLR_WAVTBL_4"			/* asynInt32,   r/w  */  

#define P_ClrTrigTblString				"PI_CLR_TRIGTBL"			/* asynInt32,   r/w  */  

#define P_NumCyclesString					"PI_NUM_CYCLES"			/* asynInt32,   r/w  */			
#define P_WaveTblRateString				"PI_WAVE_TABLE_RATE"			/* asynInt32,   r/w  */	

#define P_StopWavegenString				"PI_STOP_WAVGEN"			/* asynInt32,   r/w  */			
#define P_StartWavegenString				"PI_START_WAVGEN"			/* asynInt32,   r/w  */			
#define P_ExecWavegenString				"PI_EXEC_WAVGEN"			/* asynInt32,   r/w  */			

#define P_CalcDDLParmsString				"PI_CALC_DDL_PARMS"			/* asynInt32,   r/w  */		

#define P_WaveTbl1LenString				"PI_WAVE_TABLE_1_LEN"			/* asynInt32,   r/w  */		
#define P_WaveTbl2LenString				"PI_WAVE_TABLE_2_LEN"			/* asynInt32,   r/w  */		
#define P_WaveTbl3LenString				"PI_WAVE_TABLE_3_LEN"			/* asynInt32,   r/w  */		
#define P_WaveTbl4LenString				"PI_WAVE_TABLE_4_LEN"			/* asynInt32,   r/w  */		

#define P_WaveGen1UseTblNumString	"PI_WAVEGEN_1_USETABLE_NUM"			/* asynInt32,   r/w  */		
#define P_WaveGen2UseTblNumString	"PI_WAVEGEN_2_USETABLE_NUM"			/* asynInt32,   r/w  */		
#define P_WaveGen3UseTblNumString	"PI_WAVEGEN_3_USETABLE_NUM"			/* asynInt32,   r/w  */		
#define P_WaveGen4UseTblNumString	"PI_WAVEGEN_4_USETABLE_NUM"			/* asynInt32,   r/w  */		

#define P_DDLTbl1LenString				"PI_DDL_TABLE_1_LEN"			/* asynInt32,   r/w  */		
#define P_DDLTbl2LenString				"PI_DDL_TABLE_2_LEN"			/* asynInt32,   r/w  */		
#define P_DDLTbl3LenString				"PI_DDL_TABLE_3_LEN"			/* asynInt32,   r/w  */		
#define P_DDLTbl4LenString				"PI_DDL_TABLE_4_LEN"			/* asynInt32,   r/w  */		


#define P_WaveTbl1UseDDLString				"PI_WVT1_USE_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl2UseDDLString				"PI_WVT2_USE_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl3UseDDLString				"PI_WVT3_USE_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl4UseDDLString				"PI_WVT4_USE_DDL"	/* asynInt32,   r/w  */		

#define P_WaveTbl1UseReinitDDLString				"PI_WVT1_USEREINIT_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl2UseReinitDDLString				"PI_WVT2_USEREINIT_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl3UseReinitDDLString				"PI_WVT3_USEREINIT_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl4UseReinitDDLString				"PI_WVT4_USEREINIT_DDL"	/* asynInt32,   r/w  */		

#define P_WaveTbl1StartAtEndString				"PI_WVT1_START_AT_END_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl2StartAtEndString				"PI_WVT2_START_AT_END_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl3StartAtEndString				"PI_WVT3_START_AT_END_DDL"	/* asynInt32,   r/w  */		
#define P_WaveTbl4StartAtEndString				"PI_WVT4_START_AT_END_DDL"	/* asynInt32,   r/w  */		


#define P_WaveTbl1StartModeString 	"PI_WAV1_START_MODE"	/* asynInt32,   r/w  */		
#define P_WaveTbl2StartModeString 	"PI_WAV2_START_MODE"	/* asynInt32,   r/w  */		
#define P_WaveTbl3StartModeString 	"PI_WAV3_START_MODE"	/* asynInt32,   r/w  */		
#define P_WaveTbl4StartModeString 	"PI_WAV4_START_MODE"	/* asynInt32,   r/w  */		

#define P_XStartPosString						"PI_X_START_POS"	/* asynFloat64,    r/w */ 
#define P_YStartPosString						"PI_Y_START_POS"	/* asynFloat64,    r/w */ 

#define P_ScanModeString						"PI_SCAN_MODE" /* asynInt32,   r/w  */		
#define P_LastErrString							"PI_LAST_ERR"	 /* asynOctet, r */
#define P_TotalPointsLeftString						"PI_TTL_POINTS_LEFT" /* asynInt32,   r  */		


#define P_SlewRateString 						"PI_SLEW_RATE"	/* asynFloat64,    r */ 
#define P_NotchFreq1String 					"PI_NFREQ_1"	/* asynFloat64,    r */ 		
#define P_NotchFreq2String 					"PI_NFREQ_2"	/* asynFloat64,    r */ 		
#define P_NotchRejection1String 		"PI_NREJ_1"	/* asynFloat64,    r */ 		
#define P_NotchRejection2String 		"PI_NREJ_2"	/* asynFloat64,    r */ 		
#define P_NotchBW1String 						"PI_NBW_1"	/* asynFloat64,    r */ 			
#define P_NotchBW2String 						"PI_NBW_2"	/* asynFloat64,    r */ 			

#define P_ParamFileNameString      		"PI_PARAM_FILE_NAME"							/* asynOctet */   	
#define P_LoadParamFileString      		"PI_LOAD_PARAM_FILE"							/* asynInt32 */   	

#define P_ATZVoltString 						"PI_ATZ_VOLTS"	/* asynFloat64,    r */ 

#define P_PosFromE712ScalerString		"PI_POS_FROM_E712_SCALER"	/* asynFloat64,    r/w */ 


#define P_LastParamString       	"PI_LAST_PARAM"    /* asynInt32,    r/w */ 

#define POS 0
#define NEG 1

#define CTO_PAM_TRIGGER_STEP 		1
#define CTO_PAM_AXIS_SEL 				2
#define CTO_PAM_TRIGGER_MODE 		3
#define CTO_PAM_MIN_THRESHOLD 	5
#define CTO_PAM_MAX_THRESHOLD 	6
#define CTO_PAM_POLARITY 				7
#define CTO_PAM_START_THRESHOLD 8
#define CTO_PAM_STOP_THRESHOLD 	9

#define CTO_TRIGMODE_POSDIST 	0
#define CTO_TRIGMODE_ONTRGT 	2
#define CTO_TRIGMODE_MINMAX 	3
#define CTO_TRIGMODE_GENTRIG 	4


#define REFMODE_ABS_SENSOR 	1
#define REFMODE_NEG_LIMIT 	2
#define REFMODE_POS_LIMIT 	3
#define REFMODE_SIGNED_MARK 4
#define REFMODE_ON_IMPULSE 	5

#define REFERENCE_MODE					0x02000a00

#define SLEW_RATE	0x07000200

#define PTERM_PARAM	0x07000300
#define ITERM_PARAM	0x07000301
#define DTERM_PARAM	0x07000302

#define POSITION_FROM_SENSOR_1    0x07000500		/* E-711.C82 Board#1 "motor 1" connector */
#define POSITION_FROM_SENSOR_2    0x07000501		/* E-711.C82 Board#1 "motor 2" connector */
#define POSITION_FROM_SENSOR_3    0x07000502		/* E-711.C82 Board#2 "motor 1" connector */
#define POSITION_FROM_SENSOR_4    0x07000503		/* E-711.C82 Board#2 "motor 2" connector */
#define POSITION_FROM_SENSOR_5    0x07000504		/* Capacitance sensor Axis x1 */
#define POSITION_FROM_SENSOR_6    0x07000505
#define POSITION_FROM_SENSOR_7    0x07000506		/* Capacitance sensor Axis z1 */
#define POSITION_FROM_SENSOR_8    0x07000507		/* Capacitance sensor Axis x2 */
#define POSITION_FROM_SENSOR_9    0x07000508
#define POSITION_FROM_SENSOR_10   0x07000509		/* Capacitance sensor Axis z2 */
#define POSITION_FROM_SENSOR_11   0x0700050a
#define POSITION_FROM_SENSOR_12   0x0700050b
#define POSITION_FROM_SENSOR_13   0x0700050c
#define POSITION_FROM_SENSOR_14   0x0700050d

#define NOTCH_FREQ_1  			0x08000100
#define NOTCH_FREQ_2  			0x08000101
#define NOTCH_REJECTION_1  	0x08000200
#define NOTCH_REJECTION_2  	0x08000201
#define NOTCH_BWIDTH_1  		0x08000300
#define NOTCH_BWIDTH_2  		0x08000301

#define DRIVING_FACTOR_OF_PIEZO_1  0x09000000
#define DRIVING_FACTOR_OF_PIEZO_2  0x09000001
#define DRIVING_FACTOR_OF_PIEZO_4  0x09000003
#define DRIVING_FACTOR_OF_PIEZO_5  0x09000004
#define DRIVING_FACTOR_OF_PIEZO_6  0x09000005
#define DRIVING_FACTOR_OF_PIEZO_7  0x09000006
#define DRIVING_FACTOR_OF_PIEZO_8  0x09000007
#define DRIVING_FACTOR_OF_PIEZO_9  0x09000008
#define DRIVING_FACTOR_OF_PIEZO_10 0x09000009
#define DRIVING_FACTOR_OF_PIEZO_11 0x0900000a
#define DRIVING_FACTOR_OF_PIEZO_12 0x0900000b
#define DRIVING_FACTOR_OF_PIEZO_13 0x0900000c
#define DRIVING_FACTOR_OF_PIEZO_14 0x0900000d



#define DIGFILT_TYPE		0x5000000 /*	1	10	INT	input	Digital Filter Type */ 
#define DIGFILT_BWIDTH	0x5000001 /*	1	10	FLOAT	input	Digital Filter Bandwidth  */
#define DIGFILT_ORDER		0x5000002 /*	1	10	INT	input	Digital Filter Order  */
#define DIGFILT_PARM1		0x5000101 /*	1	10	FLOAT	input	User Filter Param. 1  */
#define DIGFILT_PARM2		0x5000102 /*	1	10	FLOAT	input	User Filter Param. 2  */
#define DIGFILT_PARM3		0x5000103 /*	1	10	FLOAT	input	User Filter Param. 3  */
#define DIGFILT_PARM4		0x5000104 /*	1	10	FLOAT	input	User Filter Param. 4  */
#define DIGFILT_PARM5		0x5000105 /*	1	10	FLOAT	input	User Filter Param. 5  */

/* setup some convienient definitions */
#define USE_CAPACITIVE_SENSOR 0
#define USE_QUADRATURE_SENSOR 1

#define AUTOZERO_VOLTS 50.0

#define AXIS1_QUADRATURE POSITION_FROM_SENSOR_1
#define AXIS1_CAPACITIVE POSITION_FROM_SENSOR_5
#define AXIS1_DRIVING_FACTOR_PARAM DRIVING_FACTOR_OF_PIEZO_5

#define AXIS2_QUADRATURE POSITION_FROM_SENSOR_2
#define AXIS2_CAPACITIVE POSITION_FROM_SENSOR_7
#define AXIS2_DRIVING_FACTOR_PARAM DRIVING_FACTOR_OF_PIEZO_7

/* leave connectors from RLE 20 in board #1 for quadrature*/
#define AXIS3_QUADRATURE POSITION_FROM_SENSOR_1
#define AXIS3_CAPACITIVE POSITION_FROM_SENSOR_8
#define AXIS3_DRIVING_FACTOR_PARAM DRIVING_FACTOR_OF_PIEZO_9

#define AXIS4_QUADRATURE POSITION_FROM_SENSOR_2
#define AXIS4_CAPACITIVE POSITION_FROM_SENSOR_10
#define AXIS4_DRIVING_FACTOR_PARAM DRIVING_FACTOR_OF_PIEZO_11

#define AXIS1_OUTPUT_CHANNEL 1 //5
#define AXIS2_OUTPUT_CHANNEL 2 //7
#define AXIS3_OUTPUT_CHANNEL 3 //9
#define AXIS4_OUTPUT_CHANNEL 4 //11

#define AXIS1_INPUT_CHANNEL_CAP 1 //5
#define AXIS2_INPUT_CHANNEL_CAP 2 //7
#define AXIS3_INPUT_CHANNEL_CAP 3 //8
#define AXIS4_INPUT_CHANNEL_CAP 4 //10

#define AXIS1_INPUT_CHANNEL_INT 1
#define AXIS2_INPUT_CHANNEL_INT 2
#define AXIS3_INPUT_CHANNEL_INT 1
#define AXIS4_INPUT_CHANNEL_INT 2


/* to be used in a function that builds a string to use in an SPA call to the E-712 */

#define epicsExportSharedSymbols
#include "shareLib.h"

class epicsShareClass PIE712Axis : public asynMotorAxis
{
public:
    //PIE712Axis(class PIE712Controller *pController, PIGCSController* pGCSController, int axis, const char* szName);
    PIE712Axis(class PIE712Controller *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start );

    asynStatus Init(const char *portName);
		
		void pie712_forceDone_Thread(void);
    
    int getAxisNo() { return axisNo_; }

    asynStatus poll(bool *moving);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus setPosition(double position);
    asynStatus setOpenLoopPosition(double volts);
    
    asynStatus setVelocityCts(double velocity );
		asynStatus setVelocityEgu(double velocity );
		asynStatus setAccelerationCts(double acceleration)	{ return asynSuccess; }
		asynStatus setAcceleration(double acceleration)	{ return asynSuccess; }
		asynStatus move(double target);
		asynStatus moveCts(int target);
		asynStatus moveCts( PIE712Axis** pAxesArray, int* pTargetCtsArray, int numAxes);
		asynStatus referenceVelCts(double velocity, int forwards);  
		asynStatus haltAxis(void);

		asynStatus sendAccelAndVelocity(double acceleration, double velocity); 
    asynStatus setAxisPositionCts(double positionCts);
    asynStatus setAxisPosition(double position);

    asynStatus getAxisPosition(double &position);
    asynStatus getAxisVelocity(void);
    asynStatus getAxisPositionCts(void);
    asynStatus setServo(int servoState);
    
    asynStatus getStatus(int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);
    asynStatus getGlobalState( asynMotorAxis** Axes, int numAxes ) { return asynSuccess; }
    asynStatus getMoving(int& homing);
    asynStatus getBusy(int& busy);
    asynStatus getTravelLimits(double& negLimit, double& posLimit);
    asynStatus hasLimitSwitches(void);
    asynStatus hasReferenceSensor(void);
    asynStatus getReferencedState(void);
    
    asynStatus SetPivotX(double value);
    asynStatus SetPivotY(double value);
    asynStatus SetPivotZ(double value);
    
    asynStatus setPGain(double pGain);
  	asynStatus setIGain(double iGain);
  	asynStatus setDGain(double dGain);
  	
  	asynStatus setDigFiltType(double dtype);
		asynStatus setDigFiltBWidth(double dbwidth);
		asynStatus setDigFiltOrder(double dOrder);
		asynStatus setDigFiltParm1(double dP);
		asynStatus setDigFiltParm2(double dP);
		asynStatus setDigFiltParm3(double dP);
		asynStatus setDigFiltParm4(double dP);
		asynStatus setDigFiltParm5(double dP);
		asynStatus setDigFiltParm(int param_id, double dP);
		
		asynStatus setForceDoneWTime(double dtime);
		asynStatus resetVoltWarning(int warn);
    
    double GetPivotX() { return 0.0; }
    double GetPivotY() { return 0.0; }
    double GetPivotZ() { return 0.0; }

    bool AcceptsNewTarget() { return true; }
    bool CanCommunicateWhileHoming() { return true; }
    
    bool IsGCS2(void);
    
    //asynStatus getPositionEgu(double * result);
    double getPositionEGU(void);
    double getPositionCTS(void);
    asynStatus setPositionEGU(double pos);
    
	  asynStatus getRRBV(double * result);
	  asynStatus getVolt(double * result);
	  asynStatus getADSensor(double *result);
	  asynStatus getCapSensor(double *result);
	  asynStatus zeroADSensor(void);
	  asynStatus getMaxRange(double * result);
	  asynStatus motorOff(void);
	  asynStatus motorOffRelaxToCenter(void);
		asynStatus motorOn(void);
		asynStatus reInitPositioner(void);
		asynStatus set_params(void);
		asynStatus setMarker(double pos);
		asynStatus setMarkerWindow(void);
		asynStatus setPositionDistanceMarker(void);
		asynStatus setOnTargetMarker(void);
		asynStatus setMinMaxMarker(void);
		asynStatus setGeneratorTrigMarker(void);
		asynStatus setMarkerPolarity(int pol);
		asynStatus setMarkerStart(double pos);
		asynStatus setMarkerStop(double pos);
		asynStatus setScanStart(double pos);
		asynStatus setScanStop(double pos);
		asynStatus setMode(int mode);
		asynStatus getServo(int *servo);
		asynStatus selectEncoderSource(int type);
		asynStatus autoZero(void);
		asynStatus selectPiezoOutputDirection(int dir);
		asynStatus get_status(unsigned int *sts);
		asynStatus getAutoZeroStatus(bool *atz_sts);
		asynStatus getWaveGenStatus(bool *wvg_sts);
		
		bool isWithInRange(double target);
		bool isMoving(void);
	
	asynStatus configure_for_mode(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
		
		/* these need to be consolodated or removed */
		asynStatus config(char *axisName, int hiHardLimit, int lowHardLimit, int home, int start, int simulate);
		asynStatus initPositioner(void);
		


    char m_axisName[50];			///< GCS name

    int m_isHoming;				///< if \b TRUE indicating that axis is currently homing/referencing
    double deferred_position;   ///< currently not used
    int deferred_move;   		///< currently not used
    int deferred_relative;   	///< currently not used
    int m_homed;   				///< if \b TRUE axis was homed and absolute positions are correct

    double m_velocity;
    double m_acceleration;
    double m_maxAcceleration;
    int m_positionCts;
    double m_position;
    int m_lastDirection;
    double m_adcSensor;
    double m_capSensor;

    int m_CPUnumerator;
    int m_CPUdenominator;
    double m_default_velo;

    asynUser* m_pasynUser;

    bool m_bHasLimitSwitches;
    bool m_bHasReference;
    bool m_bProblem;
    bool m_bServoControl;
    bool m_bMoving;
    int m_movingStateMask;
    int m_ForceDone;
    int m_ForceDonePollIters;
    bool m_ForceDoneNOW;
    bool m_movePending;
    //bool m_wavegenStatus;
    
  double m_dPiezoVoltPerMicron;
  double m_dMinVolt;
	double m_dMaxVolt;
  double m_dVolt;  
  
  
  double m_dCoEffProp;
	double m_dCoEffDiff;
	double m_dCoEffInt;
	
	int m_dDigFiltOrder;
	int m_dDigFiltType;
	
	double m_dDigFiltBWidth;
	double m_dDigFiltP1;
	double m_dDigFiltP2;
	double m_dDigFiltP3;
	double m_dDigFiltP4;
	double m_dDigFiltP5;
	
	double m_dPTerm;
	double m_dITerm;
	double m_dDTerm;
  
  int m_mode;
	bool m_disabledByMode;
	double m_markerStart;
	double m_markerStop;
	double m_scanStart;
	double m_scanStop;
	bool m_enableStatus;
	bool m_voltRngWarning;
	
	double m_simSetPoint;
	bool m_powerOn;
	double m_startupPos;
	int m_axisNo;
	double m_dPiezoMaxRange;	
	
	int m_quad_parm;
	int m_cap_parm;
	int m_cap_input_chan;
	int m_quad_input_chan;
	
	int m_input_chan;
	
	double m_piezo_driving_factor;
  int m_piezo_driving_factor_param;
  
  char m_portName[50];
	bool m_simchan;
	double m_piezo_atz_voltages[4];
	int m_numxtra_polliters;
	
	double m_force_done_wait_time;
	
	
  friend class PIE712Controller;
private:
		
		PIE712Controller *pC_;
    double negLimit_;
    double posLimit_;
    double lowHardLimit_;
  	double hiHardLimit_;
    double home_;
    bool moving_;
    int deferred_move_;
    double stepSize_;
    double m_rrbv_;
    epicsEventId m_forceDoneWaiteventId_;
  
};


class epicsShareClass  PIE712Controller : asynMotorController {
public:

  /* These are the fucntions we override from the base class */
  PIE712Controller(const char *portName, const char *asynportName, const char* dr_asynPort, int numAxes, double movingPollPeriod, double idlePollPeriod);
  
  //asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  //asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
  asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
  
  void report(FILE *fp, int level);
  asynStatus poll();
  //PIE712Axis* getAxis(asynUser *pasynUser);
  //PIE712Axis* getAxis(int axisNo);
  int commandStringToList(const char *cmndString);
  asynStatus sendCommandList(const char *cmds);
  PIE712Axis* getAxis(asynUser *pasynUser) { return (PIE712Axis*)asynMotorController::getAxis(pasynUser); }
  PIE712Axis* getAxis(int axisNo) { return (PIE712Axis*)asynMotorController::getAxis(axisNo); }
  
  asynStatus toggleEncoderSource(int axis_no, int type);
  asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
  asynStatus triggerProfile(asynUser *pasynUser);
  
  bool getValue(const char* szMsg, double& value);
  bool getValue(const char* szMsg, int& value);
  bool getValue(const char* szMsg, bool& value);
  
  asynStatus setGCSParameter(PIE712Axis* pAxis, unsigned int paramID, double value);
  asynStatus getGCSParameter(int itemID, unsigned int paramID, double& value);
  asynStatus getGCSParameter(PIE712Axis* pAxis, unsigned int paramID, double& value);
  
  
  asynStatus findConnectedAxes();

  /* These are the functions that are new to this class */
  /*void AgilentMotorTask();  */// Should be pivate, but called from non-member function
 	void processServoErrorStatusRegister(unsigned int sts);
	void processServoSystemStatusRegister(unsigned int sts);
	bool isBitOn(int reg_offset, int bitnum, unsigned int sts);
	
	size_t getNrFoundAxes() { return m_nrFoundAxes; }
	int getGCSError();
	
	asynStatus connectWavtablesToGenerator(void);
	asynStatus getWavTblLength(int tblid, int &value);
	asynStatus getWavDatatbl(int tblid);
	asynStatus getDDLTblLength(int tblid, int& value);
	asynStatus getDDLDatatbl(int tblid);
	asynStatus putDDLDatatbl(int tblid, int num_points, epicsFloat64 *data);
	
	asynStatus getTrigtbl(void);
	asynStatus getTriggerMode(int wavegen_id, int &value);
	asynStatus getDDLTblLength(int tblid);
	asynStatus clearDDLTable(int tblid);
	asynStatus clearWavTable(int tblid);
	asynStatus clearTriggers(void);
	
	asynStatus getWaveGenStatus(int wavgen_num, bool *wvg_sts);
	asynStatus startWavegen(void);
	asynStatus stopWavegen(int tblid);
	asynStatus calcDDLProcParms(int waveTbl);
	asynStatus getWaveTableLength(int waveTbl, int &result);
	asynStatus setWaveTableOffset(int wavegen_id, double offset);
	asynStatus getWaveTableOffset(int wavegen_id, double &offset);
	asynStatus setWaveTableToGenerator(int wavegen_id, int tblid);
	asynStatus getWaveTableToGenerator(int wavegen_id, int &offset);
	asynStatus setWaveTableCycles(int wavegen_id, int cycles);
	asynStatus getWaveTableCycles(int wavegen_id, int &cycles);
	asynStatus setWaveTableRate(int wavegen_id, int rate);
	asynStatus getWaveTableRate(int wavegen_id, int &rate);
	
	asynStatus getDDLFlags(int wavegen_id, int &value);
	asynStatus getStartMode(int wavegen_id, int &value);
	
	/* data recorder commands */
	asynStatus getDRRDatatbls(void);
	asynStatus getDRRTblLength(int tblid, int& value);
	asynStatus readDRRDatatbls(char *);
	asynStatus saveDataRecFile(char *lines);
	asynStatus startDataRecorder(void);
	asynStatus configDataRecorder(void);
	asynStatus setDataRecTrigSrc(int src);
	asynStatus setRecTblRate(int rate);
	asynStatus getRecTblRate(int& value);
	asynStatus fix_NDATA_value(char *strbuf, int num_data);
	
	
	bool m_bAnyAxisMoving;
	bool m_bDataTransfering;
	
	char* m_axesIDs[MAX_NR_AXES];	
	char m_allAxesIDs[255];
	int m_LastError;
	
	char m_rcvbuf[WAVE_MAX_NUM_SAMPLES];
	char *m_cmnd_list[WAVE_MAX_NUM_SAMPLES];
	int m_numCmnds;
	char m_pDataRecFPath[1000];
	char *m_strbuf;
	
	int m_simController;
	int m_xAxis_id;
	int m_yAxis_id;
	int m_trigOutput;
	int m_numCycles;
	int m_scanMode;
	
	int m_wvgen1_startMode;
	int m_wvgen2_startMode;
	int m_wvgen3_startMode;
	int m_wvgen4_startMode;
	
	int m_wvgen1_flags;
	int m_wvgen2_flags;
	int m_wvgen3_flags;
	int m_wvgen4_flags;
	
	double m_xStartPos;
	double m_yStartPos;
	
	bool m_exec_pending;
	
	bool m_wavegen1_Status;
	bool m_wavegen2_Status;
	bool m_wavegen3_Status;
	bool m_wavegen4_Status;
	
	epicsFloat64 *m_pWavTbl1Data;
	epicsFloat64 *m_pWavTbl2Data;
	epicsFloat64 *m_pWavTbl3Data;
	epicsFloat64 *m_pWavTbl4Data;
	
	epicsFloat64 *m_pDDL1Data;
	epicsFloat64 *m_pDDL2Data;
	epicsFloat64 *m_pDDL3Data;
	epicsFloat64 *m_pDDL4Data;
	
	epicsFloat64 *m_pTrigData;
	
	
	int m_iDataRecTblParamIds[PI_DR_MAX_TABLES];
	int m_iDataRecTblLens[PI_DR_MAX_TABLES];
	int m_iDataRecTblSrcs[PI_DR_MAX_TABLES];
	int m_iDataRecTblOptions[PI_DR_MAX_TABLES];
	int m_iDataRecTblEnabled[PI_DR_MAX_TABLES];
	
	
	char *m_pDDLPutStr;
	char m_lastErr[512];
	char m_lastCmnd[256];
	
	bool m_suspend_fbk;
	
	PIInterface* m_pInterface;
	PIInterface* m_pDataRecInterface;
	
	int m_ddlTblNORD;
	
protected:



	int PI_SUP_POSITION;
#define FIRST_PI_E712_PARAM PI_SUP_POSITION    
  int PI_SUP_TARGET;
  int PI_SUP_SERVO;
  int PI_SUP_LAST_ERR;
  int PI_SUP_PIVOT_X;
  int PI_SUP_PIVOT_Y;
  int PI_SUP_PIVOT_Z;
  int PI_SUP_RBPIVOT_X;
  int PI_SUP_RBPIVOT_Y;
  int PI_SUP_RBPIVOT_Z;


  int P_SetMarker;          /**< set digital marker on position */        

  int  P_FW;  				/**< Firmware of motor driver */ 
  int P_Power;			/* axis power on */
  int P_SysStatus;    
  int P_OutputVolt;
  
  int P_Mode;
	int P_MarkerStart;
	int P_MarkerStop;
	int P_ScanStart;
	int P_ScanStop;
	int P_SetVelo;
	int P_SetStartupPos;
	int P_ReInit;
	int P_AutoZero;
	int P_SelectEncoderSrc;
	int P_SelectOutputDir;
	int P_AdcSensor;
	int P_CapSensor;
	int P_ForceDone;
	int P_ForceDoneWaitTime;
	int P_ZeroADSensor;
	int P_TriggerOutputSel;
	
	int P_DigitalFilterType;
	int P_DigitalFilterBWidth;
	int P_DigitalFilterORder;
	int P_DigitalFilterParm1;
	int P_DigitalFilterParm2;
	int P_DigitalFilterParm3;
	int P_DigitalFilterParm4;
	int P_DigitalFilterParm5;
	
	int P_VoltRangeWarning;
	int P_ResetVoltRangeWarning;
	int P_ServoOffAndCenter;
	
	int P_CapSensorParmB;
	int P_CapSensorParmM;
	
	int P_PTerm;
	int P_ITerm;
	int P_DTerm;

	
	int P_CommStatus;
	int P_WaveGen1_Status;
	int P_WaveGen2_Status;
	int P_WaveGen3_Status;
	int P_WaveGen4_Status;
	
	int P_SendCommands;
	int P_GetIDN;
	
	int P_WaveTbl1Wf;
	int P_WaveTbl2Wf;
	int P_WaveTbl3Wf;
	int P_WaveTbl4Wf;
	
	int P_DDLTbl1Wf;
	int P_DDLTbl2Wf;
	int P_DDLTbl3Wf;
	int P_DDLTbl4Wf;
	
	int P_TrigTblWf;
	
	int P_GetWaveTbl1;
	int P_GetWaveTbl2;
	int P_GetWaveTbl3;
	int P_GetWaveTbl4;
	                          
	int P_GetDDLTbl1;
	int P_GetDDLTbl2;
	int P_GetDDLTbl3;
	int P_GetDDLTbl4;

	int P_GetTrigTbl;
                  
	int P_TrigOutput;
	int P_TrigModeWg1;
	int P_TrigModeWg2;
	int P_TrigModeWg3;
	int P_TrigModeWg4;
	                  
	int P_XAxisId;		
	int P_YAxisId;		
	                  
	int P_ClrDDLTbl1;
	int P_ClrDDLTbl2;
	int P_ClrDDLTbl3;
	int P_ClrDDLTbl4;
	
	int P_ClrWavTbl1;
	int P_ClrWavTbl2;
	int P_ClrWavTbl3;
	int P_ClrWavTbl4;
	
	int P_ClrTrigTbl;

	int P_NumCycles;
  int P_WaveTblRate;
  
  int P_StopWavegen;
  int P_StartWavegen;
  int P_ExecWavegen;
  
  int P_CalcDDLParms;
  
  int P_WaveTbl1Len;
  int P_WaveTbl2Len;
  int P_WaveTbl3Len;
  int P_WaveTbl4Len;
  
  int P_WaveGen1UseTblNum;
	int P_WaveGen2UseTblNum;
	int P_WaveGen3UseTblNum;
	int P_WaveGen4UseTblNum;

  
  int P_DDLTbl1Len;
  int P_DDLTbl2Len;
  int P_DDLTbl3Len;
  int P_DDLTbl4Len;
  
  int P_WaveTbl1UseDDL;
	int P_WaveTbl2UseDDL;
	int P_WaveTbl3UseDDL;
	int P_WaveTbl4UseDDL;
	
	int P_WaveTbl1UseReinitDDL;
	int P_WaveTbl2UseReinitDDL;
	int P_WaveTbl3UseReinitDDL;
	int P_WaveTbl4UseReinitDDL;
	
	int P_WaveTbl1StartAtEnd;
	int P_WaveTbl2StartAtEnd;
	int P_WaveTbl3StartAtEnd;
	int P_WaveTbl4StartAtEnd;
	 
	int P_WaveTbl1StartMode;                                  
  int P_WaveTbl2StartMode;
  int P_WaveTbl3StartMode;
  int P_WaveTbl4StartMode;
  
  int P_XStartPos;
  int P_YStartPos;
  int P_ScanMode;
  int P_LastErr;
  int P_TotalPointsLeft;
  
  
  int P_SlewRate;
	int P_NotchFreq1;
	int P_NotchFreq2;
	int P_NotchRejection1;
	int P_NotchRejection2;
	int P_NotchBW1;
	int P_NotchBW2;

  int P_DDLTblNORD;
  
  int P_ParamFileName;
	int P_LoadParamFile;
	
	int P_ATZVolt;
	
	int P_PosFromE712Scaler;
	
  int P_LastParam;
  
#define LAST_PI_E712_PARAM P_LastParam
  
#define NUM_PI_E712_PARAMS ((&LAST_PI_E712_PARAM - &FIRST_PI_E712_PARAM + 1) + NUM_PI_E712_DATAREC_PARAMS)
  
  
private:
	int movesDeferred;
  asynStatus processDeferredMoves();
  epicsThreadId motorThread_;
  epicsTimeStamp prevTime_;
  int movesDeferred_;
  size_t m_nrFoundAxes;
  
friend class PIE712Axis;
};





#endif // PI_E712_ASYN_DRIVER_INCLUDED_
