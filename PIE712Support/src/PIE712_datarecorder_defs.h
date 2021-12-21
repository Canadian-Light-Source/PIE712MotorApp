

#ifndef PI_E712_DATARECORDER_DEFS_INCLUDED_
#define PI_E712_DATARECORDER_DEFS_INCLUDED_


/* date recorder definitions:
	#RecordOptions 
	1=Target Position of axis 
	2=Current Position of axis 
	3=Position Error of axis 
	7=Control Voltage of output chan 
	13=DDL Output of axis 
	14=Open Loop Control of axis 
	15=Control Output of axis 
	16=Voltage of output chan 
	17=Sensor Normalized of input chan 
	18=Sensor Filtered of input chan 
	19=Sensor ElecLinear of input chan 
	20=Sensor MechLinear of input chan 
	22=Slowed Target position of axis 
	23=Target velocity of axis 
	24=Target acceleration of axis 
	25=Target jerk of axis 
	26=Value of Digital Input 
	27=Value of Digital Output 
	
	#TriggerOptions 
	0=Default Setting 
	1=Any CMD Changing Pos 
	3=External Trigger 
	4=Immediate Trigger 
	
	#Parameters to be set with SPA 
	0x16000000=Data Recorder Table Rate 
	0x16000300=Data Recorder Chan Number 
	0x16000700=DRC Data Source 
	0x16000701=DRC Record Option 
*/
/* OPTIONS */
#define PI_DR_TRGT_POS 											1
#define PI_DR_CURPOS_OF_AXIS 								2
#define PI_DR_POS_ERR 												3
#define PI_DR_CTRL_VOLTS_OUTPUT_CHAN 				7
#define PI_DR_DDL_OUTPUT_OF_AXIS 						13
#define PI_DR_OPEN_LOOP_CTRL_OF_AXIS 				14
#define PI_DR_CTRL_OUTPUT_OF_AXIS 						15
#define PI_DR_VOLTS_OF_OUTPUT_CHAN 					16
#define PI_DR_SNSR_NORM_OF_INPUT_CHAN 				17
#define PI_DR_SNSR_FILTERD_OF_INPUT_CHAN 		18
#define PI_DR_SNSR_ELECLINEAR_OF_INPUT_CHAN 	19
#define PI_DR_SNSR_MECHLINEAR_OF_INPUT_CHAN 	20
#define PI_DR_SLOWED_TRGT_POS_OF_AXIS 				22
#define PI_DR_TRGT_VELO_OF_AXIS 							23
#define PI_DR_TRGT_ACCL_OF_AXIS 							24
#define PI_DR_TRGT_JERK_OF_AXIS 							25
#define PI_DR_VALUE_OF_DIGITAL_INPUT 				26
#define PI_DR_VALUE_OF_DIGITAL_OUTPUT 				27

/*TRIGGER OPTIONS */
#define PI_DR_TRG_DFLT			0
#define PI_DR_TRG_ANY_CMD		1
#define PI_DR_TRG_EXTERNAL	3
#define PI_DR_TRG_IMMEDIATE	4

/* data recorder SPA param addrs */
#define	PI_DR_DRTR	0x16000000 /* =Data Recorder Table Rate */
#define	PI_DR_DRCN	0x16000300	/*	=Data Recorder Chan Number */
#define	PI_DR_DRC_DS	0x16000700	/*	=DRC Data Source */
#define	PI_DR_DRC_RO	0x16000701	/*	=DRC Record Option */

/*****/

#define PI_DR_DEFAULT_TABLE_ID 1
#define PI_DR_MAX_TABLES 12
#define PI_DR_MAX_POINTS 699050
#define PI_GCS_DATA_HDR_BYTES 1000
#define PI_DR_MIN_REC_TBL_RATE 0.000050 /* seconds */


/* data recorder driver attrs */
/* there are 12 data recorder tables */
#define P_DatRec_T1EnabledString		"PI_DR_T1_EN"		/* asynInt32 */
#define P_DatRec_T1SrcString				"PI_DR_T1_SRC"		/* asynInt32 */
#define P_DatRec_T1OptionString		  "PI_DR_T1_OPTION"		/* asynInt32 */
#define P_DatRec_T1BitmMaskString		"PI_DR_T1_BITMASK"		/* asynInt32 */

#define P_DatRec_T2EnabledString		"PI_DR_T2_EN"		/* asynInt32 */
#define P_DatRec_T2SrcString					"PI_DR_T2_SRC"		/* asynInt32 */
#define P_DatRec_T2OptionString		  "PI_DR_T2_OPTION"		/* asynInt32 */
#define P_DatRec_T2BitmMaskString		"PI_DR_T2_BITMASK"		/* asynInt32 */

#define P_DatRec_T3EnabledString		"PI_DR_T3_EN"		/* asynInt32 */
#define P_DatRec_T3SrcString					"PI_DR_T3_SRC"		/* asynInt32 */
#define P_DatRec_T3OptionString		  "PI_DR_T3_OPTION"		/* asynInt32 */
#define P_DatRec_T3BitmMaskString		"PI_DR_T3_BITMASK"		/* asynInt32 */

#define P_DatRec_T4EnabledString		"PI_DR_T4_EN"		/* asynInt32 */
#define P_DatRec_T4SrcString					"PI_DR_T4_SRC"		/* asynInt32 */
#define P_DatRec_T4OptionString		  "PI_DR_T4_OPTION"		/* asynInt32 */
#define P_DatRec_T4BitmMaskString		"PI_DR_T4_BITMASK"		/* asynInt32 */

#define P_DatRec_T5EnabledString		"PI_DR_T5_EN"		/* asynInt32 */
#define P_DatRec_T5SrcString					"PI_DR_T5_SRC"		/* asynInt32 */
#define P_DatRec_T5OptionString		  "PI_DR_T5_OPTION"		/* asynInt32 */
#define P_DatRec_T5BitmMaskString		"PI_DR_T5_BITMASK"		/* asynInt32 */

#define P_DatRec_T6EnabledString		"PI_DR_T6_EN"		/* asynInt32 */
#define P_DatRec_T6SrcString					"PI_DR_T6_SRC"		/* asynInt32 */
#define P_DatRec_T6OptionString		  "PI_DR_T6_OPTION"		/* asynInt32 */
#define P_DatRec_T6BitmMaskString		"PI_DR_T6_BITMASK"		/* asynInt32 */

#define P_DatRec_T7EnabledString		"PI_DR_T7_EN"		/* asynInt32 */
#define P_DatRec_T7SrcString					"PI_DR_T7_SRC"		/* asynInt32 */
#define P_DatRec_T7OptionString		  "PI_DR_T7_OPTION"		/* asynInt32 */
#define P_DatRec_T7BitmMaskString		"PI_DR_T7_BITMASK"		/* asynInt32 */

#define P_DatRec_T8EnabledString		"PI_DR_T8_EN"		/* asynInt32 */
#define P_DatRec_T8SrcString					"PI_DR_T8_SRC"		/* asynInt32 */
#define P_DatRec_T8OptionString		  "PI_DR_T8_OPTION"		/* asynInt32 */
#define P_DatRec_T8BitmMaskString		"PI_DR_T8_BITMASK"		/* asynInt32 */

#define P_DatRec_T9EnabledString		"PI_DR_T9_EN"		/* asynInt32 */
#define P_DatRec_T9SrcString					"PI_DR_T9_SRC"		/* asynInt32 */
#define P_DatRec_T9OptionString		  "PI_DR_T9_OPTION"		/* asynInt32 */
#define P_DatRec_T9BitmMaskString		"PI_DR_T9_BITMASK"		/* asynInt32 */

#define P_DatRec_T10EnabledString		"PI_DR_T10_EN"		/* asynInt32 */
#define P_DatRec_T10SrcString					"PI_DR_T10_SRC"		/* asynInt32 */
#define P_DatRec_T10OptionString		  "PI_DR_T10_OPTION"		/* asynInt32 */
#define P_DatRec_T10BitmMaskString		"PI_DR_T10_BITMASK"		/* asynInt32 */

#define P_DatRec_T11EnabledString		"PI_DR_T11_EN"		/* asynInt32 */
#define P_DatRec_T11SrcString					"PI_DR_T11_SRC"		/* asynInt32 */
#define P_DatRec_T11OptionString		  "PI_DR_T11_OPTION"		/* asynInt32 */
#define P_DatRec_T11BitmMaskString		"PI_DR_T11_BITMASK"		/* asynInt32 */

#define P_DatRec_T12EnabledString		"PI_DR_T12_EN"		/* asynInt32 */
#define P_DatRec_T12SrcString					"PI_DR_T12_SRC"		/* asynInt32 */
#define P_DatRec_T12OptionString		  "PI_DR_T12_OPTION"		/* asynInt32 */
#define P_DatRec_T12BitmMaskString		"PI_DR_T12_BITMASK"		/* asynInt32 */


/* data recorder commands */                                         
#define P_DatRec_ExecConfigString		"PI_DR_EXEC_CFG"		/* asynInt32 */
#define P_DatRec_setDRCString				"PI_DR_SET_DRC"		/* asynInt32 */
#define P_DatRec_getDRCString				"PI_DR_GET_DRC"		/* asynInt32 */
#define P_DatRec_getDRLString				"PI_DR_GET_DRL"		/* asynInt32 */
#define P_DatRec_getDRRString				"PI_DR_GET_DRR"		/* asynInt32 */
#define P_DatRec_DRRProgressString	"PI_DR_DRR_PROG"	/* asynInt32,  r/o */
#define P_DatRec_setDRTString				"PI_DR_SET_DRT"		/* asynInt32 */
#define P_DatRec_getDRTString				"PI_DR_GET_DRT"		/* asynInt32 */
#define P_DatRec_getHDRString				"PI_DR_GET_HDR"		/* asynInt32 */
#define P_DatRec_setIMPString				"PI_DR_SET_IMP"		/* asynInt32 */
#define P_DatRec_setRTRString				"PI_DR_SET_RTR"		/* asynInt32 */
#define P_DatRec_getRTRString				"PI_DR_GET_RTR"		/* asynInt32 */
#define P_DatRec_setSTEString				"PI_DR_SET_STE"		/* asynInt32 */
#define P_DatRec_getTNRString				"PI_DR_GET_TNR"		/* asynInt32 */
#define P_DatRec_setWGOString				"PI_DR_SET_WGO"		/* asynInt32 */
#define P_DatRec_setWGRString				"PI_DR_SET_WGR"		/* asynInt32 */

#define P_DatRec_getRTRInSecString	"PI_DR_GET_RTR_IN_SEC"	/* asynFloat64,    r/w */

#define P_DatRec_FPathString			"PI_DR_FPATH"	 /* asynOctet, rw */
//#define P_DatRec_FileNameString			"PI_DR_FILE_NAME"	 /* asynOctet, rw */

#define P_DatRec_StartString							"PI_DR_START"			/* asynInt32 */

#define P_DatRec_getDRRStsString		"PI_DR_GET_DRR_STS"			/* asynInt32 */

/*************/
/* data recorder command results */
/* result of DRR? */

/* result of DRL? */
#define P_DatRec_NptsString					"PI_DR_NPTS"		/* asynInt32,  r/o */

/* result of DRC? <source>*/
#define P_DatRec_T1SrcFbkString				"PI_DR_T1_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T2SrcFbkString				"PI_DR_T2_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T3SrcFbkString				"PI_DR_T3_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T4SrcFbkString				"PI_DR_T4_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T5SrcFbkString				"PI_DR_T5_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T6SrcFbkString				"PI_DR_T6_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T7SrcFbkString				"PI_DR_T7_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T8SrcFbkString				"PI_DR_T8_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T9SrcFbkString				"PI_DR_T9_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T10SrcFbkString			"PI_DR_T10_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T11SrcFbkString			"PI_DR_T11_SRC_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T12SrcFbkString			"PI_DR_T12_SRC_FBK"		/* asynInt32,  r/o */


/* result of DRC? <rec option> */



#define P_DatRec_T1OptionFbkString		"PI_DR_T1_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T2OptionFbkString		"PI_DR_T2_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T3OptionFbkString		"PI_DR_T3_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T4OptionFbkString		"PI_DR_T4_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T5OptionFbkString		"PI_DR_T5_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T6OptionFbkString		"PI_DR_T6_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T7OptionFbkString		"PI_DR_T7_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T8OptionFbkString		"PI_DR_T8_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T9OptionFbkString		"PI_DR_T9_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T10OptionFbkString		"PI_DR_T10_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T11OptionFbkString		"PI_DR_T11_OPTN_FBK"		/* asynInt32,  r/o */
#define P_DatRec_T12OptionFbkString		"PI_DR_T12_OPTN_FBK"		/* asynInt32,  r/o */


/* result of DRT? <trig src> <value>*/                                                        
#define P_DatRec_TrgSrcString				"PI_DR_TRGSRC"		/* asynInt32,  r/w */


/* result of DRT? <trig src> <value>*/                                                        
#define P_DatRec_TrgSrcFbkString				"PI_DR_TRGSRC_FBK"		/* asynInt32,  r/o */


/* result of RTR? <value>*/                                                         
#define P_DatRec_RTRFbkString					"PI_DR_RTR_FBK"		/* asynInt32,  r/o */

/* result of TNR? <value>*/                                                         
#define P_DatRec_TNRFbkString					"PI_DR_TNR_FBK"		/* asynInt32,  r/o */

#define P_DataRec_AutoEnableString					"PI_DR_AUTO_ENABLE"		/* asynInt32,  r/w */

#define P_DatRec_AbortString					"PI_DR_ABORT"		/* asynInt32,  r/o only used internally*/



/* all data recorder related commands and feedbacks */
	int P_DatRec_T1Enabled;
#define FIRST_PI_E712_DATAREC_PARAM P_DatRec_T1Enabled

	int P_DatRec_T2Enabled;
	int P_DatRec_T3Enabled;
	int P_DatRec_T4Enabled;
	int P_DatRec_T5Enabled;
	int P_DatRec_T6Enabled;
	int P_DatRec_T7Enabled;
	int P_DatRec_T8Enabled;
	int P_DatRec_T9Enabled;
	int P_DatRec_T10Enabled;
	int P_DatRec_T11Enabled;
	int P_DatRec_T12Enabled;

	int P_DatRec_ExecConfig;
	int P_DatRec_setDRC;
	int P_DatRec_getDRC;
	int P_DatRec_getDRR;
	int P_DatRec_DRRProgress;
	int P_DatRec_getDRL;
	int P_DatRec_setDRT;
	int P_DatRec_getDRT;
	int P_DatRec_getHDR;
	int P_DatRec_setIMP;
	int P_DatRec_setRTR;
	int P_DatRec_getRTR;
	int P_DatRec_setSTE;
	int P_DatRec_getTNR;
	int P_DatRec_setWGO;
	int P_DatRec_setWGR;
	
	int P_DatRec_getRTRInSec;
	
	int P_DatRec_FPath;
	int P_DatRec_FileName;
	
	int P_DatRec_Start;
	
	int P_DatRec_getDRRSts;

  int P_DatRec_Npts;	
	
	int P_DatRec_T1Src;
	int P_DatRec_T2Src;
	int P_DatRec_T3Src;
	int P_DatRec_T4Src;
	int P_DatRec_T5Src;
	int P_DatRec_T6Src;
	int P_DatRec_T7Src;
	int P_DatRec_T8Src;
	int P_DatRec_T9Src;
	int P_DatRec_T10Src;
	int P_DatRec_T11Src;
	int P_DatRec_T12Src;
	
	int P_DatRec_T1SrcFbk;
	int P_DatRec_T2SrcFbk;
	int P_DatRec_T3SrcFbk;
	int P_DatRec_T4SrcFbk;
	int P_DatRec_T5SrcFbk;
	int P_DatRec_T6SrcFbk;
	int P_DatRec_T7SrcFbk;
	int P_DatRec_T8SrcFbk;
	int P_DatRec_T9SrcFbk;
	int P_DatRec_T10SrcFbk;
	int P_DatRec_T11SrcFbk;
	int P_DatRec_T12SrcFbk;
	
	int P_DatRec_T1Option;
	int P_DatRec_T2Option;
	int P_DatRec_T3Option;
	int P_DatRec_T4Option;
	int P_DatRec_T5Option;
	int P_DatRec_T6Option;
	int P_DatRec_T7Option;
	int P_DatRec_T8Option;
	int P_DatRec_T9Option;
	int P_DatRec_T10Option;
	int P_DatRec_T11Option;
	int P_DatRec_T12Option;
		
	
	int P_DatRec_T1OptionFbk;
	int P_DatRec_T2OptionFbk;
	int P_DatRec_T3OptionFbk;
	int P_DatRec_T4OptionFbk;
	int P_DatRec_T5OptionFbk;
	int P_DatRec_T6OptionFbk;
	int P_DatRec_T7OptionFbk;
	int P_DatRec_T8OptionFbk;
	int P_DatRec_T9OptionFbk;
	int P_DatRec_T10OptionFbk;
	int P_DatRec_T11OptionFbk;
	int P_DatRec_T12OptionFbk;
	int P_DatRec_TrgSrc;
	int P_DatRec_TrgSrcFbk;
	int P_DatRec_RTRFbk;
	int P_DatRec_TNRFbk;
	int P_DatRec_Abort;
	int P_DataRec_AutoEnable;

	int P_DatRec_LastParam;
  
#define LAST_PI_E712_DATAREC_PARAM P_DatRec_LastParam
  

#define NUM_PI_E712_DATAREC_PARAMS (&LAST_PI_E712_DATAREC_PARAM - &FIRST_PI_E712_DATAREC_PARAM + 1) +50

//#define NUM_PI_E712_DATAREC_PARAMS 131

#endif // PI_E712_DATARECORDER_DEFS_INCLUDED_