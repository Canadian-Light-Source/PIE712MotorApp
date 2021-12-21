program calibrateLimit
%%#include <windows.h>
%%#include <stdio.h>
/*
	This calibration routine is to calibrate the Aps Motor record:
	
        *  the sequence for calibrating the XPS Q/C8 is this:
        * for now just
        *  1) Get the coarse position
        *  2) Get coarse MRES
        *  3) Set fine position to ((coarse pos) / coarse MRES ) * -1.0
        *       DONE
*/


option +r;  /* allow multiple instances.     */

%%#define mmToMicron(x) x*1000.0
%%#define micronsToMM(x) x*0.001

char motorName[40];
char coarseMotorName[40];
char fineMotorName[40];
char homeDir[40]; /* either HOMR or HOMF */
double finePos;
double fineOffset;
int coarseDir;

int fineDeviceSET;
int fineDeviceHome;
int fineDeviceSTATUS;
double fineDeviceVAL;
double fineDeviceRBV;
int fineDevicePower;

double motorRBV;
double motorMRES;

assign motorRBV to "{clsName}{motor}.RBV";
assign motorMRES to "{clsName}{motor}.MRES";


int    calibrateRun;	/* used to turn calibration procedure on and off */
int    calibrateDone;	/* set if calibration is succesfully completed */
string calibrateLog;

assign calibrateRun       to "{clsName}{motor}:calibRun";
assign calibrateDone      to "{clsName}{motor}:calibDone";
assign calibrateLog    	  to "{clsName}{motor}:log";


monitor calibrateRun;
monitor motorRBV;
monitor motorMRES;

ss calibrate_xps
{
    state init_calibration
    {
	  	entry	
			{
				if (macValueGet("clsName") != 0)
				{
					sprintf(motorName, "%s", macValueGet("clsName"));
				} else{
					strcpy(motorName, "Unknown");
				}

				if (macValueGet("coarseMotorName") != 0)
				{
					sprintf(coarseMotorName, "%s", macValueGet("coarseMotorName"));
				} else{
					strcpy(coarseMotorName, "Unknown");
				}
				if (macValueGet("fineMotorName") != 0)
				{
					sprintf(fineMotorName, "%s", macValueGet("fineMotorName"));
				} else{
					strcpy(fineMotorName, "Unknown");
				}
				if (macValueGet("fineOffset") != 0)
				{
					fineOffset = atof(macValueGet("fineOffset"));
				} else{
					fineOffset = 0.0;
				}
				if (macValueGet("coarseDir") != 0)
				{
					coarseDir = atoi(macValueGet("coarseDir"));
				} else{
					coarseDir = 1;
				}
				
			}

			/*
		 	*  start calibration procedure by moving fineDevice to limit switch.
		 	*/
	    when (delay(1))
			{
			
	    } state wait_for_cal
	  }


	 state wait_for_cal
   {
        when (calibrateRun == 1)
			{
				sprintf(calibrateLog,"%s: start calibration", motorName);
				pvPut(calibrateLog);
				printf("%s\n", calibrateLog);
		
				calibrateDone = 0;
				pvPut(calibrateDone);
			
	    } state start_cal
	  }
	
   state start_cal
   {
        when ()
				{
				 	sprintf(calibrateLog, "%s: coarse pos =%.3f", motorName,motorRBV);
				 	pvPut(calibrateLog);
				 	printf("%s\n", calibrateLog);
				 	sprintf(calibrateLog, "%s: coarse mres =%.3f", motorName,motorMRES);
				 	pvPut(calibrateLog);
				 	printf("%s\n", calibrateLog);
				 	
				 	fineDevicePower = 0;
				 	pvPut(fineDevicePower, SYNC);

				 	%%Sleep(25);
					/* put SET to SET*/
				 	fineDeviceSET = 1;
				 	pvPut(fineDeviceSET, SYNC);
				 	
				 	/*finePos = (mmToMicron(motorRBV) + fineOffset) * coarseDir;*/
				 	finePos = (motorRBV + fineOffset) * coarseDir;
				 	pvPut(calibrateLog);
				 	
				 	sprintf(calibrateLog, "%s: setting fine to %.3f", motorName,finePos);
				 	pvPut(calibrateLog);
				 	printf("%s\n", calibrateLog);
				 	
				 	
				 	/* set new position*/
				 	fineDeviceVAL = finePos;
				 	pvPut(fineDeviceVAL, SYNC);

				 	%%Sleep(25);
				 	
				 	sprintf(calibrateLog, "%s: setting SET back to USE", motorName);
				 	pvPut(calibrateLog);
				 	printf("%s\n", calibrateLog);
				 	
				 	/* put SET  to USE*/
				 	fineDeviceSET = 0;
				 	pvPut(fineDeviceSET, SYNC);
				 	
				 	%%Sleep(25);
				 	
				 	sprintf(calibrateLog, "%s: turning Closed loop back on", motorName);
				 	pvPut(calibrateLog);
				 	printf("%s\n", calibrateLog);
				 	
				 	
				 	/* now go back to coarse position, should be in middle of range for finestage*/
				 	/*fineDeviceVAL = (mmToMicron(motorRBV)) * coarseDir;*/
				 	fineDeviceVAL = motorRBV * coarseDir;
				 	pvPut(fineDeviceVAL, SYNC);
				 	
				 	fineDevicePower = 1;
				 	pvPut(fineDevicePower, SYNC);
				 	
				 	sprintf(calibrateLog, "%s: Done", motorName,finePos);
			    pvPut(calibrateLog);
			    printf("%s\n", calibrateLog);
			    
					
				} state finish_up 

        /*
         *  if time out abort
         */
        when (delay(380))
				{
					sprintf(calibrateLog, "%s: timed out - NOT calibrated", motorName);
					pvPut(calibrateLog);
					printf("%s\n", calibrateLog);
				} state stop_calibration

        when (calibrateRun == 0)
				{
				} state stop_calibration
     }
        
		
	
    state finish_up
    {
				when (delay(1))
				{
					sprintf(calibrateLog, "%s: calibration completed", motorName);
					pvPut(calibrateLog);
					printf("%s\n", calibrateLog);
					
					calibrateRun = 0;
					pvPut(calibrateRun);
			
					calibrateDone = 1;
					pvPut(calibrateDone);
			
					
				} state wait_for_cal
		}

    /*
     *  Stop calibration procedure prematurely
     *		- restore original velocity setting
     *		- set run state to 0
     *		- stop motor movement
     */
		state stop_calibration
		{
			entry
			{
				sprintf(calibrateLog, "%s: calibration discontinued", motorName);
				pvPut(calibrateLog);
				printf("%s\n", calibrateLog);
			}
			
			when ()
			{
	
				/*fineDeviceStop = 3;
				pvPut(fineDeviceStop);*/
	
			} state init_calibration
		}
}
