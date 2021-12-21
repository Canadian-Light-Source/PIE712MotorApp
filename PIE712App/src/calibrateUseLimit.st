program calibrateUseLimit
option +r;

%%#include <string.h>

#define VME_HOME_VELOCITY 1.0  /* in engineering units */
#define ATLIMIT 2

char motorName[40];
char calibLimitName[40];
char buf[40];
char limitStr[40];

int Power;

double devicePosition;
double deviceMoveRelative;
double deviceVelocity;
double goHomeVelocity;
double deviceLoadEncPosn;
double deviceSetPosn;

double deviceEncSlope;
double deviceStepSlope;

double move1Vel;
double move2Vel;
int deviceEncFbk;

double finalPosition;
double saveVelocity;
int deviceStep;
int deviceLoadStep;


int    calibrateRun;	/* used to turn calibration procedure on and off */
int    calibrateDone;	/* set if calibration is succesfully completed */
string calibrateLog;
double calibrateFirstMove;
double calibrateSecondMove;
int    goHome;		/* cause motor to move to home position */
double calibratedPosition;
double eguOffset;
double encoderOffset;
double stepOffset;
int closedLoop;
int closedLoopRval;
int saveclosedLoop;
int backLash;
int savebackLash;

string limitSwitchName;
int deviceLimitSwitch;
int deviceHomeSwitch;
int deviceMoving;
int deviceStop;
int deviceLimitSwitchType;
int deviceMvComplete;
int deviceStatus;


assign calibrateRun       to "{clsName}:calibRun";
assign calibrateDone      to "{clsName}:calibDone";
assign calibrateLog    	  to "{clsName}:calibLog";
assign calibrateFirstMove to "{clsName}:calibFirstMv";

assign devicePosition     to "{clsName}.VAL";
assign deviceMoveRelative to "{clsName}.RLV";
assign deviceVelocity     to "{clsName}.VELO";
assign deviceSetPosn 			to "{clsName}.SET";
assign deviceEncFbk       to "{clsName}.RBV";


assign goHome    	  			to "{clsName}:goHome";
assign calibratedPosition to "{clsName}:{unit}:calibPosn";
assign limitSwitchName    to "{clsName}:calibLimit";
assign deviceHomeSwitch   to "{clsName}:home";
assign deviceMoving       to "{clsName}:moving";
assign deviceMvComplete       to "{clsName}:scanMv";
assign deviceStop         to "{clsName}:stop";
assign deviceStatus               to "{clsName}:status.RVAL";

assign deviceLimitSwitch to "";/* will dynamically assign later*/

monitor finalPosition;
monitor move1Vel;
monitor calibrateRun;
monitor deviceLimitSwitch;
monitor deviceHomeSwitch;
monitor deviceMoving;
monitor deviceMvComplete;
monitor deviceStatus;
monitor deviceEncSlope;
monitor deviceStepSlope;
monitor deviceEncFbk;

ss calibrate_using_home
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
                
		if (macValueGet("lim") != 0)
        	{
        		sprintf(limitStr, "%s", macValueGet("lim"));
						printf("limitStr = %s\n",limitStr);
            sprintf(calibLimitName, "%s.%s", motorName, limitStr);
            pvAssign(deviceLimitSwitch, calibLimitName);
        	} else {
        		strcpy(limitStr, "Unknown");
        	}
	
	}

	/*
	 *  start calibration procedure by moving device to limit switch.
	 */
    when (calibrateRun == 1)
		{
			snprintf(calibrateLog, 40, "%s: start calibration", motorName);
			pvPut(calibrateLog);
			printf("%s\n", calibrateLog);

			calibrateDone = 0;
			pvPut(calibrateDone);
	
			/*
			 *  Move device out of home zone.
			 */
			pvGet(calibrateFirstMove);
			deviceMoveRelative = calibrateFirstMove;
			pvPut(deviceMoveRelative, SYNC);
	
			snprintf(calibrateLog, 40, "%s: move to %s limit", motorName, limitStr);
			/*snprintf(calibrateLog, 40, "c:%d b:%d sc:%d sb:%d", closedLoop,backLash,saveclosedLoop,savebackLash);*/
			pvPut(calibrateLog);
			printf("%s\n", calibrateLog);
		} state move_to_limit_switch
    }
    
    state move_to_limit_switch
        {
        /*
         *  When device reaches limit switch:
		 *	- save old velocity
		 *	- set slower velocity required to find home switch
		 *	- move in opposite direction to find home/index signal
         */
		/*when ((deviceLimitSwitch == 1) && (deviceStatus == ATLIMIT))*/
		when (delay(1.0) && (deviceLimitSwitch == 1))
		{
		snprintf(calibrateLog, 40, "%s: at %s limit", motorName, limitStr);
		pvPut(calibrateLog);
		printf("%s\n", calibrateLog);

		/* move the motors close to home/index */
		pvGet(move2Vel);	
		deviceVelocity = move2Vel;	
	 	pvPut(deviceVelocity, SYNC);
		
		pvGet(calibrateSecondMove);
		deviceMoveRelative = calibrateSecondMove;
		pvPut(deviceMoveRelative);
		
		
		snprintf(calibrateLog, 40, "moving %f to homeindex",deviceMoveRelative);
		pvPut(calibrateLog);
		printf("%s\n", calibrateLog);

	
		/*} state move_close_to_homeIndex*/
		} state set_calibrated_position

        /*
         *  if time out abort
         */
        when (delay(380))
		{
			snprintf(calibrateLog, 40, "%s: timed out - NOT calibrated", motorName);
			pvPut(calibrateLog);
			printf("%s\n", calibrateLog);
		} state stop_calibration

        when (calibrateRun == 0)
		{
			deviceStop = 1;
			pvPut(deviceStop, SYNC);
		} state stop_calibration
   }

   state move_close_to_homeIndex
   {
        /*
         *  When device reaches end of move:
         *      - save old velocity
         *      - set slower velocity required to find home switch
         *      - move in opposite direction to find home/index signal
         */
        /*when (deviceMvComplete==0 && delay(2.0))*/
        when (deviceMoving==0 && delay(2.0))
        {
        	snprintf(calibrateLog, 40, "%s: near home/index", motorName);
            	pvPut(calibrateLog);
            	printf("%s\n", calibrateLog);
	    
		pvGet(goHomeVelocity);
	    	deviceVelocity = goHomeVelocity;
	    	pvPut(deviceVelocity, SYNC);
            	goHome = 1;
            	pvPut(goHome, SYNC);
		printf("just requested goHome\n");
		

        } state calibrate_to_index_home_pulse

        /*
         *  if time out abort
         */
        when (delay(120))
        {
        	snprintf(calibrateLog, 40, "%s: timed out - NOT calibrated", motorName);
            	pvPut(calibrateLog);
            	printf("%s\n", calibrateLog);
        } state stop_calibration

        when (calibrateRun == 0)
        {
		deviceStop = 1;
                pvPut(deviceStop, SYNC);

    	} state stop_calibration
    }

    state calibrate_to_index_home_pulse 
    {
         when(deviceMoving== 0 && delay(2.0))
         {
	    snprintf(calibrateLog, 40, "%s: restoring blash/cloop", motorName);
            pvPut(calibrateLog, SYNC);
            printf("%s\n", calibrateLog);
            /*restore closed loop and backlash settings*/
	    closedLoop = saveclosedLoop;
       	    backLash = savebackLash;
            pvPut(closedLoop, SYNC);
            pvPut(backLash, SYNC);



		/* here we have to work around a bug in teh maxV firmware, the 
		   current encoder position has just been pushed into the :step:sp
		   of our motor which is incorrect, here we calc a new current step 
		   by converting the current encoder value into steps then pushing
		   that value into :step:sp via the :loadStep record */
	    deviceLoadStep = (deviceEncSlope / deviceStepSlope) * deviceEncFbk;
	    pvPut(deviceLoadStep, SYNC);


	 
         }state goto_0_home_pulse 
	}


    state goto_0_home_pulse 
    {
         /*when(deviceMvComplete== 0 && delay(4.0))	*/
         when(deviceMoving== 0 && delay(1.0))
         {
	    snprintf(calibrateLog, 40, "%s: move to HOME", motorName);
            pvPut(calibrateLog, SYNC);
            printf("%s\n", calibrateLog);
	    /* send it to its home edge */
	    deviceStep = 0;
	    pvPut(deviceStep, SYNC);
	    printf(" sending it to Step = 0\n");

         }state set_calibrated_position 
	}



    state set_calibrated_position
	{
        /*
         *  When the device has passed the home switch and stage has stopped moving
		 *	(because the controller has detected the home/index pulse), calibration is complete.
		 *      The controller has now has zero loaded into the step position and encoder count registers
		 *	so set the offset to the home/index calibrated position.
         */

        when ((deviceMoving == 0)  &&  delay(2.0))
	{
		snprintf(calibrateLog, 40, "%s: setting calibrated position", motorName);
            	pvPut(calibrateLog, SYNC);
            	printf("%s\n", calibrateLog);

            	pvGet(calibratedPosition);
		deviceSetPosn = calibratedPosition;
		pvPut(deviceSetPosn, SYNC);

	} state calibration_complete

        /*
         *  if time out abort
         */
        when (delay(380))
		{
			snprintf(calibrateLog, 40, "%s: timed out - NOT calibrated", motorName);
			pvPut(calibrateLog);
			printf("%s\n", calibrateLog);
		} state stop_calibration

        when (calibrateRun == 0)
		{
   			deviceStop = 1;
                        pvPut(deviceStop, SYNC);

		} state stop_calibration
    }

    /*
     *  Calibration procedure completed
     *		- restore original velocity setting
     *		- set run state to 0
     *		- set calibrate state to 1
     */
    state calibration_complete
	{
		when (delay(1.0))
		{
			deviceVelocity = saveVelocity;
			pvPut(deviceVelocity);
			 
			/*restore closed loop and backlash settings*/
			closedLoop = saveclosedLoop;
        	        backLash = savebackLash;
	                pvPut(closedLoop);
                	pvPut(backLash);
			
	
			calibrateRun = 0;
			pvPut(calibrateRun, SYNC);
	
			calibrateDone = 1;
			pvPut(calibrateDone, SYNC);
		
			
            
			/*
			pvGet(finalPosition);
			devicePosition = finalPosition;	
			pvPut(devicePosition);
			*/

			 snprintf(calibrateLog, 40, "%s: successful calibration", motorName);
			pvPut(calibrateLog);
			printf("%s\n", calibrateLog);
		} state calibration_delay
	}

    state calibration_delay
	{
		when (delay(1.0))
		{
			pvGet(finalPosition);
			printf("Final Pos = %fmm\n",finalPosition);
                        devicePosition = finalPosition;
                        pvPut(devicePosition, SYNC);

		} state init_calibration
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
		snprintf(calibrateLog, 40, "%s: calibration discontinued", motorName);
		pvPut(calibrateLog, SYNC);
		printf("%s\n", calibrateLog);
		deviceStop = 1;
                pvPut(deviceStop, SYNC);

		}
	/* wait until stopped to reset the closed loop and other settings, else closed loop can start to check position before its stopped == hell */
	when (deviceMoving == 0  &&  delay(1.0))
		{
		deviceVelocity = saveVelocity;
		pvPut(deviceVelocity, SYNC);

                closedLoop = saveclosedLoop;
                backLash = savebackLash;
		pvPut(closedLoop, SYNC);
		pvPut(backLash, SYNC);

		calibrateRun = 0;
		pvPut(calibrateRun, SYNC);

		} state init_calibration
	}
}
