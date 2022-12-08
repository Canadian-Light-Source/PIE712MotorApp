/*
FILENAME...     PIE712Controller.cpp
USAGE...        PI GCS2 Motor Support.
 
*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************
 
Version:        $Revision: 1.3 $
Modified By:    $Author: Russ Berg (bergr) $
Last Modified:  $Date: 2019/07/18 13:16:07CST $
HeadURL:        $URL$
 
Original Author: Steffen Rau
Created: January 2011

Based on drvMotorSim.c, Mark Rivers, December 13, 2009

*/


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>

//#include <windows.h>

#include <asynStandardInterfaces.h>
#include <asynOctetSyncIO.h>
//#include <motorVersion.h>
#include <motor_interface.h>
#include <ctype.h>

#include "PIE712Controller.h"
#include "PIE712_datarecorder_defs.h"

#include <epicsExport.h>



#define TEST_DATARECORDER_XFER 1




static const char *driverName = "PIE712Driver";

void pie712_forceDone_Thread(void *drvPvt);
void readDatarecorderTask(void *drvPvt);

#define DIGITAL_OUTPUT_PIN 1
#define TRIG_OUT_ID DIGITAL_OUTPUT_PIN
#define MARKER_WINDOW 1.0

static ELLLIST PIE712ControllerList;
static int PIE712ControllerListInitialized = 0;

typedef struct PIE712ControllerNode {
    ELLNODE node;
    const char *portName;
    PIE712Controller *pController;
} PIE712ControllerNode;

extern char *load_param_file(const char* fname);
extern int exec_python_code(const char* code);




extern "C" {
int TranslatePIError(const int error, char* szBuffer, const int maxlen);

}

/*****************************************************************************/

PIE712Axis::PIE712Axis(PIE712Controller *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start )
  : asynMotorAxis(pController, axis), pC_(pController), lowHardLimit_(lowHardLimit), hiHardLimit_(hiHardLimit), home_(home)
{           
	int err = 0;
  deferred_move_ = 0;
	stepSize_ = 1.0;
	

/*  
  The motor status as received from the hardware.  The MSTA bits are defined as follows:
	DIRECTION: last raw direction; (0:Negative, 1:Positive)
	DONE: motion is complete.
	PLUS_LS: plus limit switch has been hit.
	HOMELS: state of the home limit switch.
	Unused
	POSITION: closed-loop position control is enabled.
	SLIP_STALL: Slip/Stall detected (eg. fatal following error)
	HOME: if at home position.
	PRESENT: encoder is present.
	PROBLEM: driver stopped polling, or hardware problem
	MOVING: non-zero velocity present.
	GAIN_SUPPORT: motor supports closed-loop position control.
	COMM_ERR: Controller communication error.
	MINUS_LS: minus limit switch has been hit.
	HOMED: the motor has been homed.
	The record is put into MAJOR STATE alarm if either SLIP_STALL or PROBLEM bits are detected. If HLSV is set, then the record is put into HIGH alarm if either a high soft limit or hard limit (PLUS_LS) has been reached. Similary for the low limits.
*/
	// We assume servo motor with encoder for now
  setIntegerParam(pController->motorStatusGainSupport_, 1);
  setIntegerParam(pController->motorStatusHasEncoder_, 1);
	setDoubleParam(pController->P_MarkerStart, 0.0);
	setDoubleParam(pController->P_MarkerStop, 0.0);
	setDoubleParam(pController->P_ScanStart, 0.0);
	setDoubleParam(pController->P_ScanStop, 0.0);
	
	setIntegerParam(pController->motorStatusHasEncoder_, 1);
	
	setIntegerParam(pController->P_VoltRangeWarning, 0);
	
	setDoubleParam(pController->motorVelocity_, 100000.0);
	
	m_enableStatus = true;
  m_simSetPoint = 0.0;
  m_axisNo = axis + 1;
  
  m_numxtra_polliters = 0;
    
  /*
  if(m_axisNo < 3){
  	m_piezo_atz_voltages[ axis] = -50.0; 
  } else {
  	m_piezo_atz_voltages[ axis ] = 50.0; 
  }
  */		
  m_CPUnumerator = 1000;
  m_CPUdenominator = 1;
  	
  m_default_velo = 1000000.0; /* um per sec */
	m_dPiezoMaxRange = 100.0; /* um */
	
	/* POS */
	m_piezo_driving_factor = 1.0;
	
	//m_forceDoneWaiteventId_ = epicsEventCreate(epicsEventEmpty);
	m_ForceDoneNOW = false;
	m_movePending = false;
	    	
	/* Create the thread that computes the waveforms in the background */
 /* status = (asynStatus)(epicsThreadCreate("PIE712AxisForceDoneWaitTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pie712_forceDone_Thread,
                          this) == NULL);
   */
}


/************************************************************/
void pie712_forceDone_Thread(void *drvPvt)
{
    PIE712Axis *pPvt = (PIE712Axis *)drvPvt;
    pPvt->pie712_forceDone_Thread();
}


/****************************************************/
void PIE712Axis::pie712_forceDone_Thread(void)
{
    /* This thread computes the waveform and does callbacks with it */
    
		//getIntegerParam(P_PointsPerRow, 		&p_points_per_row);
		

		while(1)
		{
	  
	  	(void) epicsEventWait(m_forceDoneWaiteventId_);
	  	printf("pie712_forceDone_Thread: starting wait:\n");
	  	m_movePending = true;
	  	epicsThreadSleep(m_force_done_wait_time);
	  	printf("pie712_forceDone_Thread: wait done:\n");
			m_ForceDoneNOW = true;
			     
		}
		//asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "Leaving daqmxCi_ReadPointTaskThread\n");
		return;
		

}


/*****************************************************************************/
asynStatus PIE712Axis::getStatus(int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
	
	if(m_simchan)
	{	
		   	homing = 0;
   			negLimit = 0;
   			posLimit = 0;
    		servoControl = 0;
    		return asynSuccess;
    }
    
    asynStatus status = getMoving(moving);
    
    if (status != asynSuccess)
    {
    	return status;
    }

   	homing = 0;
   	negLimit = 0;
   	posLimit = 0;
   	
   	getServo(&servoControl);
   	   	

    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::poll(bool *returnMoving)
{
    int done = 0;
    int moving, negLimit, posLimit, servoControl;
    int oldHoming = m_isHoming;
    static int update_slow_fbks = 0;
    double t_dval = 0.0;
		
		// make assumptions 
		
		 	negLimit = 0;
	   	posLimit = 0;
	   	
	    servoControl = 1;
	    
	    moving = 0;
		
		/* if the controller is transferring data reom the data recorder then just skip updating feedback */
		if(pC_->m_bDataTransfering){
			return asynSuccess;
		}	
				
		if(m_simchan)
		{	
			*returnMoving = false;
    	return asynSuccess;
		}			
	  
	  if(m_ForceDone == FORCE_DONE_NORMAL){
	  	// actually get the moving status 
	  	//printf("poll[%s] m_Force == FORCE_DONE_NORMAL, getting moving status\n", m_axisName);
	  	getStatus(m_isHoming, moving, negLimit, posLimit, servoControl);
	  	
	  } else if(m_ForceDone == FORCE_DONE_FORCED){	
	  	// force status to show its done moving
	  	//printf("poll[%s] m_Force == FORCE_DONE_FORCED, setting moving = 0\n", m_axisName);
	  	moving = 0;
	  } else if(m_ForceDone == FORCE_DONE_INTERNAL_TIMED){	
	   	//printf("poll[%s] m_Force == FORCE_DONE_INTERNAL_TIMED, getting moving status\n", m_axisName);
      if(m_ForceDonePollIters >= 0){
	    	m_ForceDonePollIters -= 1;
	    	//printf("poll[%s]: decrementing m_ForceDonePollIters = %d\n", m_axisName, m_ForceDonePollIters);
	    	if(m_ForceDonePollIters <= 0){
	    		moving = 0;	
	    		//printf("poll[%s]: setting moving = 0\n",m_axisName);
	    	}
	    }
	       
	  }
	  
		moving_ = moving>0?true:false;;
    if (moving == 0 && m_isHoming == 0)
    	done = 1;

    m_bMoving = (done!=1);
    if (!m_isHoming || CanCommunicateWhileHoming())
    {
		if (oldHoming && oldHoming != m_isHoming)
		{	
			/* the following call sets member var m_homed */
			getReferencedState();
		    asynPrint(pasynUser_, ASYN_TRACE_ERROR, //FIXME: ASYN_TRACE_FLOW,
		        "PIE712Axis::poll() axis %d referencing state changed, homed = %d\n",
		        m_axisNo, m_homed );
		}
		
		if (!m_isHoming )
		{
			
			m_bServoControl = (servoControl == 1);
			getAxisPositionCts();
			double realPosition;
			getAxisPosition( realPosition);
			/*
			setDoubleParam(pC_->PI_SUP_POSITION,      realPosition );
			*/
			
		}
    }
    if (m_isHoming)
    {
	    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
	        "PIE712Axis::poll() axis %d referencing ...\n", m_axisNo );
    }
    
	
    /*update_slow_fbks++;
   	if(update_slow_fbks > 20){
   	*/   	
   	getVolt(&m_dVolt);
		setDoubleParam(pC_->P_OutputVolt,          m_dVolt);
		
		/* track and latch the bool flag if the voltage is nearing its outer regions */
		if(m_dVolt < MIN_WARN_VOLTS){
			m_voltRngWarning = true;
		}
		if(m_dVolt > MAX_WARN_VOLTS){
			m_voltRngWarning = true;
		}
			
		
		getADSensor(&m_adcSensor);
		setDoubleParam(pC_->P_AdcSensor,	      	m_adcSensor );
		    
		getCapSensor(&m_capSensor);
		setDoubleParam(pC_->P_CapSensor,	      	m_capSensor );
    update_slow_fbks = 0;
    
    
  	/*}*/
  	setDoubleParam(pC_->motorPosition_,          m_positionCts );
    setDoubleParam(pC_->motorEncoderPosition_,   m_positionCts);
    setIntegerParam(pC_->motorStatusDirection_,   m_lastDirection);
    setIntegerParam(pC_->motorStatusDone_,        done );
    setIntegerParam(pC_->motorStatusHighLimit_,   posLimit);
    setIntegerParam(pC_->motorStatusHomed_,       m_homed );
    setIntegerParam(pC_->motorStatusMoving_,      !done );
    setIntegerParam(pC_->motorStatusLowLimit_,    negLimit);
    setIntegerParam(pC_->motorStatusGainSupport_,	true);
    setIntegerParam(pC_->motorStatusProblem_,		m_bProblem);
    setIntegerParam(pC_->motorStatusPowerOn_,		m_bServoControl);
    setIntegerParam(pC_->P_Power,	      	m_bServoControl );
    setIntegerParam(pC_->P_VoltRangeWarning,	 m_voltRngWarning );

		setIntegerParam(pC_->motorStatusDone_, moving_? 0:1);
		setIntegerParam(pC_->motorStatusMoving_,    moving_);
	
	
#ifdef DO_I_NEED_THESE
		/* as per the E712 PZ195E User Manual */
		/* the following use the input channel as the itemID */
    pC_->getGCSParameter(m_input_chan, DIGFILT_ORDER, t_dval);
    m_dDigFiltOrder = (int)t_dval;
    setIntegerParam(pC_->P_DigitalFilterORder,          m_dDigFiltOrder );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_TYPE, t_dval);
    m_dDigFiltType = (int)t_dval;
    setIntegerParam(pC_->P_DigitalFilterType,          m_dDigFiltType );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_BWIDTH, m_dDigFiltBWidth);
    setDoubleParam(pC_->P_DigitalFilterBWidth,          m_dDigFiltBWidth );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_PARM1, m_dDigFiltP1);
    setDoubleParam(pC_->P_DigitalFilterParm1,          m_dDigFiltP1 );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_PARM2, m_dDigFiltP2);    
    setDoubleParam(pC_->P_DigitalFilterParm2,          m_dDigFiltP2 );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_PARM3, m_dDigFiltP3);    
    setDoubleParam(pC_->P_DigitalFilterParm3,          m_dDigFiltP3 );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_PARM4, m_dDigFiltP4);    
    setDoubleParam(pC_->P_DigitalFilterParm4,          m_dDigFiltP4 );
    
    pC_->getGCSParameter(m_input_chan, DIGFILT_PARM5, m_dDigFiltP5);    
    setDoubleParam(pC_->P_DigitalFilterParm5,          m_dDigFiltP5);
    
    /* the following params use the logical axis as the itemID */

    pC_->getGCSParameter(m_axisNo, PTERM_PARAM, m_dPTerm);    
    setDoubleParam(pC_->P_PTerm, m_dPTerm);
    
    pC_->getGCSParameter(m_axisNo, ITERM_PARAM, m_dITerm);    
    setDoubleParam(pC_->P_ITerm, m_dITerm);
    
    pC_->getGCSParameter(m_axisNo, DTERM_PARAM, m_dDTerm);    
    setDoubleParam(pC_->P_DTerm, m_dDTerm);
    
    pC_->getGCSParameter(m_axisNo, PTERM_PARAM, m_dPTerm);    
    setDoubleParam(pC_->P_PTerm, m_dPTerm);
    
    pC_->getGCSParameter(m_axisNo, SLEW_RATE, t_dval);
    setDoubleParam(pC_->P_SlewRate, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_FREQ_1, t_dval);
    setDoubleParam(pC_->P_NotchFreq1, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_FREQ_2, t_dval);
    setDoubleParam(pC_->P_NotchFreq2, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_REJECTION_1, t_dval);
    setDoubleParam(pC_->P_NotchRejection1, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_REJECTION_2, t_dval);
    setDoubleParam(pC_->P_NotchRejection2, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_BWIDTH_1, t_dval);
    setDoubleParam(pC_->P_NotchBW1, t_dval);
    
    pC_->getGCSParameter(m_axisNo, NOTCH_BWIDTH_2, t_dval);
    setDoubleParam(pC_->P_NotchBW2, t_dval);
#endif
    
    callParamCallbacks();

    *returnMoving = m_bMoving;
    return asynSuccess;
}


/***************************************************************************/
/* set the velocity and acceleration in EGU */
asynStatus PIE712Axis::sendAccelAndVelocity(double accelerationEgu, double velocityEgu) 
{
	/* the E-712 doesnt support setting acceleration */
  asynStatus status = asynSuccess;
  if (velocityEgu != 0)
	{
		status = setVelocityEgu( velocityEgu);
		if (asynSuccess != status)
		{
			return status;
		}
	}
	
	if(status != asynSuccess){
		printf("ERROR setting velocity for axis [%s]\n", m_axisName);
		
	}
  return status;
}

/***************************************************************************/
asynStatus PIE712Axis::move(double positionCts, int relative, double minVelocityCts, double maxVelocityCts, double accelerationCts)
{
  double cur_posEgu;
  double positionEgu = 0.0;
  double minvelocityEgu = 0.0;
  double maxvelocityEgu = 0.0;
  double accelerationEgu = 0.0;
  asynStatus status = asynSuccess;
  static const char *functionName = "move";
	double velo_fbk= 0.0;
	int p_mode;
	
	
	if(m_simchan)
	{	
			return(	asynSuccess);
	}	
		
	/*positionEgu = positionCts *1 / 1000.0 */
	positionEgu = positionCts * m_CPUdenominator / m_CPUnumerator;
	minvelocityEgu = minVelocityCts * m_CPUdenominator / m_CPUnumerator;
	maxvelocityEgu = maxVelocityCts * m_CPUdenominator / m_CPUnumerator;
	accelerationEgu = accelerationCts * m_CPUdenominator / m_CPUnumerator;

  //printf("PIE712Axis:[%s]:move(positionEgu=%f, maxvelocityEgu=%f, accelerationEgu=%f)\n", m_axisName, positionEgu, maxvelocityEgu, accelerationEgu);
	//getPositionEgu(&cur_posEgu);
	cur_posEgu = getPositionEGU();
	
	if(abs(cur_posEgu - positionEgu) <= MAX_MOVE_WITH_FORCED_STATUS){
		m_ForceDonePollIters = MAX_MOVE_WITH_FORCED_STATUS_VAL;
		//printf("E712 move[%s]: m_ForceDonePollIters = %d\n", m_axisName, m_ForceDonePollIters);
	} else {
		m_ForceDonePollIters = int(abs((cur_posEgu - positionEgu)/maxvelocityEgu)) + m_numxtra_polliters;
		//printf("E712 move[%s]: m_ForceDonePollIters = %d\n", m_axisName, m_ForceDonePollIters);
	}	
	
	pC_->getIntegerParam(axisNo_, pC_->P_Mode, &p_mode);
	

	if( (strstr(m_axisName, "SampleFineX") != NULL) || (strstr(m_axisName, "ZoneplateX") != NULL) ){
			/*configure_for_mode(position, relative, minVelocity*stepSize_, maxVelocity*stepSize_,  acceleration*stepSize_);*/
			configure_for_mode(positionEgu, relative, minvelocityEgu, maxvelocityEgu,  accelerationEgu);
	} else {
			/* Y, need to set the velocity here */
			status = sendAccelAndVelocity(accelerationEgu, maxvelocityEgu);
	}
	
	/* if the mode is coarse then the marker will now be set so just push the setpoint to agilent (COARSE means motor is already off, then leave */
	if(p_mode == MODE_COARSE)
	{
		return(asynSuccess);
	}
	
	/*printf("inside [%s]->move() position=%f, minVelocity=%f maxVelocity=%f, acceleration=%f\n",m_axisName, position, minVelocity, maxVelocity, acceleration);
	printf("\n");
	*/
	setIntegerParam(pC_->motorStatusDone_, 0);
	callParamCallbacks();
	
	if (relative) {
  	//getPositionEgu(&cur_posEgu);
  	status = move( cur_posEgu + positionEgu);
  } else {
		status = move( positionEgu);
  }
  
  
	epicsEventSignal(pC_->pollEventId_);

  //asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, maxVel=%f, accel=%f\n", driverName, functionName, pC_->portName, axisNo_, new_pos, minVelocity, maxVelocity, acceleration );
  return asynSuccess;
}

/*****************************************************************************/
asynStatus PIE712Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
	
	asynStatus status = asynError;
    static const char *functionName = "moveVelocityAxis";
	

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	


	if (!AcceptsNewTarget())
	{
	    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
	        "%s:%s: Set port %s, axis %d - controller does not accept new target (busy?)",
	        driverName, functionName, pC_->portName, m_axisNo );
		return status;
	}


    setIntegerParam(pC_->motorStatusDone_, 0);
    callParamCallbacks();


    double target = maxVelocity > 0 ? posLimit_ : negLimit_;

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f / target %f - BEFORE MOV\n",
        driverName, functionName, pC_->portName, m_axisNo, maxVelocity, acceleration, target );

    setVelocityCts( maxVelocity);
    move( target);

    epicsEventSignal(pC_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f / target %f - AFTER MOV\n",
        driverName, functionName, pC_->portName, m_axisNo, maxVelocity, acceleration, target );
    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::stop(double acceleration)
{
	
    static const char *functionName = "stopAxis";


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	

    deferred_move = 0;

    haltAxis();

    epicsEventSignal(pC_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set axis %d to stop with accel=%f",
        driverName, functionName, m_axisNo, acceleration );
    return asynSuccess;
}
/*****************************************************************************/

asynStatus PIE712Axis::referenceVelCts( double velocity, int forwards)
{
	asynStatus status = motorOn();
    if (asynSuccess != status)
    	return status;

	char cmd[100];
	

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	
	if (m_bHasReference)
	{
		// call FRF - find reference
		sprintf(cmd,"FRF %d", m_axisNo);
	}
	else if (m_bHasLimitSwitches)
	{
		if (forwards)
		{
			// call FPL - find positive limit switch
			sprintf(cmd,"FPL %d", m_axisNo);
		}
		else
		{
			// call FNL - find negative limit switch
			sprintf(cmd,"FNL %d", m_axisNo);
		}
	}
	else
	{
	    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
	    		"PIE712Axis::referenceVelCts() failed - axis has no reference/limit switch\n");
		epicsSnprintf(pasynUser_->errorMessage,pasynUser_->errorMessageSize,
			"PIE712Axis::referenceVelCts() failed - axis has no reference/limit switch\n");
		return asynError;
	}
	status = pC_->m_pInterface->sendOnly(cmd);
	if (asynSuccess != status)
		return status;
	int errorCode = pC_->getGCSError();
	if (errorCode == 0)
	{
		return asynSuccess;
	}
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
    		"PIE712Axis::referenceVelCts() failed\n");
	epicsSnprintf(pasynUser_->errorMessage,pasynUser_->errorMessageSize,
		"PIE712Axis::referenceVelCts() failed - GCS Error %d\n",errorCode);
	return asynError;

}

/*****************************************************************************/
/*** to home the E712 uses ATZ (autozero) not FRF **/
asynStatus PIE712Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
	
    asynStatus status = asynError;
    static const char *functionName = "homeAxis";
    /*double positionEgu, positionCts;*/

    m_isHoming = 1;
    setIntegerParam(pC_->motorStatusDone_, 0 );
	
    callParamCallbacks();

    //status = referenceVelCts(maxVelocity, forwards);
		
		status = autoZero();
		
    if (asynSuccess != status)
    {
    	return status;
    }
	
		//pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &positionCts );
		
		/* even though we are turning on the servo now, it doesnt recognize the 
		current setpoint so push it in after the servo is on */	
		//status = motorOn();
		/* push new setpoint */
		//setPosition(positionCts);
		
	  setIntegerParam(pC_->motorStatusHomed_, m_homed );
		callParamCallbacks();
	  epicsEventSignal(pC_->pollEventId_);
	
	  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, pC_->portName, m_axisNo, (forwards?"FORWARDS":"REVERSE"), minVelocity, maxVelocity, acceleration );
    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::setPosition(double position)
{


	if(m_simchan)
	{	
		return(	asynSuccess);
	}	
		
	asynStatus status = asynError;
	status = setAxisPositionCts( position);
    epicsEventSignal(pC_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d set position to %f - status=%d\n",
        driverName, "setPositionAxis", pC_->portName, m_axisNo, position, int(status) );
    return status;
}


/** reset the vaoltage warning flag
   */
asynStatus PIE712Axis::resetVoltWarning(int warn)
{
	if(warn){
		m_voltRngWarning = true;
	} else {
		m_voltRngWarning = false;
	}
	setIntegerParam(pC_->P_VoltRangeWarning,	m_voltRngWarning );
	
  return asynSuccess;

}




/** Set the proportional gain of the motor.
  * \param[in] pGain The new proportional gain. */
asynStatus PIE712Axis::setPGain(double pGain)
{
	pC_->setGCSParameter(this, PTERM_PARAM, pGain);
	
	m_dCoEffProp = pGain; 
  return asynSuccess;

}


/** Set the integral gain of the motor.
  * \param[in] iGain The new integral gain. */
asynStatus PIE712Axis::setIGain(double iGain)
{
	pC_->setGCSParameter(this,ITERM_PARAM, iGain);
	m_dCoEffInt = iGain;
  return asynSuccess;
}


/** Set the derivative gain of the motor.
  * \param[in] dGain The new derivative gain. */
asynStatus PIE712Axis::setDGain(double dGain)
{
	pC_->setGCSParameter(this, DTERM_PARAM, dGain);
	m_dCoEffDiff = dGain;
  return asynSuccess;
}

/** Set the digital filter type of the motor.
*/
asynStatus PIE712Axis::setDigFiltType(double dtype)
{
	//pC_->setGCSParameter(this, DIGFILT_TYPE, dtype);
	//m_iDigFiltType = itype;
  return asynSuccess;
}

/** Set the digital filter bandwidth of the motor.
*/
asynStatus PIE712Axis::setDigFiltBWidth(double dbwidth)
{
	//pC_->setGCSParameter(this, DIGFILT_BWIDTH, dbwidth);
	//m_dDigFiltBWidth = dbwidth;
  return asynSuccess;
}

/** Set the digital filter order of the motor.
*/
asynStatus PIE712Axis::setDigFiltOrder(double dOrder)
{
	//pC_->setGCSParameter(this, DIGFILT_ORDER, dOrder);
	//m_iDigFiltOrder = iOrder;
  return asynSuccess;
}


/** Set the digital filter parameter 1 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm1(double dP)
{
	//setDigFiltParm(DIGFILT_PARM1, dP);
	//m_dDigFiltP1 = dP;
	return asynSuccess;
}
/** Set the digital filter parameter 2 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm2(double dP)
{
	//setDigFiltParm(DIGFILT_PARM2, dP);
	//m_dDigFiltP2 = dP;
	return asynSuccess;
}
/** Set the digital filter parameter 3 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm3(double dP)
{
	//setDigFiltParm(DIGFILT_PARM3, dP);
	//m_dDigFiltP3 = dP;
	return asynSuccess;
}
/** Set the digital filter parameter 4 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm4(double dP)
{
	//setDigFiltParm(DIGFILT_PARM4, dP);
	//m_dDigFiltP4 = dP;
	return asynSuccess;
}
/** Set the digital filter parameter 5 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm5(double dP)
{
	//setDigFiltParm(DIGFILT_PARM5, dP);
	//m_dDigFiltP5 = dP;
	m_numxtra_polliters = int(dP);
	//printf("TESTING: setting m_numxtra_polliters = %d\n", m_numxtra_polliters);
	return asynSuccess;
}

/** Set the digital filter parameter 5 of the motor.
*/
asynStatus PIE712Axis::setForceDoneWTime(double dtime)
{
	m_force_done_wait_time = dtime;
	//printf("TESTING: setting m_force_done_wait_time = %.3f\n", m_force_done_wait_time);
	return asynSuccess;
}


/** Set the digital filter parameter 1 of the motor.
*/
asynStatus PIE712Axis::setDigFiltParm(int param_id, double dP)
{
	//pC_->setGCSParameter(this, param_id, dP);
	return asynSuccess;
}


/****************************************************************************************/
asynStatus PIE712Axis::Init(const char *portname)
{
	char cmd[100];
	char buf[255];
	double val, cap, quad = 0.0;
	

	/*
	if(m_simchan)
	{	
		return(	asynSuccess);
	}
	*/	
  sprintf(cmd, "CST? %d", m_axisNo);
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
  	return status;
  }
  if (NULL != pasynUser_)
  {
  	asynPrint(pasynUser_, ASYN_TRACE_FLOW,
  	 "PIE712Axis::initAxis() stage configuration: %s\n", buf);
  }
  m_movingStateMask = int(pow(2.0, m_axisNo - 1));
	status = pC_->findConnectedAxes();


	/* assign sensor designations */
	if(m_axisNo == 1){
		m_quad_parm = AXIS1_QUADRATURE;
		m_cap_parm = AXIS1_CAPACITIVE;
		m_cap_input_chan = AXIS1_INPUT_CHANNEL_CAP;
		m_quad_input_chan = AXIS1_INPUT_CHANNEL_INT;
		m_piezo_driving_factor_param = AXIS1_DRIVING_FACTOR_PARAM;
	} else if(m_axisNo == 2){
		m_quad_parm = AXIS2_QUADRATURE;
		m_cap_parm = AXIS2_CAPACITIVE;
		m_cap_input_chan = AXIS2_INPUT_CHANNEL_CAP;
		m_quad_input_chan = AXIS2_INPUT_CHANNEL_INT;
		m_piezo_driving_factor_param = AXIS2_DRIVING_FACTOR_PARAM;
	} else if(m_axisNo == 3){
		m_quad_parm = AXIS3_QUADRATURE;
		m_cap_parm = AXIS3_CAPACITIVE;
		m_cap_input_chan = AXIS3_INPUT_CHANNEL_CAP;
		m_quad_input_chan = AXIS3_INPUT_CHANNEL_INT;
		m_piezo_driving_factor_param = AXIS3_DRIVING_FACTOR_PARAM;
	} else if(m_axisNo == 4){
		m_quad_parm = AXIS4_QUADRATURE;
		m_cap_parm = AXIS4_CAPACITIVE;
		m_cap_input_chan = AXIS4_INPUT_CHANNEL_CAP;
		m_quad_input_chan = AXIS4_INPUT_CHANNEL_INT;
		m_piezo_driving_factor_param = AXIS4_DRIVING_FACTOR_PARAM;
	}			
	
	pC_->getGCSParameter(this, m_quad_parm, quad);
	pC_->getGCSParameter(this, m_cap_parm, cap);


	/* both encoder inputs use same voltage for ATZ */
	m_input_chan = m_cap_input_chan;
	m_piezo_atz_voltages[ m_axisNo - 1] = AUTOZERO_VOLTS; 
	
	pC_->getGCSParameter(this, m_piezo_driving_factor_param, val);
	
	//setIntegerParam(pC_->P_SelectOutputDir, int(val));


	callParamCallbacks();
	
	/* leave it off to start */
	return setServo(0);
	
}



/****************************************************************************************/
asynStatus PIE712Axis::autoZero(void)
{
	char cmd[100];
	asynStatus status = asynSuccess;
	double p_atzVolts = 0.0;
	
	pC_->getDoubleParam(axisNo_, pC_->P_ATZVolt, &p_atzVolts);
	
		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
  
  /* full votage range is -20v to 120v so mid range is 50v */
  /* <autozero command> <axis num> <0um at (50) volts> */
  /*sprintf(cmd, "ATZ %d NAN", m_axisNo);*/
  /*sprintf(cmd, "ATZ %d %.2f", m_axisNo, m_piezo_atz_voltages[m_axisNo-1]);*/
  sprintf(cmd, "ATZ %d %.2f", m_axisNo, p_atzVolts);
  
  
  printf("autoZero: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
  	asynPrint(pasynUser_,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
  	 "PIE712Axis::autoZero() unable to [%s]\n", cmd);
  }
  
	return status;
}	


/****************************************************************************************/
asynStatus PIE712Axis::getAutoZeroStatus(bool *atz_sts)
{
	char cmd[100];
	char buf[256];
	double sts = 0;
	asynStatus status = asynSuccess;


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
  
  /* full votage range is -20v to 120v so mid range is 50v */
  /* <autozero command> <axis num> <0um at (50) volts> */
  /*sprintf(cmd, "ATZ %d NAN", m_axisNo);*/
  sprintf(cmd, "ATZ? %d", m_axisNo);
  status = pC_->m_pInterface->sendAndReceive(cmd, buf, 255);
  
  if (!pC_->getValue(buf, sts))
	{
		return asynSuccess;
	}
	if(sts > 0){
		*atz_sts = true;
	} else {
		*atz_sts = false;	
	}	
  
  
	return asynSuccess;
}	


/****************************************************************************************
#define REFMODE_ABS_SENSOR 1
#define REFMODE_NEG_LIMIT 2
#define REFMODE_POS_LIMIT 3
#define REFMODE_SIGNED_MARK 4
#define REFMODE_ON_IMPULSE 5


#define REFERENCE_MODE					0x02000a00
So in order to zero the X interferometer input AD Sensor which is always channel 1 I had to:
>>SPA <input channel> <param> <newval>
>>FRF <axisID> <param> <newval>

>>SVO 1 0
>>SPA 1 0x02000a00 2
>>SPA? 1 0x02000a00
<<1 0x2000a00=2
>>FRF 3
>>TAD? 1
<<1=-58
>>SPA 1 0x02000a00 1
>>ATZ 3 NAN

-	Switch servo mode of the correspondent axis to OFF
-	Regarding correspondent input channel:
Change the value of parameter ID 0x02000a00 (Sensor reference mode) from 1 (absolute sensor) to 2 (ref. on neg. limit)
-	Reference the correspondent axis via command FRF, e.g. for axis 1: FRF 1
?	Axis is moving to the neg. limit (neg. soft limit of the correspondent amp output channel), in case of Russ’s system, the correspondent amp output voltage is set to the “soft low limit” -> e.g. -30V, which means, the HERA stage is moving
?	Check via FRF? 1, if the axis has been referenced -> 1 means “referenced”, 0 means “failed”
?	The TAD value of the correspondent input channel is supposed to be around ZERO
-	Perform an auto zero
-	Switch servo mode to ON


*/

asynStatus PIE712Axis::zeroADSensor(void)
{
	char cmd[100];
	char buf[256];
	asynStatus status = asynSuccess;


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

  
  motorOff();
  
  sprintf(cmd, "CCL 1 advanced");
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendOnly(cmd);
  
  
  /* as per instructions from Andreas Haas for zeroing the ADSensor */
  sprintf(cmd, "SPA %d %#8x %d", m_input_chan, REFERENCE_MODE, REFMODE_NEG_LIMIT);
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendOnly(cmd);
  
  sprintf(cmd, "SPA? %d %#8x", m_input_chan, REFERENCE_MODE);
  status = pC_->m_pInterface->sendAndReceive(cmd, buf, 255);
  printf("zeroADSensor: Rcvd [%s] from controller\n", buf);
  
  sprintf(cmd, "FRF %d", m_axisNo);
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendOnly(cmd);
  
  printf("zeroADSensor: waiting 5 seconds for FRF to complete\n");
  epicsThreadSleep(5.0);

  sprintf(cmd, "TAD? %d", m_input_chan);
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendAndReceive(cmd, buf, 255);
  printf("zeroADSensor: Rcvd AD sensor value reading of[%s] from controller\n", buf);
  
  
  sprintf(cmd, "SPA %d %#8x %d", m_input_chan, REFERENCE_MODE, REFMODE_ABS_SENSOR);
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendOnly(cmd);
  
  printf("zeroADSensor: performing an autozero\n");
  autoZero();
  
  printf("zeroADSensor: waiting 10 seconds for AutoZero to complete\n");
  epicsThreadSleep(10.0);
  
  sprintf(cmd, "TAD? %d", m_input_chan);
  printf("zeroADSensor: Sending [%s] to controller\n", cmd);
  status = pC_->m_pInterface->sendAndReceive(cmd, buf, 255);
  printf("zeroADSensor: Rcvd AD sensor value reading of[%s] from controller\n", buf);
  
  printf("zeroADSensor: zeroADSensor complete\n");
  
  if (status != asynSuccess)
  {
  	asynPrint(pasynUser_,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
  	 "PIE712Axis::zeroADSensor() unable to [%s]\n", cmd);
  }
	return status;
}	


/****************************************************************************************/
asynStatus PIE712Axis::setOpenLoopPosition(double volts)
{
	char cmd[100];
	asynStatus status = asynSuccess;


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

  /* <Set Openloop> <axis num> <volts> */
  
  sprintf(cmd, "SVA %d %f", m_axisNo, volts);
  status = pC_->m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
  	asynPrint(pasynUser_,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
  	 "PIE712Axis::setOpenLoopPosition() unable to [%s]\n", cmd);
  }
	return status;
}	


/** this function sets the sign of the following parameters
															for axis x1:
		   Position From Sensor:	0x07000504
		Driving Factor of Piezo:  0x09000004
	
say the quadrature encoder counts:
 	left(+) and right (-)
but the piezo stage and its capacitive feedback counts:
  left(-) and right (+) 

If your desire is to be able to switch back and forth in software between the capacitive encoder 
and the quadrature (in the UHV STXM case an interferometer) you have to change the sign of these parameters
so that closed loop will continue to work. So when the SelectOutputDirection portDriver parameter is:
	POS: the output voltage and counting of the current input sensor is the default which is + volts == + counts
	NEG: the output voltage and counting of the current input sensor is the default which is - volts == - counts 
		
*/		
asynStatus PIE712Axis::selectPiezoOutputDirection(int dir)
{

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
#ifdef TESTING
	if(dir == POS)
	{
		/* POS */
		m_piezo_driving_factor = 1.0;
	} else {
		/* NEG */
		m_piezo_driving_factor = -1.0;
	}	
	setIntegerParam(pC_->P_SelectOutputDir, m_piezo_driving_factor);
		
	callParamCallbacks();
#endif
	return(asynSuccess);
}
/****************************************************************************************/
/*
This function handles the fact that only 1 of the "X" axis' (SampleX or ZoneplateX, this goes for 
the "Y" axis' as well), can use the quadrature from the interferometer at a time, so this function
toggle the "other" axis so that the one calling this function gets what it has selected
*/
asynStatus PIE712Axis::selectEncoderSource(int type)
{
	int on_axis = 0;
	int off_axis = 0;
	int on_type = 0;
	int off_type = 0;
	asynStatus status = asynSuccess;

  return(asynSuccess);

#ifdef TESTING
  if(m_axisNo == 1){
  	on_axis = 1;	/* x1 */
  	off_axis = 3; /* x2 */
  	
  } else if(m_axisNo == 2){
  	on_axis = 2;	/* z1 */
  	off_axis = 4; /* z2 */
  	
  } else if(m_axisNo == 3){
  	on_axis = 3;	/* x2 */
  	off_axis = 1; /* x1 */
  	
  } else if(m_axisNo == 4){
  	on_axis = 4;	/* z2 */
  	off_axis = 2; /* z1 */
  } 
  
  if(type == USE_CAPACITIVE_SENSOR){
  	/* turn on CAPACITIVE and turn off QUADRATURE */
  	on_type = USE_CAPACITIVE_SENSOR;
  	off_type = USE_QUADRATURE_SENSOR;
  } else {
  	/* turn on QUADRATURE and turn off CAPACITIVE */
  	on_type = USE_QUADRATURE_SENSOR;
  	off_type = USE_CAPACITIVE_SENSOR;
  }	
  
  motorOff();
  /* now toggle the axis' */
  pC_->toggleEncoderSource(off_axis, off_type);
  pC_->toggleEncoderSource(on_axis, on_type);
  
  motorOn();
  		
  	
	return status;
#endif	
}

/*****************************************************************************/

bool PIE712Axis::IsGCS2(void)
{
	char buf[256];
	float csv=0.0;
	

		if(m_simchan)
		{	
			return(	true);
		}	
	
	asynStatus status = pC_->m_pInterface->sendAndReceive("CSV?", buf, 255);
	if (asynTimeout == status)
	{
		return false;
	}
	else if (asynSuccess != status)
	{
		return false;
	}
	
	csv = float(atof(buf));
	return (csv >= 2.0);
}

/*****************************************************************************/
asynStatus PIE712Axis::setVelocityCts(double velocity )
{
	char cmd[100];


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

	
	velocity = fabs(velocity) * m_CPUdenominator / m_CPUnumerator;
    sprintf(cmd,"VEL %d %f", m_axisNo, velocity);
    asynStatus status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess == status)
    {
    	m_velocity = velocity;
    }
    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::setVelocityEgu(double velocity )
{
	char cmd[100];


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
    sprintf(cmd,"VEL %d %f", m_axisNo, velocity);
    asynStatus status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess == status)
    {
    	m_velocity = velocity;
    }
    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::moveCts( PIE712Axis** pAxesArray, int* pTargetCtsArray, int numAxes)
{

	asynStatus status;
	char cmd[1000] = "MOV";
	char subCmd[100];


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	for (int axis = 0; axis <numAxes; axis++)
	{
		PIE712Axis* pAxis = pAxesArray[axis];
		double target = double(pTargetCtsArray[axis]) * m_CPUdenominator / m_CPUnumerator;
		//sprintf(subCmd," %d %f", m_axisNo, target);
		//strcat(cmd, subCmd);
		sprintf(subCmd,"MOV %d %f", m_axisNo, target);
	    m_lastDirection = (pTargetCtsArray[axis] > m_positionCts) ? 1 : 0;
	}
    status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    int errorCode = pC_->getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::moveCts(array) failed, GCS error %d\n", errorCode);
    return asynError;
}

/*****************************************************************************/
asynStatus PIE712Axis::setAxisPositionCts(double positionCts)
{
	double position = double(positionCts) * m_CPUdenominator / m_CPUnumerator;

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

	asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
		"PIE712Axis::setAxisPositionCts(, %f) \n", positionCts);
	printf("PIE712Axis::setAxisPositionCts(, %f) \n", positionCts);
	return setAxisPosition( position);
}

/*****************************************************************************/
asynStatus PIE712Axis::setAxisPosition(double position)
{
	asynStatus status;
	char cmd[100];
	

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
    sprintf(cmd,"RON %d 0", m_axisNo);
    status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::setAxisPosition() sent \"%s\"\n", cmd);
    sprintf(cmd,"POS %d %f", m_axisNo, position);
    status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::setAxisPosition() sent \"%s\"\n", cmd);    
    printf("PIE712Axis::setAxisPosition() sent \"%s\"\n", cmd);
    
    sprintf(cmd,"RON %d 1", m_axisNo);
    status = pC_->m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::setAxisPosition() sent \"%s\"\n", cmd);

    int errorCode = pC_->getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::setAxisPosition() failed, GCS error %d\n", errorCode);
    return asynError;

}

/*****************************************************************************/
asynStatus PIE712Axis::moveCts(int targetCts )
{
	double target;

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	 target = double(targetCts) * m_CPUdenominator / m_CPUnumerator;
    asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::moveCts(, %d) \n", targetCts);
    		
	return move(target);
}

/*****************************************************************************/
asynStatus PIE712Axis::move(double target )
{
	asynStatus status;
	char cmd[100];


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
  //m_movePending = true;
  
  sprintf(cmd,"MOV %d %f", m_axisNo, target);
  status = pC_->m_pInterface->sendOnly(cmd);
  
  /*
  if(m_ForceDone == FORCE_DONE_INTERNAL_TIMED){	
  	epicsEventSignal(m_forceDoneWaiteventId_);
	}
	*/
	
  if (asynSuccess != status)
  {
  	return status;
  }
  //asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,"PIE712Axis::move() sent \"%s\"\n", cmd);
  
  m_lastDirection = (target > m_position) ? 1 : 0;
  int errorCode = pC_->getGCSError();
	/* error code 10 is "stopped by command" which shouldnt be listed as an error */
  if ((errorCode == 0) || (errorCode == 10))
  	return asynSuccess;
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIE712Axis::move() failed, GCS error %d\n", errorCode);
  return asynError;
}

/*****************************************************************************/
asynStatus PIE712Axis::haltAxis(void)
{
	/* the E-712 doesnt support the stopping of individual channels, all you can call is
	the STP command which unfortunately stops all channels
	*/
	char cmd[100];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
    sprintf(cmd,"STP");
    asynStatus status = pC_->m_pInterface->sendOnly(cmd);
    if (status != asynSuccess)
    {
    	return status;
    }
    return status;
}



/**
 *  get position of axis in physical units (EGU) as defined on the controller
 */
asynStatus PIE712Axis::getAxisPosition(double& position)
{
	char cmd[100];
	char buf[255];\
	int enc_src = 0; // 0 == Capacitance
	double e712_pos_scaler = 1.0;

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	sprintf(cmd, "POS? %d", m_axisNo);
	asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
	//printf("PIE712Axis::getAxisPosition, sent[%s] received [%s]\n", cmd, buf);
	if (status != asynSuccess)
	{
		return status;
	}
	if (!pC_->getValue(buf, position))
	{
		status = asynError;
	}
	
	// scale the value coming from the E712 as it was adjusted on the controller to make the system tunable
	pC_->getDoubleParam(axisNo_, pC_->P_PosFromE712Scaler, &e712_pos_scaler);
	position = position * e712_pos_scaler;
	
	
	// if using the interferometer for encoder input reduce the value by a 1000 so that it will
	// fit into the 32 bit motor Record RRBV field
	pC_->getIntegerParam(axisNo_, pC_->P_SelectEncoderSrc, &enc_src);
	if(enc_src == USE_QUADRATURE_SENSOR)
	{
		position = position * 0.001;
	}
	
	// if it is set for CAPACITANCE then just leave it, NOTE: the ERES and MRES must be set to account for 
	// which encoder is selected
	
	return status;
}

/**
 *  get velocity of axis in physical units (EGU) as defined on the controller
 *  and set PIE712Axis::m_velocity
 */
asynStatus PIE712Axis::getAxisVelocity()
{
	char cmd[100];
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	sprintf(cmd, "VEL? %d", m_axisNo);
	asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	if (!pC_->getValue(buf, m_velocity))
	{
		status = asynError;
	}
    return status;

}

/**
 * Find travel range for axis.
 */
asynStatus PIE712Axis::getTravelLimits(double& negLimit, double& posLimit)
{
	char cmd[100];
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
    sprintf(cmd, "TMN? %d", m_axisNo);
    asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }
	if (!pC_->getValue(buf, negLimit))
	{
		return asynError;
	}
	sprintf(cmd, "TMX? %d", m_axisNo);
     status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
     if (status != asynSuccess)
     {
     	return status;
     }
	if (!pC_->getValue(buf, posLimit))
	{
		return asynError;
	}

     return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::hasLimitSwitches()
{
	char cmd[100];
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
     sprintf(cmd, "LIM? %d", m_axisNo);
     asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
     if (status != asynSuccess)
     {
     	return status;
     }
		if (!pC_->getValue(buf, m_bHasLimitSwitches))
		{
			return asynError;
		}
    if (!m_bHasLimitSwitches)
     {
         sprintf(cmd, "HAR? %d", m_axisNo);
         asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
         if (status == asynSuccess)
         {
     		if (!pC_->getValue(buf, m_bHasLimitSwitches))
     		{
     			return asynError;
     		}
        }
         else if (status == asynTimeout)
         {
        	 int err = pC_->getGCSError();
        	 if (err == PI_CNTR_UNKNOWN_COMMAND)
        	 {
            	 // "HAR?" not known
        		 m_bHasLimitSwitches = false;
        	 }
        	 else
        	 {
        		 return status;
        	 }
         }
         else
         {
        	 return status;
         }
     }

     asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    		 "PIE712Axis::hasLimitSwitches() axis has %slimit switches\n",
    		 m_bHasLimitSwitches?"":"no ");
     return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::hasReferenceSensor(void)
{
	char cmd[100];
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
    sprintf(cmd, "TRS? %d", m_axisNo);
    asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }

	if (!pC_->getValue(buf, m_bHasReference))
	{
		return asynError;
	}

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
   		 "PIE712Axis::hasReferenceSwitch() axis has %sreference sensor\n",
   		 m_bHasReference?"":"no ");
    return status;
}

/**
 *  get position of axis in counts as used in EPICS.
 *  getAxisPosition() is called and position is covnerted to counts using the
 *  counts-per-unit (CPU) fraction of the axis.
 */
asynStatus PIE712Axis::getAxisPositionCts(void)
{
	double pos;

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	asynStatus status = getAxisPosition(pos);
    if (status != asynSuccess)
    {
    	return status;
    }
    m_position = pos;
    if (m_CPUdenominator==0 || m_CPUnumerator==0)
    {
    	m_positionCts = int(pos);
    	return status;
    }

    m_positionCts = int( (pos * double(m_CPUnumerator) / double(m_CPUdenominator))+0.5);
    if (pasynUser_ != NULL)
    {
    	asynPrint(pasynUser_, ASYN_TRACE_FLOW,
               "PIE712Axis::getAxisPositionCts() pos:%d\n",
               m_positionCts);
    }
    return status;
}

//void PIE712Axis::calcAxisPositionCts();
/*****************************************************************************/
asynStatus PIE712Axis::setServo(int servoState)
{
    char cmd[100];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	    
    sprintf(cmd, "SVO %d %d", m_axisNo, servoState);
    asynStatus status = pC_->m_pInterface->sendOnly(cmd);
    if (status != asynSuccess)
    {
    	return status;
    }
    int err = pC_->getGCSError();
    if (COM_NO_ERROR == err)
    {
    	m_bServoControl = (servoState == 1);
    	if (m_bProblem && m_bServoControl)
    	{
    		m_bProblem = false;
    	}
    	return asynSuccess;
    }
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "Could not set servo state!\n");
	return asynError;

}

/*****************************************************************************/
asynStatus PIE712Axis::getMoving(int& moving)
{
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
    asynStatus status = pC_->m_pInterface->sendAndReceive(char(5), buf, 99);
    if (status != asynSuccess)
    {
			//printf("PIE712Axis::getMoving() failed, status %d", status);
			return status;
    }
		/*printf("PIE712Axis::getMoving() buf= %s\n", buf);*/
    char* pStr;
    long movingState = strtol(buf, &pStr, 16);
    
    moving = (movingState & m_movingStateMask) != 0 ? 1 : 0;

    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::getBusy(int& busy)
{
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
    asynStatus status = pC_->m_pInterface->sendAndReceive(char(7), buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }

    unsigned char c = (unsigned char)buf[0];
    busy = (c==0xB0);

    return status;
}

/*****************************************************************************/
asynStatus PIE712Axis::getReferencedState(void)
{
	char cmd[100];
	char buf[255];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
    sprintf(cmd, "FRF? %d", m_axisNo);
    asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }
    if (!pC_->getValue(buf, m_homed))
    {
    	return asynError;
    }
    return status;
}



/*****************************************************************************/
asynStatus PIE712Axis::SetPivotX(double value)
{
    if (NULL != pasynUser_)
    {
    	asynPrint(pasynUser_, ASYN_TRACE_FLOW,
   		 "PIE712Axis::SetPivotX() ignored");
    }
	return asynSuccess;
}

/*****************************************************************************/
asynStatus PIE712Axis::SetPivotY(double value)
{
    if (NULL != pasynUser_)
    {
    	asynPrint(pasynUser_, ASYN_TRACE_FLOW,
   		 "PIE712Axis::SetPivotY() ignored");
    }
	return asynSuccess;
}

/*****************************************************************************/
asynStatus PIE712Axis::SetPivotZ(double value)
{
    if (NULL != pasynUser_)
    {
    	asynPrint(pasynUser_, ASYN_TRACE_FLOW,
   		 "PIE712Axis::SetPivotZ() ignored");
    }
	return asynSuccess;
}

/********************************************************/

/***************************************************************************/
asynStatus PIE712Axis::getMaxRange(double * result)
{
	*result = MAX_PIEZO_RANGE;
	return asynSuccess;
}

/***************************************************************************/
#ifdef REPLACED
asynStatus PIE712Axis::getPositionEgu(double * result)
{
	double pos = 0.0;

		if(m_simchan)
		{	
			*result=pos;
			return(	asynSuccess);
		}	
		
	getAxisPosition(pos);
  *result=pos;
	m_rrbv_ = *result;
	//printf("PIE712Axis::[%s]getPositionEgu(%.5f)\n",m_axisName, *result);	
	return asynSuccess;
}
#endif

/***************************************************************************/
double PIE712Axis::getPositionEGU(void)
{
	int status = 0;
	double encPos = -9.5;
	double mres = 0.0;
	double encRatio, eres = 0.0;
	double mrec_res = 0.0;
	double mpos = 0.0;
	double rbv = 0.0;
	double mrec_offset = 0.0;

  
  pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &mpos);
	pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
	pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encRatio);
	pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &mrec_res);
	pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_, &encPos);
	pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &mrec_offset);
	eres = mrec_res/encRatio;
	rbv = (encPos * eres) + mrec_offset;
	
	//printf("PIE712Axis::getPosition: axisNo[%d] rbv=%f, motorEncoderPosition_=%f, mres=%f, eres=%f, mrec_res=%f, mpos=%f\n",axisNo_, encPos, mres, eres, mrec_res, mpos );
	//printf("PIE712Axis::getPositionEGU: axisNo[%d] rbv=%f, mrec_offset=%f\n",axisNo_, rbv, mrec_offset );
	
	return rbv;
	
}	

/***************************************************************************/
double PIE712Axis::getPositionCTS(void)
{
	int status = 0;
	double encPos = -9.5;
	double mres = 0.0;
	double encRatio, eres = 0.0;
	double mrec_res = 0.0;
	double mpos = 0.0;
	double cts = 0;
	double mrec_offset = 0.0;

  
  pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &mpos);
	pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
	pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encRatio);
	pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &mrec_res);
	pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_, &encPos);
	pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &mrec_offset);
	eres = mrec_res/encRatio;
	cts = encPos + (mrec_offset / eres);
	
	//printf("PIE712Axis::getPosition: axisNo[%d] rbv=%f, motorEncoderPosition_=%f, mres=%f, eres=%f, mrec_res=%f, mpos=%f\n",axisNo_, encPos, mres, eres, mrec_res, mpos );
	//printf("PIE712Axis::getPositionEGU: axisNo[%d] cts=%ld, mrec_offset=%f\n",axisNo_, cts, mrec_offset );
	
	return cts;
	
}	

/* setPositionEGU(setpoint position) setpoint position in engineering units */
asynStatus PIE712Axis::setPositionEGU(double pos)
{
	double mres = 0.0;
	double steps = 0.0; 
	
	pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
	pC_->setDoubleParam(axisNo_, pC_->motorRecOffset_, 0.0);
	
	steps = pos / mres;
	steps = int(steps);
	//printf("PIE712Axis::setPositionEGU: axisNo[%d] pos_egu=%f, steps=%d\n",axisNo_, pos, steps );
	setPosition(steps);
	//callParamCallbacks();

	return asynSuccess;
}		

/***************************************************************************/
asynStatus PIE712Axis::getRRBV(double * result)
{	
	double pos = 0.;

		if(m_simchan)
		{	
			*result = 0.0  ;
			return(	asynSuccess);
		}	
	
	getAxisPositionCts();
	*result = m_positionCts  ;
	//printf("PIE712Axis::[%s]getRRBV(%.5f)\n",m_axisName, *result);	

	return asynSuccess;
}


/***************************************************************************/
asynStatus PIE712Axis::getServo(int *result)
{
	char cmd[100];
	char buf[255];
	int t_val = 0;

		if(m_simchan)
		{	
			*result = 0;
			return(	asynSuccess);
		}	
		
  sprintf(cmd, "SVO? %d", m_axisNo);
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!pC_->getValue(buf, t_val))
  {
   	return asynError;
  }
  *result = t_val;
  return status;
}

/***************************************************************************/
asynStatus PIE712Axis::getADSensor(double *result)
{
	char cmd[100];
	char buf[255];
	double t_val = 0;
	int chan = 0;

		if(m_simchan)
		{	
			*result = 0.0  ;
			return(	asynSuccess);
		}	
		
	//sprintf(cmd, "TNS? %d", m_axisNo);
	sprintf(cmd, "TAD? %d", m_input_chan);
	
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!pC_->getValue(buf, t_val))
  {
   	return asynError;
  }
  *result = t_val;
  return status;
}

/***************************************************************************/
/*#ifdef ORIGINAL*/
asynStatus PIE712Axis::getCapSensor(double *result)
{
	char cmd[100];
	char buf[255];
	double t_val = 0;
	int chan = 0;

		if(m_simchan)
		{	
			*result = 0.0  ;
			return(	asynSuccess);
		}	
		
	sprintf(cmd, "TSP? %d", m_cap_input_chan);
	
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!pC_->getValue(buf, t_val))
  {
   	return asynError;
  }
  *result = t_val;
  
  return status;
}
/*#endif*/
  
  
#ifdef COMMENTED_OUT_MAY_16_2018
 /***************************************************************************/
asynStatus PIE712Axis::getCapSensor(double *result)
{
	char cmd[100];
	char buf[255];
	double t_val = 0;
	int chan = 0;
	double m = 0.0; // = -0.9943;
	double b = 0.0; //-17.25;
	double y = 0.0;

		if(m_simchan)
		{	
			*result = 0.0  ;
			return(	asynSuccess);
		}	
		
	sprintf(cmd, "TSP? %d", m_cap_input_chan);
	
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!pC_->getValue(buf, t_val))
  {
   	return asynError;
  }
  //*result = t_val;
  pC_->getDoubleParam(axisNo_, pC_->P_CapSensorParmB, &b);
  pC_->getDoubleParam(axisNo_, pC_->P_CapSensorParmM, &m);
  
  y = m*(t_val) + b;
  *result = y;
  
  return status;
} 
  
#endif  
 
 /***************************************************************************/
/***************************************************************************/
asynStatus PIE712Axis::getVolt(double *result)
{
	char cmd[100];
	char buf[255];
	double t_val = 0.0;
	int chan = 0;

		if(m_simchan)
		{	
			*result = 0.0  ;
			return(	asynSuccess);
		}	
		
	if(m_axisNo == 1){
		chan = AXIS1_OUTPUT_CHANNEL;
	} else if(m_axisNo == 2){
		chan = AXIS2_OUTPUT_CHANNEL;
	} else if(m_axisNo == 3){
		chan = AXIS3_OUTPUT_CHANNEL;
	} else if(m_axisNo == 4){
		chan = AXIS4_OUTPUT_CHANNEL;
	} 
	
  sprintf(cmd, "VOL? %d", chan);
  asynStatus status = pC_->m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!pC_->getValue(buf, t_val))
  {
   	return asynError;
  }
  *result = t_val;
  return status;
}
/***************************************************************************/	
asynStatus PIE712Axis::setMarkerStart(double pos){


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

	setDoubleParam(pC_->P_MarkerStart, pos);
	m_markerStart = pos;
	
	callParamCallbacks();
	/*printf("PIE712Axis [%s]::setMarkerStart(%.3f)\n", m_axisName, pos);*/
	return(asynSuccess);
}
/***************************************************************************/	
asynStatus PIE712Axis::setMarkerStop(double pos){


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	setDoubleParam(pC_->P_MarkerStop, pos);
	m_markerStop = pos;
	callParamCallbacks();
	/*printf("PIE712Axis [%s]::setMarkerStop(%.3f)\n", m_axisName, pos);*/
	return(asynSuccess);
}

/***************************************************************************/	
asynStatus PIE712Axis::setScanStart(double pos){


		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	setDoubleParam(pC_->P_ScanStart, pos);
	callParamCallbacks();
	/*printf("PIE712Axis [%s]::setScanStart(%.3f)\n", m_axisName, pos);*/
	return(asynSuccess);
}
/***************************************************************************/	
asynStatus PIE712Axis::setScanStop(double pos)
{

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	setDoubleParam(pC_->P_ScanStop, pos);
	callParamCallbacks();
	/*printf("PIE712Axis [%s]::setScanStop(%.3f)\n", m_axisName, pos);*/
	return(asynSuccess);
}
/***************************************************************************/
asynStatus PIE712Axis::motorOff(void)
{
		double cur_posEgu = 0.0;
		double newPos = 0.0;
		

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	//motorOffRelaxToCenter();
	printf("PIE712Axis::[%d] motorOff()\n", m_axisNo);
	setServo( 0);
	return asynSuccess;
	
}		

/***************************************************************************/
asynStatus PIE712Axis::motorOffRelaxToCenter(void)
{
		double cur_posEgu = 0.0;
		double newPosCts = 0.0;
		
		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	//getPositionEgu(&cur_posEgu);
	//newPos = cur_posEgu - m_capSensor;
	/* we want a relative move in the opposite direction of the m_capSensor IN STEPS! */
	newPosCts = -1.0 * (m_capSensor * double(m_CPUnumerator) / double(m_CPUdenominator));
	move(newPosCts, 1, m_default_velo, m_default_velo, m_maxAcceleration);
	/* assume a time to make the move to be 100ms */
	epicsThreadSleep(0.1);
	
	printf("PIE712Axis::[%d] motorOffRelaxToCenter(%f) cts\n", m_axisNo, newPosCts);
	setServo( 0);
	
	return asynSuccess;
	
}		

/***************************************************************************/
asynStatus PIE712Axis::motorOn(void)
{

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	
	//the driver documentation reccommends this as the way to turn the servo on as opposed to SrvoDriveOn()
	if(m_disabledByMode){
		//printf("PIE712Axis::[%d] motorOn disabled by mode\n", m_axisNo);
		return(asynSuccess);
	} else {
		//printf("PIE712Axis::[%d] motorOn()\n", m_axisNo);
	}
	
	setServo( 1);
	return asynSuccess;
}	

/**************************************************************************
	For a detailed description see page 123 of the E-711/E-712 PZ195E Release 1.3.1 manual
	
	The command for setting a digital output pulse is of the form:
		CTO {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} ...
		
		<TrigOutID> 			
			is the number of the output line
		<CTOPam> 		
			1 = Trigger StepSize setting
			2 = Axis Selection
			3 = Trigger Mode Selection
					Mode Values:
						0 =	Position Distance
						2 = On Target
						3 = MinMax Threshold
						4 = Generator Trigger 
			5 = Min Threshold Setting
			6 = Max Threshold Setting
			7 = Output Line Polarity
					Polarity Values:
						0 = Low
						1 = High
			8 = StartThreshold Setting
			9 = StopThreshold Setting
		
	The setMarker() function uses the MinMax Threshold Trigger mode
*****************************************************************************/
asynStatus PIE712Axis::setMarker(double pos)
{
	char cmd[100];
	char axisSel[50];
	char trigMode[50];
	char minThresh[50];
	char maxThresh[50];

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	

	
	sprintf(axisSel, "%d 2 %d", TRIG_OUT_ID, m_axisNo);
	sprintf(trigMode, "%d 3 3", TRIG_OUT_ID);
	sprintf(minThresh, "%d 5 %.3f", TRIG_OUT_ID, pos);
	sprintf(maxThresh, "%d 6 %.3f", TRIG_OUT_ID, pos + MARKER_WIDTH);
	
	printf("PIE712Axis [%s]::setMarker(%.3f) with MARKER_WIDTH(%.3f)\n", m_axisName, pos, MARKER_WIDTH);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s %s %s %s", axisSel, trigMode, minThresh, maxThresh);
	printf("PIE712Axis::setMarker: %s\n", cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;

#ifdef TEST
	sprintf(cmd, "CTO %d 2 %d", TRIG_OUT_ID, m_axisNo);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

	sprintf(cmd, "CTO %d 3 3", TRIG_OUT_ID);
        status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

        sprintf(cmd, "CTO %d 5 %.3f", TRIG_OUT_ID, pos);
        status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

        sprintf(cmd, "CTO %d 6 %.3f", TRIG_OUT_ID, pos + MARKER_WIDTH);
        status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

#endif

}

/**************************************************************************
	For a detailed description see page 123 of the E-711/E-712 PZ195E Release 1.3.1 manual
	
	The command for setting a digital output pulse is of the form:
		CTO {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} ...
		
		<TrigOutID> 			
			is the number of the output line
		<CTOPam> 		
			1 = Trigger StepSize setting
			2 = Axis Selection
			3 = Trigger Mode Selection
					Mode Values:
						0 =	Position Distance
						2 = On Target
						3 = MinMax Threshold
						4 = Generator Trigger 
			5 = Min Threshold Setting
			6 = Max Threshold Setting
			7 = Output Line Polarity
					Polarity Values:
						0 = Low
						1 = High
			8 = StartThreshold Setting
			9 = StopThreshold Setting
		
	The setMarker() function uses the MinMax Threshold Trigger mode
*****************************************************************************/
asynStatus PIE712Axis::setMarkerWindow(void)
{
	char cmd[100];
	char axisSel[50];
	char trigMode[50];
	char minThresh[50];
	char maxThresh[50];
	

		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStart, &m_markerStart);
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStop, &m_markerStop);
	


	//sprintf(axisSel, "%d 2 %d", m_axisNo, m_axisNo);
	sprintf(axisSel, "%d 2 %d", TRIG_OUT_ID, m_axisNo);
	sprintf(trigMode, "%d 3 3", TRIG_OUT_ID);
	sprintf(minThresh, "%d 5 %.3f", TRIG_OUT_ID, m_markerStart);
	//sprintf(maxThresh, "%d 6 %.3f", m_axisNo, m_markerStop);
	// MAX here is defined here assumes moving left(negative) to the right(positive)
	
	//JULY 6 2022 sprintf(maxThresh, "%d 6 %.3f", TRIG_OUT_ID, m_markerStart + 1.0);
	sprintf(maxThresh, "%d 6 %.3f", TRIG_OUT_ID, m_markerStart + 100.0);
	
	
	//printf("\n\nPIE712Axis [%s]::setMarkerWindow(%.5f, %.5f) with MARKER_WIDTH(%.3f)\n\n", m_axisName, m_markerStart, m_markerStop, MARKER_WINDOW);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s %s %s %s", axisSel, trigMode, minThresh, maxThresh);
	
	printf("PIE712Axis::setMarkerWindow: %s\n", cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;

#ifdef TEST
	sprintf(cmd, "CTO %d 2 %d", m_axisNo, m_axisNo);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

	sprintf(cmd, "CTO %d 2 %d", TRIG_OUT_ID, m_axisNo);
	status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

  sprintf(cmd, "CTO %d 3 3", TRIG_OUT_ID);
	status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

  sprintf(cmd, "CTO %d 5 %.3f", TRIG_OUT_ID, m_markerStart);
	status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");

  sprintf(cmd, "CTO %d 6 %.3f", TRIG_OUT_ID, m_markerStart + 100.0);
	status = pC_->m_pInterface->sendOnly(cmd);
	printf(cmd);
	printf("\n");
	
        return status;

#endif


}



/**************************************************************************
	For a detailed description see page 126 of the E-711/E-712 PZ195E Release 1.3.1 manual
	
	The command for setting a digital output pulse for POSITION DISTANCE is of the form:
		CTO {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} {<TrigOutID> <CTOPam> <Value>} ...
		#define CTO_PAM_TRIGGER_STEP 		1
		#define CTO_PAM_AXIS_SEL 				2
		#define CTO_PAM_TRIGGER_MODE 		3
		#define CTO_PAM_MIN_THRESHOLD 	5
		#define CTO_PAM_MAX_THRESHOLD 	6
		#define CTO_PAM_POLARITY 				7
		#define CTO_PAM_START_THRESHOLD 8
		#define CTO_PAM_STOP_THRESHOLD 	9

		<TrigOutID> 			
			is the number of the output line
		<CTOPam> 		
			1 = Trigger StepSize setting
			2 = Axis Selection
			3 = Trigger Mode Selection
					Mode Values:
					#define CTO_TRIGMODE_POSDIST 	0
					#define CTO_TRIGMODE_ONTRGT 	2
					#define CTO_TRIGMODE_MINMAX 	3
					#define CTO_TRIGMODE_GENTRIG 	4

						0 =	Position Distance
						2 = On Target
						3 = MinMax Threshold
						4 = Generator Trigger 
			5 = Min Threshold Setting
			6 = Max Threshold Setting
			7 = Output Line Polarity
					Polarity Values:
						0 = Low
						1 = High
			8 = StartThreshold Setting
			9 = StopThreshold Setting
***************************************************************************/
/*
The "Position Distance" trigger mode is designed for scanning applications. 
A trigger pulse is written whenever the axis has covered the distance set with 
CTO (<TriggerStep>). The time interval of the trigger pulses depends on the position 
distance set with <TriggerStep> and on the current speed of the axis.
The unit of <TriggerStep> is µm or µrad.

The following parameters must be set for the digital output line which is to be used for trigger output (<TrigOutID>):

¦ Axis (<CTOPam> = 2)
¦ TriggerMode (<CTOPam> = 3)
¦ TriggerStep (<CTOPam> = 1)

	CTO <TrigOutID> 2 Axis <TrigOutID> 3 0 <TrigOutID> 1 Stepsize

¦ Axis (<CTOPam> = 2)
¦ TriggerMode (<CTOPam> = 3)
¦ TriggerStep (<CTOPam> = 1)
¦ StartThreshold (<CTOPam> = 8
¦ StopThreshold (<CTOPam> = 9
 
 CTO <TrigOutID> 2 Axis <TrigOutID> 3 0 <TrigOutID> 1 Stepsize <TrigOutID> 8 Startpos. <TrigOutID> 9 Stoppos.

*/
asynStatus PIE712Axis::setPositionDistanceMarker(void)
{
	char cmd[100];
	char axisSel[50];
	char trigMode[50];
	char trigStep[50];
	char startThresh[50];
	char stopThresh[50];
	double stepSize = 0.0;
	int trig_mode = CTO_TRIGMODE_POSDIST;	/* 0 =	Position Distance */

	if(m_simchan)
	{	
		return(	asynSuccess);
	}	
	
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStart, &m_markerStart);
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStop, &m_markerStop);
	pC_->getDoubleParam(axisNo_, pC_->P_ScanStart, &m_scanStart);
	pC_->getDoubleParam(axisNo_, pC_->P_ScanStop, &m_scanStop);

	stepSize = m_markerStart - m_scanStart;
	
	
	sprintf(axisSel, "%d %d %d", TRIG_OUT_ID, CTO_PAM_AXIS_SEL, m_axisNo);
	sprintf(trigMode, "%d %d %d", TRIG_OUT_ID, CTO_PAM_TRIGGER_MODE, trig_mode);
	sprintf(trigStep, "%d %d %.3f", TRIG_OUT_ID, CTO_PAM_TRIGGER_STEP, m_markerStart - m_scanStart);
	sprintf(startThresh, "%d %d %.3f", TRIG_OUT_ID, CTO_PAM_START_THRESHOLD, m_markerStart);
	sprintf(stopThresh, "%d %d %.3f", TRIG_OUT_ID, CTO_PAM_STOP_THRESHOLD, m_markerStart + MARKER_WIDTH);
	
	//printf("PIE712Axis [%s]::setPositionDistanceMarker(trigStep=%.3f) with (startThresh=%.3f stopThresh=%.3f)\n", m_axisName, stepSize, m_markerStart, m_markerStart + MARKER_WIDTH);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s %s %s %s", axisSel, trigMode, startThresh, stopThresh);
	printf("PIE712Axis::setPositionDistanceMarker: %s\n",cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;
}

/*
With the "On Target" trigger mode, the on-target status of the selected axis is 
written to the selected trigger line. It is the same on-target status flag 
which can also be read by the ONT? command. The on-target status is influenced 
by two parameters: settling window (On Target Tolerance, ID 0x07000900) and 
settling time (Settling Time, ID 0x07000901). The on-target status is true 
when the current position is inside the settling window and stays there for 
at least the settling time. The settling window is centered around the target 
position.

The following parameters must be set for the digital output line which is to be used for trigger output (<TrigOutID>):
¦ Axis (<CTOPam> = 2)
¦ TriggerMode (<CTOPam> = 3)

CTO <TrigOutID> 2 Axis <TrigOutID> 3 2
CTO <TrigOutID> 2 <axis> <TrigOutID> 3 2
*/
/***************************************************************************/
asynStatus PIE712Axis::setOnTargetMarker(void)
{
	char cmd[100];
	char axisSel[50];
	char trigMode[50];
	int trig_mode = CTO_TRIGMODE_ONTRGT;	/* 2 =	On Target */
	
	if(m_simchan)
	{	
		return(	asynSuccess);
	}	
	sprintf(axisSel, "%d %d %d", TRIG_OUT_ID, CTO_PAM_AXIS_SEL, m_axisNo);
	sprintf(trigMode, "%d %d %d", TRIG_OUT_ID, CTO_PAM_TRIGGER_MODE, trig_mode);
	
	//printf("PIE712Axis [%s]::setOnTargetMarker\n", m_axisName);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s %s", axisSel, trigMode);
	printf("PIE712Axis::setOnTargetMarker: %s\n",cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;
}




/*
MinThreshold and MaxThreshold (<CTOPam> IDs 5 and 6). 
When the axis position is inside the specified band then 
the trigger output line is set high, otherwise it is set low.
The following parameters must be set for the digital output line which is to be used for trigger output (<TrigOutID>):

¦ Axis (<CTOPam> = 2)
¦ TriggerMode (<CTOPam> = 3)
¦ MinThreshold (<CTOPam> = 5)
¦ MaxThreshold (<CTOPam> = 6)

*/


/***************************************************************************/
asynStatus PIE712Axis::setMinMaxMarker(void)
{
	char cmd[100];
	char axisSel[50];
	char trigMode[50];
	char minThresh[50];
	char maxThresh[50];
	int trig_mode = CTO_TRIGMODE_MINMAX;	/* 3 =	Position Distance */
	
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStart, &m_markerStart);
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStop, &m_markerStop);

	if(m_simchan)
	{	
		return(	asynSuccess);
	}	

	sprintf(axisSel, "%d %d %d", TRIG_OUT_ID, CTO_PAM_AXIS_SEL, m_axisNo);
	sprintf(trigMode, "%d %d %d", TRIG_OUT_ID, CTO_PAM_TRIGGER_MODE, trig_mode);
	sprintf(minThresh, "%d %d %.3f", TRIG_OUT_ID, CTO_PAM_MIN_THRESHOLD, m_markerStart);
	sprintf(maxThresh, "%d %d %.3f", TRIG_OUT_ID, CTO_PAM_MAX_THRESHOLD, m_markerStop);
	
	//printf("PIE712Axis [%s]::setMinMaxMarker(%.3f ->%.3f)\n", m_axisName, m_markerStart, m_markerStop);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s %s %s %s", axisSel, trigMode, minThresh, maxThresh);
	printf("PIE712Axis::setMinMaxMarker: %s\n",cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;
}

/*
With the "Generator Trigger" mode, the trigger output will be synchronized with the wave 
generator output, and CTO must be used in combination with TWS.
The following parameter must be set for the digital output line which is to be used for 
trigger output (<TrigOutID>):
¦ TriggerMode (<CTOPam> = 3)

General notation of the CTO command for this mode:
	CTO <TrigOutID> 3 4
*/
/***************************************************************************/
asynStatus PIE712Axis::setGeneratorTrigMarker(void)
{
	char cmd[100];
	char trigMode[50];
	int trig_mode = CTO_TRIGMODE_GENTRIG;	/* 4 =	Generator Trigger */
	
	if(m_simchan)
	{	
		return(	asynSuccess);
	}	

	sprintf(trigMode, "%d %d %d", TRIG_OUT_ID, CTO_PAM_TRIGGER_MODE, trig_mode);
	//printf("PIE712Axis [%s]::setGeneratorTrigMarker(%.3f ->%.3f)\n", m_axisName, m_markerStart, m_markerStop);
	
	/* for now the output line is the same number as the axis number (axis 1 uses output line 1 etc) */
	sprintf(cmd, "CTO %s", trigMode);
	printf("PIE712Axis::setGeneratorTrigMarker: %s\n",cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;
}




/*
It is possible to select the signal polarity (active high = 1, default / active low = 0) for the digital output line which is to be used for trigger output.
The following parameter must be set for the digital output line (<TrigOutID>):
¦ Polarity (<CTOPam> = 7)
 General notation of the CTO command for polarity selection:
 Command mnemonic
 Trigger mode selection
 CTO <TrigOutID> 7 pol.code
*/
/***************************************************************************/
asynStatus PIE712Axis::setMarkerPolarity(int pol)
{
	char cmd[100];
	char trigMode[50];
	int trig_mode = CTO_PAM_POLARITY;	/* 7 =	Polarity */
	
	if(m_simchan)
	{	
		return(	asynSuccess);
	}	

	sprintf(trigMode, "%d %d %d", TRIG_OUT_ID, trig_mode, pol);
	//printf("PIE712Axis [%s]::?Polarity(%d)\n", m_axisName, pol);
	
	sprintf(cmd, "CTO %s", trigMode);
	printf("%s\n",cmd);
	asynStatus status = pC_->m_pInterface->sendOnly(cmd);
	return status;
}

/***************************************************************************/
asynStatus PIE712Axis::setMode(int mode)
{
	int servoPwr = 0;
	
	
	getServo(&servoPwr);
	
		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
	
	
	if(mode == MODE_COARSE){
		/* turn Agilent off and set mode_disabled flag so that any attempt to turn it on 
		while in coarse mode will be denied */
		motorOff();
		m_disabledByMode = true;
	} else {
		/* any other mode will enable the Agilent */
		m_disabledByMode = false;
		
		/* if it was on urn it back on */
		if(servoPwr){
			motorOn();
		}	
	}
	//setIntegerParam(pC_->P_Mode, mode);
	pC_->setIntegerParam(axisNo_, pC_->P_Mode, mode);
	callParamCallbacks();
		
  return asynSuccess;
}


/**********************************************************************/
asynStatus PIE712Axis::configure_for_mode(double positionEgu, int relative, double minVelocityEgu, double maxVelocityEgu, double accelerationEgu)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "configure_for_mode";
	int p_mode;
	double p_startscan, p_stopscan, p_markerstart, p_markerstop, p_fbkpos, p_fbkvelo;
	double marker_rb;
	static double previous_gotopos_egu=0.0;
	
		if(m_simchan)
		{	
			return(	asynSuccess);
		}	
		
	pC_->getIntegerParam(axisNo_, pC_->P_Mode, &p_mode);
	pC_->getDoubleParam(axisNo_, pC_->P_ScanStart, &p_startscan);
	pC_->getDoubleParam(axisNo_, pC_->P_ScanStop, &p_stopscan);
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStart, &p_markerstart);
	pC_->getDoubleParam(axisNo_, pC_->P_MarkerStop, &p_markerstop);
	pC_->getDoubleParam(axisNo_, pC_->motorVelocity_, &p_fbkvelo);

	//getPositionEgu(&p_fbkpos);
	p_fbkpos = getPositionEGU();
	//printf("configure_for_mode [%s]: p_fbkpos = %f, position = %f\n", m_axisName, p_fbkpos, positionEgu);

	if(p_mode == MODE_NORMAL){
		/* NORMAL */
		m_enableStatus = true;
		/*printf("set_for_mode: MODE_NORMAL: enabled status\n");*/
		status = sendAccelAndVelocity(accelerationEgu, maxVelocityEgu);
		
	} else if(p_mode == MODE_LINE_UNIDIR){
		/* MODE_LINE_UNIDIR */
		if(previous_gotopos_egu == positionEgu)
		{
			// already done this, for seom
			return asynSuccess;
		}
				
		//printf("positionEgu=%.5f, previous_gotopos_egu=%.5f, p_stopscan=%.5f, p_markerstop=%.5f\n",positionEgu, previous_gotopos_egu, p_stopscan, p_markerstop); 		
		m_enableStatus = true;
		if(positionEgu >= p_markerstop)
		{
				//marker_rb = setMarker(p_markerstart);	
				setMarkerWindow();	
				//printf("set_for_mode: [position=%.5f] MODE_LINE_UNIDIR: p_fbkpos < p_startscan : SET_MARKER(%.5f) rb=%.5f\n", position, p_markerstart, marker_rb);
				//printf("set_for_mode: MODE_LINE_UNIDIR: position=%.5f >= p_markerstop=%.5f : SET_MARKER(%.5f) rb=%.5f\n", positionEgu, p_markerstop, p_markerstart);
				status = sendAccelAndVelocity(accelerationEgu, maxVelocityEgu);
			
		} else {
			marker_rb = setMarker(100000.);	
			//printf("set_for_mode: [position=%.5f] MODE_LINE_UNIDIR: p_fbkpos ELSE p_startscan : SET_MARKER(%.5f) rb=%.5f\n", position, 100000., marker_rb);
			//printf("set_for_mode: MODE_LINE_UNIDIR: position=%.5f < p_markerstop=%.5f : SET_MARKER(%.5f) rb=%.5f\n", positionEgu, p_markerstop, 100000., marker_rb);
			status = sendAccelAndVelocity(accelerationEgu, m_default_velo);
			
		}	
		previous_gotopos_egu = positionEgu;	
		
	} else if(p_mode == MODE_LINE_BIDIR){
		/* MODE_LINE_BIDIR */
		m_enableStatus = true;
		if(p_fbkpos > p_markerstop)
		{
			marker_rb = setMarker(p_markerstop);	
			/*printf("set_for_mode: MODE_LINE_BIDIR: p_fbkpos > p_startscan : SET_MARKER(%.5f) rb=%.5f\n", p_markerstop, marker_rb);*/
			status = sendAccelAndVelocity(accelerationEgu, maxVelocityEgu);
			
		} else if(p_fbkpos < p_markerstart){
			marker_rb = setMarker(p_markerstart);	
			/*printf("set_for_mode: MODE_LINE_BIDIR: p_fbkpos ELSE p_startscan : SET_MARKER(%.5f) rb=%.5f\n", p_markerstart, marker_rb);*/
			status = sendAccelAndVelocity(accelerationEgu, maxVelocityEgu);
		}
		
	} else if(p_mode == MODE_POINT){
		/* MODE_POINT */
		m_enableStatus = false;
		marker_rb = setMarker(100000.0);	
		/*printf("set_for_mode: MODE_POINT: disabled status : SET_MARKER   %.5f\n", marker_rb); */
		status = sendAccelAndVelocity(accelerationEgu, m_default_velo);
		
	} else if(p_mode == MODE_COARSE){
		/* MODE_COARSE */
		if(positionEgu >= p_markerstop)
		{
				marker_rb = setMarkerWindow();	
				//marker_rb = setMarker(p_markerstart);	
				//printf("set_for_mode: MODE_COARSE: position=%.5f >= p_markerstop=%.5f : SET_MARKER(%.5f)\n", positionEgu, p_markerstop, p_markerstart);
			
		} else {
			marker_rb = setMarker(100000.);	
			//printf("set_for_mode: MODE_COARSE: position=%.5f < p_markerstop=%.5f : SET_MARKER(%.5f) rb=%.5f\n", positionEgu, p_markerstop, 100000., marker_rb);
		}	
		previous_gotopos_egu = positionEgu;
		
	} else {
		/*printf("set_for_mode: unknown p_mode[%d]\n", p_mode);*/
	}
	
  //asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s:%s: set_for_mode %s, axis %d move to %f, min vel=%f, maxVel=%f, accel=%f\n", driverName, functionName, pC_->portName, axisNo_, new_pos, minVelocity, maxVelocity, acceleration );
  return asynSuccess;
}

/***************************************************************************/
asynStatus PIE712Axis::get_status(unsigned int *sts)
{
	bool moving = false;	
	moving = isMoving();

		if(m_simchan)
		{	
			*sts = 0;
			return(	asynSuccess);
		}	

	
	/*printf("PIE712Axis[%d]::get_status: [%d]\n", axisNo_, moving);  */
	if(moving){
		*sts = 1;
	} else {
		*sts = 0;
	}
	return asynSuccess;
}


/***************************************************************************/
bool  PIE712Axis::isMoving(void)
{

		if(m_simchan)
		{	
			return(false);
		}	
		
	if(moving_){
		return(true);
	} else {
		return(false);
	}	
}	


/********************************************
Function name	: IsWithInRange
Description	    : returns true if can be reached by piezo, all units need to be either converted to or exist in microns
Return type		: bool 
Argument        : double target
********************************************/
bool PIE712Axis::isWithInRange(double target_microns)
{
	double dCurrPos_microns, deltaPos;
	double rng_left = 0.0;
	
	if(m_simchan)
	{	
		return(false);
		
	}	
	//getPositionEgu(&dCurrPos_microns);
	dCurrPos_microns = getPositionEGU();
	deltaPos = fabs(target_microns - dCurrPos_microns);
	
	if(MAX_PIEZO_EGU > m_capSensor){
		rng_left = MAX_PIEZO_EGU - fabs(m_capSensor);
		
		if(deltaPos > rng_left){
			return(false);
		}
		return(true);
		
	} else {
		return(false);
	}
	
	
}


/***************************************************************************/
asynStatus PIE712Axis::config(char *axisName, int hiHardLimit, int lowHardLimit, int home, int start, int simulate)
{
	printf("simulate = %d\n", simulate);

  //Init(m_portName);
	
  hiHardLimit_ = hiHardLimit;
  lowHardLimit_ = lowHardLimit;
  home_ = home;
  //enc_offset_ = start;
  //m_simchan = simulate>0?true:false;
  
  if(simulate>0){
  	m_simchan = true;
  } else { 
  	m_simchan = false;
  }
  
  
  sprintf(m_axisName, "%s", axisName);   
	initPositioner();
	
	return(asynSuccess);

}

/***************************************************************************/
asynStatus PIE712Axis::initPositioner(void)
{
		
	printf("PIE712Axis::initPositioner()\n");
	return asynSuccess;
}


/********************************************************
*******************************************************
*********************************************************
**********************************************************/


PIE712Controller::PIE712Controller(const char *portName, const char* asynPort, const char* dr_asynPort, int numAxes, double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PI_E712_PARAMS,
            asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
            asynInt32Mask | asynFloat64Mask | asynOctetMask ,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE,
            1, // autoconnect
            0, 0)  // Default priority and stack size
{	
	
		int axis;
    PIE712Axis *pAxis;
    PIE712ControllerNode *pNode;
    int num_points_expected;
    int num_dest_bytes_expected;
    int bytes_per_point;
    
    createParam(PI_SUP_POSITION_String,		asynParamFloat64,	&PI_SUP_POSITION);
    createParam(PI_SUP_TARGET_String,			asynParamFloat64,	&PI_SUP_TARGET);
    createParam(PI_SUP_SERVO_String,			asynParamInt32,		&PI_SUP_SERVO);
    createParam(PI_SUP_LAST_ERR_String,		asynParamInt32,		&PI_SUP_LAST_ERR);
    createParam(PI_SUP_PIVOT_X_String,		asynParamFloat64,	&PI_SUP_PIVOT_X);
    createParam(PI_SUP_PIVOT_Y_String,		asynParamFloat64,	&PI_SUP_PIVOT_Y);
    createParam(PI_SUP_PIVOT_Z_String,		asynParamFloat64,	&PI_SUP_PIVOT_Z);
    createParam(PI_SUP_RBPIVOT_X_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_X);
    createParam(PI_SUP_RBPIVOT_Y_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_Y);
    createParam(PI_SUP_RBPIVOT_Z_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_Z);
		
		/* added by Russ for scanning capability */
		createParam(P_SetMarkerString,  			asynParamFloat64,  &P_SetMarker);
	  createParam(P_FWString, 							asynParamOctet,    &P_FW);
	  createParam(P_PowerString,   					asynParamInt32,    &P_Power);
	  createParam(P_SysStatusString, 				asynParamInt32,    &P_SysStatus);
	  createParam(P_OutputVoltString,  			asynParamFloat64,  &P_OutputVolt);
	  createParam(P_ModeString,   					asynParamInt32,    &P_Mode);
		createParam(P_MarkerStartString,  		asynParamFloat64,  &P_MarkerStart);
		createParam(P_MarkerStopString,  			asynParamFloat64,  &P_MarkerStop);
		createParam(P_ScanStartString,  			asynParamFloat64,  &P_ScanStart);
		createParam(P_ScanStopString,  				asynParamFloat64,  &P_ScanStop);
		createParam(P_SetVeloString,  				asynParamFloat64,  &P_SetVelo);
		createParam(P_SetStartupPosString,  	asynParamFloat64,  &P_SetStartupPos);
		createParam(P_ReInitString,   				asynParamInt32,    &P_ReInit);
		
		createParam(P_AutoZeroString,   			asynParamInt32,    &P_AutoZero);
		createParam(P_SelectEncoderSrcString, asynParamInt32,    &P_SelectEncoderSrc);
		createParam(P_SelectOutputDirString, asynParamInt32,    	&P_SelectOutputDir);
		
		createParam(P_AdcSensorString,  			asynParamFloat64,  &P_AdcSensor);
		createParam(P_CapSensorString,  			asynParamFloat64,  &P_CapSensor);
		
		createParam(P_ForceDoneString,  			asynParamInt32,  &P_ForceDone);
		createParam(P_ForceDoneWaitTimeString, asynParamFloat64,  &P_ForceDoneWaitTime);
		
		
		createParam(P_ZeroADSensorString,  		asynParamInt32,  &P_ZeroADSensor);
		createParam(P_TriggerOutputSelString,  		asynParamInt32,  &P_TriggerOutputSel);
		
		createParam(P_DigitalFilterTypeString,		 		asynParamInt32,  &P_DigitalFilterType);   
		createParam(P_DigitalFilterBWidthString,   		asynParamFloat64,  &P_DigitalFilterBWidth);  
		createParam(P_DigitalFilterORderString,	  		asynParamInt32,  &P_DigitalFilterORder); 	 
		createParam(P_DigitalFilterParm1String, 	    asynParamFloat64,  &P_DigitalFilterParm1); 	 
		createParam(P_DigitalFilterParm2String, 	    asynParamFloat64,  &P_DigitalFilterParm2); 	 
		createParam(P_DigitalFilterParm3String, 	    asynParamFloat64,  &P_DigitalFilterParm3); 	 
		createParam(P_DigitalFilterParm4String, 	    asynParamFloat64,  &P_DigitalFilterParm4); 	 
		createParam(P_DigitalFilterParm5String, 	    asynParamFloat64,  &P_DigitalFilterParm5); 	 

		createParam(P_VoltRangeWarningString,   				asynParamInt32,    &P_VoltRangeWarning);
		createParam(P_ResetVoltRangeWarningString,   				asynParamInt32,    &P_ResetVoltRangeWarning);
		createParam(P_ServoOffAndCenterString,   				asynParamInt32,    &P_ServoOffAndCenter);
		
		/* thr following params are used in a linear capsensor value to um conversion */
		createParam(P_CapSensorParmBString, 	    asynParamFloat64,  &P_CapSensorParmB); 	 
		createParam(P_CapSensorParmMString, 	    asynParamFloat64,  &P_CapSensorParmM); 	 
		
		createParam(P_PTermString, 	    asynParamFloat64,  &P_PTerm); 	 
		createParam(P_ITermString, 	    asynParamFloat64,  &P_ITerm); 	 
		createParam(P_DTermString, 	    asynParamFloat64,  &P_DTerm); 	 

		
		createParam(P_CommStatusString, 	    		asynParamInt32,  &P_CommStatus); 	
		createParam(P_WaveGen1_StatusString, 	    asynParamInt32,  &P_WaveGen1_Status); 	
		createParam(P_WaveGen2_StatusString, 	    asynParamInt32,  &P_WaveGen2_Status); 	
		createParam(P_WaveGen3_StatusString, 	    asynParamInt32,  &P_WaveGen3_Status); 	
		createParam(P_WaveGen4_StatusString, 	    asynParamInt32,  &P_WaveGen4_Status); 	
		
		createParam(P_SendCommandsString, 	    asynParamOctet,  &P_SendCommands); 
		createParam(P_GetIDNString, 	    asynParamOctet,  &P_GetIDN	 ); 
		
		createParam(P_WaveTbl1WfString,           asynParamFloat64Array,  &P_WaveTbl1Wf);
		createParam(P_WaveTbl2WfString,           asynParamFloat64Array,  &P_WaveTbl2Wf);
		createParam(P_WaveTbl3WfString,           asynParamFloat64Array,  &P_WaveTbl3Wf);
		createParam(P_WaveTbl4WfString,           asynParamFloat64Array,  &P_WaveTbl4Wf);
		
		createParam(P_DDLTbl1WfString,           asynParamFloat64Array,  &P_DDLTbl1Wf);
		createParam(P_DDLTbl2WfString,           asynParamFloat64Array,  &P_DDLTbl2Wf);
		createParam(P_DDLTbl3WfString,           asynParamFloat64Array,  &P_DDLTbl3Wf);
		createParam(P_DDLTbl4WfString,           asynParamFloat64Array,  &P_DDLTbl4Wf);
		
		createParam(P_TrigTblWfString,           asynParamFloat64Array,  &P_TrigTblWf);
		
		createParam(P_GetWaveTbl1String,  		asynParamInt32,  &P_GetWaveTbl1);
		createParam(P_GetWaveTbl2String,  		asynParamInt32,  &P_GetWaveTbl2);
		createParam(P_GetWaveTbl3String,  		asynParamInt32,  &P_GetWaveTbl3);
		createParam(P_GetWaveTbl4String,  		asynParamInt32,  &P_GetWaveTbl4);
		
		createParam(P_GetDDLTbl1String,  		asynParamInt32,  &P_GetDDLTbl1);
		createParam(P_GetDDLTbl2String,  		asynParamInt32,  &P_GetDDLTbl2);
		createParam(P_GetDDLTbl3String,  		asynParamInt32,  &P_GetDDLTbl3);
		createParam(P_GetDDLTbl4String,  		asynParamInt32,  &P_GetDDLTbl4);
		
		createParam(P_GetTrigTblString,  		asynParamInt32,  &P_GetTrigTbl);
		
		createParam(P_TrigOutputString,  		asynParamInt32,  &P_TrigOutput);
		
		createParam(P_TrigModeWg1String,  			asynParamInt32,  &P_TrigModeWg1);
		createParam(P_TrigModeWg2String,  			asynParamInt32,  &P_TrigModeWg2);
		createParam(P_TrigModeWg3String,  			asynParamInt32,  &P_TrigModeWg3);
		createParam(P_TrigModeWg4String,  			asynParamInt32,  &P_TrigModeWg4);
		
		createParam(P_XAxisIdString,  			asynParamInt32,  &P_XAxisId);
		createParam(P_YAxisIdString,  			asynParamInt32,  &P_YAxisId);
		
		createParam(P_ClrDDLTbl1String,  		asynParamInt32,  &P_ClrDDLTbl1);
		createParam(P_ClrDDLTbl2String,  		asynParamInt32,  &P_ClrDDLTbl2);
		createParam(P_ClrDDLTbl3String,  		asynParamInt32,  &P_ClrDDLTbl3);
		createParam(P_ClrDDLTbl4String,  		asynParamInt32,  &P_ClrDDLTbl4);
		
		createParam(P_ClrWavTbl1String,  		asynParamInt32,  &P_ClrWavTbl1);
		createParam(P_ClrWavTbl2String,  		asynParamInt32,  &P_ClrWavTbl2);
		createParam(P_ClrWavTbl3String,  		asynParamInt32,  &P_ClrWavTbl3);
		createParam(P_ClrWavTbl4String,  		asynParamInt32,  &P_ClrWavTbl4);
		
		createParam(P_ClrTrigTblString,  		asynParamInt32,  &P_ClrTrigTbl);
		
		createParam(P_NumCyclesString,  		asynParamInt32,  &P_NumCycles);
		createParam(P_WaveTblRateString,  	asynParamInt32,  &P_WaveTblRate);
		
		createParam(P_StopWavegenString,  	asynParamInt32,  &P_StopWavegen);
		createParam(P_StartWavegenString,  	asynParamInt32,  &P_StartWavegen);
		createParam(P_ExecWavegenString,  	asynParamInt32,  &P_ExecWavegen);
		
		createParam(P_CalcDDLParmsString,  	asynParamInt32,  &P_CalcDDLParms);
		
		createParam(P_WaveTbl1LenString,  		asynParamInt32,  &P_WaveTbl1Len);
		createParam(P_WaveTbl2LenString,  		asynParamInt32,  &P_WaveTbl2Len);
		createParam(P_WaveTbl3LenString,  		asynParamInt32,  &P_WaveTbl3Len);
		createParam(P_WaveTbl4LenString,  		asynParamInt32,  &P_WaveTbl4Len);
		
		createParam(P_WaveGen1UseTblNumString,	asynParamInt32,  &P_WaveGen1UseTblNum);
		createParam(P_WaveGen2UseTblNumString,	asynParamInt32,  &P_WaveGen2UseTblNum);
		createParam(P_WaveGen3UseTblNumString,	asynParamInt32,  &P_WaveGen3UseTblNum);
		createParam(P_WaveGen4UseTblNumString,	asynParamInt32,  &P_WaveGen4UseTblNum);
		
		createParam(P_DDLTbl1LenString,  		asynParamInt32,  &P_DDLTbl1Len);
		createParam(P_DDLTbl2LenString,  		asynParamInt32,  &P_DDLTbl2Len);
		createParam(P_DDLTbl3LenString,  		asynParamInt32,  &P_DDLTbl3Len);
		createParam(P_DDLTbl4LenString,  		asynParamInt32,  &P_DDLTbl4Len);
		
		createParam(P_WaveTbl1UseDDLString, asynParamInt32,  &P_WaveTbl1UseDDL);
		createParam(P_WaveTbl2UseDDLString, asynParamInt32,  &P_WaveTbl2UseDDL);
		createParam(P_WaveTbl3UseDDLString, asynParamInt32,  &P_WaveTbl3UseDDL);
		createParam(P_WaveTbl4UseDDLString, asynParamInt32,  &P_WaveTbl4UseDDL);
		
		createParam(P_WaveTbl1UseReinitDDLString, asynParamInt32,  &P_WaveTbl1UseReinitDDL);
		createParam(P_WaveTbl2UseReinitDDLString, asynParamInt32,  &P_WaveTbl2UseReinitDDL);
		createParam(P_WaveTbl3UseReinitDDLString, asynParamInt32,  &P_WaveTbl3UseReinitDDL);
		createParam(P_WaveTbl4UseReinitDDLString, asynParamInt32,  &P_WaveTbl4UseReinitDDL);
		
		createParam(P_WaveTbl1StartAtEndString, asynParamInt32,  &P_WaveTbl1StartAtEnd);
		createParam(P_WaveTbl2StartAtEndString, asynParamInt32,  &P_WaveTbl2StartAtEnd);
		createParam(P_WaveTbl3StartAtEndString, asynParamInt32,  &P_WaveTbl3StartAtEnd);
		createParam(P_WaveTbl4StartAtEndString, asynParamInt32,  &P_WaveTbl4StartAtEnd);
		
		createParam(P_WaveTbl1StartModeString, asynParamInt32,  &P_WaveTbl1StartMode);                                  
		createParam(P_WaveTbl2StartModeString, asynParamInt32,  &P_WaveTbl2StartMode);
		createParam(P_WaveTbl3StartModeString, asynParamInt32,  &P_WaveTbl3StartMode);
		createParam(P_WaveTbl4StartModeString, asynParamInt32,  &P_WaveTbl4StartMode);
		
		createParam(P_XStartPosString, asynParamFloat64,  &P_XStartPos);
		createParam(P_YStartPosString, asynParamFloat64,  &P_YStartPos);
		
		createParam(P_ScanModeString, asynParamInt32,  &P_ScanMode);
		
		createParam(P_LastErrString, asynParamOctet,  &P_LastErr);
		createParam(P_TotalPointsLeftString, asynParamInt32,  &P_TotalPointsLeft);
		
		createParam(P_SlewRateString, 	    asynParamFloat64,  &P_SlewRate); 	 
		createParam(P_NotchFreq1String, 	    asynParamFloat64,  &P_NotchFreq1); 	 
		createParam(P_NotchFreq2String, 	    asynParamFloat64,  &P_NotchFreq2); 	 
		createParam(P_NotchRejection1String, 	    asynParamFloat64,  &P_NotchRejection1); 	 
		createParam(P_NotchRejection2String, 	    asynParamFloat64,  &P_NotchRejection2); 	 
		createParam(P_NotchBW1String, 	    asynParamFloat64,  &P_NotchBW1); 	 
		createParam(P_NotchBW2String, 	    asynParamFloat64,  &P_NotchBW2); 	 
		
		createParam(P_DDLTblNORDString, asynParamInt32,  &P_DDLTblNORD);
		
		createParam(P_ParamFileNameString, asynParamOctet,  &P_ParamFileName);
		createParam(P_LoadParamFileString, asynParamInt32,  &P_LoadParamFile);
		
		
		/* data recorder params */
		createParam(P_DatRec_AbortString, 		asynParamInt32, 	&P_DatRec_Abort);  
		createParam(P_DatRec_T1EnabledString, asynParamInt32, 	&P_DatRec_T1Enabled);  
		createParam(P_DatRec_T2EnabledString, asynParamInt32, 	&P_DatRec_T2Enabled);  
		createParam(P_DatRec_T3EnabledString, asynParamInt32, 	&P_DatRec_T3Enabled);  
		createParam(P_DatRec_T4EnabledString, asynParamInt32, 	&P_DatRec_T4Enabled);  
		createParam(P_DatRec_T5EnabledString, asynParamInt32, 	&P_DatRec_T5Enabled);  
		createParam(P_DatRec_T6EnabledString, asynParamInt32, 	&P_DatRec_T6Enabled);  
		createParam(P_DatRec_T7EnabledString, asynParamInt32, 	&P_DatRec_T7Enabled);  
		createParam(P_DatRec_T8EnabledString, asynParamInt32, 	&P_DatRec_T8Enabled);  
		createParam(P_DatRec_T9EnabledString, asynParamInt32, 	&P_DatRec_T9Enabled);  
		createParam(P_DatRec_T10EnabledString, asynParamInt32, 	&P_DatRec_T10Enabled);
		createParam(P_DatRec_T11EnabledString, asynParamInt32, 	&P_DatRec_T11Enabled);
		createParam(P_DatRec_T12EnabledString, asynParamInt32, 	&P_DatRec_T12Enabled);
		
		createParam(P_DatRec_T1SrcString,					asynParamInt32,			&P_DatRec_T1Src);
		createParam(P_DatRec_T2SrcString,					asynParamInt32,			&P_DatRec_T2Src);
		createParam(P_DatRec_T3SrcString,					asynParamInt32,			&P_DatRec_T3Src);
		createParam(P_DatRec_T4SrcString,					asynParamInt32,			&P_DatRec_T4Src);
		createParam(P_DatRec_T5SrcString,					asynParamInt32,			&P_DatRec_T5Src);
		createParam(P_DatRec_T6SrcString,					asynParamInt32,			&P_DatRec_T6Src);
		createParam(P_DatRec_T7SrcString,					asynParamInt32,			&P_DatRec_T7Src);
		createParam(P_DatRec_T8SrcString,					asynParamInt32,			&P_DatRec_T8Src);
		createParam(P_DatRec_T9SrcString,					asynParamInt32,			&P_DatRec_T9Src);
		createParam(P_DatRec_T10SrcString,				asynParamInt32,			&P_DatRec_T10Src);
		createParam(P_DatRec_T11SrcString,				asynParamInt32,			&P_DatRec_T11Src);
		createParam(P_DatRec_T12SrcString,				asynParamInt32,			&P_DatRec_T12Src);
		
		createParam(P_DatRec_T1OptionString,					asynParamInt32,			&P_DatRec_T1Option);
		createParam(P_DatRec_T2OptionString,					asynParamInt32,			&P_DatRec_T2Option);
		createParam(P_DatRec_T3OptionString,					asynParamInt32,			&P_DatRec_T3Option);
		createParam(P_DatRec_T4OptionString,					asynParamInt32,			&P_DatRec_T4Option);
		createParam(P_DatRec_T5OptionString,					asynParamInt32,			&P_DatRec_T5Option);
		createParam(P_DatRec_T6OptionString,					asynParamInt32,			&P_DatRec_T6Option);
		createParam(P_DatRec_T7OptionString,					asynParamInt32,			&P_DatRec_T7Option);
		createParam(P_DatRec_T8OptionString,					asynParamInt32,			&P_DatRec_T8Option);
		createParam(P_DatRec_T9OptionString,					asynParamInt32,			&P_DatRec_T9Option);
		createParam(P_DatRec_T10OptionString,					asynParamInt32,			&P_DatRec_T10Option);
		createParam(P_DatRec_T11OptionString,					asynParamInt32,			&P_DatRec_T11Option);
		createParam(P_DatRec_T12OptionString,					asynParamInt32,			&P_DatRec_T12Option);
		
		
		createParam(P_DatRec_ExecConfigString,								asynParamInt32,  &P_DatRec_ExecConfig);
		createParam(P_DatRec_setDRCString,										asynParamInt32,  &P_DatRec_setDRC);
		createParam(P_DatRec_getDRCString,                    asynParamInt32,  &P_DatRec_getDRC);
		createParam(P_DatRec_getDRRString,                    asynParamInt32,  &P_DatRec_getDRR);
		createParam(P_DatRec_DRRProgressString,               asynParamInt32,  &P_DatRec_DRRProgress);
		
		createParam(P_DatRec_getDRLString,                    asynParamInt32,  &P_DatRec_getDRL);
		createParam(P_DatRec_setDRTString,                    asynParamInt32,  &P_DatRec_setDRT);
		createParam(P_DatRec_getDRTString,                    asynParamInt32,  &P_DatRec_getDRT);
		createParam(P_DatRec_getHDRString,                    asynParamInt32,  &P_DatRec_getHDR);
		createParam(P_DatRec_setIMPString,                    asynParamInt32,  &P_DatRec_setIMP);
		createParam(P_DatRec_setRTRString,                    asynParamInt32,  &P_DatRec_setRTR);
		createParam(P_DatRec_getRTRString,                    asynParamInt32,  &P_DatRec_getRTR);
		createParam(P_DatRec_setSTEString,                    asynParamInt32,  &P_DatRec_setSTE);
		createParam(P_DatRec_getTNRString,                    asynParamInt32,  &P_DatRec_getTNR);
		createParam(P_DatRec_setWGOString,                    asynParamInt32,  &P_DatRec_setWGO);
		createParam(P_DatRec_setWGRString,                    asynParamInt32,  &P_DatRec_setWGR);
		
		createParam(P_DatRec_getRTRInSecString, 						asynParamFloat64, &P_DatRec_getRTRInSec);
		                                                      
		createParam(P_DatRec_NptsString,		                asynParamInt32,  &P_DatRec_Npts);
		
		createParam(P_DatRec_FPathString,                			asynParamOctet,    &P_DatRec_FPath);
		
		createParam(P_DatRec_StartString, 										asynParamInt32, &P_DatRec_Start);
		
		createParam(P_DatRec_getDRRStsString,									asynParamInt32, &P_DatRec_getDRRSts);
     
		createParam(P_DatRec_T1SrcFbkString,				          asynParamInt32,  &P_DatRec_T1SrcFbk);			 
		createParam(P_DatRec_T2SrcFbkString,				          asynParamInt32,  &P_DatRec_T2SrcFbk);			 
		createParam(P_DatRec_T3SrcFbkString,				          asynParamInt32,  &P_DatRec_T3SrcFbk);			 
		createParam(P_DatRec_T4SrcFbkString,				          asynParamInt32,  &P_DatRec_T4SrcFbk);			 
		createParam(P_DatRec_T5SrcFbkString,				          asynParamInt32,  &P_DatRec_T5SrcFbk);			 
		createParam(P_DatRec_T6SrcFbkString,				          asynParamInt32,  &P_DatRec_T6SrcFbk);			 
		createParam(P_DatRec_T7SrcFbkString,				          asynParamInt32,  &P_DatRec_T7SrcFbk);			 
		createParam(P_DatRec_T8SrcFbkString,				          asynParamInt32,  &P_DatRec_T8SrcFbk);			 
		createParam(P_DatRec_T9SrcFbkString,				          asynParamInt32,  &P_DatRec_T9SrcFbk);			 
		createParam(P_DatRec_T10SrcFbkString,			            asynParamInt32,  &P_DatRec_T10SrcFbk);
		createParam(P_DatRec_T11SrcFbkString,			            asynParamInt32,  &P_DatRec_T11SrcFbk);
		createParam(P_DatRec_T12SrcFbkString,			            asynParamInt32,  &P_DatRec_T12SrcFbk);
		
		createParam(P_DatRec_T1OptionFbkString,		            asynParamInt32,  &P_DatRec_T1OptionFbk);
		createParam(P_DatRec_T2OptionFbkString,		            asynParamInt32,  &P_DatRec_T2OptionFbk);
		createParam(P_DatRec_T3OptionFbkString,		            asynParamInt32,  &P_DatRec_T3OptionFbk);
		createParam(P_DatRec_T4OptionFbkString,		            asynParamInt32,  &P_DatRec_T4OptionFbk);
		createParam(P_DatRec_T5OptionFbkString,		            asynParamInt32,  &P_DatRec_T5OptionFbk);
		createParam(P_DatRec_T6OptionFbkString,		            asynParamInt32,  &P_DatRec_T6OptionFbk);
		createParam(P_DatRec_T7OptionFbkString,		            asynParamInt32,  &P_DatRec_T7OptionFbk);
		createParam(P_DatRec_T8OptionFbkString,		            asynParamInt32,  &P_DatRec_T8OptionFbk);
		createParam(P_DatRec_T9OptionFbkString,		            asynParamInt32,  &P_DatRec_T9OptionFbk);
		createParam(P_DatRec_T10OptionFbkString,		          asynParamInt32,  &P_DatRec_T10OptionFbk);
		createParam(P_DatRec_T11OptionFbkString,		          asynParamInt32,  &P_DatRec_T11OptionFbk);
		createParam(P_DatRec_T12OptionFbkString,		          asynParamInt32,  &P_DatRec_T12OptionFbk);
		
		createParam(P_DatRec_TrgSrcString,		            asynParamInt32,  &P_DatRec_TrgSrc);
		createParam(P_DatRec_TrgSrcFbkString,	          asynParamInt32,  &P_DatRec_TrgSrcFbk);
		
		createParam(P_DatRec_RTRFbkString,					    asynParamInt32,      &P_DatRec_RTRFbk);
			
		createParam(P_DatRec_TNRFbkString,					    asynParamInt32,      &P_DatRec_TNRFbk);
		createParam(P_DataRec_AutoEnableString, 				asynParamInt32,	&P_DataRec_AutoEnable);
		
		createParam(P_ATZVoltString, 						asynParamFloat64, &P_ATZVolt);
		
		createParam(P_PosFromE712ScalerString, 						asynParamFloat64, &P_PosFromE712Scaler);

		
		m_pWavTbl1Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pWavTbl2Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pWavTbl3Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pWavTbl4Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		
		m_pDDL1Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pDDL2Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pDDL3Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		m_pDDL4Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		
		m_pTrigData = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		
		m_pDDLPutStr = (char *)calloc(8, sizeof(char));
		
		/*m_strbuf = (char *)calloc(((16000 * 11) * 12) + PI_GCS_DATA_HDR_BYTES + 100, sizeof(char));*/
		bytes_per_point = 11;
		num_dest_bytes_expected = ((PI_DR_MAX_POINTS * bytes_per_point) * PI_DR_MAX_TABLES) + PI_GCS_DATA_HDR_BYTES;
		m_strbuf = (char *)calloc(num_dest_bytes_expected + 1, sizeof(char));
		
		m_iDataRecTblEnabled[0] = P_DatRec_T1Enabled;
		m_iDataRecTblEnabled[1] = P_DatRec_T2Enabled;
		m_iDataRecTblEnabled[2] = P_DatRec_T3Enabled;
		m_iDataRecTblEnabled[3] = P_DatRec_T4Enabled;
		m_iDataRecTblEnabled[4] = P_DatRec_T5Enabled;
		m_iDataRecTblEnabled[5] = P_DatRec_T6Enabled;
		m_iDataRecTblEnabled[6] = P_DatRec_T7Enabled;
		m_iDataRecTblEnabled[7] = P_DatRec_T8Enabled;
		m_iDataRecTblEnabled[8] = P_DatRec_T9Enabled;
		m_iDataRecTblEnabled[9] = P_DatRec_T10Enabled;
		m_iDataRecTblEnabled[10] = P_DatRec_T11Enabled;
		m_iDataRecTblEnabled[11] = P_DatRec_T12Enabled;
		
		
		m_iDataRecTblSrcs[0] = P_DatRec_T1Src;
		m_iDataRecTblSrcs[1] = P_DatRec_T2Src;
		m_iDataRecTblSrcs[2] = P_DatRec_T3Src;
		m_iDataRecTblSrcs[3] = P_DatRec_T4Src;
		m_iDataRecTblSrcs[4] = P_DatRec_T5Src;
		m_iDataRecTblSrcs[5] = P_DatRec_T6Src;
		m_iDataRecTblSrcs[6] = P_DatRec_T7Src;
		m_iDataRecTblSrcs[7] = P_DatRec_T8Src;
		m_iDataRecTblSrcs[8] = P_DatRec_T9Src;
		m_iDataRecTblSrcs[9] = P_DatRec_T10Src;
		m_iDataRecTblSrcs[10] = P_DatRec_T11Src;
		m_iDataRecTblSrcs[11] = P_DatRec_T12Src;

		m_iDataRecTblOptions[0] = P_DatRec_T1Option;
		m_iDataRecTblOptions[1] = P_DatRec_T2Option;
		m_iDataRecTblOptions[2] = P_DatRec_T3Option;
		m_iDataRecTblOptions[3] = P_DatRec_T4Option;
		m_iDataRecTblOptions[4] = P_DatRec_T5Option;
		m_iDataRecTblOptions[5] = P_DatRec_T6Option;
		m_iDataRecTblOptions[6] = P_DatRec_T7Option;
		m_iDataRecTblOptions[7] = P_DatRec_T8Option;
		m_iDataRecTblOptions[8] = P_DatRec_T9Option;
		m_iDataRecTblOptions[9] = P_DatRec_T10Option;
		m_iDataRecTblOptions[10] = P_DatRec_T11Option;
		m_iDataRecTblOptions[11] = P_DatRec_T12Option;
		                 
		m_simController = 0;
		m_exec_pending = false;
		m_ddlTblNORD = 0;
		m_suspend_fbk = false;
		m_bDataTransfering = false;
		/*               
		setStringParam(P_ParamFileName, "test_param_file.dat");
		getStringParam(P_ParamFileName , 255, fname);
		printf("the fname that will be passed is [%s]\n", fname);
		*/               
                     
    if (!PIE712ControllerListInitialized)
    {
        PIE712ControllerListInitialized = 1;
        ellInit(&PIE712ControllerList);
    }

    // We should make sure this portName is not already in the list */
    pNode = (PIE712ControllerNode*) calloc(1, sizeof(PIE712ControllerNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = this;
    ellAdd(&PIE712ControllerList, (ELLNODE *)pNode);
		
		this->movesDeferred_ = 0;

    asynStatus status;
    asynUser* pAsynCom;
    status = pasynOctetSyncIO->connect(asynPort, 0, &pAsynCom, NULL);
    if (status)
    {
    	asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "echoHandler: unable to connect to port %s\n",
          asynPort);
    	return;
	}
	
	
	status = pasynOctetSyncIO->setInputEos(pAsynCom, "\n", 1);
	if (status) {
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set input EOS on %s: %s\n",
			  asynPort, pAsynCom->errorMessage);
		return;
	}
	status = pasynOctetSyncIO->setOutputEos(pAsynCom, "", 0);
	if (status) {
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set output EOS on %s: %s\n",
			  asynPort, pAsynCom->errorMessage);
		return;
	}
	
	m_pInterface = new PIInterface(pAsynCom);
	m_pInterface->m_pCurrentLogSink = pAsynCom;
	
	
	/******* setup data recorder interface ****************/
#ifdef USE_DATARECORDER
	  asynUser* dr_pAsynCom;
    status = pasynOctetSyncIO->connect(dr_asynPort, 0, &dr_pAsynCom, NULL);
    if (status)
    {
    	asynPrint(dr_pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "echoHandler: unable to connect to port %s\n",
          dr_asynPort);
    	return;
	}
	
	
	status = pasynOctetSyncIO->setInputEos(dr_pAsynCom, "\n\0", 2);
	if (status) {
		asynPrint(dr_pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set input EOS on %s: %s\n",
			  dr_asynPort, dr_pAsynCom->errorMessage);
		return;
	}
	status = pasynOctetSyncIO->setOutputEos(dr_pAsynCom, "", 0);
	if (status) {
		asynPrint(dr_pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set output EOS on %s: %s\n",
			  dr_asynPort, dr_pAsynCom->errorMessage);
		return;
	}
	
	m_pDataRecInterface = new PIInterface(dr_pAsynCom);
	m_pDataRecInterface->m_pCurrentLogSink = dr_pAsynCom;
	printf("Data Recorder interface has been created\n");
#endif
	/**********************************************************/
	
	
	char inputBuff[256];
	inputBuff[0] = '\0';
	status = m_pInterface->sendAndReceive("*IDN?", inputBuff, 255, pAsynCom);
	asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW, "read from %s: %s\n",  asynPort, inputBuff);
	setStringParam(P_GetIDN, inputBuff);
	
	
	/* some commands we will send require the higher mode of control (level 1) so set it here */
	status = m_pInterface->sendOnly("CCL 1 advanced");
	
	callParamCallbacks();
	
	/* findout what axes there are */
	findConnectedAxes();
	
	 if (numAxes < 1 ) numAxes = 1;
    this->numAxes_ = numAxes;

	if (getNrFoundAxes()<size_t(numAxes))
	{
		// more axes configured than connected to controller
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW, "PIE712Controller: requested number of axes (%d) out of range, only %d axis/axes supported\n",  numAxes, int(getNrFoundAxes()));
		return;
	}
	
	for (axis=0; axis<numAxes; axis++) {
		pAxis  = new PIE712Axis(this, axis, DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_START);
		sprintf(pAxis->m_portName, "%s",portName);
    //moved this to support simulation pAxis->Init(portName);
    /* put back Jan 13 2022*/
    pAxis->Init(portName);
  }

    //startPoller(double(movingPollPeriod)/1000, double(idlePollPeriod)/1000, 10);
    startPoller(double(movingPollPeriod), double(idlePollPeriod), 10);
    
    
    getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);


}

/**************************************************/
int PIE712Controller::commandStringToList(const char *cmndString)
{
    int i;
    int ttl_cmnds_idx = 0;
    int cmnd_start_idx, cmnd_stop_idx;
    char cmnd[5000];
    int num_cmnd_chars;
    int num_bytes = 0;
    
	m_numCmnds = 0;
    num_cmnd_chars = strlen(cmndString);
    i = 0;
    cmnd_start_idx = 0;
    for(i=0; i < num_cmnd_chars; i++) 
    {
        if (cmndString[i] == ';')
        { 
        	/* found separator */
        	cmnd_stop_idx = i;
        	num_bytes = cmnd_stop_idx - cmnd_start_idx;
        	epicsSnprintf(cmnd, num_bytes + 1, "%s", &cmndString[cmnd_start_idx]);
        	/*printf("commandStringToList: [%d] [%s]\n", m_numCmnds, cmnd);*/
        	m_cmnd_list[m_numCmnds] = (char *)malloc(sizeof(cmnd));
        	strncpy(m_cmnd_list[m_numCmnds], cmnd, num_bytes);
					m_cmnd_list[m_numCmnds][num_bytes] = '\0';
		  		cmnd_start_idx = i + 1;
					m_numCmnds += 1;
        }
    }
		/*printf("commandStringToList: DONE\n");*/
    return 0;
}

/**************************************************/
asynStatus PIE712Controller::sendCommandList(const char *cmds)
{
	int i=0;
	int errorCode = 0;
	
	
	/*
	printf("here is the list of commands to send:\n");
	printf(cmds);
	printf("\n");
	*/
	commandStringToList(cmds);
	printf("sendCommandList: starting\n");
	for(i=0; i <m_numCmnds; i++)
	{
		printf("sendCommandList:[%d] sending: [%s]\n", i, m_cmnd_list[i]);
		asynStatus status = m_pInterface->sendOnly(m_cmnd_list[i]);
		//sprintf(m_lastCmnd, "%s", m_cmnd_list[i]); 
		errorCode = getGCSError();
		
		free(m_cmnd_list[i]);
	}
	
	printf("sendCommandList: done\n");
	return(asynSuccess);

}

/**************************************************/
asynStatus PIE712Controller::getWavTblLength(int tblid, int& value)
{
    /*
    command: 
    	GWD? <startpoint=1> <numberOfPoints> <wave tbaleID>
    	
    valid response from the E712 is:
      3 1=150000\n


    */
    char buf[255];
    char cmd[100];
		sprintf(cmd, "WAV? %d 1", tblid);
		asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
	  if (!getValue(buf, value))
		{
			return asynError;
		}
    return status;
}

/**************************************************/
asynStatus PIE712Controller::clearDDLTable(int tblid)
{
    /*
    command: 
    	DTC <wave tbaleID>
    	
    Response:
    	None
      
    */
    
    char cmd[100];
		sprintf(cmd, "DTC %d", tblid);
		asynStatus status = m_pInterface->sendOnly(cmd);
	  return status;
}

/**************************************************/
asynStatus PIE712Controller::clearWavTable(int tblid)
{
    /*
    command: 
    	WCL <wave tbaleID>
    	
    Response:
    	None
      
    */
    
    char cmd[100];
		sprintf(cmd, "WCL %d", tblid);
		asynStatus status = m_pInterface->sendOnly(cmd);
	  return status;
}

/**************************************************/
asynStatus PIE712Controller::clearTriggers(void)
{
    /*
    command: 
    	TWC 
    	
    Response:
    	None
      
    */
    
    char cmd[100];
		sprintf(cmd, "TWC");
		asynStatus status = m_pInterface->sendOnly(cmd);
	  return status;
}


/**************************************************/
asynStatus PIE712Controller::setWaveTableOffset(int wavegen_id, double offset)
{
    /*
    :param wavegen_id:
    :param offset:
    :return:
    */
    char cmd[100];
    asynStatus status;
    
    sprintf(cmd, "WOS %d %f", wavegen_id, offset);
		status = m_pInterface->sendOnly(cmd);
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getWaveTableOffset(int wavegen_id, double &value)
{
	  /*
    :param wavegen_id:
    :return:
    */
    char cmd[100];
    char buf[250];
    asynStatus status;
    
    sprintf(cmd, "WOS? %d", wavegen_id);
		status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
		if (!getValue(buf, value))
		{
			return asynError;
		}
		
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::setWaveTableToGenerator(int wavegen_id, int tblid)
{
    /*
    :param wavegen_id:
    :param offset:
    :return:
    */
    char cmd[100];
    asynStatus status;
    
    sprintf(cmd, "WSL %d %d", wavegen_id, tblid);
		status = m_pInterface->sendOnly(cmd);
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getWaveTableToGenerator(int wavegen_id, int &value)
{
	  /*
    :param wavegen_id:
    :return:
    */
    char cmd[100];
    char buf[250];
    asynStatus status;
    
    sprintf(cmd, "WSL? %d", wavegen_id);
		status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
		if (!getValue(buf, value))
		{
			return asynError;
		}
		
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::setWaveTableCycles(int wavegen_id, int cycles)
{
    /*
    :param wavegen_id:
    :param offset:
    :return:
    */
    char cmd[100];
    asynStatus status;
    
    sprintf(cmd, "WGC %d %d", wavegen_id, cycles);
		status = m_pInterface->sendOnly(cmd);
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getWaveTableCycles(int wavegen_id, int &value)
{
	  /*
    :param wavegen_id:
    :return:
    */
    char cmd[100];
    char buf[250];
    asynStatus status;
    
    sprintf(cmd, "WGC? %d", wavegen_id);
		status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
		if (!getValue(buf, value))
		{
			return asynError;
		}
		
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getTriggerMode(int wavegen_id, int &value)
{
	  /*
    :param wavegen_id:
    :return:
    */
    char cmd[100];
    char buf[250];
    int output_id, mode;
    asynStatus status;
    /* 3 is the TriggerMode CTOPam id */
    /* valid response is: 1 3=4 */

    
    sprintf(cmd, "CTO? %d 3", wavegen_id);
		status = m_pInterface->sendAndReceive(cmd, buf, 99);
		sscanf( buf, "%d 3=%d", &output_id, &mode);
		value = mode;
		
		
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::setWaveTableRate(int wavegen_id, int rate)
{
    /*
    the wave table rate is the same for all wave generators so only need to get from 1 to know
    what all of the rates are but specifying the specific wave gen is supported

	command:
		WTR <wavegenID> <waveTableRate> <interpolationType>

	args:
		<wavegenID>			self explanatory but for some reason this is 0 based so use 0,
				discovered this when using base 1 didn't work  as it reported "Parameter Out Of Range" so used log to
				watch and see what was sent from PI's wavegen app.
		<waveTableRate>		is the table rate to be used for wavegen output
			(unit: number of servo loop cycles), must be an integer value larger than 0
		<interpolationType>	When a awavegen table rate is higher than 1 is set, this option can be used to 
			apply interpolation to the wavegen output between wave table points. 
			The following interpolation types can be selected:
				0 = no interpolation
				1 = straightline (default)

    */
    char cmd[100];
    asynStatus status;
    
    sprintf(cmd, "WTR 0 %d 1", rate);
		status = m_pInterface->sendOnly(cmd);
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getWaveTableRate(int wavegen_id, int &value)
{
	  /*
    the wave table rate is the same for all wave generators so only need to get from 1 to know
    what all of the rates are but specifying the specific wave gen is supported
    */
    char cmd[100];
    char buf[250];
    asynStatus status;
    
    sprintf(cmd, "WTR? %d", wavegen_id);
		status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
		if (!getValue(buf, value))
		{
			return asynError;
		}
		
		return(status);
}

/**************************************************/
asynStatus PIE712Controller::getDDLFlags(int wavegen_id, int &value)
{
	  /*
     OR the 3 DDL flags into one and return
     WVGEN_FLAGS_USE_AND_REINIT	64
			WVGEN_FLAGS_USE							128
			WVGEN_FLAGS_START_AT_ENDPOS	256
     
    */
    int use_ddl = 0;
    int reinit = 0;
    int start_at_end = 0;
    int result = 0;
    
    if(wavegen_id == 1)
    {
    	getIntegerParam(P_WaveTbl1UseDDL, &use_ddl);
    	getIntegerParam(P_WaveTbl1UseReinitDDL, &reinit);
    	getIntegerParam(P_WaveTbl1StartAtEnd, &start_at_end);
    	
    } else if(wavegen_id == 2)
    {
    	getIntegerParam(P_WaveTbl2UseDDL, &use_ddl);
    	getIntegerParam(P_WaveTbl2UseReinitDDL, &reinit);
    	getIntegerParam(P_WaveTbl2StartAtEnd, &start_at_end);
    	
    } else if(wavegen_id == 3)
    {
    	getIntegerParam(P_WaveTbl3UseDDL, &use_ddl);
    	getIntegerParam(P_WaveTbl3UseReinitDDL, &reinit);
    	getIntegerParam(P_WaveTbl3StartAtEnd, &start_at_end);
    	
    } else if(wavegen_id == 4)
    {
    	getIntegerParam(P_WaveTbl4UseDDL, &use_ddl);
    	getIntegerParam(P_WaveTbl4UseReinitDDL, &reinit);
    	getIntegerParam(P_WaveTbl4StartAtEnd, &start_at_end);
    }
    
    if(reinit){
    	result += WVGEN_FLAGS_USE_AND_REINIT;
    }
    if(use_ddl){
    	result += WVGEN_FLAGS_USE ;
    }
    if(start_at_end){
    	result += WVGEN_FLAGS_START_AT_ENDPOS;
    }	
    
    /* this is an mbbo so these are mutually exclusive */
    value = result;
		
		return(asynSuccess);
}
/**************************************************/
asynStatus PIE712Controller::getStartMode(int wavegen_id, int &value)
{
	 /*
     OR the 3 DDL flags into one and return#define 
	    WVGEN_STRTMODE_DO_NOT_START 	0
		 WVGEN_STRTMODE_IMEDDIATELY	 	1
		 WVGEN_STRTMODE_EXTERNAL_TRIG  2
    */
    int start_mode = 0;
    
    
    if(wavegen_id == 1)
    {
    	getIntegerParam(P_WaveTbl1StartMode, &start_mode);
    	
    } else if(wavegen_id == 2)
    {
    	getIntegerParam(P_WaveTbl2StartMode, &start_mode);
    	
    } else if(wavegen_id == 3)
    {
    	getIntegerParam(P_WaveTbl3StartMode, &start_mode);
    	
    } else if(wavegen_id == 4)
    {
    	getIntegerParam(P_WaveTbl4StartMode, &start_mode);
    }
    
    /* this is an mbbo so these are mutually exclusive */
    value = start_mode;
		
		return(asynSuccess);
}
/*
	There are 4 wave generators on the E712, each of which can be connected
	to any one of 120 available wavetables, this function gets the user
	selected wave table numbers and assigns them to each of the 4 waveform 
	generators

*/
asynStatus PIE712Controller::connectWavtablesToGenerator(void)
{	
	int wvgen_usetbl_num = 0;
	int num_wvgens = 0;
	
	getNumWavGenerators(num_wvgens);
	//printf("connectWavtablesToGenerator: num_wvgens=%d\n", num_wvgens);
	
	if(num_wvgens >= 1){
		getIntegerParam(P_WaveGen1UseTblNum, &wvgen_usetbl_num);
		printf("connectWavtablesToGenerator: connecting tbl[%d] to wgen[%d]\n", 1,wvgen_usetbl_num); 
		setWaveTableToGenerator(1, wvgen_usetbl_num);	
	}
	if(num_wvgens >= 2){
		getIntegerParam(P_WaveGen2UseTblNum, &wvgen_usetbl_num);
		printf("connectWavtablesToGenerator: connecting tbl[%d] to wgen[%d]\n", 2,wvgen_usetbl_num); 
		setWaveTableToGenerator(2, wvgen_usetbl_num);	
	}
	
	if(num_wvgens >= 3){
		getIntegerParam(P_WaveGen3UseTblNum, &wvgen_usetbl_num);
		printf("connectWavtablesToGenerator: connecting tbl[%d] to wgen[%d]\n", 3,wvgen_usetbl_num); 
		setWaveTableToGenerator(3, wvgen_usetbl_num);	
	}
	
	if(num_wvgens >= 4){
		getIntegerParam(P_WaveGen4UseTblNum, &wvgen_usetbl_num);
		printf("connectWavtablesToGenerator: connecting tbl[%d] to wgen[%d]\n", 4,wvgen_usetbl_num); 
		setWaveTableToGenerator(4, wvgen_usetbl_num);	
  }  
	
	return(asynSuccess);
	
}

/**************************************************/

asynStatus PIE712Controller::startWavegen(void)
{
    /*
    Check all of hte wave table flags to see which ones are to be used and which are not
    command: 
    	WGO <wave tbaleID> <flags> [<wave tbaleID> <flags> , <wave tbaleID> <flags>, <wave tbaleID> <flags>]
    	
    */
    
    char cmd[100];
    int startMode = 0;
    int ddlFlags = 0;
    int wvgen_usetbl_num = 0;
    int num_wvgens = 0;
    
    char axis1StartStr[25];
    char axis2StartStr[25];
    char axis3StartStr[25];
    char axis4StartStr[25];
    asynStatus status;
		
		getNumWavGenerators(num_wvgens);
		//printf("startWavegen: num_wvgens=[%d]\n",num_wvgens);
    
		/* make sure velo ius set up for fast*/
    //sprintf(cmd, "VEL %d 1000000", m_xAxis_id);
		//status = m_pInterface->sendOnly(cmd);
		
		   
    setWaveTableOffset(m_xAxis_id, m_xStartPos);
		setWaveTableOffset(m_yAxis_id, m_yStartPos);
    
    /* set teh number of cycles to run */
		getIntegerParam(P_NumCycles, &m_numCycles);
    setWaveTableCycles(m_xAxis_id, m_numCycles);
    
    /* first connect wavetable to wavegenreator, here useing 1:1, 2:2, 3:3, 4:4
     I think this is actually hardcoded on the e712 (I think Andreas told me that in an email)
    */
    /*
    setWaveTableToGenerator(m_xAxis_id, m_xAxis_id);
    setWaveTableToGenerator(m_yAxis_id, m_yAxis_id);
    */
    /* start the command string */
    sprintf(cmd, "WGO");
    
    connectWavtablesToGenerator();
    
     /* determine which wavegenerators are configured to be used */
    if(num_wvgens >= 1){
	    getStartMode(1, startMode);
	    getDDLFlags(1, ddlFlags);
	    sprintf(axis1StartStr, " 1 %d", startMode + ddlFlags);
	    strcat(cmd, axis1StartStr); 
    }
    
    if(num_wvgens >= 2){
	    getStartMode(2, startMode);
	    getDDLFlags(2, ddlFlags);
	    sprintf(axis2StartStr, " 2 %d", startMode + ddlFlags);
	    strcat(cmd, axis2StartStr);  
    }
    
    if(num_wvgens >= 3){
	    getStartMode(3, startMode);
	    getDDLFlags(3, ddlFlags);
	    sprintf(axis3StartStr, " 3 %d", startMode + ddlFlags);
	    strcat(cmd, axis3StartStr); 
    }
    
    if(num_wvgens >= 4){
	    getStartMode(4, startMode);
	    getDDLFlags(4, ddlFlags);
	    sprintf(axis4StartStr, " 4 %d", startMode + ddlFlags);
	    strcat(cmd, axis4StartStr); 
    }	
  	/* build a single string for all 4 wavegenerators */      
  	//printf("sending: [%s]\n", cmd);
    status = m_pInterface->sendOnly(cmd);
		
	  return status;
}



/**************************************************/
asynStatus PIE712Controller::stopWavegen(void)
{
    /*
    command: 
    	WGO <wave tbaleID> 0
    	
    Response:
    	None
      
    */
    
    char cmd[100];
    int num_wvgens = 0;
    int i=0;
    asynStatus status;
    
    getNumWavGenerators(num_wvgens);
    
    for(int i=1; i <= num_wvgens; i++){
			sprintf(cmd, "WGO %d %d", i, WVGEN_STRTMODE_DO_NOT_START);
			status = m_pInterface->sendOnly(cmd);
		}
	  return status;
}

/**************************************************/
asynStatus PIE712Controller::getDDLTblLength(int tblid, int& value)
{
    /*
    command: 
    	DTL? <wave tbaleID>
    	
    valid response from the E712 is:
      3=150000\n


    */
    char buf[255];
    char cmd[100];
		sprintf(cmd, "DTL? %d", tblid);
		asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
	  if (!getValue(buf, value))
		{
			return asynError;
		}
    return status;
}

/**************************************************/
asynStatus PIE712Controller::startDataRecorder(void)
{
    /*
    command:
    	DRT <data recorder tableID> <trig src 4 = immediately> <value, dependant on trig src>
    	DRT 1 4
    
    this starts the data recorder immediately
    	
    */
    configDataRecorder();
   	setDataRecTrigSrc(PI_DR_TRG_IMMEDIATE);
		
    return(asynSuccess);
}

/**************************************************/
asynStatus PIE712Controller::setRecTblRate(int rate)
{
    /*
    command:
    	RTR <record table rate= number of servo loop cycles>
    	RTR 1
    
    */
    char cmd[100];
    int rateFbk = 0;
    double rateInSec = 0.0;
    asynStatus status;
    /* june 30 2022 */
    return(asynSuccess);
    
#ifdef NOT_SURE_IF_I_WANT_TO_KEEP
    sprintf(cmd, "RTR %d", rate);
		status = m_pDataRecInterface->sendOnly(cmd);
		
		getRecTblRate(rateFbk);
		setIntegerParam(P_DatRec_getRTR, rateFbk);
		
		rateInSec = rateFbk * PI_DR_MIN_REC_TBL_RATE;
		setDoubleParam(P_DatRec_getRTRInSec, rateInSec);
		
		status = (asynStatus)callParamCallbacks();
		
    return status;
#endif    
}

/**************************************************/
asynStatus PIE712Controller::getRecTblRate(int& value)
{
    /*
    command: 
    	RTR? <rec table rate= number of servo loop cycles>
    	
    valid response from the E712 is:
      1


    */
    char buf[255];
    char cmd[100];
		sprintf(cmd, "RTR?");
		asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
	value = atoi(buf);
    return status;
}

/**************************************************/
asynStatus PIE712Controller::setDataRecTrigSrc(int src)
{
    /*
    command:
    	DRT <data recorder tableID> <trig src 4 = immediately> <value, dependant on trig src>
    	DRT 1 4
    
    this starts the data recorder immediately
    	
    */
    
    
	char cmd[100];
	asynStatus status;
	/* june 30 2022 */
	 return(asynSuccess);

#ifdef NOT_SURE_IF_I_WANT_TO_KEEP    
    /* get only the one table length */
		sprintf(cmd, "DRT 1 %d 1", src);
		status = m_pDataRecInterface->sendOnly(cmd);
		
    return status;
#endif    
}


/**************************************************/
asynStatus PIE712Controller::configDataRecorder(void)
{
    /*
    grab all of the sources and options for all tables and send them to the controller
    command:
    	DRC 1 4
    
    this starts the data recorder immediately
    	
    */
    
    char cmd[100];
    int src, option = 0;
    asynStatus status=asynError;
    /* june 30 2022 */
     return(status);
#ifdef NOT_SURE_IF_I_WANT_TO_KEEP    
    /* walk all 12 channels setting up the src and options */
   	for(int i=0; i < PI_DR_MAX_TABLES; i++){
   		getIntegerParam( m_iDataRecTblSrcs[i], &src);
   		getIntegerParam( m_iDataRecTblOptions[i], &option);
   		sprintf(cmd, "DRC %d %d %d", i+1, src, option);
		status = m_pDataRecInterface->sendOnly(cmd);
	}	
    return status;
#endif    
}





/**************************************************/
/**************************************************/
asynStatus PIE712Controller::getDRRTblLength(int tblid, int& value)
{
    /*
    command:
    	DRL? <data recorder tableID>
    	DRL? 1
    	
    	valid response from the E712 is:
      3=699050\n


    */
    char buf[255 * PI_DR_MAX_TABLES];
    char cmd[100];
   	/* get only the one table length */
		sprintf(cmd, "DRL? %d", tblid);
		asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
		
	  if (!getValue(buf, value))
		{
			return asynError;
		}
		if(value > PI_DR_MAX_POINTS)
		{	
			value = PI_DR_MAX_POINTS;
		}
    return status;
}
/**************************************************/
asynStatus stringToDoubleArray(char *rcvbuf, epicsFloat64 *data_array)
{
	char *token;
	double t_fval = 0.0;
	char *strbuf;
	const char *last_idx;
	int i;
	
	
	if(strlen(rcvbuf) <= 0)
	{
		return(asynSuccess);
	}

	last_idx = strstr( rcvbuf, "# END_HEADER");
	//sprintf(strbuf, last_idx+14);
	strbuf = (char *)last_idx+14;
	/* walk string converting float strings to floats and assign to correct array */
	token = strtok(strbuf, "\n");
  /* walk through other tokens */
  i = 0;
  while( token != NULL ) 
  {
  	t_fval = atof(token);
  	data_array[i] = t_fval;
    //printf( " %s\n", token );
    token = strtok(NULL, "\n");
    i++;
   }	
  
	return(asynSuccess);
}



/**************************************************/
asynStatus PIE712Controller::getTrigtbl(void)
{
	int num_points_expected = 0;
	int num_bytes_expected = 0;
	int trig_output = 0;
	int BYTES_PER_POINT = 33 * 4;
	int start_idx = 1;
	char *rcvbuf;
	char *strbuf;
	
  char cmd[100];
  float t_fval = 0.0;
  int i = 0;
  
  
	asynStatus status = asynSuccess;

	getIntegerParam(P_TrigOutput, &trig_output);
	/* use the x table length as the determiner of how many points to get*/
	getWavTblLength(m_xAxis_id, num_points_expected);
	
	num_bytes_expected = num_points_expected * BYTES_PER_POINT;
	rcvbuf = (char *)malloc(num_bytes_expected + 1);
	strbuf = (char *)malloc(num_bytes_expected + 1);

  free(m_pTrigData);
	m_pTrigData = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
  
  if(num_points_expected > 0){
  	/* only send a command if there are points expected otherwise clear the array */
  	sprintf(cmd, "TWS? %d %d %d", start_idx, num_points_expected, trig_output);
  	status = m_pInterface->sendAndReceive(cmd, rcvbuf, num_bytes_expected);
  	stringToDoubleArray(rcvbuf, m_pTrigData);
  } else {
  		/* reset the waveform array */
		for (i=0; i<WAVE_MAX_NUM_SAMPLES; i++){
			m_pTrigData[i] = 0.0;
		}
  }
  
  //callParamCallbacks();
  doCallbacksFloat64Array(m_pTrigData, num_points_expected , P_TrigTblWf, 0);
	    
	return (status);

}

/**************************************************/
asynStatus PIE712Controller::getWavDatatbl(int tblid)
{
	int num_points_expected = 0;
	int num_bytes_expected = 0;
	int BYTES_PER_POINT = 33 * 4;
	int start_idx = 1;
	char *rcvbuf;
	char *strbuf;
	char cmd[100];
  float t_fval = 0.0;
  int i = 0;
  asynStatus status = asynSuccess;
	epicsFloat64 *t_array = 0;
	int param_id = P_WaveTbl1Wf;
	
	getWavTblLength(tblid, num_points_expected);
	if(num_points_expected == 0){
		return(asynSuccess);
	}
	num_bytes_expected = num_points_expected * BYTES_PER_POINT;
	rcvbuf = (char *)malloc(num_bytes_expected + 1);
	strbuf = (char *)malloc(num_bytes_expected + 1);
  
  if(num_points_expected > 0){
  	/* only send a command if there are points expected otherwise clear the array */
  	sprintf(cmd, "GWD? %d %d %d", start_idx, num_points_expected, tblid);
  	status = m_pInterface->sendAndReceive(cmd, rcvbuf, num_bytes_expected);
	}
		
	if(tblid == 1)
	{
		param_id = P_WaveTbl1Wf;
		free(m_pWavTbl1Data);
		m_pWavTbl1Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pWavTbl1Data;
  	
	} else if(tblid == 2)
	{
		param_id = P_WaveTbl2Wf;
		free(m_pWavTbl2Data);
		m_pWavTbl2Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pWavTbl2Data;
  	
	} else if(tblid == 3)
	{
		param_id = P_WaveTbl3Wf;
		free(m_pWavTbl3Data);
		m_pWavTbl3Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pWavTbl3Data;
  	
	} else if(tblid == 4)
	{
		param_id = P_WaveTbl4Wf;
		free(m_pWavTbl4Data);
		m_pWavTbl4Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pWavTbl4Data;
	} 
	
	if(num_points_expected > 0){
		stringToDoubleArray(rcvbuf, t_array);	
	} else {
		/* reset the waveform array */
		for (i=0; i<WAVE_MAX_NUM_SAMPLES; i++){
			m_pTrigData[i] = 0.0;
		}
	}

  doCallbacksFloat64Array(t_array, num_points_expected , param_id, 0);
	    
	return (status);

}

/**************************************************/
/**************************************************/
asynStatus PIE712Controller::getNumWavGenerators(int& num_wvgens)
{
    /*
    command: 
    	TWG? 
    	
    valid response from the E712 is:
      3\n

		which is the nuumber of waveform genereators available

    */
    char buf[255];
    char cmd[100];
		sprintf(cmd, "TWG?");
		asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
		//printf("getNumWavGenerators: the response is [%s]\n", buf);
		num_wvgens = atoi(buf);
	  return status;
}

/**************************************************/
asynStatus PIE712Controller::getDDLDatatbl(int tblid)
{
	int num_points_expected = 0;
	int num_bytes_expected = 0;
	int BYTES_PER_POINT = 33 * 4;
	int start_idx = 1;
	char *rcvbuf;
	char *strbuf;
	//const char *frst_idx;
	//const char *last_idx;
	char cmd[100];
  float t_fval = 0.0;
  int i = 0;
  //char *token;
	asynStatus status;
	epicsFloat64 *t_array = 0;
	int param_id = P_DDLTbl1Wf;
	
	getDDLTblLength(tblid, num_points_expected);
	
	num_bytes_expected = num_points_expected * BYTES_PER_POINT;
	rcvbuf = (char *)malloc(num_bytes_expected + 1);
	strbuf = (char *)malloc(num_bytes_expected + 1);
  
  if(num_points_expected > 0){
  	/* only send a command if there are points expected otherwise clear the array */
  	sprintf(cmd, "DDL? %d %d %d", start_idx, num_points_expected, tblid);
  	status = m_pInterface->sendAndReceive(cmd, rcvbuf, num_bytes_expected);
	}
	if(tblid == 1)
	{
		free(m_pDDL1Data);
		m_pDDL1Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pDDL1Data;
		param_id = P_DDLTbl1Wf;
		
	} else if(tblid == 2)
	{
		free(m_pDDL2Data);
		m_pDDL2Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pDDL2Data;
		param_id = P_DDLTbl2Wf;
		
		
	} else if(tblid == 3)
	{
		free(m_pDDL3Data);
		m_pDDL3Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pDDL3Data;
		param_id = P_DDLTbl3Wf;
		
		
	} else if(tblid == 4)
	{
		free(m_pDDL4Data);
		m_pDDL4Data = (epicsFloat64 *)calloc(WAVE_MAX_NUM_SAMPLES, sizeof(epicsFloat64));
		t_array = m_pDDL4Data;
		param_id = P_DDLTbl4Wf;
		
	} 
	
	if(num_points_expected > 0){
		stringToDoubleArray(rcvbuf, t_array);	
	} else {
		/* reset the waveform array */
		for (i=0; i<WAVE_MAX_NUM_SAMPLES; i++){
			m_pTrigData[i] = 0.0;
		}
	}
	doCallbacksFloat64Array(t_array, num_points_expected , param_id, 0);
	    
	return (status);

}


asynStatus PIE712Controller::getDRRDatatbls(void)
{
	asynStatus status;
		/* create data recorder thread to send the DRR command and read out all of the data*/
		status = (asynStatus)(epicsThreadCreate("PIE712DatarecorderTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::readDatarecorderTask,
                          this) == NULL);

		return(status);


}

/************************************************************/
void readDatarecorderTask(void *drvPvt)
{
    PIE712Controller *pPvt = (PIE712Controller *)drvPvt;
    pPvt->readDRRDatatbls(pPvt->m_strbuf);
}

/**************************************************/
/* get data from all data recorder tables, this is executed in its own thread */
asynStatus PIE712Controller::readDRRDatatbls(char *strbuf)
{
	int num_points_expected = 0;
	int num_bytes_expected = 0;
	int num_dest_bytes_expected = 0;
	int rcv_bytes_expected = 0;
	char pathname[1000];
	//int BYTES_PER_POINT = 11;
	int BYTES_PER_POINT = 20;
	int start_idx = 1;
	char *ptr;
	char *destbuf;
	char cmd[100];
	char t_cmd[100];
	int t_bytes = 0;
  int i = 0;
  int t_idx = 1;
  int read_points = 10000;
  int datstr_len = 0;
  asynStatus status;
  int enable = 0;
  float total_points_read = 0.0;
  int points_to_read = 500;
  int start_point = 1;
  char *last_idx;
  int dest_ptr = 0;
  int num_tables = 0;
  float progress = 0.0;
  bool not_ready = true;
  int abort = 0;
  int bytes_per_point=0;
  int j = 0;
  
  /* June 28 2022*/
  return(asynSuccess);

#ifdef NOT_SURE_IF_I_WANT_TO_KEEP
  setIntegerParam(P_DatRec_Abort, abort);
  /* clear previous data */
  bytes_per_point = 11;
	//num_dest_bytes_expected = ((PI_DR_MAX_POINTS * bytes_per_point) * PI_DR_MAX_TABLES) + PI_GCS_DATA_HDR_BYTES;
  memset(&strbuf[0], 0, sizeof(strbuf));
  
  strbuf[0] = '\0';
  getStringParam(P_DatRec_FPath , 1000, m_pDataRecFPath);
	sprintf(pathname, "%s",m_pDataRecFPath);
	
	if(strlen(pathname) < 1){
		printf("readDRRDatatbls: pathname is empty\n");
		return(asynSuccess);
	}
  
  not_ready = true;
  i = 0;
	while(not_ready)
  {
  	getIntegerParam(P_DatRec_Abort, &abort);
  	if(abort){
  			break;
  	} else {
			getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);
			if(num_points_expected < 1){
				printf("getDRRDatatbls: DRL? returned num_points_expected < 1, waiting\n");
				/* wait a second */
				epicsThreadSleep(1.0);
				i += 1;
			} else {
				/* its ready so giver */
				not_ready = false;
			}
			
			/* check to see if we waited long enough */
			if(i > 10){
				printf("getDRRDatatbls: datarecorder does not appear to be recording, leaving\n");
				return(asynError);
			}
			
		}
	}
	
	printf("executing readDRRDatatbls from thread\n");
  setIntegerParam(P_DatRec_getDRRSts, 1);
  setIntegerParam(P_DatRec_DRRProgress, progress);
  
  status = (asynStatus)callParamCallbacks();
  
  
  num_dest_bytes_expected = ((PI_DR_MAX_POINTS * BYTES_PER_POINT) * PI_DR_MAX_TABLES) + PI_GCS_DATA_HDR_BYTES;
  destbuf = (char *)calloc(num_dest_bytes_expected + 1, sizeof(char));
  
  
  /* while 699050 not reached, request data points */
  /* build the command string */
  ptr = t_cmd;
  for(int x=0; x<PI_DR_MAX_TABLES; x++){
		getIntegerParam(m_iDataRecTblEnabled[x], &enable);
		if(enable){	
			num_tables += 1;
			sprintf(ptr, "%d ", x+1); 
			ptr += strlen(ptr);
		}
	}
  
  while(PI_DR_MAX_POINTS > total_points_read){
  	/* check to see if we have been aborted */
  	getIntegerParam(P_DatRec_Abort, &abort);
 		if(abort){
 			break;
 		}

  	//getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);
  	getIntegerParam(P_DatRec_getDRL, &num_points_expected);
  	
  	while(num_points_expected < (total_points_read + points_to_read))
  	{
  		getIntegerParam(P_DatRec_Abort, &abort);
  		/* check to see if we have been aborted */
  		if(abort){
  			break;
  		} else {
  		
	  		getIntegerParam(P_DatRec_getDRL, &num_points_expected);
	  		printf("Waiting for more points to be acquired [%d]\n", num_points_expected);
	  		if(num_points_expected >= PI_DR_MAX_POINTS){
	  			break;
	  		}
	  		epicsThreadSleep(2.0);
	  	}
  	}
  	
  	/* check to see if we were broken out of the loop by an abort */
  	if(abort){
  		printf("!!!!!!!!!!!!!!! ABORTING DATA TRANSFER !!!!!!!!!!!!!!!!!!!!!!\n");
  		/* cleanup and leave*/
  		free(destbuf);
  		/* reset the GetDataRecTables status */
			setIntegerParam(P_DatRec_getDRRSts, 0);
			setIntegerParam(P_DatRec_Abort, 0);
		  status = (asynStatus)callParamCallbacks();
  		return(asynSuccess);
  	}
  	
  	sprintf(cmd, "DRR? %d %d %s\n", start_point, points_to_read, t_cmd);
	printf("%s\n", cmd);
	
	num_bytes_expected = ((points_to_read * BYTES_PER_POINT) * num_tables) + PI_GCS_DATA_HDR_BYTES;
			
	status = m_pDataRecInterface->sendAndReceiveDRR(cmd, strbuf, num_bytes_expected);
	start_point = start_point + points_to_read;
	i = strlen(strbuf);
  	
  	if(points_to_read == 500){
  			/* keep the header but modify the total to 699050 */
  			
  	} else {
  		/* skip the header */
  		last_idx = strstr( strbuf, "# END_HEADER");
		strbuf = (char *)last_idx+14;
	}
		
	//strcpy(&destbuf[dest_ptr], "!");
	//dest_ptr += 1;
	j = strlen(strbuf);
	strcpy(&destbuf[dest_ptr], strbuf);
	j = strlen(strbuf);
	dest_ptr += strlen(strbuf);
	//dest_ptr += i;
	i = strlen(destbuf);
		
	points_to_read = points_to_read * 2;
  	if(points_to_read > 64000){
  		points_to_read = 64000;
  	}
  	
  	total_points_read += points_to_read;
  	
  	progress = float(total_points_read / PI_DR_MAX_POINTS) * 100.0;
  	setIntegerParam(P_DatRec_DRRProgress, progress);
  	
  	/* give the controller a break */
  	epicsThreadSleep(1.0);
  	
  	
	}
	

	/* unset flag that suspended most of the polling feedback */
	m_bDataTransfering = false;

	if(abort){
  		printf("!!!!!!!!!!!!!!! ABORTING DATA TRANSFER !!!!!!!!!!!!!!!!!!!!!!\n");
  		/* cleanup and leave*/
  		free(destbuf);
  		/* reset the GetDataRecTables status */
			setIntegerParam(P_DatRec_getDRRSts, 0);
			setIntegerParam(P_DatRec_Abort, 0);
		  status = (asynStatus)callParamCallbacks();
  		return(asynSuccess);
  	} else {
  
		datstr_len = strlen(destbuf);
		saveDataRecFile(destbuf);
		free(destbuf);
		printf("getDRRDatatbls: Done saving datarec file\n");
		/* reset the GetDataRecTables status */
		setIntegerParam(P_DatRec_getDRRSts, 0);
		status = (asynStatus)callParamCallbacks();
		
		return (status);
	}
#endif
}

#ifdef NOT_SURE_IF_I_WANT_TO_KEEP
/**************************************************/
/* get data from all data recorder tables, this is executed in its own thread */
asynStatus PIE712Controller::readDRRDatatbls(char *strbuf)
{
	int num_points_expected = 0;
	int num_bytes_expected = 0;
	int num_dest_bytes_expected = 0;
	int rcv_bytes_expected = 0;
	char pathname[1000];
	//int BYTES_PER_POINT = 11;
	int BYTES_PER_POINT = 11;
	int start_idx = 1;
	char *ptr;
	char *destbuf;
	char cmd[100];
	char t_cmd[100];
	int t_bytes = 0;
  int i = 0;
  int t_idx = 1;
  int read_points = 10000;
  int datstr_len = 0;
  asynStatus status;
  int enable = 0;
  float total_points_read = 0.0;
  int points_to_read = 500;
  int start_point = 1;
  char *last_idx;
  int dest_ptr = 0;
  int num_tables = 0;
  float progress = 0.0;
  bool not_ready = true;
  int abort = 0;
  char new_header[2000]= {'\n'};
  char *start_ptr;
  char *stop_ptr;
  int num_cpy_chars = 0;
  
  /*setIntegerParam(P_DatRec_Abort, abort);*/
  m_bDataTransfering = false;
  strbuf[0] = '\0';
  getStringParam(P_DatRec_FPath , 1000, m_pDataRecFPath);
	sprintf(pathname, "%s",m_pDataRecFPath);
	
	if(strlen(pathname) < 1){
		printf("readDRRDatatbls: pathname is empty\n");
		return(asynSuccess);
	}
  
  not_ready = true;
  i = 0;
	while(not_ready)
  {
  	getIntegerParam(P_DatRec_Abort, &abort);
  	if(abort){
  			break;
  	} else {
			//getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);
			getIntegerParam(P_DatRec_getDRL, &num_points_expected);
			if(num_points_expected < 1){
				printf("getDRRDatatbls: DRL? returned num_points_expected < 1, waiting\n");
				/* wait a second */
				epicsThreadSleep(1.0);
				i += 1;
			} else {
				/* its ready so giver */
				not_ready = false;
			}
			
			/* check to see if we waited long enough */
			if(i > 10){
				printf("getDRRDatatbls: datarecorder does not appear to be recording, leaving\n");
				return(asynError);
			}
			
		}
	}
	
	printf("executing readDRRDatatbls from thread\n");
  setIntegerParam(P_DatRec_getDRRSts, 1);
  setIntegerParam(P_DatRec_DRRProgress, progress);
  
  status = (asynStatus)callParamCallbacks();
  
  
  num_dest_bytes_expected = ((PI_DR_MAX_POINTS * BYTES_PER_POINT) * PI_DR_MAX_TABLES) + PI_GCS_DATA_HDR_BYTES;
  destbuf = (char *)calloc(num_dest_bytes_expected + 1, sizeof(char));
  
  
  /* while 699050 not reached, request data points */
  /* build the command string */
  ptr = t_cmd;
  for(int x=0; x<PI_DR_MAX_TABLES; x++){
		getIntegerParam(m_iDataRecTblEnabled[x], &enable);
		if(enable){	
			num_tables += 1;
			sprintf(ptr, "%d ", x+1); 
			ptr += strlen(ptr);
		}
	}
  
  while(PI_DR_MAX_POINTS > total_points_read){
  	/* check to see if we have been aborted */
  	getIntegerParam(P_DatRec_Abort, &abort);
 		if(abort){
 			break;
 		}

  	//getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);
  	getIntegerParam(P_DatRec_getDRL, &num_points_expected);
  	
  	while(num_points_expected < (total_points_read + points_to_read))
  	{
  		getIntegerParam(P_DatRec_Abort, &abort);
  		/* check to see if we have been aborted */
  		if(abort){
  			break;
  		} else {
  		
	  		getIntegerParam(P_DatRec_getDRL, &num_points_expected);
	  		//getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, num_points_expected);
	  		printf("Waiting for more points to be acquired [%d]\n", num_points_expected);
	  		if(num_points_expected >= PI_DR_MAX_POINTS){
	  			break;
	  		}
	  		
	  		epicsThreadSleep(2.0);
	  	}
  	}
  	
  	/* check to see if we were broken out of the loop by an abort */
  	if(abort){
  		printf("!!!!!!!!!!!!!!! ABORTING DATA TRANSFER !!!!!!!!!!!!!!!!!!!!!!\n");
  		/* cleanup and leave*/
  		free(destbuf);
  		/* reset the GetDataRecTables status */
			setIntegerParam(P_DatRec_getDRRSts, 0);
			//setIntegerParam(P_DatRec_Abort, 0);
		  status = (asynStatus)callParamCallbacks();
  		return(asynSuccess);
  	}
  	
  	sprintf(cmd, "DRR? %d %d %s\n", start_point, points_to_read, t_cmd);
		printf("%s\n", cmd);
	
		num_bytes_expected = ((points_to_read * BYTES_PER_POINT) * num_tables) + PI_GCS_DATA_HDR_BYTES;
			
		status = m_pDataRecInterface->sendAndReceiveDRR(cmd, strbuf, num_bytes_expected);
	  start_point = start_point + points_to_read;
		i = strlen(strbuf);
  	
  		if(points_to_read == 500){
  				/* keep the header but modify the total to 699050 */
  			start_ptr = strbuf;
  			last_idx = strstr( strbuf, "# END_HEADER");
			strbuf = (char *)last_idx+14;
			stop_ptr = (char *)last_idx+14;
			num_cpy_chars = stop_ptr - start_ptr;
			strncpy(new_header, start_ptr, num_cpy_chars);
			fix_NDATA_value(new_header, 699050);
			printf("%s\n", new_header);
			/* copy the new header*/
			strcpy(&destbuf[dest_ptr], new_header);
			dest_ptr += strlen(new_header);
			/* now adjust strbuf ptr to start of the data */
			last_idx = strstr( start_ptr, "# END_HEADER");
			strbuf = start_ptr;
			strbuf = (char *)last_idx+14;
	  			
  		} else {
  			/* skip the header */
  			last_idx = strstr( strbuf, "# END_HEADER");
				strbuf = (char *)last_idx+14;
		}
		
		strcpy(&destbuf[dest_ptr], strbuf);
		dest_ptr += strlen(strbuf);
		i = strlen(destbuf);
		
		points_to_read = points_to_read * 2;
  		if(points_to_read > 64000){
  			points_to_read = 64000;
  		}
	  	
  		total_points_read += points_to_read;
	  	
  		progress = float(total_points_read / PI_DR_MAX_POINTS) * 100.0;
  		setIntegerParam(P_DatRec_DRRProgress, progress);
	  	
  		/* give the controller a break */
  		epicsThreadSleep(1.0);
  	
  	
	}
	
	

	/* unset flag that suspended most of the polling feedback */
	m_bDataTransfering = false;

	if(abort){
  		printf("!!!!!!!!!!!!!!! ABORTING DATA TRANSFER !!!!!!!!!!!!!!!!!!!!!!\n");
  		/* cleanup and leave*/
  		free(destbuf);
  		/* reset the GetDataRecTables status */
			setIntegerParam(P_DatRec_getDRRSts, 0);
			//setIntegerParam(P_DatRec_Abort, 0);
		  status = (asynStatus)callParamCallbacks();
  		return(asynSuccess);
  	} else {
  
		datstr_len = strlen(destbuf);
		saveDataRecFile(destbuf);
		free(destbuf);
		printf("getDRRDatatbls: Done saving datarec file\n");
		/* reset the GetDataRecTables status */
		setIntegerParam(P_DatRec_getDRRSts, 0);
		status = (asynStatus)callParamCallbacks();
		
		return (status);
	}

}

#endif //NOT_SURE_IF_I_WANT_TO_KEEP
asynStatus PIE712Controller::fix_NDATA_value(char *hdrbuf, int num_data)
{
	char *idx = NULL;
	char *ptr = NULL;
	bool done = false;
	char tbuf[50000] = {0};
	char *tptr = NULL;
	int nchars = 0;
	char t_line[100] = {0};

	/* # NDATA = 699050 */
	tptr = tbuf;
	char* line = strtok(hdrbuf, "\n");
	while (line != NULL)
	{
		/* add a plus 1 for the newline char */
		//nchars = strlen(line) + 1;
		idx = strstr( line, "# NDATA = ");
		if(idx != NULL)
		{
			sprintf(t_line, "# NDATA = %d\n", num_data);
			nchars = strlen(t_line);
			strncpy(tptr, t_line, nchars);
		} else {
			/* copy this line */
			sprintf(t_line, "%s\n", line);
			nchars = strlen(t_line);
			strncpy(tptr, t_line, nchars);
			//strncpy(tptr, line, nchars);
		}
		tptr += nchars;
		
		line = strtok(NULL, "\n");
	}
	nchars = strlen(tbuf);
	strncpy(hdrbuf, tbuf, nchars);

	return(asynSuccess);		
}

/*********************************************************/
/****
	TODO:
		need to split directory and filename into two diff vars so that I 
		can check to see of the directory exists before trying to save to it, 
		same with checking the validity of the filename
		
	DIR* dir = opendir("mydir");
	if (dir)
	{
	    Directory exists. 
	    closedir(dir);
	}
	else if (ENOENT == errno)
	{
	    Directory does not exist. 
	}
	else
	{
	    opendir() failed for some other reason. 
	}

****/
asynStatus PIE712Controller::saveDataRecFile(char *lines)
{
	FILE * fptr;
	char pathname[1000];
	
	//getStringParam(P_DatRec_FileName , 255, fname);
	getStringParam(P_DatRec_FPath , 1000, m_pDataRecFPath);
	//sprintf(fpath, "%s\\%s",m_pDataRecFPath, fname);
	sprintf(pathname, "%s",m_pDataRecFPath);
	
	if(strlen(pathname) < 1){
		printf("saveDataRecFile: pathname is empty\n");
		return(asynSuccess);
	}
	fptr = fopen (pathname,"wb"); 
	fwrite(lines, sizeof(char), strlen(lines), fptr);
  fclose (fptr);
  printf("saveDataRecFile: saved [%s]\n", pathname);
  return(asynSuccess);
  
 }



/***********************************************************************
This fuinction takes an array of data points and constructs a string that
ets sent to the E712g

	DDL <table id> <starting point> val1 val2 val3 ....valN

*/
asynStatus PIE712Controller::putDDLDatatbl(int tblid, int num_points, epicsFloat64 *data)
{
	//epicsFloat64 *t_array;
	//int param_id;
	char *ptr;
	int i = 0;
	bool new_cmnd = false;
	asynStatus status = asynSuccess;
	
	free(m_pDDLPutStr);
	m_pDDLPutStr = (char *)calloc(250 * 33, sizeof(char));
	printf("putDDLDatatbl: tblid<%d> num_points<%d> \n", tblid, num_points);
	for (i=0; i < num_points; )
	{
		/* one line of the command has the command DDL <table id> <starting point> followed by up to 10 floating point vals */
		sprintf(m_pDDLPutStr, "DDL %d %d ", tblid, i+1);
		ptr = m_pDDLPutStr + strlen(m_pDDLPutStr);
		
		for(int x=0; x < 10; x++)
		{	
			sprintf(ptr, "%05.5f ", data[i]);
			ptr += strlen(ptr);
			i++;
		}
		sprintf(ptr, "\n");
		/*printf("%s", m_pDDLPutStr);*/
		
		/* send one line (10 points) at a time */
		status = m_pInterface->sendOnly(m_pDDLPutStr);

		if((num_points - i) < 10)
		{	
			/* handle the last lines one at a time */
			break;
		}
	}
	/* now finish off any < 10 remaining */
	if((i < num_points))
	{
		for (	; i < num_points; i++)
		{
			sprintf(m_pDDLPutStr, "DDL %d %d %.5f ", tblid, i+1, data[i]);
			status = m_pInterface->sendOnly(m_pDDLPutStr);
		}
		/*printf("%s", m_pDDLPutStr);*/
	}

	printf("done\n");
	return(status);

}


/***************************************************************************/
asynStatus PIE712Controller::getWaveGenStatus(bool *wvg_1, bool *wvg_2, bool *wvg_3, bool *wvg_4 )
{
	char cmd = 0x09;
	char buf[255];
	int t_val = 0;
	
	
	//sprintf(cmd, "#9");
	//sprintf(cmd, "%s", GCS_REQ_WAVGEN_STS); printf("%x", ch & 0xff);
	//printf("getWaveGenStatus: sending [%x]\n", cmd);
	
  asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
  
  if (status != asynSuccess)
  {
    	return status;
  }
  t_val = (int)strtol(buf, NULL, 16);
  
  if(t_val & WVGEN_1_RUNNING)
	{
  	*wvg_1 = true;
	} else {
			*wvg_1 = false;
	}
	
  
  if(t_val & WVGEN_2_RUNNING)
	{
  	*wvg_2 = true;
	} else {
		*wvg_2 = false;
	}
	
  if(t_val & WVGEN_3_RUNNING)
	{
  	*wvg_3 = true;
	} else {
		*wvg_3 = false;
	}
	
  if(t_val & WVGEN_4_RUNNING)
	{
  	*wvg_4 = true;
	} else {
		*wvg_4 = false;
  }
  return status;
} 

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including PilatusBadPixelFile, ADFilePath, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus PIE712Controller::writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    PIE712Axis *pAxis = (PIE712Axis *)this->getAxis(pasynUser);
    const char *functionName = "writeOctet";
    char fname[256];

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);

    if (function == P_SendCommands) {
    		printf("in writeOctet for P_SendCommands\n");
    		/*printf("writeOctet: [%s]\n", value);*/
    		
    		/* BUSY */
    		status = pAxis->setIntegerParam(P_CommStatus, 1);
    		status = (asynStatus)callParamCallbacks();
    		
    		sendCommandList(value);
    		/* READY */
    		status = pAxis->setIntegerParam(P_CommStatus, 0);
    		status = (asynStatus)callParamCallbacks();
    } else if (function == P_ParamFileName) {
    		printf("in writeOctet for P_ParamFileName\n");
    		setStringParam(P_ParamFileName, (char *)value);
				getStringParam(P_ParamFileName , 255, fname);
				printf("the fname that will be passed is [%s]\n", fname);
    		
    } else if(function == P_DatRec_FPath){
    		setStringParam(P_DatRec_FPath, (char *)value);
				getStringParam(P_DatRec_FPath , 1000, m_pDataRecFPath);
				printf("the data rec file path name that will be used is [%s]\n", m_pDataRecFPath);
  	
  	}
    else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PI_E712_PARAM){
        	 status = asynMotorController::writeOctet(pasynUser, value, nChars, nActual);
        }
    }
    
     /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, value=%s", 
                  driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%s\n", 
              driverName, functionName, function, value);
    *nActual = nChars;
    return status;
}


/****************************************************************************************/
void PIE712Controller::report(FILE *fp, int level)
{
    int axis;
    PIE712Axis *pAxis;

    fprintf(fp, "motor driver %s, numAxes=%d\n", this->portName, this->numAxes_);

    for (axis=0; axis<this->numAxes_; axis++)
    {
        pAxis = getAxis(axis);
        fprintf(fp, "  axis %d  m_quad_parm 0x%08x  m_cap_parm 0x%08x \n", 
            pAxis->m_axisNo, pAxis->m_quad_parm, pAxis->m_cap_parm);

        if (level > 0)
        {
            if (pAxis->m_isHoming)
            {
            	fprintf(fp, "    Currently homing axis\n" );
            }
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}
/**************************************************************/

asynStatus PIE712Controller::findConnectedAxes()
{
	m_nrFoundAxes = 0;
	for (size_t i=0; i<MAX_NR_AXES; i++)
	{
		m_axesIDs[i] = NULL;
	}
	asynStatus status = m_pInterface->sendAndReceive("SAI?", m_allAxesIDs, 255);

	if (asynSuccess != status)
	{
		return status;
	}
	char* szAxis = strtok(m_allAxesIDs, "\n");
	while (szAxis != NULL)
	{
		int i=strlen(szAxis)-1;
		while (szAxis[i] == ' ')
		{
			szAxis[i] = '\0';
			i--;
		}
		if (MAX_NR_AXES <= m_nrFoundAxes)
		{
			return asynError;
		}
		m_axesIDs[m_nrFoundAxes] = szAxis;
		m_nrFoundAxes++;
		szAxis = strtok(NULL, "\n");
	}
	return status;
}


/****************************************************************/    
bool PIE712Controller::getValue(const char* szMsg, double& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	value = atof(p+1);
	return true;
}

/****************************************************************/    
bool PIE712Controller::getValue(const char* szMsg, int& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	value = atoi(p+1);
	return true;
}

/****************************************************************/    
bool PIE712Controller::getValue(const char* szMsg, bool& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	int ivalue = atoi(p+1);
	value = (ivalue =! 0);
	return true;
}

/****************************************************************/    
asynStatus PIE712Controller::setGCSParameter(PIE712Axis* pAxis, unsigned int paramID, double value)
{
    char cmd[100];
    sprintf(cmd, "SPA %d %d %.12g", pAxis->m_axisNo, paramID, value);
    asynStatus status = m_pInterface->sendOnly(cmd);
    return status;
}

/****************************************************************/    
asynStatus PIE712Controller::getGCSParameter(PIE712Axis* pAxis, unsigned int paramID, double& value)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "SPA? %d %d", pAxis->m_axisNo, paramID);
	asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}

	if (!getValue(buf, value))
	{
		return asynError;
	}
    return status;

}

/****************************************************************/    
asynStatus PIE712Controller::getGCSParameter(int itemID, unsigned int paramID, double& value)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "SPA? %d %d", itemID, paramID);
	asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}

	if (!getValue(buf, value))
	{
		return asynError;
	}
    return status;

}


/****************************************************************************************/
asynStatus PIE712Controller::processDeferredMoves()
{
	
		return(asynSuccess);
/*		
    asynStatus status = asynError;
    int axis;
    PIE712Axis *pAxesArray[PIE712Axis::MAX_NR_AXES];
    int targetsCts[PIE712Axis::MAX_NR_AXES];

    int numDeferredAxes = 0;
    for (axis=0; axis<this->numAxes_; axis++)
    {
    	PIE712Axis *pAxis = getAxis(axis);
        if (pAxis->deferred_move)
        {
        	pAxesArray[numDeferredAxes] = pAxis;
        	targetsCts[numDeferredAxes] = pAxis->deferred_position;
        	pAxis->setIntegerParam(motorStatusDone_, 0);
        	pAxis->callParamCallbacks();
        	numDeferredAxes++;
        }
    }
    if (numDeferredAxes > 0)
    {
    	status = moveCts(pAxesArray, targetsCts, numDeferredAxes);
    }

    for (axis=0; axis<this->numAxes_; axis++)
    {
        if (getAxis(axis)->deferred_move)
        {
        	getAxis(axis)->deferred_move = 0;
        }
    }
    epicsEventSignal(pollEventId_);

    return status;
*/
}

/****************************************************************************************/
asynStatus PIE712Controller::toggleEncoderSource(int axis_no, int type)
{
	PIE712Axis *pAxis;
	char cmd[100];
	asynStatus status = asynSuccess;
	/*int m_quad_parm = 0;*/
	int m_cap_parm = 0;
	
	pAxis = getAxis(axis_no-1);
	
	
	/* set appropriate CCL level in order to be able to change the param */
  /*
  sprintf(cmd, "CCL 1 advanced");
  status = m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
  	asynPrint(m_pInterface->m_pCurrentLogSink,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
  	 "PIE712Controller::toggleEncoderSource() unable to set CCL level\n");
  	return status;
  }
  */
  
  if(type == USE_CAPACITIVE_SENSOR){
  	/* deselect quadrature sensor */
  	sprintf(cmd, "SPA %d 0x%08x 0", axis_no, pAxis->m_quad_parm); 
  	printf("%s\n",cmd);
		status = m_pInterface->sendOnly(cmd);
	  if (status != asynSuccess)
	  {
	  	asynPrint(m_pInterface->m_pCurrentLogSink,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	  	 "PIE712Controller::toggleEncoderSource() unable to [%s]\n", cmd);
	  	return status;
	  }
	  /* select capacitance sensor, the sign of the m_piezo_driving_factor determines the direction the controller will count*/
  	sprintf(cmd, "SPA %d 0x%08x %.2f", axis_no, pAxis->m_cap_parm, pAxis->m_piezo_driving_factor); 
  	printf("%s\n",cmd);
		status = m_pInterface->sendOnly(cmd);
	  if (status != asynSuccess)
	  {
	  	asynPrint(m_pInterface->m_pCurrentLogSink,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	  	 "PIE712Controller::toggleEncoderSource() unable to [%s]\n", cmd);
	  	return status;
	  }
	  
  } else {	
  	/* USE_QUADRATURE_SENSOR */
  	/* deselect capacitive sensor */
  	sprintf(cmd, "SPA %d 0x%08x 0", axis_no, pAxis->m_cap_parm); 
  	printf("%s\n",cmd);
		status = m_pInterface->sendOnly(cmd);
	  if (status != asynSuccess)
	  {
	  	asynPrint(m_pInterface->m_pCurrentLogSink,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	  	 "PIE712Controller::toggleEncoderSource() unable to [%s]\n", cmd);
	  	return status;
	  }
	  /* select quadrature sensor */
  	//sprintf(cmd, "SPA %d 0x%08x %.2f", axis_no, m_quad_parm, m_piezo_driving_factor); 
  	sprintf(cmd, "SPA %d 0x%08x 1.0", axis_no, pAxis->m_quad_parm); 
  	printf("%s\n",cmd);
		status = m_pInterface->sendOnly(cmd);
	  if (status != asynSuccess)
	  {
	  	asynPrint(m_pInterface->m_pCurrentLogSink,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	  	 "PIE712Controller::toggleEncoderSource() unable to [%s]\n", cmd);
	  	return status;
	  }
	 } 
	
	/* set driving factor of piezo */
	/* this sets the sign of the output voltage, basically reverses the direction */
  /*
  sprintf(cmd, "SPA %d 0x%08x %.2f", axis_no, m_piezo_driving_factor_param, m_piezo_driving_factor); 
	status = pC_->m_pInterface->sendOnly(cmd);
	if (status != asynSuccess)
	{
	 	asynPrint(pasynUser_,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	 	 "PIE712Axis::toggleEncoderSource() unable to set driving factor of piezo [%s]\n", cmd);
	 	return status;
	}
	*/
	
	 /* write params to flash */
/*
  sprintf(cmd, "WPA 100"); 
	status = pC_->m_pInterface->sendOnly(cmd);
	if (status != asynSuccess)
	 {
	 	asynPrint(pasynUser_,ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
	 	 "PIE712Axis::toggleEncoderSource() unable to [%s]\n", cmd);
	 	return status;
	} 
	
*/	
	return status;
	
}





/*****************************************************/
asynStatus PIE712Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	  int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    PIE712Axis *pAxis = (PIE712Axis *)this->getAxis(pasynUser);
    static const char *functionName = "writeInt32";
    char fname[256];
    char *cmd_ptr;
    int auto_dr_enable = 0;
    int num_wvgens = 0;

    lock();
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setIntegerParam(function, value);
    
    if (function == motorClosedLoop_)
    {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
        		"%s:%s: %sing Closed-Loop Control flag on driver %s\n",
        		value != 0.0?"Enabl":"Disabl",
        		driverName, functionName, this->portName);
        status = pAxis->setServo((value!=0)?1:0);

    }
    else if (function == motorDeferMoves_)
    {
    	asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %sing Deferred Move flag on driver %s\n",
            value != 0.0?"Sett":"Clear",
            driverName, functionName, this->portName);
        if (value == 0.0 && this->movesDeferred != 0)
        {
            processDeferredMoves();
        }
        this->movesDeferred = value;
    } else {
    	
    	if (function == P_Power)
			{
			  	/*printf("writeInt32: P_Power[%d]: changed\n", function);*/
			  	if(value){
			  		pAxis->motorOn();
			  		
			  	} else {
			  		pAxis->motorOff();
			  	}
			}
			else if (function == P_ServoOffAndCenter)
			{
			  	/*printf("writeInt32: P_ServoOffAndCenter[%d]: changed\n", function);*/
			  	if(value){
			  		pAxis->motorOffRelaxToCenter();
			  	}
			}
			
			else if (function == P_Mode)
			{
				status = pAxis->setMode( value);
			}
			else if (function == P_AutoZero)
			{
				status = pAxis->autoZero();
			}
			/*
			else if (function == P_SelectEncoderSrc)
			{
				status = pAxis->selectEncoderSource(value);
			}
			
    		else if (function == P_SelectOutputDir)
			{
				status = pAxis->selectPiezoOutputDirection(value);
			}
			*/
			else if (function == P_ForceDone)
			{
				pAxis->m_ForceDone = value;
			}
			else if (function == P_ZeroADSensor)
			{
				if(value){
					pAxis->zeroADSensor();
				}	
			}
			else if (function == P_ResetVoltRangeWarning)
			{
				status = pAxis->resetVoltWarning( value);
			}
			
			else if (function == P_DigitalFilterType)
			{
					pAxis->setDigFiltType(value);
			}
			else if (function == P_DigitalFilterORder)
			{
					pAxis->setDigFiltOrder(value);
			}
			
			else if (function == P_TriggerOutputSel)
			{
				
				if(value == CTO_TRIGMODE_POSDIST)
				{
						pAxis->setPositionDistanceMarker();
						
				} else if(value == CTO_TRIGMODE_ONTRGT)
				{
						pAxis->setOnTargetMarker();
						
				} else if(value == CTO_TRIGMODE_MINMAX)
				{
						pAxis->setMinMaxMarker();
						
				} else if(value == CTO_TRIGMODE_GENTRIG)
				{
						pAxis->setGeneratorTrigMarker();
						
				} 
				
				
				//asynStatus setMarkerPolarity(int pol);

			}
			else if (function == P_GetWaveTbl1)
			{
					if(value == 1) getWavDatatbl(1);
			}
			else if (function == P_GetWaveTbl2)
			{
					if(value == 1) getWavDatatbl(2);
			}
			else if (function == P_GetWaveTbl3)
			{
					if(value == 1) getWavDatatbl(3);
			}
			else if (function == P_GetWaveTbl4)
			{
					if(value == 1) getWavDatatbl(4);
			}
			
			else if (function == P_GetDDLTbl1)
			{
					if(value == 1) getDDLDatatbl(1);
			}
			else if (function == P_GetDDLTbl2)
			{
					if(value == 1) getDDLDatatbl(2);
			}
			else if (function == P_GetDDLTbl3)
			{
					if(value == 1) getDDLDatatbl(3);
			}
			else if (function == P_GetDDLTbl4)
			{
					if(value == 1) getDDLDatatbl(4);
			}
			else if (function == P_GetTrigTbl)
			{
					if(value == 1) getTrigtbl();
			}
			
			else if (function == P_NumCycles)
			{
				/* the num cycles is the same for all wave generators so only need to set for wavegen 1 */
				setWaveTableCycles(1, value);
					
			}
			else if (function == P_WaveTblRate)
			{
				/* the wave table rate is the same for all wave generators so only need to set for wavegen 1 */
				setWaveTableRate(1, value);
					
			}
			


			else if (function == P_TrigOutput)
			{
					m_trigOutput = value;
			}
			
			else if (function == P_XAxisId)
			{
					m_xAxis_id = value;
			}
			else if (function == P_YAxisId)
			{
					m_yAxis_id = value;
			}
			else if (function == P_ClrDDLTbl1)
			{
					if(value == 1) clearDDLTable(1);
			}
			else if (function == P_ClrDDLTbl2)
			{
					if(value == 1) clearDDLTable(2);
			}
			else if (function == P_ClrDDLTbl3)
			{
					if(value == 1) clearDDLTable(3);
			}
			else if (function == P_ClrDDLTbl4)
			{
					if(value == 1) clearDDLTable(4);
			}
			
			else if (function == P_ClrWavTbl1)
			{
					if(value == 1) clearWavTable(1);
			}
			else if (function == P_ClrWavTbl2)
			{
					if(value == 1) clearWavTable(2);
			}
			else if (function == P_ClrWavTbl3)
			{
					if(value == 1) clearWavTable(3);
			}
			else if (function == P_ClrWavTbl4)
			{
					if(value == 1) clearWavTable(4);
			}
			else if (function == P_ClrTrigTbl)
			{
					if(value == 1) clearTriggers();
			}
			
			
			else if (function == P_StopWavegen)
			{	
				if(value == 1)
				{
					stopWavegen();
					//stopWavegen(2);
					//stopWavegen(3);
					//stopWavegen(4);
				}
			}
			else if (function == P_StartWavegen)
			{
					if(value == 1) {
						startWavegen();
						getIntegerParam(P_DataRec_AutoEnable, &auto_dr_enable);
						if(auto_dr_enable)
						{
							
							/* cause an abort of a current recording if noe is being taken */
							//setIntegerParam(P_DatRec_Abort, 1);
							/* Do callbacks so higher layers see any changes */
	    				callParamCallbacks();
							epicsThreadSleep(5.0);
							getDRRDatatbls();
						}
					}
					
			}
			else if (function == P_ExecWavegen)
			{
					if(value == 1){
						m_exec_pending = true;
						startWavegen();
						getIntegerParam(P_DataRec_AutoEnable, &auto_dr_enable);
						if(auto_dr_enable)
						{
							/* cause an abort of a current recording if noe is being taken */
							//setIntegerParam(P_DatRec_Abort, 1);
							/* Do callbacks so higher layers see any changes */
	    				callParamCallbacks();
							epicsThreadSleep(5.0);
							getDRRDatatbls();
						}
					} else {
						m_exec_pending = false;
					}
					
			}
			
			
			else if (function == P_CalcDDLParms)
			{
				if(value == 1) 
				{
					calcDDLProcParms(1);
					calcDDLProcParms(2);
					calcDDLProcParms(3);
					calcDDLProcParms(4);
				}
			}
			else if (function == P_WaveTbl1StartMode)
			{
					m_wvgen1_startMode = value;
			}
			else if (function == P_WaveTbl2StartMode)
			{
					m_wvgen2_startMode = value;
			}
			else if (function == P_WaveTbl3StartMode)
			{
					m_wvgen3_startMode = value;
			}
			else if (function == P_WaveTbl4StartMode)
			{
					m_wvgen4_startMode = value;
			}
			else if (function == P_LoadParamFile)
			{
					if(value == 1){
						/* tell poll() to skip updates until we are done */
						m_suspend_fbk = true;
						
						getStringParam(P_ParamFileName , 255, fname);
						printf("writeInt32: the fname is [%s]\n", fname);
						cmd_ptr = load_param_file(fname);
						if(cmd_ptr != NULL){
							sendCommandList(cmd_ptr);
						}
						m_suspend_fbk = false;
					}
			}
			
			else if (function == P_DatRec_ExecConfig)
			{
					if(value == 1) configDataRecorder();
					
			}
			
			else if (function == P_DatRec_getDRR)
			{
					if(value == 1) {
						/* cause an abort of a current recording if noe is being taken */
						//setIntegerParam(P_DatRec_Abort, 1);
    				getDRRDatatbls();
					}
					
			}
			else if (function == P_DatRec_Start)
			{
					if(value == 1) {
						startDataRecorder();
						/* cause an abort of a current recording if noe is being taken */
						//setIntegerParam(P_DatRec_Abort, 1);
						/* Do callbacks so higher layers see any changes */
    				callParamCallbacks();
						epicsThreadSleep(5.0);
						getDRRDatatbls();
					}
					
			}
			else if (function == P_DatRec_TrgSrc)
			{
					setDataRecTrigSrc(value);
					
			}
			
			else if (function == P_DatRec_setRTR)
			{
					setRecTblRate(value);
					
			}
			
			
    	
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeInt32(pasynUser, value);
    }
    unlock();
    /* Do callbacks so higher layers see any changes */
    pAxis->callParamCallbacks();
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}


/*****************************************************/
asynStatus PIE712Controller::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	
	//pasynUser_ = pasynUser;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    PIE712Axis *pAxis = (PIE712Axis *)this->getAxis(pasynUser);
    static const char *functionName = "writeFloat64";
    double p_markerstop = 0.0;
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setDoubleParam(function, value);
    
    if (function == PI_SUP_TARGET)
    {
    	printf("PI_SUP_TargetAO: %f for axis %d\n", value, pAxis->m_axisNo);
    }
    else if (function == PI_SUP_PIVOT_X)
    {
    	status = pAxis->SetPivotX(value);
    }
    else if (function == PI_SUP_PIVOT_Y)
    {
    	status = pAxis->SetPivotY(value);
    }
    else if (function == PI_SUP_PIVOT_Z)
    {
    	status = pAxis->SetPivotZ(value);
    }
//    else if (function == motorPosition_) // Entspricht das DFH ?
//    {
//  //      pAxis->enc_offset = (double) value - pAxis->nextpoint.axis[0].p;
//        asynPrint(pasynUser, ASYN_TRACE_FLOW,
//            "%s:%s: Set axis %d to position %d",
//            driverName, functionName, pAxis->m_axisNo, value);
//    }
    else if (function == motorResolution_ )
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    else if (function == motorEncoderRatio_)
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
     else if (function == P_DigitalFilterBWidth)
    {
        status = pAxis->setDigFiltBWidth(value);
    }
    else if (function == P_DigitalFilterORder)
    {
        status = pAxis->setDigFiltOrder(value);
    }
    else if (function == P_DigitalFilterParm1)
    {
        status = pAxis->setDigFiltParm1(value);
    }
    else if (function == P_DigitalFilterParm2)
    {
        status = pAxis->setDigFiltParm2(value);
    }
    else if (function == P_DigitalFilterParm3)
    {
        status = pAxis->setDigFiltParm3(value);
    }
    else if (function == P_DigitalFilterParm4)
    {
        status = pAxis->setDigFiltParm4(value);
    }
    else if (function == P_DigitalFilterParm5)
    {
        status = pAxis->setDigFiltParm5(value);
    }
    else if (function == P_ForceDoneWaitTime)
    {
        pAxis->setForceDoneWTime(value);
    }
    
    
    else
    {
    	
    		if (function == P_SetMarker)
			  {
			  	/*printf("writeFloat64: P_SetMarker[%d]: changed\n", function);*/
			  	//getDoubleParam(axisNo_, pC_->P_MarkerStop, &p_markerstop);
			  	//status = pAxis->setMarker(value);
			  	status = pAxis->setMarkerWindow();
			  	
			  }
			  
	   		if (function == P_OutputVolt)
			  {
			  	/*printf("writeFloat64: setOpenLoopPosition[%.2f]: changed\n", value);*/
			  	status = pAxis->setOpenLoopPosition(value);
			  }
	
    	
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    /* Do callbacks so higher layers see any changes */
    pAxis->callParamCallbacks();
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "%s:%s: error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    return status;
}


/** Called when asyn clients call pasynFloat64Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus PIE712Controller::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
  int function = pasynUser->reason;
  static const char *functionName = "writeFloat64Array";
   
  getIntegerParam(P_DDLTblNORD, &m_ddlTblNORD);
	
  if (function == P_DDLTbl1Wf) {
    putDDLDatatbl(1, m_ddlTblNORD, value);
  }
  else if (function == P_DDLTbl2Wf) {
    putDDLDatatbl(2, m_ddlTblNORD, value);
  } 
  else if (function == P_DDLTbl3Wf) {
  	/*printf("writeFloat64Array: calling putDDLDatatbl(3, m_ddlTblNORD, value);\n");*/
    putDDLDatatbl(3, m_ddlTblNORD, value);
  }
  else if (function == P_DDLTbl4Wf) {
    putDDLDatatbl(4, m_ddlTblNORD, value);
  }
  else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: unknown parameter number %d\n", driverName, functionName, function);
    return asynError ;
  }
  return asynSuccess;
}

#ifdef TESTING
/****************************** readFloat64 read interface *******************************/
asynStatus PIE712Controller::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  PIE712Axis *pAxis = getAxis(pasynUser);
  //PIE712Axis *pAxis = (PIE712Axis *)this->getAxis(pasynUser);
  static const char *functionName = "readFloat64";
  static char outputBuffer[8];
  double t_dval = 0.0;
  
  
  if (function == P_DigitalFilterBWidth) {
  		getGCSParameter(pAxis, DIGFILT_BWIDTH, t_dval);
  		*value = t_dval;
  } else if (function == P_DigitalFilterParm1) {
     	getGCSParameter(pAxis, DIGFILT_PARM1, t_dval);
  		*value = t_dval;
 	} else if (function == P_DigitalFilterParm2) {
     	getGCSParameter(pAxis, DIGFILT_PARM2, t_dval);
  		*value = t_dval;
  } else if (function == P_DigitalFilterParm3) {
     	getGCSParameter(pAxis, DIGFILT_PARM3, t_dval);
  		*value = t_dval;
  } else if (function == P_DigitalFilterParm4) {
     	getGCSParameter(pAxis, DIGFILT_PARM4, t_dval);
  		*value = t_dval;
  } else if (function == P_DigitalFilterParm5) {
     	getGCSParameter(pAxis, DIGFILT_PARM5, t_dval);
  		*value = t_dval;
  } else if (function == P_PTerm) {
     	getGCSParameter(pAxis, PTERM_PARAM, t_dval);
  		*value = t_dval;
  } else if (function == P_ITerm) {
     	getGCSParameter(pAxis, ITERM_PARAM, t_dval);
  		*value = t_dval;
  } else if (function == P_DTerm) {
     	getGCSParameter(pAxis, DTERM_PARAM, t_dval);
  		*value = t_dval;  		
  } else {
      // Call base class
   	 status = asynMotorController::readFloat64(pasynUser, value);
  }


  return status;
}
#endif

/***************************************************************************/
asynStatus PIE712Controller::getWaveTableLength(int waveTbl, int &result)
{
	char cmd[100];
	char buf[255];
	int t_val = 0;
	int chan = 0;

		if(m_simController)
		{	
			result = 0  ;
			return(	asynSuccess);
		}	
		
	sprintf(cmd, "WAV? %d 1", waveTbl);
	
  asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);;
  if (status != asynSuccess)
  {
    	return status;
  }
  
  if (!getValue(buf, t_val))
  {
   	return asynError;
  }
  result = t_val;
  return status;
}


/***************************************************************************/
asynStatus PIE712Controller::calcDDLProcParms(int waveTbl)
{
	char cmd[100];
	int t_val = 0;
	int chan = 0;
	asynStatus status;

		if(m_simController)
		{	
			return(	asynSuccess);
		}	
		
	/*
	sprintf(cmd, "CCL 1 advanced");
	status = m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
    	return status;
  }
  */
  
  sprintf(cmd, "DPO %d", waveTbl);
	status = m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
    	return status;
  }
  
  /*
  sprintf(cmd, "CCL 0 advanced", waveTbl);
	status = m_pInterface->sendOnly(cmd);
  if (status != asynSuccess)
  {
    	return status;
  }
  */
  
  return status;
}

	
/** Polls the controller, rather than individual axis
  * Used during profile moves */
asynStatus PIE712Controller::poll(void)
{
		int wg_sts, exec_ival, t_ival = 0;
		double t_dval = 0.0;
		int total_points = WAVE_MAX_NUM_SAMPLES;

		if(m_suspend_fbk)
		{	
			return(asynSuccess);
		}
			
		/* update the wavegen status */
    getWaveGenStatus(&m_wavegen1_Status, &m_wavegen2_Status, &m_wavegen3_Status, &m_wavegen4_Status);
    getIntegerParam(P_ExecWavegen, &exec_ival);
    
    setIntegerParam(P_WaveGen1_Status,   m_wavegen1_Status);
    setIntegerParam(P_WaveGen2_Status,   m_wavegen2_Status);
    setIntegerParam(P_WaveGen3_Status,   m_wavegen3_Status);
    setIntegerParam(P_WaveGen4_Status,   m_wavegen4_Status);
    
    /* check the wavegenerator status, if it is stopped and the busy record P_ExecWavegen to 0*/
    wg_sts = (m_wavegen1_Status + m_wavegen2_Status + m_wavegen3_Status + m_wavegen4_Status);
    
#ifdef TESTING    
    /* wavegen is stopped, exec has been requested, pending flag has been cleared so set it to stopped */
    if((wg_sts == 0) && (exec_ival == 1) && (m_exec_pending == false))
    {
    	/* set it to stopped */
    	setIntegerParam(P_ExecWavegen, 0);
    }
    /*if((wg_sts > 0) && (m_exec_pending))*/
    if(wg_sts > 0)
    {
    	/* ok the wavegen is now active so reset the pending flag */
    	m_exec_pending = false;
    }
#endif
		if(wg_sts > 0)
		{
			setIntegerParam(P_ExecWavegen, 1);
		} else {
			setIntegerParam(P_ExecWavegen, 0);
		}
		
//#ifdef RUSS_SEPT_7_2022    
    
    /* only use the value returned from table 1 as all the tables report the same (most recently set) value */
    getWaveTableCycles(1, t_ival);
    setIntegerParam(P_NumCycles,   t_ival);
    
    getTriggerMode(1, t_ival);
    setIntegerParam(P_TrigModeWg1, t_ival);
    getTriggerMode(2, t_ival);
    setIntegerParam(P_TrigModeWg2, t_ival);
    getTriggerMode(3, t_ival);
    setIntegerParam(P_TrigModeWg3, t_ival);
    getTriggerMode(4, t_ival);
    setIntegerParam(P_TrigModeWg4, t_ival);
    
    
    /* only use the value returned from table 1 as all the tables report the same (most recently set) value */
    getWaveTableRate(1, t_ival);
    setIntegerParam(P_WaveTblRate,   t_ival);
    
    /****************************************/
    getWaveTableLength(1, t_ival);
    setIntegerParam(P_WaveTbl1Len,   t_ival);
    total_points -= t_ival;
    
    getWaveTableLength(2, t_ival);
    setIntegerParam(P_WaveTbl2Len,   t_ival);
    total_points -= t_ival;
    
    getWaveTableLength(3, t_ival);
    setIntegerParam(P_WaveTbl3Len,   t_ival);
    total_points -= t_ival;
    
    getWaveTableLength(4, t_ival);
    setIntegerParam(P_WaveTbl4Len,   t_ival);
    total_points -= t_ival;
    
    
    getDDLTblLength(1, t_ival);
    setIntegerParam(P_DDLTbl1Len,   t_ival);
    
    getDDLTblLength(2, t_ival);
    setIntegerParam(P_DDLTbl2Len,   t_ival);
    
    getDDLTblLength(3, t_ival);
    setIntegerParam(P_DDLTbl3Len,   t_ival);
    
    getDDLTblLength(4, t_ival);
    setIntegerParam(P_DDLTbl4Len,   t_ival);
    
    /*****************************************/
    setIntegerParam(P_TotalPointsLeft,   total_points);
    
    /* update some member variables */
    getIntegerParam(P_XAxisId, &m_xAxis_id);
    getIntegerParam(P_YAxisId, &m_yAxis_id);
		getDoubleParam(P_XStartPos, &m_xStartPos);
		getDoubleParam(P_YStartPos, &m_yStartPos);
		getIntegerParam(P_ScanMode, &m_scanMode);
		
		getDRRTblLength(PI_DR_DEFAULT_TABLE_ID, t_ival);
		setIntegerParam(P_DatRec_getDRL, t_ival);

//#endif // RUSS_SEPT_7_2022    		
    
    callParamCallbacks();
		
	return asynSuccess;
}


/****************************************************************************************/
asynStatus PIE712Controller::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger )
{
	asynPrint(pasynUser, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
			"PIE712Controller::profileMove() - not implemented\n");
	return asynError;
}
/****************************************************************************************/
asynStatus PIE712Controller::triggerProfile(asynUser *pasynUser)
{
	asynPrint(pasynUser, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
			"PIE712Controller::profileMove() - not implemented\n");
	return asynError;
}

/****************************************************************************************/
int PIE712Controller::getGCSError()
{
	char buf[256];
	asynStatus status = m_pInterface->sendAndReceive("ERR?", buf, 255);
	if (asynTimeout == status)
	{
		return COM_TIMEOUT;
	}
	else if (asynSuccess != status)
	{
		return COM_ERROR;
	}
	int errorCode = atoi(buf);

	/* errcode 10 is "stopped by command" which should not be listed as an error */
	if ((0 != errorCode) && (10 != errorCode) )
	{
		m_LastError = errorCode;
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW, "PIGCSController::getGCSError() GCS error code = %d\n", errorCode);
		char szErrorMsg[1024];
		if (TranslatePIError(errorCode, szErrorMsg, 1024))
		{
	        asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,"PIGCSController::getGCSError() GCS error, %s\n", szErrorMsg);
	        sprintf(m_lastErr, "[%s] %s", m_lastCmnd, szErrorMsg);
	        setStringParam(P_LastErr, m_lastErr);
		
		}
	} else {
		if(m_suspend_fbk){
			sprintf(m_lastErr, "Currently Updating E712 Parameters");
		} else {
			sprintf(m_lastErr, " ");
		}
	  setStringParam(P_LastErr, m_lastErr);
	}
	callParamCallbacks();
	return errorCode;
}

/** Configuration command, called directly or from iocsh */
extern "C" int PI_E712_CreateController(const char *portName, const char* asynPort, const char* dr_asynPort, int numAxes, int movingPollPeriod, int idlePollPeriod)
{
	double d_movingPollPeriod = movingPollPeriod;
	double d_idlePollPeriod = idlePollPeriod;
  PIE712Controller *pSimController = new PIE712Controller(portName, asynPort, dr_asynPort, numAxes, d_movingPollPeriod*0.001, d_idlePollPeriod * 0.001); 
  //piezoMotorController *pSimController = new piezoMotorController(portName, numAxes, 0.2, 0.2, priority, stackSize);
  pSimController = NULL;
  return(asynSuccess);
}


/** Code for iocsh registration */
/***************************************************************************/
static const iocshArg PI_E712_CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg PI_E712_CreateControllerArg1 = {"asyn Port name", iocshArgString};
static const iocshArg PI_E712_CreateControllerArg2 = {"data recorder asyn Port name", iocshArgString};
static const iocshArg PI_E712_CreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg PI_E712_CreateControllerArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg PI_E712_CreateControllerArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg * const PI_E712_CreateControllerArgs[] =  {&PI_E712_CreateControllerArg0,
                                                                 &PI_E712_CreateControllerArg1,
                                                                 &PI_E712_CreateControllerArg2,
																																&PI_E712_CreateControllerArg3,
																																&PI_E712_CreateControllerArg4,
																																&PI_E712_CreateControllerArg5};
																																 
/***************************************************************************/																																 
static const iocshFuncDef PI_E712_CreateControllerDef = {"PI_E712_CreateController", 6, PI_E712_CreateControllerArgs};
static void PI_E712_CreateControllerCallFunc(const iocshArgBuf *args)
{
	PI_E712_CreateController(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}



/***************************************************************************/
extern "C" int PI_E712_MotorConfigAxis(const char *portName, int axis, char *axisName, int hiHardLimit, int lowHardLimit, int home, int start, int simulate)
{
  PIE712ControllerNode *pNode;
  static const char *functionName = "PI_E712_MotorConfigAxis";
  
  // Find this controller
  if (!PIE712ControllerListInitialized) {
    printf("%s:%s: ERROR, controller list not initialized\n", driverName, functionName);
    return(-1);
  }
  pNode = (PIE712ControllerNode*)ellFirst(&PIE712ControllerList);
  while(pNode) {
    if (strcmp(pNode->portName, portName) == 0) {
			printf("%s:%s: configuring controller %s axis %d\n", driverName, functionName, pNode->portName, axis); 
      //pNode->pController->getAxis(axis)->config(axisName, hiHardLimit, lowHardLimit, home, start, simulate);
      PIE712Axis *pAxis = pNode->pController->getAxis(axis);
      pAxis->config(axisName, hiHardLimit, lowHardLimit, home, start, simulate);
      
      return(0);
    }
    pNode = (PIE712ControllerNode*)ellNext((ELLNODE*)pNode);
  }
  printf("PIE712 Controller not found\n");
  return(-1);
}
/***************************************************************************/
static const iocshArg PI_E712_MotorConfigAxisArg0 = { "Port name",     iocshArgString};
static const iocshArg PI_E712_MotorConfigAxisArg1 = { "Axis #",        iocshArgInt};
static const iocshArg PI_E712_MotorConfigAxisArg2 = { "Axis name",     iocshArgString};
static const iocshArg PI_E712_MotorConfigAxisArg3 = { "High limit",    iocshArgInt};
static const iocshArg PI_E712_MotorConfigAxisArg4 = { "Low limit",     iocshArgInt};
static const iocshArg PI_E712_MotorConfigAxisArg5 = { "Home position", iocshArgInt};
static const iocshArg PI_E712_MotorConfigAxisArg6 = { "Start posn",    iocshArgInt};
static const iocshArg PI_E712_MotorConfigAxisArg7 = { "Simulate",    iocshArgInt};

static const iocshArg *const PI_E712_MotorConfigAxisArgs[] = {
  &PI_E712_MotorConfigAxisArg0,
  &PI_E712_MotorConfigAxisArg1,
  &PI_E712_MotorConfigAxisArg2,
  &PI_E712_MotorConfigAxisArg3,
  &PI_E712_MotorConfigAxisArg4,
  &PI_E712_MotorConfigAxisArg5,
  &PI_E712_MotorConfigAxisArg6,
  &PI_E712_MotorConfigAxisArg7
};
/***************************************************************************/
static const iocshFuncDef PI_E712_MotorConfigAxisDef ={"PI_E712_MotorConfigAxis", 8, PI_E712_MotorConfigAxisArgs};
/***************************************************************************/
static void PI_E712_MotorConfigAxisCallFunc(const iocshArgBuf *args)
{
  PI_E712_MotorConfigAxis(args[0].sval, args[1].ival, args[2].sval, args[3].ival, args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}

static void PIE712DriverRegister(void)
{
    iocshRegister(&PI_E712_CreateControllerDef, PI_E712_CreateControllerCallFunc);
    iocshRegister(&PI_E712_MotorConfigAxisDef, PI_E712_MotorConfigAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(PIE712DriverRegister);
}
