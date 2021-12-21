/*
FILENAME...     translateerror.c

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 1.1 $
Modified By:    $Author: Russ Berg (bergr) $
Last Modified:  $Date: 2018/09/27 13:19:47CST $
HeadURL:        $URL$

Original Author: Steffen Rau 
*/

#include <stdio.h>
#include <string.h>

#include "picontrollererrors.h"

#ifndef BOOL
#define BOOL int
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * translate error code to c-string
 */
BOOL TranslatePIError(const int error, char* szBuffer, const int maxlen)
{	
    switch (error)
    {
/*************************************************
 **
 ** Dll Errors - DLL errors occured in GCS DLL
 */
        case(PI_UNKNOWN_AXIS_IDENTIFIER): /* -1001 */
        {
            if ((int)strlen("Unknown axis identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown axis identifier");
                return TRUE;
            }
            break;
        }
        case(PI_NR_NAV_OUT_OF_RANGE): /* -1002 */
        {
            if ((int)strlen("Number for NAV out of range--must be in [1,10000]")<maxlen)
            {
                sprintf(szBuffer, "Number for NAV out of range--must be in [1,10000]");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SGA): /* -1003 */
        {
            if ((int)strlen("Invalid value for SGA--must be one of {1, 10, 100, 1000}")<maxlen)
            {
                sprintf(szBuffer, "Invalid value for SGA--must be one of {1, 10, 100, 1000}");
                return TRUE;
            }
            break;
        }
        case(PI_UNEXPECTED_RESPONSE): /* -1004 */
        {
            if ((int)strlen("Controller sent unexpected response")<maxlen)
            {
                sprintf(szBuffer, "Controller sent unexpected response");
                return TRUE;
            }
            break;
        }
        case(PI_NO_MANUAL_PAD): /* -1005 */
        {
            if ((int)strlen("No manual control pad installed, calls to SMA and related commands are not allowed")<maxlen)
            {
                sprintf(szBuffer, "No manual control pad installed, calls to SMA and related commands are not allowed");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_MANUAL_PAD_KNOB): /* -1006 */
        {
            if ((int)strlen("Invalid number for manual control pad knob")<maxlen)
            {
                sprintf(szBuffer, "Invalid number for manual control pad knob");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_MANUAL_PAD_AXIS): /* -1007 */
        {
            if ((int)strlen("Axis not currently controlled by a manual control pad")<maxlen)
            {
                sprintf(szBuffer, "Axis not currently controlled by a manual control pad");
                return TRUE;
            }
            break;
        }
        case(PI_CONTROLLER_BUSY): /* -1008 */
        {
            if ((int)strlen("Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)")<maxlen)
            {
                sprintf(szBuffer, "Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)");
                return TRUE;
            }
            break;
        }
        case(PI_THREAD_ERROR): /* -1009 */
        {
            if ((int)strlen("Internal error--could not start thread")<maxlen)
            {
                sprintf(szBuffer, "Internal error--could not start thread");
                return TRUE;
            }
            break;
        }
        case(PI_IN_MACRO_MODE): /* -1010 */
        {
            if ((int)strlen("Controller is (already) in macro mode--command not valid in macro mode")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) in macro mode--command not valid in macro mode");
                return TRUE;
            }
            break;
        }
        case(PI_NOT_IN_MACRO_MODE): /* -1011 */
        {
            if ((int)strlen("Controller not in macro mode--command not valid unless macro mode active")<maxlen)
            {
                sprintf(szBuffer, "Controller not in macro mode--command not valid unless macro mode active");
                return TRUE;
            }
            break;
        }
        case(PI_MACRO_FILE_ERROR): /* -1012 */
        {
            if ((int)strlen("Could not open file to write or read macro")<maxlen)
            {
                sprintf(szBuffer, "Could not open file to write or read macro");
                return TRUE;
            }
            break;
        }
        case(PI_NO_MACRO_OR_EMPTY): /* -1013 */
        {
            if ((int)strlen("No macro with given name on controller, or macro is empty")<maxlen)
            {
                sprintf(szBuffer, "No macro with given name on controller, or macro is empty");
                return TRUE;
            }
            break;
        }
        case(PI_MACRO_EDITOR_ERROR): /* -1014 */
        {
            if ((int)strlen("Internal error in macro editor")<maxlen)
            {
                sprintf(szBuffer, "Internal error in macro editor");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_ARGUMENT): /* -1015 */
        {
            if ((int)strlen("One or more arguments given to function is invalid (empty string, index out of range, ...)")<maxlen)
            {
                sprintf(szBuffer, "One or more arguments given to function is invalid (empty string, index out of range, ...)");
                return TRUE;
            }
            break;
        }
        case(PI_AXIS_ALREADY_EXISTS): /* -1016 */
        {
            if ((int)strlen("Axis identifier is already in use by a connected stage")<maxlen)
            {
                sprintf(szBuffer, "Axis identifier is already in use by a connected stage");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_AXIS_IDENTIFIER): /* -1017 */
        {
            if ((int)strlen("Invalid axis identifier")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis identifier");
                return TRUE;
            }
            break;
        }
        case(PI_COM_ARRAY_ERROR): /* -1018 */
        {
            if ((int)strlen("Could not access array data in COM server")<maxlen)
            {
                sprintf(szBuffer, "Could not access array data in COM server");
                return TRUE;
            }
            break;
        }
        case(PI_COM_ARRAY_RANGE_ERROR): /* -1019 */
        {
            if ((int)strlen("Range of array does not fit the number of parameters")<maxlen)
            {
                sprintf(szBuffer, "Range of array does not fit the number of parameters");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SPA_CMD_ID): /* -1020 */
        {
            if ((int)strlen("Invalid parameter ID given to SPA or SPA?")<maxlen)
            {
                sprintf(szBuffer, "Invalid parameter ID given to SPA or SPA?");
                return TRUE;
            }
            break;
        }
        case(PI_NR_AVG_OUT_OF_RANGE): /* -1021 */
        {
            if ((int)strlen("Number for AVG out of range--must be >0")<maxlen)
            {
                sprintf(szBuffer, "Number for AVG out of range--must be >0");
                return TRUE;
            }
            break;
        }
        case(PI_WAV_SAMPLES_OUT_OF_RANGE): /* -1022 */
        {
            if ((int)strlen("Incorrect number of samples given to WAV")<maxlen)
            {
                sprintf(szBuffer, "Incorrect number of samples given to WAV");
                return TRUE;
            }
            break;
        }
        case(PI_WAV_FAILED): /* -1023 */
        {
            if ((int)strlen("Generation of wave failed")<maxlen)
            {
                sprintf(szBuffer, "Generation of wave failed");
                return TRUE;
            }
            break;
        }
        case(PI_MOTION_ERROR): /* -1024 */
        {
            if ((int)strlen("Motion error: position error too large, servo is switched off automatically")<maxlen)
            {
                sprintf(szBuffer, "Motion error: position error too large, servo is switched off automatically");
                return TRUE;
            }
            break;
        }
        case(PI_RUNNING_MACRO): /* -1025 */
        {
            if ((int)strlen("Controller is (already) running a macro")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) running a macro");
                return TRUE;
            }
            break;
        }
        case(PI_PZT_CONFIG_FAILED): /* -1026 */
        {
            if ((int)strlen("Configuration of PZT stage or amplifier failed")<maxlen)
            {
                sprintf(szBuffer, "Configuration of PZT stage or amplifier failed");
                return TRUE;
            }
            break;
        }
        case(PI_PZT_CONFIG_INVALID_PARAMS): /* -1027 */
        {
            if ((int)strlen("Current settings are not valid for desired configuration")<maxlen)
            {
                sprintf(szBuffer, "Current settings are not valid for desired configuration");
                return TRUE;
            }
            break;
        }
        case(PI_UNKNOWN_CHANNEL_IDENTIFIER): /* -1028 */
        {
            if ((int)strlen("Unknown channel identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown channel identifier");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_PARAM_FILE_ERROR): /* -1029 */
        {
            if ((int)strlen("Error while reading/writing wave generator parameter file")<maxlen)
            {
                sprintf(szBuffer, "Error while reading/writing wave generator parameter file");
                return TRUE;
            }
            break;
        }
        case(PI_UNKNOWN_WAVE_SET): /* -1030 */
        {
            if ((int)strlen("Could not find description of wave form. Maybe WG.INI is missing?")<maxlen)
            {
                sprintf(szBuffer, "Could not find description of wave form. Maybe WG.INI is missing?");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_FUNC_NOT_LOADED): /* -1031 */
        {
            if ((int)strlen("The WGWaveEditor DLL function was not found at startup")<maxlen)
            {
                sprintf(szBuffer, "The WGWaveEditor DLL function was not found at startup");
                return TRUE;
            }
            break;
        }
        case(PI_USER_CANCELLED): /* -1032 */
        {
            if ((int)strlen("The user cancelled a dialog")<maxlen)
            {
                sprintf(szBuffer, "The user cancelled a dialog");
                return TRUE;
            }
            break;
        }
        case(PI_C844_ERROR): /* -1033 */
        {
            if ((int)strlen("Error from C-844 Controller")<maxlen)
            {
                sprintf(szBuffer, "Error from C-844 Controller");
                return TRUE;
            }
            break;
        }
        case(PI_DLL_NOT_LOADED): /* -1034 */
        {
            if ((int)strlen("DLL necessary to call function not loaded, or function not found in DLL")<maxlen)
            {
                sprintf(szBuffer, "DLL necessary to call function not loaded, or function not found in DLL");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_PROTECTED): /* -1035 */
        {
            if ((int)strlen("The open parameter file is protected and cannot be edited")<maxlen)
            {
                sprintf(szBuffer, "The open parameter file is protected and cannot be edited");
                return TRUE;
            }
            break;
        }
        case(PI_NO_PARAMETER_FILE_OPENED): /* -1036 */
        {
            if ((int)strlen("There is no parameter file open")<maxlen)
            {
                sprintf(szBuffer, "There is no parameter file open");
                return TRUE;
            }
            break;
        }
        case(PI_STAGE_DOES_NOT_EXIST): /* -1037 */
        {
            if ((int)strlen("Selected stage does not exist")<maxlen)
            {
                sprintf(szBuffer, "Selected stage does not exist");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_ALREADY_OPENED): /* -1038 */
        {
            if ((int)strlen("There is already a parameter file open. Close it before opening a new file")<maxlen)
            {
                sprintf(szBuffer, "There is already a parameter file open. Close it before opening a new file");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_OPEN_ERROR): /* -1039 */
        {
            if ((int)strlen("Could not open parameter file")<maxlen)
            {
                sprintf(szBuffer, "Could not open parameter file");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_CONTROLLER_VERSION): /* -1040 */
        {
            if ((int)strlen("The version of the connected controller is invalid")<maxlen)
            {
                sprintf(szBuffer, "The version of the connected controller is invalid");
                return TRUE;
            }
            break;
        }
        case(PI_PARAM_SET_ERROR): /* -1041 */
        {
            if ((int)strlen("Parameter could not be set with SPA--parameter not defined for this controller!")<maxlen)
            {
                sprintf(szBuffer, "Parameter could not be set with SPA--parameter not defined for this controller!");
                return TRUE;
            }
            break;
        }
        case(PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED): /* -1042 */
        {
            if ((int)strlen("The maximum number of wave definitions has been exceeded")<maxlen)
            {
                sprintf(szBuffer, "The maximum number of wave definitions has been exceeded");
                return TRUE;
            }
            break;
        }
        case(PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED): /* -1043 */
        {
            if ((int)strlen("The maximum number of wave generators has been exceeded")<maxlen)
            {
                sprintf(szBuffer, "The maximum number of wave generators has been exceeded");
                return TRUE;
            }
            break;
        }
        case(PI_NO_WAVE_FOR_AXIS_DEFINED): /* -1044 */
        {
            if ((int)strlen("No wave defined for specified axis")<maxlen)
            {
                sprintf(szBuffer, "No wave defined for specified axis");
                return TRUE;
            }
            break;
        }
        case(PI_CANT_STOP_OR_START_WAV): /* -1045 */
        {
            if ((int)strlen("Wave output to axis already stopped/started")<maxlen)
            {
                sprintf(szBuffer, "Wave output to axis already stopped/started");
                return TRUE;
            }
            break;
        }
        case(PI_REFERENCE_ERROR): /* -1046 */
        {
            if ((int)strlen("Not all axes could be referenced")<maxlen)
            {
                sprintf(szBuffer, "Not all axes could be referenced");
                return TRUE;
            }
            break;
        }
        case(PI_REQUIRED_WAVE_NOT_FOUND): /* -1047 */
        {
            if ((int)strlen("Could not find parameter set required by frequency relation")<maxlen)
            {
                sprintf(szBuffer, "Could not find parameter set required by frequency relation");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SPP_CMD_ID): /* -1048 */
        {
            if ((int)strlen("Command ID given to SPP or SPP? is not valid")<maxlen)
            {
                sprintf(szBuffer, "Command ID given to SPP or SPP? is not valid");
                return TRUE;
            }
            break;
        }
        case(PI_STAGE_NAME_ISNT_UNIQUE): /* -1049 */
        {
            if ((int)strlen("A stage name given to CST is not unique")<maxlen)
            {
                sprintf(szBuffer, "A stage name given to CST is not unique");
                return TRUE;
            }
            break;
        }
        case(PI_FILE_TRANSFER_BEGIN_MISSING): /* -1050 */
        {
            if ((int)strlen("A uuencoded file transfered did not start with \"begin\" followed by the proper filename")<maxlen)
            {
                sprintf(szBuffer, "A uuencoded file transfered did not start with \"begin\" followed by the proper filename");
                return TRUE;
            }
            break;
        }
        case(PI_FILE_TRANSFER_ERROR_TEMP_FILE): /* -1051 */
        {
            if ((int)strlen("Could not create/read file on host PC")<maxlen)
            {
                sprintf(szBuffer, "Could not create/read file on host PC");
                return TRUE;
            }
            break;
        }
        case(PI_FILE_TRANSFER_CRC_ERROR): /* -1052 */
        {
            if ((int)strlen("Checksum error when transfering a file to/from the controller")<maxlen)
            {
                sprintf(szBuffer, "Checksum error when transfering a file to/from the controller");
                return TRUE;
            }
            break;
        }
        case(PI_COULDNT_FIND_PISTAGES_DAT): /* -1053 */
        {
            if ((int)strlen("The PiStages.dat database could not be found. This file is required to connect a stage with the CST command")<maxlen)
            {
                sprintf(szBuffer, "The PiStages.dat database could not be found. This file is required to connect a stage with the CST command");
                return TRUE;
            }
            break;
        }
        case(PI_NO_WAVE_RUNNING): /* -1054 */
        {
            if ((int)strlen("No wave being output to specified axis")<maxlen)
            {
                sprintf(szBuffer, "No wave being output to specified axis");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_PASSWORD): /* -1055 */
        {
            if ((int)strlen("Invalid password")<maxlen)
            {
                sprintf(szBuffer, "Invalid password");
                return TRUE;
            }
            break;
        }
        case(PI_OPM_COM_ERROR): /* -1056 */
        {
            if ((int)strlen("Error during communication with OPM (Optical Power Meter), maybe no OPM connected")<maxlen)
            {
                sprintf(szBuffer, "Error during communication with OPM (Optical Power Meter), maybe no OPM connected");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_WRONG_PARAMNUM): /* -1057 */
        {
            if ((int)strlen("WaveEditor: Error during wave creation, incorrect number of parameters")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Error during wave creation, incorrect number of parameters");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_FREQUENCY_OUT_OF_RANGE): /* -1058 */
        {
            if ((int)strlen("WaveEditor: Frequency out of range")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Frequency out of range");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_WRONG_IP_VALUE): /* -1059 */
        {
            if ((int)strlen("WaveEditor: Error during wave creation, incorrect index for integer parameter")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Error during wave creation, incorrect index for integer parameter");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_WRONG_DP_VALUE): /* -1060 */
        {
            if ((int)strlen("WaveEditor: Error during wave creation, incorrect index for floating point parameter")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Error during wave creation, incorrect index for floating point parameter");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_WRONG_ITEM_VALUE): /* -1061 */
        {
            if ((int)strlen("WaveEditor: Error during wave creation, could not calculate value")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Error during wave creation, could not calculate value");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_MISSING_GRAPH_COMPONENT): /* -1062 */
        {
            if ((int)strlen("WaveEditor: Graph display component not installed")<maxlen)
            {
                sprintf(szBuffer, "WaveEditor: Graph display component not installed");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_UNALLOWED_CMD): /* -1063 */
        {
            if ((int)strlen("User Profile Mode: Command is not allowed, check for required preparatory commands")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Command is not allowed, check for required preparatory commands");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_EXPECTING_MOTION_ERROR): /* -1064 */
        {
            if ((int)strlen("User Profile Mode: First target position in User Profile is too far from current position")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: First target position in User Profile is too far from current position");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_ACTIVE): /* -1065 */
        {
            if ((int)strlen("Controller is (already) in User Profile Mode")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) in User Profile Mode");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_INDEX_OUT_OF_RANGE): /* -1066 */
        {
            if ((int)strlen("User Profile Mode: Block or Data Set index out of allowed range")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Block or Data Set index out of allowed range");
                return TRUE;
            }
            break;
        }
        case(PI_PROFILE_GENERATOR_NO_PROFILE): /* -1067 */
        {
            if ((int)strlen("ProfileGenerator: No profile has been created yet")<maxlen)
            {
                sprintf(szBuffer, "ProfileGenerator: No profile has been created yet");
                return TRUE;
            }
            break;
        }
        case(PI_PROFILE_GENERATOR_OUT_OF_LIMITS): /* -1068 */
        {
            if ((int)strlen("ProfileGenerator: Generated profile exceeds limits of one or both axes")<maxlen)
            {
                sprintf(szBuffer, "ProfileGenerator: Generated profile exceeds limits of one or both axes");
                return TRUE;
            }
            break;
        }
        case(PI_PROFILE_GENERATOR_UNKNOWN_PARAMETER): /* -1069 */
        {
            if ((int)strlen("ProfileGenerator: Unknown parameter ID in Set/Get Parameter command")<maxlen)
            {
                sprintf(szBuffer, "ProfileGenerator: Unknown parameter ID in Set/Get Parameter command");
                return TRUE;
            }
            break;
        }
        case(PI_PROFILE_GENERATOR_PAR_OUT_OF_RANGE): /* -1070 */
        {
            if ((int)strlen("ProfileGenerator: Parameter out of allowed range")<maxlen)
            {
                sprintf(szBuffer, "ProfileGenerator: Parameter out of allowed range");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_OUT_OF_MEMORY): /* -1071 */
        {
            if ((int)strlen("User Profile Mode: Out of memory")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Out of memory");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_WRONG_CLUSTER): /* -1072 */
        {
            if ((int)strlen("User Profile Mode: Cluster is not assigned to this axis")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Cluster is not assigned to this axis");
                return TRUE;
            }
            break;
        }
        case(PI_EXT_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER): /* -1073 */
        {
            if ((int)strlen("Unknown cluster identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown cluster identifier");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_DEVICE_DRIVER_VERSION): /* -1074 */
        {
            if ((int)strlen("The installed device driver doesn't match the required version. Please see the documentation to determine the required device driver version.")<maxlen)
            {
                sprintf(szBuffer, "The installed device driver doesn't match the required version. Please see the documentation to determine the required device driver version.");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_LIBRARY_VERSION): /* -1075 */
        {
            if ((int)strlen("The library used doesn't match the required version. Please see the documentation to determine the required library version.")<maxlen)
            {
                sprintf(szBuffer, "The library used doesn't match the required version. Please see the documentation to determine the required library version.");
                return TRUE;
            }
            break;
        }
        case(PI_INTERFACE_LOCKED): /* -1076 */
        {
            if ((int)strlen("The interface is currently locked by another function. Please try again later.")<maxlen)
            {
                sprintf(szBuffer, "The interface is currently locked by another function. Please try again later.");
                return TRUE;
            }
            break;
        }
        case(PI_PARAM_DAT_FILE_INVALID_VERSION): /* -1077 */
        {
            if ((int)strlen("Version of parameter DAT file does not match the required version. Current files are available at www.pi.ws.")<maxlen)
            {
                sprintf(szBuffer, "Version of parameter DAT file does not match the required version. Current files are available at www.pi.ws.");
                return TRUE;
            }
            break;
        }
        case(PI_CANNOT_WRITE_TO_PARAM_DAT_FILE): /* -1078 */
        {
            if ((int)strlen("Cannot write to parameter DAT file to store user defined stage type.")<maxlen)
            {
                sprintf(szBuffer, "Cannot write to parameter DAT file to store user defined stage type.");
                return TRUE;
            }
            break;
        }
        case(PI_CANNOT_CREATE_PARAM_DAT_FILE): /* -1079 */
        {
            if ((int)strlen("Cannot create parameter DAT file to store user defined stage type.")<maxlen)
            {
                sprintf(szBuffer, "Cannot create parameter DAT file to store user defined stage type.");
                return TRUE;
            }
            break;
        }
        case(PI_PARAM_DAT_FILE_INVALID_REVISION): /* -1080 */
        {
            if ((int)strlen("Parameter DAT file does not have correct revision.")<maxlen)
            {
                sprintf(szBuffer, "Parameter DAT file does not have correct revision.");
                return TRUE;
            }
            break;
        }
        case(PI_USERSTAGES_DAT_FILE_INVALID_REVISION): /* -1081 */
        {
            if ((int)strlen("User stages DAT file does not have correct revision.")<maxlen)
            {
                sprintf(szBuffer, "User stages DAT file does not have correct revision.");
                return TRUE;
            }
            break;
        }
        case(PI_SOFTWARE_TIMEOUT): /* -1082 */
        {
            if ((int)strlen("Timeout Error. Some lengthy operation did not finish within expected time.")<maxlen)
            {
                sprintf(szBuffer, "Timeout Error. Some lengthy operation did not finish within expected time.");
                return TRUE;
            }
            break;
        }
/*
 **  End of Dll Errors
 *************************************************/

/*************************************************
 **
 ** Controller Errors - Errors set by the controller or the GCS DLL
 */
        case(PI_CNTR_NO_ERROR): /* 0 */
        {
            if ((int)strlen("No error")<maxlen)
            {
                sprintf(szBuffer, "No error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PARAM_SYNTAX): /* 1 */
        {
            if ((int)strlen("Parameter syntax error")<maxlen)
            {
                sprintf(szBuffer, "Parameter syntax error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNKNOWN_COMMAND): /* 2 */
        {
            if ((int)strlen("Unknown command")<maxlen)
            {
                sprintf(szBuffer, "Unknown command");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_COMMAND_TOO_LONG): /* 3 */
        {
            if ((int)strlen("Command length out of limits or command buffer overrun")<maxlen)
            {
                sprintf(szBuffer, "Command length out of limits or command buffer overrun");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SCAN_ERROR): /* 4 */
        {
            if ((int)strlen("Error while scanning")<maxlen)
            {
                sprintf(szBuffer, "Error while scanning");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MOVE_WITHOUT_REF_OR_NO_SERVO): /* 5 */
        {
            if ((int)strlen("Unallowable move attempted on unreferenced axis, or move attempted with servo off")<maxlen)
            {
                sprintf(szBuffer, "Unallowable move attempted on unreferenced axis, or move attempted with servo off");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_SGA_PARAM): /* 6 */
        {
            if ((int)strlen("Parameter for SGA not valid")<maxlen)
            {
                sprintf(szBuffer, "Parameter for SGA not valid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_POS_OUT_OF_LIMITS): /* 7 */
        {
            if ((int)strlen("Position out of limits")<maxlen)
            {
                sprintf(szBuffer, "Position out of limits");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_VEL_OUT_OF_LIMITS): /* 8 */
        {
            if ((int)strlen("Velocity out of limits")<maxlen)
            {
                sprintf(szBuffer, "Velocity out of limits");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SET_PIVOT_NOT_POSSIBLE): /* 9 */
        {
            if ((int)strlen("Attempt to set pivot point while U,V and W not all 0")<maxlen)
            {
                sprintf(szBuffer, "Attempt to set pivot point while U,V and W not all 0");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_STOP): /* 10 */
        {
            if ((int)strlen("Controller was stopped by command")<maxlen)
            {
                sprintf(szBuffer, "Controller was stopped by command");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SST_OR_SCAN_RANGE): /* 11 */
        {
            if ((int)strlen("Parameter for SST or for one of the embedded scan algorithms out of range")<maxlen)
            {
                sprintf(szBuffer, "Parameter for SST or for one of the embedded scan algorithms out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_SCAN_AXES): /* 12 */
        {
            if ((int)strlen("Invalid axis combination for fast scan")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis combination for fast scan");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_NAV_PARAM): /* 13 */
        {
            if ((int)strlen("Parameter for NAV out of range")<maxlen)
            {
                sprintf(szBuffer, "Parameter for NAV out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_ANALOG_INPUT): /* 14 */
        {
            if ((int)strlen("Invalid analog channel")<maxlen)
            {
                sprintf(szBuffer, "Invalid analog channel");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_AXIS_IDENTIFIER): /* 15 */
        {
            if ((int)strlen("Invalid axis identifier")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis identifier");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_STAGE_NAME): /* 16 */
        {
            if ((int)strlen("Invalid stage name")<maxlen)
            {
                sprintf(szBuffer, "Invalid stage name");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PARAM_OUT_OF_RANGE): /* 17 */
        {
            if ((int)strlen("Parameter out of range")<maxlen)
            {
                sprintf(szBuffer, "Parameter out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_MACRO_NAME): /* 18 */
        {
            if ((int)strlen("Invalid macro name")<maxlen)
            {
                sprintf(szBuffer, "Invalid macro name");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_RECORD): /* 19 */
        {
            if ((int)strlen("Error while recording macro")<maxlen)
            {
                sprintf(szBuffer, "Error while recording macro");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_NOT_FOUND): /* 20 */
        {
            if ((int)strlen("Macro not found")<maxlen)
            {
                sprintf(szBuffer, "Macro not found");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AXIS_HAS_NO_BRAKE): /* 21 */
        {
            if ((int)strlen("Axis has no brake")<maxlen)
            {
                sprintf(szBuffer, "Axis has no brake");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DOUBLE_AXIS): /* 22 */
        {
            if ((int)strlen("Axis identifier specified more than once")<maxlen)
            {
                sprintf(szBuffer, "Axis identifier specified more than once");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ILLEGAL_AXIS): /* 23 */
        {
            if ((int)strlen("Illegal axis")<maxlen)
            {
                sprintf(szBuffer, "Illegal axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PARAM_NR): /* 24 */
        {
            if ((int)strlen("Incorrect number of parameters")<maxlen)
            {
                sprintf(szBuffer, "Incorrect number of parameters");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_REAL_NR): /* 25 */
        {
            if ((int)strlen("Invalid floating point number")<maxlen)
            {
                sprintf(szBuffer, "Invalid floating point number");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MISSING_PARAM): /* 26 */
        {
            if ((int)strlen("Parameter missing")<maxlen)
            {
                sprintf(szBuffer, "Parameter missing");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SOFT_LIMIT_OUT_OF_RANGE): /* 27 */
        {
            if ((int)strlen("Soft limit out of range")<maxlen)
            {
                sprintf(szBuffer, "Soft limit out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_MANUAL_PAD): /* 28 */
        {
            if ((int)strlen("No manual pad found")<maxlen)
            {
                sprintf(szBuffer, "No manual pad found");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_JUMP): /* 29 */
        {
            if ((int)strlen("No more step-response values")<maxlen)
            {
                sprintf(szBuffer, "No more step-response values");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_JUMP): /* 30 */
        {
            if ((int)strlen("No step-response values recorded")<maxlen)
            {
                sprintf(szBuffer, "No step-response values recorded");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AXIS_HAS_NO_REFERENCE): /* 31 */
        {
            if ((int)strlen("Axis has no reference sensor")<maxlen)
            {
                sprintf(szBuffer, "Axis has no reference sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_STAGE_HAS_NO_LIM_SWITCH): /* 32 */
        {
            if ((int)strlen("Axis has no limit switch")<maxlen)
            {
                sprintf(szBuffer, "Axis has no limit switch");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_RELAY_CARD): /* 33 */
        {
            if ((int)strlen("No relay card installed")<maxlen)
            {
                sprintf(szBuffer, "No relay card installed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CMD_NOT_ALLOWED_FOR_STAGE): /* 34 */
        {
            if ((int)strlen("Command not allowed for selected stage(s)")<maxlen)
            {
                sprintf(szBuffer, "Command not allowed for selected stage(s)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_DIGITAL_INPUT): /* 35 */
        {
            if ((int)strlen("No digital input installed")<maxlen)
            {
                sprintf(szBuffer, "No digital input installed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_DIGITAL_OUTPUT): /* 36 */
        {
            if ((int)strlen("No digital output configured")<maxlen)
            {
                sprintf(szBuffer, "No digital output configured");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_MCM): /* 37 */
        {
            if ((int)strlen("No more MCM responses")<maxlen)
            {
                sprintf(szBuffer, "No more MCM responses");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_MCM): /* 38 */
        {
            if ((int)strlen("No MCM values recorded")<maxlen)
            {
                sprintf(szBuffer, "No MCM values recorded");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_CNTR_NUMBER): /* 39 */
        {
            if ((int)strlen("Controller number invalid")<maxlen)
            {
                sprintf(szBuffer, "Controller number invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_JOYSTICK_CONNECTED): /* 40 */
        {
            if ((int)strlen("No joystick configured")<maxlen)
            {
                sprintf(szBuffer, "No joystick configured");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_EGE_AXIS): /* 41 */
        {
            if ((int)strlen("Invalid axis for electronic gearing, axis can not be slave")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis for electronic gearing, axis can not be slave");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SLAVE_POSITION_OUT_OF_RANGE): /* 42 */
        {
            if ((int)strlen("Position of slave axis is out of range")<maxlen)
            {
                sprintf(szBuffer, "Position of slave axis is out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_COMMAND_EGE_SLAVE): /* 43 */
        {
            if ((int)strlen("Slave axis cannot be commanded directly when electronic gearing is enabled")<maxlen)
            {
                sprintf(szBuffer, "Slave axis cannot be commanded directly when electronic gearing is enabled");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_JOYSTICK_CALIBRATION_FAILED): /* 44 */
        {
            if ((int)strlen("Calibration of joystick failed")<maxlen)
            {
                sprintf(szBuffer, "Calibration of joystick failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_REFERENCING_FAILED): /* 45 */
        {
            if ((int)strlen("Referencing failed")<maxlen)
            {
                sprintf(szBuffer, "Referencing failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPM_MISSING): /* 46 */
        {
            if ((int)strlen("OPM (Optical Power Meter) missing")<maxlen)
            {
                sprintf(szBuffer, "OPM (Optical Power Meter) missing");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPM_NOT_INITIALIZED): /* 47 */
        {
            if ((int)strlen("OPM (Optical Power Meter) not initialized or cannot be initialized")<maxlen)
            {
                sprintf(szBuffer, "OPM (Optical Power Meter) not initialized or cannot be initialized");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPM_COM_ERROR): /* 48 */
        {
            if ((int)strlen("OPM (Optical Power Meter) Communication Error")<maxlen)
            {
                sprintf(szBuffer, "OPM (Optical Power Meter) Communication Error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MOVE_TO_LIMIT_SWITCH_FAILED): /* 49 */
        {
            if ((int)strlen("Move to limit switch failed")<maxlen)
            {
                sprintf(szBuffer, "Move to limit switch failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_REF_WITH_REF_DISABLED): /* 50 */
        {
            if ((int)strlen("Attempt to reference axis with referencing disabled")<maxlen)
            {
                sprintf(szBuffer, "Attempt to reference axis with referencing disabled");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AXIS_UNDER_JOYSTICK_CONTROL): /* 51 */
        {
            if ((int)strlen("Selected axis is controlled by joystick")<maxlen)
            {
                sprintf(szBuffer, "Selected axis is controlled by joystick");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_COMMUNICATION_ERROR): /* 52 */
        {
            if ((int)strlen("Controller detected communication error")<maxlen)
            {
                sprintf(szBuffer, "Controller detected communication error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DYNAMIC_MOVE_IN_PROCESS): /* 53 */
        {
            if ((int)strlen("MOV! motion still in progress")<maxlen)
            {
                sprintf(szBuffer, "MOV! motion still in progress");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNKNOWN_PARAMETER): /* 54 */
        {
            if ((int)strlen("Unknown parameter")<maxlen)
            {
                sprintf(szBuffer, "Unknown parameter");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_REP_RECORDED): /* 55 */
        {
            if ((int)strlen("No commands were recorded with REP")<maxlen)
            {
                sprintf(szBuffer, "No commands were recorded with REP");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_PASSWORD): /* 56 */
        {
            if ((int)strlen("Password invalid")<maxlen)
            {
                sprintf(szBuffer, "Password invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_RECORDER_CHAN): /* 57 */
        {
            if ((int)strlen("Data Record Table does not exist")<maxlen)
            {
                sprintf(szBuffer, "Data Record Table does not exist");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_RECORDER_SRC_OPT): /* 58 */
        {
            if ((int)strlen("Source does not exist; number too low or too high")<maxlen)
            {
                sprintf(szBuffer, "Source does not exist; number too low or too high");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_RECORDER_SRC_CHAN): /* 59 */
        {
            if ((int)strlen("Source Record Table number too low or too high")<maxlen)
            {
                sprintf(szBuffer, "Source Record Table number too low or too high");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PARAM_PROTECTION): /* 60 */
        {
            if ((int)strlen("Protected Param: current Command Level (CCL) too low")<maxlen)
            {
                sprintf(szBuffer, "Protected Param: current Command Level (CCL) too low");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AUTOZERO_RUNNING): /* 61 */
        {
            if ((int)strlen("Command execution not possible while Autozero is running")<maxlen)
            {
                sprintf(szBuffer, "Command execution not possible while Autozero is running");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_LINEAR_AXIS): /* 62 */
        {
            if ((int)strlen("Autozero requires at least one linear axis")<maxlen)
            {
                sprintf(szBuffer, "Autozero requires at least one linear axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INIT_RUNNING): /* 63 */
        {
            if ((int)strlen("Initialization still in progress")<maxlen)
            {
                sprintf(szBuffer, "Initialization still in progress");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_READ_ONLY_PARAMETER): /* 64 */
        {
            if ((int)strlen("Parameter is read-only")<maxlen)
            {
                sprintf(szBuffer, "Parameter is read-only");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PAM_NOT_FOUND): /* 65 */
        {
            if ((int)strlen("Parameter not found in non-volatile memory")<maxlen)
            {
                sprintf(szBuffer, "Parameter not found in non-volatile memory");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_VOL_OUT_OF_LIMITS): /* 66 */
        {
            if ((int)strlen("Voltage out of limits")<maxlen)
            {
                sprintf(szBuffer, "Voltage out of limits");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAVE_TOO_LARGE): /* 67 */
        {
            if ((int)strlen("Not enough memory available for requested wave curve")<maxlen)
            {
                sprintf(szBuffer, "Not enough memory available for requested wave curve");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NOT_ENOUGH_DDL_MEMORY): /* 68 */
        {
            if ((int)strlen("Not enough memory available for DDL table; DDL can not be started")<maxlen)
            {
                sprintf(szBuffer, "Not enough memory available for DDL table; DDL can not be started");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DDL_TIME_DELAY_TOO_LARGE): /* 69 */
        {
            if ((int)strlen("Time delay larger than DDL table; DDL can not be started")<maxlen)
            {
                sprintf(szBuffer, "Time delay larger than DDL table; DDL can not be started");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DIFFERENT_ARRAY_LENGTH): /* 70 */
        {
            if ((int)strlen("The requested arrays have different lengths; query them separately")<maxlen)
            {
                sprintf(szBuffer, "The requested arrays have different lengths; query them separately");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_GEN_SINGLE_MODE_RESTART): /* 71 */
        {
            if ((int)strlen("Attempt to restart the generator while it is running in single step mode")<maxlen)
            {
                sprintf(szBuffer, "Attempt to restart the generator while it is running in single step mode");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ANALOG_TARGET_ACTIVE): /* 72 */
        {
            if ((int)strlen("Motion commands and wave generator activation are not allowed when analog target is active")<maxlen)
            {
                sprintf(szBuffer, "Motion commands and wave generator activation are not allowed when analog target is active");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAVE_GENERATOR_ACTIVE): /* 73 */
        {
            if ((int)strlen("Motion commands are not allowed when wave generator output is active; use WGO to disable generator output")<maxlen)
            {
                sprintf(szBuffer, "Motion commands are not allowed when wave generator output is active; use WGO to disable generator output");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AUTOZERO_DISABLED): /* 74 */
        {
            if ((int)strlen("No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix)")<maxlen)
            {
                sprintf(szBuffer, "No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_WAVE_SELECTED): /* 75 */
        {
            if ((int)strlen("Generator started (WGO) without having selected a wave table (WSL).")<maxlen)
            {
                sprintf(szBuffer, "Generator started (WGO) without having selected a wave table (WSL).");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_IF_BUFFER_OVERRUN): /* 76 */
        {
            if ((int)strlen("Interface buffer did overrun and command couldn't be received correctly")<maxlen)
            {
                sprintf(szBuffer, "Interface buffer did overrun and command couldn't be received correctly");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NOT_ENOUGH_RECORDED_DATA): /* 77 */
        {
            if ((int)strlen("Data Record Table does not hold enough recorded data")<maxlen)
            {
                sprintf(szBuffer, "Data Record Table does not hold enough recorded data");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_TABLE_DEACTIVATED): /* 78 */
        {
            if ((int)strlen("Data Record Table is not configured for recording")<maxlen)
            {
                sprintf(szBuffer, "Data Record Table is not configured for recording");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPENLOOP_VALUE_SET_WHEN_SERVO_ON): /* 79 */
        {
            if ((int)strlen("Open-loop commands (SVA, SVR) are not allowed when servo is on")<maxlen)
            {
                sprintf(szBuffer, "Open-loop commands (SVA, SVR) are not allowed when servo is on");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RAM_ERROR): /* 80 */
        {
            if ((int)strlen("Hardware error affecting RAM")<maxlen)
            {
                sprintf(szBuffer, "Hardware error affecting RAM");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_UNKNOWN_COMMAND): /* 81 */
        {
            if ((int)strlen("Not macro command")<maxlen)
            {
                sprintf(szBuffer, "Not macro command");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_PC_ERROR): /* 82 */
        {
            if ((int)strlen("Macro counter out of range")<maxlen)
            {
                sprintf(szBuffer, "Macro counter out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_JOYSTICK_ACTIVE): /* 83 */
        {
            if ((int)strlen("Joystick is active")<maxlen)
            {
                sprintf(szBuffer, "Joystick is active");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MOTOR_IS_OFF): /* 84 */
        {
            if ((int)strlen("Motor is off")<maxlen)
            {
                sprintf(szBuffer, "Motor is off");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ONLY_IN_MACRO): /* 85 */
        {
            if ((int)strlen("Macro-only command")<maxlen)
            {
                sprintf(szBuffer, "Macro-only command");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_JOYSTICK_UNKNOWN_AXIS): /* 86 */
        {
            if ((int)strlen("Invalid joystick axis")<maxlen)
            {
                sprintf(szBuffer, "Invalid joystick axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_JOYSTICK_UNKNOWN_ID): /* 87 */
        {
            if ((int)strlen("Joystick unknown")<maxlen)
            {
                sprintf(szBuffer, "Joystick unknown");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_REF_MODE_IS_ON): /* 88 */
        {
            if ((int)strlen("Move without referenced stage")<maxlen)
            {
                sprintf(szBuffer, "Move without referenced stage");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NOT_ALLOWED_IN_CURRENT_MOTION_MODE): /* 89 */
        {
            if ((int)strlen("Command not allowed in current motion mode")<maxlen)
            {
                sprintf(szBuffer, "Command not allowed in current motion mode");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DIO_AND_TRACING_NOT_POSSIBLE): /* 90 */
        {
            if ((int)strlen("No tracing possible while digital IOs are used on this HW revision. Reconnect to switch operation mode.")<maxlen)
            {
                sprintf(szBuffer, "No tracing possible while digital IOs are used on this HW revision. Reconnect to switch operation mode.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_COLLISION): /* 91 */
        {
            if ((int)strlen("Move not possible, would cause collision")<maxlen)
            {
                sprintf(szBuffer, "Move not possible, would cause collision");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SLAVE_NOT_FAST_ENOUGH): /* 92 */
        {
            if ((int)strlen("Stage is not capable of following the master. Check the gear ratio(SRA).")<maxlen)
            {
                sprintf(szBuffer, "Stage is not capable of following the master. Check the gear ratio(SRA).");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CMD_NOT_ALLOWED_WHILE_AXIS_IN_MOTION): /* 93 */
        {
            if ((int)strlen("This command is not allowed while the affected axis or its master is in motion.")<maxlen)
            {
                sprintf(szBuffer, "This command is not allowed while the affected axis or its master is in motion.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPEN_LOOP_JOYSTICK_ENABLED): /* 94 */
        {
            if ((int)strlen("Servo cannot be switched on when open-loop joystick control is enabled.")<maxlen)
            {
                sprintf(szBuffer, "Servo cannot be switched on when open-loop joystick control is enabled.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_SERVO_STATE_FOR_PARAMETER): /* 95 */
        {
            if ((int)strlen("This parameter cannot be changed in current servo mode.")<maxlen)
            {
                sprintf(szBuffer, "This parameter cannot be changed in current servo mode.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNKNOWN_STAGE_NAME): /* 96 */
        {
            if ((int)strlen("Unknown stage name")<maxlen)
            {
                sprintf(szBuffer, "Unknown stage name");
                return TRUE;
            }
            break;
        }
        case(PI_LABVIEW_ERROR): /* 100 */
        {
            if ((int)strlen("PI LabVIEW driver reports error. See source control for details.")<maxlen)
            {
                sprintf(szBuffer, "PI LabVIEW driver reports error. See source control for details.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_AXIS): /* 200 */
        {
            if ((int)strlen("No stage connected to axis")<maxlen)
            {
                sprintf(szBuffer, "No stage connected to axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_AXIS_PARAM_FILE): /* 201 */
        {
            if ((int)strlen("File with axis parameters not found")<maxlen)
            {
                sprintf(szBuffer, "File with axis parameters not found");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_AXIS_PARAM_FILE): /* 202 */
        {
            if ((int)strlen("Invalid axis parameter file")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis parameter file");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_AXIS_PARAM_BACKUP): /* 203 */
        {
            if ((int)strlen("Backup file with axis parameters not found")<maxlen)
            {
                sprintf(szBuffer, "Backup file with axis parameters not found");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RESERVED_204): /* 204 */
        {
            if ((int)strlen("PI internal error code 204")<maxlen)
            {
                sprintf(szBuffer, "PI internal error code 204");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SMO_WITH_SERVO_ON): /* 205 */
        {
            if ((int)strlen("SMO with servo on")<maxlen)
            {
                sprintf(szBuffer, "SMO with servo on");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UUDECODE_INCOMPLETE_HEADER): /* 206 */
        {
            if ((int)strlen("uudecode: incomplete header")<maxlen)
            {
                sprintf(szBuffer, "uudecode: incomplete header");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UUDECODE_NOTHING_TO_DECODE): /* 207 */
        {
            if ((int)strlen("uudecode: nothing to decode")<maxlen)
            {
                sprintf(szBuffer, "uudecode: nothing to decode");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UUDECODE_ILLEGAL_FORMAT): /* 208 */
        {
            if ((int)strlen("uudecode: illegal UUE format")<maxlen)
            {
                sprintf(szBuffer, "uudecode: illegal UUE format");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CRC32_ERROR): /* 209 */
        {
            if ((int)strlen("CRC32 error")<maxlen)
            {
                sprintf(szBuffer, "CRC32 error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ILLEGAL_FILENAME): /* 210 */
        {
            if ((int)strlen("Illegal file name (must be 8-0 format)")<maxlen)
            {
                sprintf(szBuffer, "Illegal file name (must be 8-0 format)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FILE_NOT_FOUND): /* 211 */
        {
            if ((int)strlen("File not found on controller")<maxlen)
            {
                sprintf(szBuffer, "File not found on controller");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FILE_WRITE_ERROR): /* 212 */
        {
            if ((int)strlen("Error writing file on controller")<maxlen)
            {
                sprintf(szBuffer, "Error writing file on controller");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_DTR_HINDERS_VELOCITY_CHANGE): /* 213 */
        {
            if ((int)strlen("VEL command not allowed in DTR Command Mode")<maxlen)
            {
                sprintf(szBuffer, "VEL command not allowed in DTR Command Mode");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_POSITION_UNKNOWN): /* 214 */
        {
            if ((int)strlen("Position calculations failed")<maxlen)
            {
                sprintf(szBuffer, "Position calculations failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CONN_POSSIBLY_BROKEN): /* 215 */
        {
            if ((int)strlen("The connection between controller and stage may be broken")<maxlen)
            {
                sprintf(szBuffer, "The connection between controller and stage may be broken");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ON_LIMIT_SWITCH): /* 216 */
        {
            if ((int)strlen("The connected stage has driven into a limit switch, some controllers need CLR to resume operation")<maxlen)
            {
                sprintf(szBuffer, "The connected stage has driven into a limit switch, some controllers need CLR to resume operation");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNEXPECTED_STRUT_STOP): /* 217 */
        {
            if ((int)strlen("Strut test command failed because of an unexpected strut stop")<maxlen)
            {
                sprintf(szBuffer, "Strut test command failed because of an unexpected strut stop");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_POSITION_BASED_ON_ESTIMATION): /* 218 */
        {
            if ((int)strlen("While MOV! is running position can only be estimated!")<maxlen)
            {
                sprintf(szBuffer, "While MOV! is running position can only be estimated!");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_POSITION_BASED_ON_INTERPOLATION): /* 219 */
        {
            if ((int)strlen("Position was calculated during MOV motion")<maxlen)
            {
                sprintf(szBuffer, "Position was calculated during MOV motion");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_HANDLE): /* 230 */
        {
            if ((int)strlen("Invalid handle")<maxlen)
            {
                sprintf(szBuffer, "Invalid handle");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_BIOS_FOUND): /* 231 */
        {
            if ((int)strlen("No bios found")<maxlen)
            {
                sprintf(szBuffer, "No bios found");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SAVE_SYS_CFG_FAILED): /* 232 */
        {
            if ((int)strlen("Save system configuration failed")<maxlen)
            {
                sprintf(szBuffer, "Save system configuration failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_LOAD_SYS_CFG_FAILED): /* 233 */
        {
            if ((int)strlen("Load system configuration failed")<maxlen)
            {
                sprintf(szBuffer, "Load system configuration failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SEND_BUFFER_OVERFLOW): /* 301 */
        {
            if ((int)strlen("Send buffer overflow")<maxlen)
            {
                sprintf(szBuffer, "Send buffer overflow");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_VOLTAGE_OUT_OF_LIMITS): /* 302 */
        {
            if ((int)strlen("Voltage out of limits")<maxlen)
            {
                sprintf(szBuffer, "Voltage out of limits");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_OPEN_LOOP_MOTION_SET_WHEN_SERVO_ON): /* 303 */
        {
            if ((int)strlen("Open-loop motion attempted when servo ON")<maxlen)
            {
                sprintf(szBuffer, "Open-loop motion attempted when servo ON");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RECEIVING_BUFFER_OVERFLOW): /* 304 */
        {
            if ((int)strlen("Received command is too long")<maxlen)
            {
                sprintf(szBuffer, "Received command is too long");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EEPROM_ERROR): /* 305 */
        {
            if ((int)strlen("Error while reading/writing EEPROM")<maxlen)
            {
                sprintf(szBuffer, "Error while reading/writing EEPROM");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_I2C_ERROR): /* 306 */
        {
            if ((int)strlen("Error on I2C bus")<maxlen)
            {
                sprintf(szBuffer, "Error on I2C bus");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RECEIVING_TIMEOUT): /* 307 */
        {
            if ((int)strlen("Timeout while receiving command")<maxlen)
            {
                sprintf(szBuffer, "Timeout while receiving command");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_TIMEOUT): /* 308 */
        {
            if ((int)strlen("A lengthy operation has not finished in the expected time")<maxlen)
            {
                sprintf(szBuffer, "A lengthy operation has not finished in the expected time");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_OUT_OF_SPACE): /* 309 */
        {
            if ((int)strlen("Insufficient space to store macro")<maxlen)
            {
                sprintf(szBuffer, "Insufficient space to store macro");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EUI_OLDVERSION_CFGDATA): /* 310 */
        {
            if ((int)strlen("Configuration data has old version number")<maxlen)
            {
                sprintf(szBuffer, "Configuration data has old version number");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EUI_INVALID_CFGDATA): /* 311 */
        {
            if ((int)strlen("Invalid configuration data")<maxlen)
            {
                sprintf(szBuffer, "Invalid configuration data");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HARDWARE_ERROR): /* 333 */
        {
            if ((int)strlen("Internal hardware error")<maxlen)
            {
                sprintf(szBuffer, "Internal hardware error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_INDEX_ERROR): /* 400 */
        {
            if ((int)strlen("Wave generator index error")<maxlen)
            {
                sprintf(szBuffer, "Wave generator index error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_NOT_DEFINED): /* 401 */
        {
            if ((int)strlen("Wave table not defined")<maxlen)
            {
                sprintf(szBuffer, "Wave table not defined");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_TYPE_NOT_SUPPORTED): /* 402 */
        {
            if ((int)strlen("Wave type not supported")<maxlen)
            {
                sprintf(szBuffer, "Wave type not supported");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_LENGTH_EXCEEDS_LIMIT): /* 403 */
        {
            if ((int)strlen("Wave length exceeds limit")<maxlen)
            {
                sprintf(szBuffer, "Wave length exceeds limit");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_PARAMETER_NR): /* 404 */
        {
            if ((int)strlen("Wave parameter number error")<maxlen)
            {
                sprintf(szBuffer, "Wave parameter number error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WAV_PARAMETER_OUT_OF_LIMIT): /* 405 */
        {
            if ((int)strlen("Wave parameter out of range")<maxlen)
            {
                sprintf(szBuffer, "Wave parameter out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_WGO_BIT_NOT_SUPPORTED): /* 406 */
        {
            if ((int)strlen("WGO command bit not supported")<maxlen)
            {
                sprintf(szBuffer, "WGO command bit not supported");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EMERGENCY_STOP_BUTTON_ACTIVATED): /* 500 */
        {
            if ((int)strlen("The \"red knob\" is still set and disables system")<maxlen)
            {
                sprintf(szBuffer, "The \"red knob\" is still set and disables system");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EMERGENCY_STOP_BUTTON_WAS_ACTIVATED): /* 501 */
        {
            if ((int)strlen("The \"red knob\" was activated and still disables system - reanimation required")<maxlen)
            {
                sprintf(szBuffer, "The \"red knob\" was activated and still disables system - reanimation required");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_REDUNDANCY_LIMIT_EXCEEDED): /* 502 */
        {
            if ((int)strlen("Position consistency check failed")<maxlen)
            {
                sprintf(szBuffer, "Position consistency check failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_COLLISION_SWITCH_ACTIVATED): /* 503 */
        {
            if ((int)strlen("Hardware collision sensor(s) are activated")<maxlen)
            {
                sprintf(szBuffer, "Hardware collision sensor(s) are activated");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FOLLOWING_ERROR): /* 504 */
        {
            if ((int)strlen("Strut following error occurred, e.g. caused by overload or encoder failure")<maxlen)
            {
                sprintf(szBuffer, "Strut following error occurred, e.g. caused by overload or encoder failure");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_SIGNAL_INVALID): /* 505 */
        {
            if ((int)strlen("One sensor signal is not valid")<maxlen)
            {
                sprintf(szBuffer, "One sensor signal is not valid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SERVO_LOOP_UNSTABLE): /* 506 */
        {
            if ((int)strlen("Servo loop was unstable due to wrong parameter setting and switched off to avoid damage.")<maxlen)
            {
                sprintf(szBuffer, "Servo loop was unstable due to wrong parameter setting and switched off to avoid damage.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_DOES_NOT_EXIST): /* 530 */
        {
            if ((int)strlen("A command refers to a node that does not exist")<maxlen)
            {
                sprintf(szBuffer, "A command refers to a node that does not exist");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PARENT_NODE_DOES_NOT_EXIST): /* 531 */
        {
            if ((int)strlen("A command refers to a node that has no parent node")<maxlen)
            {
                sprintf(szBuffer, "A command refers to a node that has no parent node");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_IN_USE): /* 532 */
        {
            if ((int)strlen("Attempt to delete a node that is in use")<maxlen)
            {
                sprintf(szBuffer, "Attempt to delete a node that is in use");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_DEFINITION_IS_CYCLIC): /* 533 */
        {
            if ((int)strlen("Definition of a node is cyclic")<maxlen)
            {
                sprintf(szBuffer, "Definition of a node is cyclic");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_CHAIN_INVALID): /* 534 */
        {
            if ((int)strlen("The node chain does not end in the \"0\" node")<maxlen)
            {
                sprintf(szBuffer, "The node chain does not end in the \"0\" node");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_DEFINITION_NOT_CONSISTENT): /* 535 */
        {
            if ((int)strlen("The definition of a coordinate transformation is erroneous")<maxlen)
            {
                sprintf(szBuffer, "The definition of a coordinate transformation is erroneous");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HEXAPOD_IN_MOTION): /* 536 */
        {
            if ((int)strlen("Transformation cannot be defined as long as Hexapod is in motion")<maxlen)
            {
                sprintf(szBuffer, "Transformation cannot be defined as long as Hexapod is in motion");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_TRANSFORMATION_TYPE_NOT_SUPPORTED): /* 537 */
        {
            if ((int)strlen("Transformation node cannot be activated")<maxlen)
            {
                sprintf(szBuffer, "Transformation node cannot be activated");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_TYPE_DIFFERS): /* 538 */
        {
            if ((int)strlen("A node can only be replaced by a node of the same type")<maxlen)
            {
                sprintf(szBuffer, "A node can only be replaced by a node of the same type");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_PARENT_IDENTICAL_TO_CHILD): /* 539 */
        {
            if ((int)strlen("A node cannot be linked to itself")<maxlen)
            {
                sprintf(szBuffer, "A node cannot be linked to itself");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_DEFINITION_INCONSISTENT): /* 540 */
        {
            if ((int)strlen("Node definition is erroneous or not complete (replace or delete it)")<maxlen)
            {
                sprintf(szBuffer, "Node definition is erroneous or not complete (replace or delete it)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ZERO_NODE_CANNOT_BE_CHANGED_OR_REPLACED): /* 541 */
        {
            if ((int)strlen("0 is the root node and cannot be modified")<maxlen)
            {
                sprintf(szBuffer, "0 is the root node and cannot be modified");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODES_NOT_IN_SAME_CHAIN): /* 542 */
        {
            if ((int)strlen("The nodes are not part of the same chain")<maxlen)
            {
                sprintf(szBuffer, "The nodes are not part of the same chain");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NODE_MEMORY_FULL): /* 543 */
        {
            if ((int)strlen("Unused nodes must be deleted before new nodes can be stored")<maxlen)
            {
                sprintf(szBuffer, "Unused nodes must be deleted before new nodes can be stored");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PIVOT_POINT_FEATURE_NOT_SUPPORTED): /* 544 */
        {
            if ((int)strlen("With some transformations pivot point usage is not supported")<maxlen)
            {
                sprintf(szBuffer, "With some transformations pivot point usage is not supported");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNKNOWN_ERROR): /* 555 */
        {
            if ((int)strlen("BasMac: unknown controller error")<maxlen)
            {
                sprintf(szBuffer, "BasMac: unknown controller error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NOT_ENOUGH_MEMORY): /* 601 */
        {
            if ((int)strlen("Not enough memory")<maxlen)
            {
                sprintf(szBuffer, "Not enough memory");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HW_VOLTAGE_ERROR): /* 602 */
        {
            if ((int)strlen("Hardware voltage error")<maxlen)
            {
                sprintf(szBuffer, "Hardware voltage error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HW_TEMPERATURE_ERROR): /* 603 */
        {
            if ((int)strlen("Hardware temperature out of range")<maxlen)
            {
                sprintf(szBuffer, "Hardware temperature out of range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_POSITION_ERROR_TOO_HIGH): /* 604 */
        {
            if ((int)strlen("Position error of any axis in the system is too high")<maxlen)
            {
                sprintf(szBuffer, "Position error of any axis in the system is too high");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INPUT_OUT_OF_RANGE): /* 606 */
        {
            if ((int)strlen("Maximum value of input signal has been exceeded")<maxlen)
            {
                sprintf(szBuffer, "Maximum value of input signal has been exceeded");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_INTEGER): /* 607 */
        {
            if ((int)strlen("Value is not integer")<maxlen)
            {
                sprintf(szBuffer, "Value is not integer");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_TOO_MANY_NESTED_MACROS): /* 1000 */
        {
            if ((int)strlen("Too many nested macros")<maxlen)
            {
                sprintf(szBuffer, "Too many nested macros");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_ALREADY_DEFINED): /* 1001 */
        {
            if ((int)strlen("Macro already defined")<maxlen)
            {
                sprintf(szBuffer, "Macro already defined");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NO_MACRO_RECORDING): /* 1002 */
        {
            if ((int)strlen("Macro recording not activated")<maxlen)
            {
                sprintf(szBuffer, "Macro recording not activated");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_MAC_PARAM): /* 1003 */
        {
            if ((int)strlen("Invalid parameter for MAC")<maxlen)
            {
                sprintf(szBuffer, "Invalid parameter for MAC");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_DELETE_ERROR): /* 1004 */
        {
            if ((int)strlen("Deleting macro failed")<maxlen)
            {
                sprintf(szBuffer, "Deleting macro failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CONTROLLER_BUSY): /* 1005 */
        {
            if ((int)strlen("Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)")<maxlen)
            {
                sprintf(szBuffer, "Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_IDENTIFIER): /* 1006 */
        {
            if ((int)strlen("Invalid identifier (invalid special characters, ...)")<maxlen)
            {
                sprintf(szBuffer, "Invalid identifier (invalid special characters, ...)");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_UNKNOWN_VARIABLE_OR_ARGUMENT): /* 1007 */
        {
            if ((int)strlen("Variable or argument not defined")<maxlen)
            {
                sprintf(szBuffer, "Variable or argument not defined");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RUNNING_MACRO): /* 1008 */
        {
            if ((int)strlen("Controller is (already) running a macro")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) running a macro");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_INVALID_OPERATOR): /* 1009 */
        {
            if ((int)strlen("Invalid or missing operator for condition. Check necessary spaces around operator.")<maxlen)
            {
                sprintf(szBuffer, "Invalid or missing operator for condition. Check necessary spaces around operator.");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MACRO_NO_ANSWER): /* 1010 */
        {
            if ((int)strlen("No answer was received while executing WAC/MEX/JRC/...")<maxlen)
            {
                sprintf(szBuffer, "No answer was received while executing WAC/MEX/JRC/...");
                return TRUE;
            }
            break;
        }
        case(PI_CMD_NOT_VALID_IN_MACRO_MODE): /* 1011 */
        {
            if ((int)strlen("Command not valid during macro execution")<maxlen)
            {
                sprintf(szBuffer, "Command not valid during macro execution");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_MOTION_ERROR): /* 1024 */
        {
            if ((int)strlen("Motion error: position error too large, servo is switched off automatically")<maxlen)
            {
                sprintf(szBuffer, "Motion error: position error too large, servo is switched off automatically");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EXT_PROFILE_UNALLOWED_CMD): /* 1063 */
        {
            if ((int)strlen("User Profile Mode: Command is not allowed, check for required preparatory commands")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Command is not allowed, check for required preparatory commands");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_EXT_PROFILE_EXPECTING_MOTION_ERROR): /* 1064 */
        {
            if ((int)strlen("User Profile Mode: First target position in User Profile is too far from current position")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: First target position in User Profile is too far from current position");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PROFILE_ACTIVE): /* 1065 */
        {
            if ((int)strlen("Controller is (already) in User Profile Mode")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) in User Profile Mode");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PROFILE_INDEX_OUT_OF_RANGE): /* 1066 */
        {
            if ((int)strlen("User Profile Mode: Block or Data Set index out of allowed range")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Block or Data Set index out of allowed range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PROFILE_OUT_OF_MEMORY): /* 1071 */
        {
            if ((int)strlen("User Profile Mode: Out of memory")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Out of memory");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PROFILE_WRONG_CLUSTER): /* 1072 */
        {
            if ((int)strlen("User Profile Mode: Cluster is not assigned to this axis")<maxlen)
            {
                sprintf(szBuffer, "User Profile Mode: Cluster is not assigned to this axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER): /* 1073 */
        {
            if ((int)strlen("Unknown cluster identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown cluster identifier");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_TOO_MANY_TCP_CONNECTIONS_OPEN): /* 1090 */
        {
            if ((int)strlen("There are too many open tcpip connections")<maxlen)
            {
                sprintf(szBuffer, "There are too many open tcpip connections");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_ALREADY_HAS_SERIAL_NUMBER): /* 2000 */
        {
            if ((int)strlen("Controller already has a serial number")<maxlen)
            {
                sprintf(szBuffer, "Controller already has a serial number");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SECTOR_ERASE_FAILED): /* 4000 */
        {
            if ((int)strlen("Sector erase failed")<maxlen)
            {
                sprintf(szBuffer, "Sector erase failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FLASH_PROGRAM_FAILED): /* 4001 */
        {
            if ((int)strlen("Flash program failed")<maxlen)
            {
                sprintf(szBuffer, "Flash program failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FLASH_READ_FAILED): /* 4002 */
        {
            if ((int)strlen("Flash read failed")<maxlen)
            {
                sprintf(szBuffer, "Flash read failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HW_MATCHCODE_ERROR): /* 4003 */
        {
            if ((int)strlen("HW match code missing/invalid")<maxlen)
            {
                sprintf(szBuffer, "HW match code missing/invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FW_MATCHCODE_ERROR): /* 4004 */
        {
            if ((int)strlen("FW match code missing/invalid")<maxlen)
            {
                sprintf(szBuffer, "FW match code missing/invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_HW_VERSION_ERROR): /* 4005 */
        {
            if ((int)strlen("HW version missing/invalid")<maxlen)
            {
                sprintf(szBuffer, "HW version missing/invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FW_VERSION_ERROR): /* 4006 */
        {
            if ((int)strlen("FW version missing/invalid")<maxlen)
            {
                sprintf(szBuffer, "FW version missing/invalid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FW_UPDATE_ERROR): /* 4007 */
        {
            if ((int)strlen("FW update failed")<maxlen)
            {
                sprintf(szBuffer, "FW update failed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FW_CRC_PAR_ERROR): /* 4008 */
        {
            if ((int)strlen("FW Parameter CRC wrong")<maxlen)
            {
                sprintf(szBuffer, "FW Parameter CRC wrong");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_FW_CRC_FW_ERROR): /* 4009 */
        {
            if ((int)strlen("FW CRC wrong")<maxlen)
            {
                sprintf(szBuffer, "FW CRC wrong");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_PCC_SCAN_DATA): /* 5000 */
        {
            if ((int)strlen("PicoCompensation scan data is not valid")<maxlen)
            {
                sprintf(szBuffer, "PicoCompensation scan data is not valid");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PCC_SCAN_RUNNING): /* 5001 */
        {
            if ((int)strlen("PicoCompensation is running, some actions can not be executed during scanning/recording")<maxlen)
            {
                sprintf(szBuffer, "PicoCompensation is running, some actions can not be executed during scanning/recording");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_INVALID_PCC_AXIS): /* 5002 */
        {
            if ((int)strlen("Given axis can not be defined as PPC axis")<maxlen)
            {
                sprintf(szBuffer, "Given axis can not be defined as PPC axis");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PCC_SCAN_OUT_OF_RANGE): /* 5003 */
        {
            if ((int)strlen("Defined scan area is larger than the travel range")<maxlen)
            {
                sprintf(szBuffer, "Defined scan area is larger than the travel range");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PCC_TYPE_NOT_EXISTING): /* 5004 */
        {
            if ((int)strlen("Given PicoCompensation type is not defined")<maxlen)
            {
                sprintf(szBuffer, "Given PicoCompensation type is not defined");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PCC_PAM_ERROR): /* 5005 */
        {
            if ((int)strlen("PicoCompensation parameter error")<maxlen)
            {
                sprintf(szBuffer, "PicoCompensation parameter error");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_PCC_TABLE_ARRAY_TOO_LARGE): /* 5006 */
        {
            if ((int)strlen("PicoCompensation table is larger than maximum table length")<maxlen)
            {
                sprintf(szBuffer, "PicoCompensation table is larger than maximum table length");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NEXLINE_ERROR): /* 5100 */
        {
            if ((int)strlen("Common error in Nexline firmware module")<maxlen)
            {
                sprintf(szBuffer, "Common error in Nexline firmware module");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_CHANNEL_ALREADY_USED): /* 5101 */
        {
            if ((int)strlen("Output channel for Nexline can not be redefined for other usage")<maxlen)
            {
                sprintf(szBuffer, "Output channel for Nexline can not be redefined for other usage");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_NEXLINE_TABLE_TOO_SMALL): /* 5102 */
        {
            if ((int)strlen("Memory for Nexline signals is too small")<maxlen)
            {
                sprintf(szBuffer, "Memory for Nexline signals is too small");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RNP_WITH_SERVO_ON): /* 5103 */
        {
            if ((int)strlen("RNP can not be executed if axis is in closed loop")<maxlen)
            {
                sprintf(szBuffer, "RNP can not be executed if axis is in closed loop");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_RNP_NEEDED): /* 5104 */
        {
            if ((int)strlen("relax procedure (RNP) needed")<maxlen)
            {
                sprintf(szBuffer, "relax procedure (RNP) needed");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_AXIS_NOT_CONFIGURED): /* 5200 */
        {
            if ((int)strlen("Axis must be configured for this action")<maxlen)
            {
                sprintf(szBuffer, "Axis must be configured for this action");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_INVALID_VALUE): /* 6000 */
        {
            if ((int)strlen("Invalid preset value of absolute sensor")<maxlen)
            {
                sprintf(szBuffer, "Invalid preset value of absolute sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_WRITE_ERROR): /* 6001 */
        {
            if ((int)strlen("Error while writing to sensor")<maxlen)
            {
                sprintf(szBuffer, "Error while writing to sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_READ_ERROR): /* 6002 */
        {
            if ((int)strlen("Error while reading from sensor")<maxlen)
            {
                sprintf(szBuffer, "Error while reading from sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_CRC_ERROR): /* 6003 */
        {
            if ((int)strlen("Checksum error of absolute sensor")<maxlen)
            {
                sprintf(szBuffer, "Checksum error of absolute sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_ERROR): /* 6004 */
        {
            if ((int)strlen("General error of absolute sensor")<maxlen)
            {
                sprintf(szBuffer, "General error of absolute sensor");
                return TRUE;
            }
            break;
        }
        case(PI_CNTR_SENSOR_ABS_OVERFLOW): /* 6005 */
        {
            if ((int)strlen("Overflow of absolute sensor position")<maxlen)
            {
                sprintf(szBuffer, "Overflow of absolute sensor position");
                return TRUE;
            }
            break;
        }
/*
 **  End of Controller Errors
 *************************************************/

/*************************************************
 **
 ** Interface Errors - Interface errors occuring while communicating with the controller
 */
        case(COM_ERROR): /* -1 */
        {
            if ((int)strlen("Error during com operation (could not be specified)")<maxlen)
            {
                sprintf(szBuffer, "Error during com operation (could not be specified)");
                return TRUE;
            }
            break;
        }
        case(SEND_ERROR): /* -2 */
        {
            if ((int)strlen("Error while sending data")<maxlen)
            {
                sprintf(szBuffer, "Error while sending data");
                return TRUE;
            }
            break;
        }
        case(REC_ERROR): /* -3 */
        {
            if ((int)strlen("Error while receiving data")<maxlen)
            {
                sprintf(szBuffer, "Error while receiving data");
                return TRUE;
            }
            break;
        }
        case(NOT_CONNECTED_ERROR): /* -4 */
        {
            if ((int)strlen("Not connected (no port with given ID open)")<maxlen)
            {
                sprintf(szBuffer, "Not connected (no port with given ID open)");
                return TRUE;
            }
            break;
        }
        case(COM_BUFFER_OVERFLOW): /* -5 */
        {
            if ((int)strlen("Buffer overflow")<maxlen)
            {
                sprintf(szBuffer, "Buffer overflow");
                return TRUE;
            }
            break;
        }
        case(CONNECTION_FAILED): /* -6 */
        {
            if ((int)strlen("Error while opening port")<maxlen)
            {
                sprintf(szBuffer, "Error while opening port");
                return TRUE;
            }
            break;
        }
        case(COM_TIMEOUT): /* -7 */
        {
            if ((int)strlen("Timeout error")<maxlen)
            {
                sprintf(szBuffer, "Timeout error");
                return TRUE;
            }
            break;
        }
        case(COM_MULTILINE_RESPONSE): /* -8 */
        {
            if ((int)strlen("There are more lines waiting in buffer")<maxlen)
            {
                sprintf(szBuffer, "There are more lines waiting in buffer");
                return TRUE;
            }
            break;
        }
        case(COM_INVALID_ID): /* -9 */
        {
            if ((int)strlen("There is no interface or DLL handle with the given ID")<maxlen)
            {
                sprintf(szBuffer, "There is no interface or DLL handle with the given ID");
                return TRUE;
            }
            break;
        }
        case(COM_NOTIFY_EVENT_ERROR): /* -10 */
        {
            if ((int)strlen("Event/message for notification could not be opened")<maxlen)
            {
                sprintf(szBuffer, "Event/message for notification could not be opened");
                return TRUE;
            }
            break;
        }
        case(COM_NOT_IMPLEMENTED): /* -11 */
        {
            if ((int)strlen("Function not supported by this interface type")<maxlen)
            {
                sprintf(szBuffer, "Function not supported by this interface type");
                return TRUE;
            }
            break;
        }
        case(COM_ECHO_ERROR): /* -12 */
        {
            if ((int)strlen("Error while sending \"echoed\" data")<maxlen)
            {
                sprintf(szBuffer, "Error while sending \"echoed\" data");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EDVR): /* -13 */
        {
            if ((int)strlen("IEEE488: System error")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: System error");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ECIC): /* -14 */
        {
            if ((int)strlen("IEEE488: Function requires GPIB board to be CIC")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Function requires GPIB board to be CIC");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ENOL): /* -15 */
        {
            if ((int)strlen("IEEE488: Write function detected no listeners")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Write function detected no listeners");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EADR): /* -16 */
        {
            if ((int)strlen("IEEE488: Interface board not addressed correctly")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Interface board not addressed correctly");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EARG): /* -17 */
        {
            if ((int)strlen("IEEE488: Invalid argument to function call")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Invalid argument to function call");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ESAC): /* -18 */
        {
            if ((int)strlen("IEEE488: Function requires GPIB board to be SAC")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Function requires GPIB board to be SAC");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EABO): /* -19 */
        {
            if ((int)strlen("IEEE488: I/O operation aborted")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: I/O operation aborted");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ENEB): /* -20 */
        {
            if ((int)strlen("IEEE488: Interface board not found")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Interface board not found");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EDMA): /* -21 */
        {
            if ((int)strlen("IEEE488: Error performing DMA")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Error performing DMA");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EOIP): /* -22 */
        {
            if ((int)strlen("IEEE488: I/O operation started before previous operation completed")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: I/O operation started before previous operation completed");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ECAP): /* -23 */
        {
            if ((int)strlen("IEEE488: No capability for intended operation")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: No capability for intended operation");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EFSO): /* -24 */
        {
            if ((int)strlen("IEEE488: File system operation error")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: File system operation error");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_EBUS): /* -25 */
        {
            if ((int)strlen("IEEE488: Command error during device call")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Command error during device call");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ESTB): /* -26 */
        {
            if ((int)strlen("IEEE488: Serial poll-status byte lost")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Serial poll-status byte lost");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ESRQ): /* -27 */
        {
            if ((int)strlen("IEEE488: SRQ remains asserted")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: SRQ remains asserted");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ETAB): /* -28 */
        {
            if ((int)strlen("IEEE488: Return buffer full")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Return buffer full");
                return TRUE;
            }
            break;
        }
        case(COM_GPIB_ELCK): /* -29 */
        {
            if ((int)strlen("IEEE488: Address or board locked")<maxlen)
            {
                sprintf(szBuffer, "IEEE488: Address or board locked");
                return TRUE;
            }
            break;
        }
        case(COM_RS_INVALID_DATA_BITS): /* -30 */
        {
            if ((int)strlen("RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits with 1.5 stop bits")<maxlen)
            {
                sprintf(szBuffer, "RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits with 1.5 stop bits");
                return TRUE;
            }
            break;
        }
        case(COM_ERROR_RS_SETTINGS): /* -31 */
        {
            if ((int)strlen("RS-232: Error configuring the COM port")<maxlen)
            {
                sprintf(szBuffer, "RS-232: Error configuring the COM port");
                return TRUE;
            }
            break;
        }
        case(COM_INTERNAL_RESOURCES_ERROR): /* -32 */
        {
            if ((int)strlen("Error dealing with internal system resources (events, threads, ...)")<maxlen)
            {
                sprintf(szBuffer, "Error dealing with internal system resources (events, threads, ...)");
                return TRUE;
            }
            break;
        }
        case(COM_DLL_FUNC_ERROR): /* -33 */
        {
            if ((int)strlen("A DLL or one of the required functions could not be loaded")<maxlen)
            {
                sprintf(szBuffer, "A DLL or one of the required functions could not be loaded");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_INVALID_HANDLE): /* -34 */
        {
            if ((int)strlen("FTDIUSB: invalid handle")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: invalid handle");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_DEVICE_NOT_FOUND): /* -35 */
        {
            if ((int)strlen("FTDIUSB: device not found")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: device not found");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_DEVICE_NOT_OPENED): /* -36 */
        {
            if ((int)strlen("FTDIUSB: device not opened")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: device not opened");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_IO_ERROR): /* -37 */
        {
            if ((int)strlen("FTDIUSB: IO error")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: IO error");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_INSUFFICIENT_RESOURCES): /* -38 */
        {
            if ((int)strlen("FTDIUSB: insufficient resources")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: insufficient resources");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_INVALID_PARAMETER): /* -39 */
        {
            if ((int)strlen("FTDIUSB: invalid parameter")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: invalid parameter");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_INVALID_BAUD_RATE): /* -40 */
        {
            if ((int)strlen("FTDIUSB: invalid baud rate")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: invalid baud rate");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_ERASE): /* -41 */
        {
            if ((int)strlen("FTDIUSB: device not opened for erase")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: device not opened for erase");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_WRITE): /* -42 */
        {
            if ((int)strlen("FTDIUSB: device not opened for write")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: device not opened for write");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_FAILED_TO_WRITE_DEVICE): /* -43 */
        {
            if ((int)strlen("FTDIUSB: failed to write device")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: failed to write device");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_EEPROM_READ_FAILED): /* -44 */
        {
            if ((int)strlen("FTDIUSB: EEPROM read failed")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: EEPROM read failed");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_EEPROM_WRITE_FAILED): /* -45 */
        {
            if ((int)strlen("FTDIUSB: EEPROM write failed")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: EEPROM write failed");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_EEPROM_ERASE_FAILED): /* -46 */
        {
            if ((int)strlen("FTDIUSB: EEPROM erase failed")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: EEPROM erase failed");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_EEPROM_NOT_PRESENT): /* -47 */
        {
            if ((int)strlen("FTDIUSB: EEPROM not present")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: EEPROM not present");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_EEPROM_NOT_PROGRAMMED): /* -48 */
        {
            if ((int)strlen("FTDIUSB: EEPROM not programmed")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: EEPROM not programmed");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_INVALID_ARGS): /* -49 */
        {
            if ((int)strlen("FTDIUSB: invalid arguments")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: invalid arguments");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_NOT_SUPPORTED): /* -50 */
        {
            if ((int)strlen("FTDIUSB: not supported")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: not supported");
                return TRUE;
            }
            break;
        }
        case(COM_FTDIUSB_OTHER_ERROR): /* -51 */
        {
            if ((int)strlen("FTDIUSB: other error")<maxlen)
            {
                sprintf(szBuffer, "FTDIUSB: other error");
                return TRUE;
            }
            break;
        }
        case(COM_PORT_ALREADY_OPEN): /* -52 */
        {
            if ((int)strlen("Error while opening the COM port: was already open")<maxlen)
            {
                sprintf(szBuffer, "Error while opening the COM port: was already open");
                return TRUE;
            }
            break;
        }
        case(COM_PORT_CHECKSUM_ERROR): /* -53 */
        {
            if ((int)strlen("Checksum error in received data from COM port")<maxlen)
            {
                sprintf(szBuffer, "Checksum error in received data from COM port");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_NOT_READY): /* -54 */
        {
            if ((int)strlen("Socket not ready, you should call the function again")<maxlen)
            {
                sprintf(szBuffer, "Socket not ready, you should call the function again");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_PORT_IN_USE): /* -55 */
        {
            if ((int)strlen("Port is used by another socket")<maxlen)
            {
                sprintf(szBuffer, "Port is used by another socket");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_NOT_CONNECTED): /* -56 */
        {
            if ((int)strlen("Socket not connected (or not valid)")<maxlen)
            {
                sprintf(szBuffer, "Socket not connected (or not valid)");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_TERMINATED): /* -57 */
        {
            if ((int)strlen("Connection terminated (by peer)")<maxlen)
            {
                sprintf(szBuffer, "Connection terminated (by peer)");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_NO_RESPONSE): /* -58 */
        {
            if ((int)strlen("Can't connect to peer")<maxlen)
            {
                sprintf(szBuffer, "Can't connect to peer");
                return TRUE;
            }
            break;
        }
        case(COM_SOCKET_INTERRUPTED): /* -59 */
        {
            if ((int)strlen("Operation was interrupted by a nonblocked signal")<maxlen)
            {
                sprintf(szBuffer, "Operation was interrupted by a nonblocked signal");
                return TRUE;
            }
            break;
        }
        case(COM_PCI_INVALID_ID): /* -60 */
        {
            if ((int)strlen("No Device with this ID is present")<maxlen)
            {
                sprintf(szBuffer, "No Device with this ID is present");
                return TRUE;
            }
            break;
        }
        case(COM_PCI_ACCESS_DENIED): /* -61 */
        {
            if ((int)strlen("Driver could not be opened (on Vista: run as administrator!)")<maxlen)
            {
                sprintf(szBuffer, "Driver could not be opened (on Vista: run as administrator!)");
                return TRUE;
            }
            break;
        }
/*
 **  End of Interface Errors
 *************************************************/

        default:
            return FALSE;
    }
    return FALSE;
}

