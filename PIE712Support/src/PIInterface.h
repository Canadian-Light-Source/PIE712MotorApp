/*
FILENAME...     PIInterface.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 1.1 $
Modified By:    $Author: Russ Berg (bergr) $
Last Modified:  $Date: 2018/09/27 13:19:43CST $
HeadURL:        $URL$

Original Author: Steffen Rau 
Created: 15.12.2010
*/

#ifndef PIINTERFACE_H_INCLUDED
#define PIINTERFACE_H_INCLUDED

#include <epicsMutex.h>


class PIInterface
{
public:
	PIInterface(asynUser* pCom);
	virtual ~PIInterface();

	virtual asynStatus sendOnly(const char *outputBuff, asynUser* logSink);
	virtual asynStatus sendOnly(char c, asynUser* logSink);

	virtual asynStatus sendAndReceive(const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink);
	virtual asynStatus DRRsendAndReceive(const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink);

	virtual asynStatus sendOnly(const char *outputBuff);
	virtual asynStatus sendOnly(char c);

	virtual asynStatus sendAndReceive(char c, char *inputBuff, int inputSize);
	virtual asynStatus sendAndReceive(const char* output, char *inputBuff, int inputSize);
	virtual asynStatus sendAndReceiveDRR(const char* output, char *inputBuff, int inputSize);
	virtual asynStatus sendAndReceive(char c, char *inputBuff, int inputSize, asynUser* logSink);

    asynUser* m_pCurrentLogSink;

protected:
    static double TIMEOUT;
	epicsMutex m_interfaceMutex;

	asynUser* m_pAsynInterface;
};

#endif // PIINTERFACE_H_INCLUDED
