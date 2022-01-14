#FILENAME...     Makefile
#USAGE...        Makefile for motor application example with Asyn

#Version:        $Revision: 1.3 $
#Modified By:    $Author: Russ Berg (bergr) $
#Last Modified:  $Date: 2019/12/09 16:32:55CST $
#HeadURL:        $URL: https://subversion.xor.aps.anl.gov/synApps/motor/tags/R6-8/motorExApp/pi_e712WithAsyn/Makefile $

# "#!" marks lines that can be uncommented.

TOP = ../..
include $(TOP)/configure/CONFIG
USR_CPPFLAGS += -I/home/beamline/tools/anaconda3/include/python3.7m
USR_LDFLAGS += -L/home/beamline/tools/anaconda3/lib


PROD_IOC_DEFAULT = pi_e712WithAsyn

COMMONDBDS  = base.dbd
COMMONDBDS += asyn.dbd
#motorSim_DBD += motorSimSupport.dbd

COMMONDBDS += transformRecord.dbd
COMMONDBDS += calcSupport.dbd
COMMONDBDS += motorSupport.dbd
COMMONDBDS += PI_E712Support.dbd
COMMONDBDS += busySupport.dbd
#COMMONDBDS += sscanSupport.dbd
COMMONDBDS += drvAsynIPPort.dbd
COMMONDBDS += as.dbd
COMMONDBDS += asSupport.dbd

DBD += pi_e712WithAsyn.dbd
pi_e712WithAsyn_DBD += $(COMMONDBDS)

pi_e712WithAsyn_SRCS   += pi_e712WithAsyn_registerRecordDeviceDriver.cpp
pi_e712WithAsyn_SRCS_DEFAULT += pi_e712WithAsynMain.cpp

COMMONLIBS += PI
COMMONLIBS += PI_E712Support
COMMONLIBS += softMotor
COMMONLIBS += motorSimSupport
COMMONLIBS += motor


pi_e712WithAsyn_LIBS += python3.7m
python3.7m_DIR = /home/beamline/tools/anaconda3/lib


# Needed for Newport SNL programs
pi_e712WithAsyn_LIBS += $(COMMONLIBS) 
pi_e712WithAsyn_LIBS += calc 
pi_e712WithAsyn_LIBS += asyn
pi_e712WithAsyn_LIBS += sscan
pi_e712WithAsyn_LIBS += autosave

ifdef SNCSEQ
pi_e712WithAsynVx_DBD += load_param_file.dbd
pi_e712WithAsyn_SRCS   += load_param_file.st
pi_e712WithAsyn_LIBS += seq pv
endif
pi_e712WithAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

pi_e712WithAsyn_LIBS += busy 

pi_e712WithAsynVx_LIBS += $(COMMONLIBS)
pi_e712WithAsynVx_LIBS += asyn
# Needed for Newport SNL programs
ifdef SNCSEQ
pi_e712WithAsynVx_LIBS   += seq pv
endif
pi_e712WithAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

