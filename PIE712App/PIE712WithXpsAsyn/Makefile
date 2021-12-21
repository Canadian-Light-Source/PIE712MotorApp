#FILENAME...     Makefile
#USAGE...        Makefile for motor application example with Asyn

#Version:        $Revision: 1.6 $
#Modified By:    $Author: Russ Berg (bergr) $
#Last Modified:  $Date: 2014/06/12 15:28:20CST $
#HeadURL:        $URL: https://subversion.xor.aps.anl.gov/synApps/motor/tags/R6-8/motorExApp/uhvAbsMotor/Makefile $

# "#!" marks lines that can be uncommented.

TOP = ../..
include $(TOP)/configure/CONFIG

USR_CFLAGS += /Zi
USR_CPPFLAGS += /Zi

PROD_IOC_DEFAULT = uhvAbsMotor
PROD_IOC_vxWorks = uhvAbsMotorVx

COMMONDBDS  = base.dbd
COMMONDBDS += transformRecord.dbd
COMMONDBDS += sCalcOutRecord.dbd
COMMONDBDS += motorSupport.dbd
COMMONDBDS += devNewport.dbd
COMMONDBDS += PI_E712Support.dbd
COMMONDBDS += AbsMotor-PI-XPSSupport.dbd
#COMMONDBDS += uhvAbsMotorSupport.dbd
COMMONDBDS += drvAsynSerialPort.dbd
COMMONDBDS += drvAsynIPPort.dbd
COMMONDBDS += busySupport.dbd
COMMONDBDS += sscanSupport.dbd
COMMONDBDS += as.dbd
COMMONDBDS += asSupport.dbd


DBD += uhvAbsMotor.dbd
uhvAbsMotor_DBD += $(COMMONDBDS)
ifdef SNCSEQ
uhvAbsMotor_DBD += devSequencer.dbd
endif

uhvAbsMotor_SRCS   += uhvAbsMotor_registerRecordDeviceDriver.cpp
uhvAbsMotor_SRCS_DEFAULT += uhvAbsMotorMain.cpp
uhvAbsMotorVx_SRCS += uhvAbsMotorVx_registerRecordDeviceDriver.cpp

COMMONLIBS += AbsMotor-PI-XPS
COMMONLIBS += PI_E712Support
COMMONLIBS += PI_GCS2Support
COMMONLIBS += Newport
COMMONLIBS += motor

# Needed for Newport SNL programs
uhvAbsMotor_LIBS += $(COMMONLIBS) 
uhvAbsMotor_LIBS += calc 
uhvAbsMotor_LIBS += asyn
uhvAbsMotor_LIBS += sscan

ifdef SNCSEQ
uhvAbsMotor_LIBS += seqDev seq pv
endif
uhvAbsMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

uhvAbsMotor_LIBS += busy 
uhvAbsMotor_LIBS += autosave

uhvAbsMotorVx_LIBS += $(COMMONLIBS)
ifdef IPAC
uhvAbsMotorVx_LIBS += Ipac TyGSOctal
endif
uhvAbsMotorVx_LIBS += asyn
# Needed for Newport SNL programs
ifdef SNCSEQ
uhvAbsMotorVx_LIBS   += seqDev seq pv
endif
uhvAbsMotorVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

