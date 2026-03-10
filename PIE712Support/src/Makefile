TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
USR_CFLAGS += -g -O0
USR_CPPFLAGS += -g -O0

USR_CPPFLAGS += -I/home/beamline/tools/anaconda3/include/python3.12 -Wno-unused-variable -Wno-unused-but-set-variable
USR_LDFLAGS += -L/home/beamline/tools/anaconda3/lib -Wno-unused-variable
USR_CPPFLAGS += -I/$(MOTOR)/include
USR_CFLAGS += -I/$(MOTOR)/include


LIBRARY_IOC += PI_E712Support

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += PI_E712Support.dbd

# The following are compiled and added to the Support library
PI_E712Support_SRCS += PIE712Controller.cpp
PI_E712Support_SRCS += translateerror.c
PI_E712Support_SRCS += PIInterface.cpp
PI_E712Support_SRCS += load_param_file.cpp

PI_E712Support_LIBS += motor
PI_E712Support_LIBS += asyn
PI_E712Support_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

