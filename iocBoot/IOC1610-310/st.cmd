#errlogInit(5000)
< envPaths
< $(BP_API_APP)/AMB_STXM_CONFIG

epicsEnvSet IOCSH_PS1 "$(STXM) E712 Ctrl App>"

# Tell EPICS all about the record types, device-support modules, drivers,
# etc. in this build from CARS

dbLoadDatabase("$(TOP)/dbd/pi_e712WithAsyn.dbd")
dbLoadDatabase("$(TOP)/dbd/PI_E712Support.dbd")    

pi_e712WithAsyn_registerRecordDeviceDriver(pdbbase)


# PI E-712 Controller
dbLoadTemplate "motor.substitutions.pi_e712"
drvAsynIPPortConfigure("L100", "192.168.50.5:50000",0,0,0)
# now one for the data recorder on same port
#drvAsynIPPortConfigure("L101", "192.168.50.5:50000",0,0,0)

# PI_E712_CreateController("Port name","asyn Port name","data recorder asyn Port name","Number of axes","moving polling time [msec]","idle polling time [msec]") 
#PI_E712_CreateController("E712", "L100", "L101", 3, 50, 250)
#PI_E712_CreateController("E712", "L100", "L101", 3, 50, 500)
PI_E712_CreateController("E712", "L100", "L101", 3, 50, 250)



# PI_E712_MotorConfigAxis("Port name", "Axis #", "Axis name",  "High limit", "Low limit", "Home position", "Start posn", "Simulate")
PI_E712_MotorConfigAxis("E712", 0, "SampleFineX", 20000, -20000,  500, 0, 0)
PI_E712_MotorConfigAxis("E712", 1, "SampleFineY", 20000, -20000,  500, 0, 0)

#asynSetTraceIOMask("E712", 0, 255)
#asynSetTraceMask("E712", 0, 255)


dbLoadRecords("db/e712.db", "P=PZAC1610-3-I12-,M=40:,PORT=E712,ADDR=0,TIMEOUT=1")
dbLoadRecords("db/e712.db", "P=PZAC1610-3-I12-,M=41:,PORT=E712,ADDR=1,TIMEOUT=1")

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(STXM):E712:, R=L100, PORT=L100, ADDR=0, OMAX=256, IMAX=256") 
dbLoadRecords("db/e712_controller.db", "P=$(STXM):E712:,PORT=E712,ADDR=0,TIMEOUT=1")
dbLoadRecords("db/e712_calib.db")
dbLoadRecords("db/e712_scan_support.db", "P=$(STXM):E712")


epicsEnvSet(AUTOSAVE_PREFIX,"astxm_e712")
< $(TOP)/support/autosave-1.cmd

iocInit


#save positions every five seconds, it will look only in the dir setup in set_requestfile_path()
create_monitor_set("$(AUTOSAVE_PREFIX).req", 5)

dbpf(PZAC1610-3-I12-40:ServoPower, "0")
dbpf(PZAC1610-3-I12-41:ServoPower, "0")

dbpf(PZAC1610-3-I12-40_able.VAL, "$(START_EN_DISABLED)")
dbpf(PZAC1610-3-I12-41_able.VAL, "$(START_EN_DISABLED)")

dbpf("PSMTR1610-3-I12-00.RTRY","1") 
dbpf("PSMTR1610-3-I12-01.RTRY","1") 

#dbpf("$(STXM):E712:ParamFileName", "oct20_2022_coarse_smplfine.pam")
#epicsThreadSleep(5.0)
 
#dbpf("$(STXM):E712:LoadParamFile", "1")
 
 
