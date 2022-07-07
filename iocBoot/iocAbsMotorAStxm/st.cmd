# The is the ASYN example for communication to 4 simulated motors
# "#!" marks lines that can be uncommented.
#THE following file must be at the top before any other calls
< C:/controls/epics/R3.14.12.4/local/src/blApiApp/UHV_STXM_CONFIG

< $(IOCDIR)/envPaths


< $(IOCDIR)/$(ABSTRACT_MTR_MODE_ST_FILE)
