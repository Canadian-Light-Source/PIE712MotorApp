# Debug-output level
#save_restoreSet_Debug(1)
epicsEnvSet(PREFIX,"$(AUTOSAVE_PREFIX)")
epicsEnvSet(AUTOSAVE,"/home/epics/src/R7-SL7/modules/support/autosave-R5-10")


# status-PV prefix, so save_restore can find its status PV's.
#save_restoreSet_status_prefix("IOC")

save_restoreSet_IncompleteSetsOk(1)
save_restoreSet_DatedBackupFiles(1)

set_savefile_path("$(AUTOSAVE_DIR)")
set_requestfile_path("$(TOP)")
save_restoreSet_FilePermissions(0755)

set_pass0_restoreFile("$(AUTOSAVE_PREFIX).sav")
set_pass1_restoreFile("$(AUTOSAVE_PREFIX).sav")

save_restoreSet_NumSeqFiles(3)
save_restoreSet_RetrySeconds(60)
save_restoreSet_CAReconnect(1)
save_restoreSet_CallbackTimeout(-1)


dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db","P=$(AUTOSAVE_PREFIX)")