
set IOCDIR=C:/controls/epics/R3.14.12.4/modules/support/motor-6-8/iocBoot/iocAbsMotorUHV/
set XPS_IOCDIR=C:/controls/epics/R3.14.12.4/modules/support/motor-6-8/iocBoot/iocXPSWithAsyn/
set AGILENT_IOCDIR=C:/controls/epics/R3.14.12.4/modules/support/motor-6-8/iocBoot/iocPIE712WithAsyn/


cd C:/controls/epics/R3.14.12.4\modules\support\motor-6-8\

#.\bin\win32-x86-debug\uhvAbsMotor.exe .\iocBoot\iocAbsMotorUHV\st.cmd.COARSE_ZONEPLATE_SCANNING_datarecorder
#.\bin\win32-x86-debug\uhvAbsMotor.exe .\iocBoot\iocAbsMotorUHV\st.cmd.GONI_ZONEPLATE_SCANNING




.\bin\win32-x86-debug\uhvAbsMotor.exe .\iocBoot\iocAbsMotorUHV\st.cmd
