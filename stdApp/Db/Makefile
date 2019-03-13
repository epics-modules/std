# Makefile.Host
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) dbd files
# <name>.dbd will be created from <name>Include.dbd
#DBD += example.dbd

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
#DB += sample.db
DB += 4step.db        pvHistory.db        recordPV.db           trend.db          softMotor.db   ramp_tweak.db
DB += all_com_0.db    all_com_4.db        all_com_8.db          all_com_12.db     all_com_16.db  all_com_24.db  all_com_28.db 
DB += all_com_20.db   all_com_22.db       all_com_32.db         all_com_36.db     all_com_40.db  all_com_44.db  all_com_48.db 
DB += all_com_52.db   all_com_56.db       all_com_64.db         all_com_72.db     all_com_80.db  all_com_88.db
DB += femto.db        femto_DDPCA_300.db  femto_DHPCA_100.db    femto_DLPCA_200.db
DB += pid_control.db  fast_pid_control.db async_pid_control.db   
DB += misc.db         Nano2k.db           IDctrl.db             sampleWheel.db
DB += scaler.db       scalerSoftCtrl.db   scaler16.db           scaler16m.db      scaler32.db
DB += genericState.db genericStateAux.db  genericStrState.db    genTweak.db       selector.db                
DB += alarmClock.vdb  delayDo.db          timer.db       timeString.db  countDownTimer.vdb
DB += userMbbos10.db 
DB += zero.db         zero2.db
DB += autoShutter.vdb remoteShutter.db    throttle.db                              

#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += dbExample2.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
