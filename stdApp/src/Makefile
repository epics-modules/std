TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# For Kate Feng's implementation of devScaler.c, which uses Till
# Straumann's basicIoOps.h to bridge any mismatch between endianness
# of Joerger scaler hardware and endianness of processor.
USR_CFLAGS += -DHAS_IOOPS_H

#PROD_HOST += caExample
#caExample_SRCS	+= caExample.c
#caExample_LIBS	+= ca
#caExample_LIBS	+= Com

ca_DIR           = $(EPICS_BASE_LIB)
Com_DIR          = $(EPICS_BASE_LIB)


#=============================
# xxxRecord.h will be created from xxxRecord.dbd

DBDINC += scalerRecord
DBDINC += scanparmRecord
#DBDINC += serialRecord
DBDINC += tableRecord
DBDINC += transformRecord
#DBDINC += vmeRecord
#DBDINC += gpibRecord
DBDINC += sscanRecord
DBDINC += sCalcoutRecord
DBDINC += busyRecord
DBDINC += swaitRecord
DBDINC += sseqRecord
DBDINC += epidRecord
#DBDINC += genSubRecord

# <name>.dbd will be created from <name>Include.dbd
DBD += std.dbd
DBD += stdVX.dbd
DBDINSTALL += ../shareBase.dbd
DBDINSTALL += ../stdShare.dbd

#=============================

#PROD_IOC_vxWorks = std

LIBRARY_IOC += std

std_SRCS_vxWorks += dbrestore.c save_restore.c fGetDateStr.c
std_SRCS_vxWorks  += initHooks.c
#std_SRCS += devAiVaroc.c drvVarocB.c
#std_SRCS += devXxHP10895LaserAxis.c
#std_SRCS += devXxHeidenhainGpib.c
#std_SRCS += devXxKeithleyDMM199Gpib.c

# Heidenhain IK320 support
#std_SRCS += devIK320.c
#std_SRCS += drvIK320.c
#std_SRCS += drvIK320ErrStat.c
std_SRCS_vxWorks  += aiCvtDouble.c

std_SRCS_vxWorks  += module_types.c
std_SRCS_vxWorks  += scalerRecord.c devScaler.c
std_SRCS   += sscanRecord.c
std_SRCS_vxWorks   += scanparmRecord.c
#std_SRCS += serialRecord.c
std_SRCS_vxWorks   += tableRecord.c
std_SRCS += transformRecord.c
std_SRCS += sCalcPostfix.c sCalcPerform.c
std_SRCS_vxWorks += test_sCalc.c
std_SRCS   += sCalcoutRecord.c
#std_SRCS += vmeRecord.c
#std_SRCS += gpibRecord.c
#std_SRCS += camacRecord.c
std_SRCS_vxWorks   += epidRecord.c
std_SRCS_vxWorks  += fastPID.cc
#std_SRCS += devEpidSoft.c
std_SRCS  += swaitRecord.c
#std_SRCS += devBunchClkGen.c
#std_SRCS += devGP307gpib.c
#std_SRCS += devXxAX301.c
std_SRCS  += busyRecord.c
std_SRCS_vxWorks  += sseqRecord.c
#std_SRCS += genSubRecord.c
#std_SRCS += devAoVMI4116.c
#std_SRCS += devAvme9440.c
std_SRCS += recDynLink.c

# save scan data stuff
std_SRCS_vxWorks += xdr_stdio.c
std_SRCS_vxWorks  += xdr_lib.c
std_SRCS_vxWorks  += saveData.c
std_SRCS_vxWorks  += req_file.c

# For machine-status board
std_SRCS_vxWorks  += devA32Vme.c
std_SRCS_vxWorks  += subAve.c

# ioc status checking stuff
std_SRCS_vxWorks   += iocCheck.c

# Try out Eric Norum's message stuff.  Should be in base by 3.14.2
#std_SRCS += epicsMessageQueue.cpp
#std_SRCS += osdMessageQueue.cpp

# Override dbLoadTemplate from base with new version
# that works around the lack of soft links since 3.13.1
#YACCOPT = -l
#LEXOPT = -L
std_SRCS_vxWorks += dbLoadTemplate.c
HTMLS += dbLoadTemplate.html
dbLoadTemplate.c: dbLoadTemplate_lex.c
clean::
	@$(RM) dbLoadTemplate.c dbLoadTemplate_lex.c

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
std_SRCS += std_registerRecordDeviceDriver.cpp
#std_SRCS_DEFAULT += stdMain.cpp #maybe only for win32
#std_SRCS_vxWorks += -nil- #?

#The following adds support from base/src/vxWorks
#std_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

### The following builds SNL programs as components of std
# NOTE: for suffix .st, code is run through cpp; for .stt, it's not
#std_SRCS += kohzuCtl.st
#std_SRCS += hrCtl.st
#std_SRCS += getFilledBuckets.st

# Pete Jemian's support for X-ray Instrumentation Associates
# HSC-1 slit controller (SNL program)
std_SRCS_vxWorks   += xiahsc.st
# customized version that works with sscan record
std_SRCS_vxWorks   += xia_slit.st

std_LIBS += seq
std_LIBS += pv
seq_DIR    = $(SNCSEQ_LIB)
pv_DIR     = $(SNCSEQ_LIB)

# publish this file so it can be included by device support for scaler
# built in the MCA module
INC += devScaler.h
INC += fastPID.h

SCRIPTS += showBurtDiff wrapCmd wrapper

std_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

