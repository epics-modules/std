TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# For Kate Feng's implementation of devScaler.c, which uses Till
# Straumann's basicIoOps.h to bridge any mismatch between endianness
# of Joerger scaler hardware and endianness of processor.
#USR_CFLAGS += -DHAS_IOOPS_H

#PROD_HOST += caExample
#caExample_SRCS	+= caExample.c
#caExample_LIBS	+= ca
#caExample_LIBS	+= Com
#ca_DIR           = $(EPICS_BASE_LIB)
#Com_DIR          = $(EPICS_BASE_LIB)


#=============================
# xxxRecord.h will be created from xxxRecord.dbd

DBDINC += scalerRecord
DBDINC += sseqRecord
DBDINC += epidRecord
DBDINC += timestampRecord

# <name>.dbd will be created from <name>Include.dbd
DBD += std.dbd
DBD += stdVX.dbd
DBD += stdSupport.dbd
DBD += stdVXSupport.dbd

#=============================

LIBRARY_IOC += std

std_SRCS_vxWorks += aiCvtDouble.c

std_SRCS += scalerRecord.c
std_SRCS += devScalerAsyn.c
std_SRCS += drvScalerSoft.c
std_SRCS += drvScaler974.cpp
std_SRCS += epidRecord.c
std_SRCS += devEpidSoft.c
std_SRCS += devEpidSoftCallback.c
std_SRCS += devEpidFast.c
std_SRCS += sseqRecord.c
#std_SRCS += devVXStats.c

# timestamp record
std_SRCS += timestampRecord.c

# pvHistory stuff
std_SRCS += devTimeOfDay.c 
std_SRCS += pvHistory.c

# Femto amplifier
std_SRCS += femto.st

# publish this file so it can be included by device support for scaler
# built in the MCA module
INC += devScaler.h
INC += devScalerAsyn.h
INC += aiCvtDouble.h

SCRIPTS += showBurtDiff wrapCmd wrapper

std_LIBS += asyn seq pv
std_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# build an ioc application for testing

PROD_IOC = stdApp

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
stdApp_SRCS_DEFAULT += std_registerRecordDeviceDriver.cpp
stdApp_SRCS_vxWorks += stdVX_registerRecordDeviceDriver.cpp
stdApp_SRCS_DEFAULT += stdMain.cpp
stdApp_SRCS_vxWorks += -nil-
#The following adds support from base/src/vxWorks
stdApp_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

stdApp_LIBS += std seq pv asyn $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# Parallel build sometimes fails.
# Make dependences on epidRecord.h explicit.
epidRecord$(OBJ):          $(COMMON_DIR)/epidRecord.h
devEpidSoft$(OBJ):         $(COMMON_DIR)/epidRecord.h
devEpidSoftCallback$(OBJ): $(COMMON_DIR)/epidRecord.h
devEpidFast$(OBJ):         $(COMMON_DIR)/epidRecord.h
