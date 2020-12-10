TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# For base-7
USR_CPPFLAGS += -DUSE_TYPED_RSET


#PROD_HOST += caExample
#caExample_SRCS	+= caExample.c
#caExample_LIBS	+= ca
#caExample_LIBS	+= Com
#ca_DIR           = $(EPICS_BASE_LIB)
#Com_DIR          = $(EPICS_BASE_LIB)


#=============================
# xxxRecord.h will be created from xxxRecord.dbd

DBDINC += epidRecord
DBDINC += timestampRecord
DBDINC += throttleRecord

# <name>.dbd will be created from <name>Include.dbd
DBD += std.dbd
DBD += stdVX.dbd
DBD += stdSupport.dbd
DBD += stdVXSupport.dbd
DBD += delayDo.dbd

#=============================

LIBRARY_IOC += std

std_SRCS += epidRecord.c
std_SRCS += devEpidSoft.c
std_SRCS += devEpidSoftCallback.c
std_SRCS += devEpidFast.c

# timestamp record
std_SRCS += timestampRecord.c

# throttle record
std_SRCS += throttleRecord.c

# pvHistory stuff
std_SRCS += devTimeOfDay.c 
std_SRCS += pvHistory.c

# Femto amplifier
std_SRCS += femto.st

# delayDo
std_SRCS += delayDo.st

#delayCmd command
std_SRCS += delayCmd.cpp

# vxCall command
std_SRCS_vxWorks += vxcall.cpp

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
stdApp_SYS_LIBS_solaris += socket

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
timestampRecord$(OBJ):     $(COMMON_DIR)/timestampRecord.h
throttleRecord$(OBJ):      $(COMMON_DIR)/throttleRecord.h
