TOP=../..
include $(TOP)/configure/CONFIG

PROD_NAME = stdTestApp
PROD_IOC = $(PROD_NAME)

DBD += $(PROD_NAME).dbd

$(PROD_NAME)_DBD += base.dbd
$(PROD_NAME)_DBD += $(STD_IOC_DBDS)

$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp
$(PROD_NAME)_SRCS_DEFAULT += $(PROD_NAME)Main.cpp
$(PROD_NAME)_SRCS_vxWorks += -nil-
$(PROD_NAME)_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

$(PROD_NAME)_LIBS += $(STD_IOC_LIBS)

# Transitive dependencies of std.
# Replace with $(MODULE_IOC_LIBS) as each module adopts cfg-deps.
$(PROD_NAME)_LIBS += seq pv asyn

$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
