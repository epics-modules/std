#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure stdApp iocBoot
stdApp_DEPEND_DIRS  = configure
iocBoot_DEPEND_DIRS = stdApp
include $(TOP)/configure/RULES_TOP
