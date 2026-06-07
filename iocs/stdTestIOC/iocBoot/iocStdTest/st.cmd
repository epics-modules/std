# This is a demonstration startup for stdTestApp on non-vxWorks systems

< envPaths

# Tell EPICS all about the record types, device-support modules, drivers,
# etc. in this build
dbLoadDatabase("../../dbd/stdTestApp.dbd")
stdTestApp_registerRecordDeviceDriver(pdbbase)

# Set up 2 serial ports
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)
asynSetOption("serial1", 0, "baud", "9600")
asynSetOption("serial1", 0, "parity", "none")
asynSetOption("serial1", 0, "bits", "8")
asynSetOption("serial1", 0, "stop", "1")

drvAsynSerialPortConfigure("serial2", "/dev/ttyS1", 0, 0, 0)
asynSetOption("serial1", 0, "baud", "19200")
asynSetOption("serial1", 0, "parity", "none")
asynSetOption("serial1", 0, "bits", "8")
asynSetOption("serial1", 0, "stop", "1")

#PID slow
dbLoadTemplate "pid_slow.template"

# A set of scan parameters for each positioner.  This is a convenience
# for the user.  It can contain an entry for each scannable thing in the
# crate.
#dbLoadTemplate "scanParms.template"

### Scan-support software
# crate-resident scan.  This executes 1D, 2D, 3D, and 4D scans, and caches
# 1D data, but it doesn't store anything to disk.  (You need the data catcher
# or the equivalent for that.)  This database is configured to use the
# "alldone" database (above) to figure out when motors have stopped moving
# and it's time to trigger detectors.
#dbLoadRecords("$(SSCAN)/db/scan.db", "P=stdTest:,MAXPTS1=2000,MAXPTS2=200,MAXPTS3=20,MAXPTS4=10,MAXPTSH=10")

# Free-standing user menu choices (mbbo records)
#dbLoadRecords("$(STD)/db/userMbbos10.db", "P=stdTest:")

# Miscellaneous PV's, such as burtResult
dbLoadRecords("$(STD)/db/misc.db", "P=stdTest:")

# Femto amplifier
#dbLoadRecords("$(STD)/db/femto.db","P=stdTest:,H=fem01:,F=seq01:")

# Throttle record
#dbLoadRecords("$(STD)/db/throttle.db", "P=stdTest:,THR=thr1")

# General purpose selector
dbLoadTemplate("selector.substitutions")

iocInit

### Start Femto amplifier sequence programs
#   P=<prefix>
#   H=<hardware-type> (i.e. fem01: - femto amplifier)
#   F=<functionality> (i.e. seq01: - sequencer)
#   G1=<gain bit1>
#   G2=<gain bit2>
#   G3=<gain bit3>
#   NO=<low/high noise bit>
#
# NOTE: Below command line is limited to 128 chars
#seq femto,"name=fm1,P=stdTest:,H=fem01:,F=seq01:,G1=,G2=,G3=,NO="

### Start up the autosave task and tell it what to do.
# The task is actually named "save_restore".
# Load the list of search directories for request files
#< ../requestFileCommands

# save positions every five seconds
#create_monitor_set("auto_positions.req", 5)
# save other things every thirty seconds
#create_monitor_set("auto_settings.req", 30)
