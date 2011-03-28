# This is a demonstration startup for for stdApp on non-vxWorks systems

# Tell EPICS all about the record types, device-support modules, drivers,
# etc. in this build from stdApp
dbLoadDatabase("../../dbd/std.dbd")
registerRecordDeviceDriver(pdbbase)

routerInit
localMessageRouterStart(0)

showSymbol

# Set up 2 serial ports
initTtyPort("serial1", "/dev/ttyS0", 9600, "N", 1, 8, "N", 1000)
initTtyPort("serial2", "/dev/ttyS1", 19200, "N", 1, 8, "N", 1000)
initSerialServer("serial1", "serial1", 1000, 20, "")
initSerialServer("serial2", "serial2", 1000, 20, "")

#PID slow
dbLoadTemplate "pid_slow.template"

# A set of scan parameters for each positioner.  This is a convenience
# for the user.  It can contain an entry for each scannable thing in the
# crate.
dbLoadTemplate "scanParms.template"

### Scan-support software
# crate-resident scan.  This executes 1D, 2D, 3D, and 4D scans, and caches
# 1D data, but it doesn't store anything to disk.  (You need the data catcher
# or the equivalent for that.)  This database is configured to use the
# "alldone" database (above) to figure out when motors have stopped moving
# and it's time to trigger detectors.
dbLoadRecords("../../../std/stdApp/Db/scan.db", "P=stdTest:,MAXPTS1=2000,MAXPTS2=200,MAXPTS3=20,MAXPTS4=10,MAXPTSH=10")

# Free-standing user string/number calculations (sCalcout records)
dbLoadRecords("../../../std/stdApp/Db/userStringCalcs10.db", "P=stdTest:")

# Free-standing user transforms (transform records)
dbLoadRecords("../../../std/stdApp/Db/userTransforms10.db", "P=stdTest:")

# Free-standing user menu choices (mbbo records)
#dbLoadRecords("../../../std/stdApp/Db/userMbbos10.db", "P=stdTest:")

# Miscellaneous PV's, such as burtResult
dbLoadRecords("../../../std/stdApp/Db/misc.db", "P=stdTest:")

# Femto amplifier
#dbLoadRecords("../../../std/femto.db","P=stdTest:,H=fem01:,F=seq01:")

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
#seq &femto,"name=fm1,P=stdTest:,H=fem01:,F=seq01:,G1=,G2=,G3=,NO="


### Start up the autosave task and tell it what to do.
# The task is actually named "save_restore".
# (See also, 'initHooks' above, which is the means by which the values that
# will be saved by the task we're starting here are going to be restored.
#
# Load the list of search directories for request files
< ../requestFileCommands

# save positions every five seconds
create_monitor_set("auto_positions.req", 5)
# save other things every thirty seconds
create_monitor_set("auto_settings.req", 30)

# Enable user string calcs and user transforms
dbpf "stdTest:EnableUserTrans.PROC","1"
dbpf "stdTest:EnableUserSCalcs.PROC","1"

