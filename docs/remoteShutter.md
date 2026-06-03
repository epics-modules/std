---
layout: default
title: Remote Shutter
nav_order: 11
---


# Remote Shutter

## Overview

The `remoteShutter.db` database provides remote open/close control of beamline
hutch shutters using a remote shutter control box connected via digital output
hardware (e.g., IpUnidig).


## Macros

| Macro | Description | Example |
|-------|-------------|---------|
| `P` | PV prefix | `xxx:` |
| `BL` | Beamline number (2 digits) | `08` |
| `S` | Sector hutch name | `A`, `B`, `C`, or `D` |
| `PPS` | PPS hutch name (usually the same as `S`) | `A` |
| `OUT` | Digital output number, where `$(P)UnidigBo$(OUT)` is the physical output PV (see IpUnidig.substitutions) | `1` |


## Startup Configuration (st.cmd)

```
# Remote Shutter Control
#    Used to 'Open' and 'Close' the shutters using the remote shutter
#    control box.

dbLoadRecords("$(STD)/stdApp/Db/remoteShutter.db", "P=xxx:,BL=08,S=A,PPS=A,OUT=1")
dbLoadRecords("$(STD)/stdApp/Db/remoteShutter.db", "P=xxx:,BL=08,S=B,PPS=B,OUT=2")
dbLoadRecords("$(STD)/stdApp/Db/remoteShutter.db", "P=xxx:,BL=08,S=C,PPS=C,OUT=3")
dbLoadRecords("$(STD)/stdApp/Db/remoteShutter.db", "P=xxx:,BL=08,S=D,PPS=D,OUT=4")
```

The database monitors the APS operations PV `PS:$(BL)ID:$(S1)_SHTRS_CLOSED`
to determine the current shutter state, and uses `$(P)UnidigBo$(OUT)` as the
physical digital output to control the shutter.
