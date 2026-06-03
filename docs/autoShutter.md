---
layout: default
title: Auto Shutter
nav_order: 10
---


# Auto Shutter

## Overview

The `autoShutter.vdb` database provides automatic shutter control based on
synchrotron storage ring current. It integrates with the `countDownTimer.vdb`
(for pre-open delay and open-time tracking) and `alarmClock.vdb` (for
time-based lease control) databases.

The auto shutter system monitors the storage ring beam current and
automatically opens or closes the shutter based on configurable thresholds and
conditions. It includes arming logic, trigger output validation, and
integration with the remote shutter control system.


## Startup Configuration

### Database Loading (st.cmd)

```
# Auto-Shutter Open Substitution File
# P   = PV prefix
# S   = Control shutter (usually A)
# BL  = Beamline ID, 4-character format: <Sector#><ID/BM> (e.g., 08ID)
# T1  = Pre-open delay timer ID (usually 1, see countDownTimer.vdb)
# T2  = Post-open count timer ID (usually 2)
# A   = Lease alarm clock ID (usually 1, see alarmClock.vdb)
dbLoadTemplate("autoShutter.substitutions")
```

### Autosave/Restore

Add to your `auto_settings.req` file:

```
file autoShutter.req P=$(P), S=A, T1=1, T2=2, A=1
```

### MEDM Display

```
autoShutter.adl P=xxx:,S=A,T1=1,T2=2,A=1
```

**MEDM macro arguments:**

| Macro | Description |
|-------|-------------|
| `P` | PV prefix |
| `S` | Control shutter (usually A) |
| `T1` | CountDownTimer used for auto-open delay function |
| `T2` | CountDownTimer used for shutter open timer (time since shutter opened) |
| `A` | AlarmClock number used for the lease function |


## Notes

- The `BL` macro is a four-character beamline ID in the format
  `<Sector#><ID/BM>` (e.g., `08ID`).
- The `Shtr:Trigger` OUT link is configured at run time (not hardcoded in the
  database) to allow flexibility in how the shutter is opened. The OUT field is
  saved/restored via autosave.
