---
layout: default
title: Auto Shutter
nav_order: 11
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


## Release Notes

### Version 1.1

Initial release, installed in synApps v5.2.

### Version 1.2 (synApps v5.2.1)

- Redefined `autoShutter.vdb` `BL` macro argument to include the `ID` or `BM`
  identifier. `BL` is now defined as a four-character beamline ID:
  `<Sector#><ID/BM>` (example: `08ID`). See comments in
  `xxx/iocBoot/iocvxWorks/autoShutter.substitutions`.

- Removed `Shtr:Trigger` OUT link to the remoteShutter Open PV. The OUT link
  should be programmed at run time to allow for more flexibility in how the
  shutter is opened. The OUT field is now in the backup/restore request file.
  The OUT link is calculated by `shtr:CalcTrigOutput` to ensure that the PP
  attribute is on the PV link to the shutter Open record. The Trigger OUT link
  is checked by `Shtr:TrigOutOK` for a valid PV. The `Shtr:Arm` calc now
  includes the TrigOutOK check.

- Renamed old `Shtr:Open` PV to `Shtr:Active` and changed the ZNAM and ONAM to
  "NO" and "YES".

- Changed `autoShutter.vdb` `ALM` macro argument to `A` to be consistent with
  MEDM argument list.

- MEDM `autoShutter.adl`:
    - Shutter Control "Edit" and status fields added to reflect the changes to
      `Shtr:Trigger.OUT` and `Shtr:Arm`.
    - Changed "Last Open Timer" label to "Open Notify Timer".
    - Added "Shutter Control" section to allow programming of "Remote Shutter
      Control" PV. Red/Green status indicates if PV has been programmed or not.
