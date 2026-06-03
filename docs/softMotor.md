---
layout: default
title: Soft Motor
nav_order: 8
---


# Soft Motor Support

The std module provides two databases that create EPICS motor records backed by
soft-channel device support. A soft motor presents any position-based PV (a
real motor, a DAC, a calculated pseudo-axis) through the standard motor record
interface, giving it all the motor record features: backlash compensation,
retries, tweaking, and integration with the scan system.

## Choosing the Right Variant

| | `softMotor.db` | `softMotorTf.db` |
|---|---|---|
| **Link configuration** | Dynamic at runtime | Static at load time (or via autosave) |
| **Drive/stop mechanism** | `calcout` with 1 channel | `transform` with 16 channels (A-P) |
| **Readback mechanism** | `calcout` with 12 channels | `calcout` with 12 channels |
| **Coordinate transforms** | Simple (single `CALC` expression) | Complex (16-channel transform) |
| **Multi-actuator fan-out** | No (single output link) | Yes (up to 16 output links) |
| **Self-initializing** | Yes (dfanout fires after iocInit) | No (links set via autosave or load-time) |
| **Record count** | 14 | 7 |

### When to use each

- **`softMotor.db`** -- Use when you need a simple 1:1 mapping between the soft
  motor and a single real PV, and you want to configure the target PV names at
  runtime by typing them into PV fields. Good for ad-hoc setups where the
  target may change.

- **`softMotorTf.db`** -- Use when you need coordinate transformations (unit
  conversion, linear scaling, coupled axes) or need to fan out a single soft
  motor command to multiple real actuators. The transform record provides 16
  channels with per-channel expressions and I/O links.


## softMotor.db -- Dynamic Link Soft Motor

### Macros

| Macro | Description |
|-------|-------------|
| `P` | PV prefix |
| `SM` | Soft motor name |

### Records

| Record | Type | Purpose |
|--------|------|---------|
| `$(P)$(SM)` | `motor` | The soft motor (DTYP: Soft Channel) |
| `$(P)$(SM)_able` | `bo` | Enable/disable switch |
| `$(P)$(SM)_ableput` | `transform` | Writes enable state to motor DISA/DISP |
| `$(P)$(SM)Init` | `dfanout` | One-shot post-iocInit link assembly |
| `$(P)$(SM)CalcFrwdOutput` | `scalcout` | Parses drive output PV name |
| `$(P)$(SM)CalcFrwd` | `calcout` | Forward (drive) calculation |
| `$(P)$(SM)CalcRevsInput` | `scalcout` | Parses readback input PV name |
| `$(P)$(SM)CalcRevs` | `calcout` | Reverse (readback) calculation |
| `$(P)$(SM)MoveLogic` | `bo` | Done-moving signal polarity (Invert/Same) |
| `$(P)$(SM)CalcMoveLogic` | `scalcout` | Sets CalcMove CALC expression |
| `$(P)$(SM)CalcMoveInput` | `scalcout` | Parses done-moving input PV name |
| `$(P)$(SM)CalcMove` | `calcout` | Done-moving calculation |
| `$(P)$(SM)CalcStopOutput` | `scalcout` | Parses stop output PV name |
| `$(P)$(SM)CalcStop` | `calcout` | Stop command output |

### Signal Flow

```
User command --> motor $(P)$(SM)
    |-- OUT  --> CalcFrwd --> [dynamic link] --> real drive PV
    |-- RDBL <-- CalcRevs <-- [dynamic link] <-- real readback PV (CP)
    |-- DINP <-- CalcMove <-- [dynamic link] <-- real done-moving PV (CP)
    |-- STOO --> CalcStop --> [dynamic link] --> real stop PV
```

### Configuration

After loading the database, write the target PV names into the `AA` fields of
the link-parser records. Do **not** include link attributes (`PP`, `CP`, etc.)
-- the parsers append these automatically.

| Field to set | Link attribute added | Purpose |
|---|---|---|
| `$(P)$(SM)CalcFrwdOutput.AA` | `PP MS` | Drive destination PV |
| `$(P)$(SM)CalcRevsInput.AA` | `CP MS` | Readback source PV |
| `$(P)$(SM)CalcMoveInput.AA` | `CP MS` | Done-moving source PV |
| `$(P)$(SM)CalcStopOutput.AA` | `PP MS` | Stop destination PV |

The `Init` dfanout processes all link-parser records once at IOC startup (scans
at 1 second, then self-disables by writing to its own DISA). This assembles the
CA links from the saved PV names.

Set `MoveLogic` to match the polarity of the upstream done-moving signal:
- **Invert** (default): upstream signal is 0 when done (e.g., motor `.DMOV`)
- **Same**: upstream signal is 1 when done

Customize the `CALC` expressions in `CalcFrwd` and `CalcRevs` for simple
transforms (default is `A` -- identity/passthrough).

### Example

```
dbLoadRecords("$(STD)/stdApp/Db/softMotor.db", "P=xxx:,SM=SM1")
```

Then at runtime (or via autosave):

```
caput xxx:SM1CalcFrwdOutput.AA "xxx:m1.VAL"
caput xxx:SM1CalcRevsInput.AA  "xxx:m1.RBV"
caput xxx:SM1CalcMoveInput.AA  "xxx:m1.DMOV"
caput xxx:SM1CalcStopOutput.AA "xxx:m1.STOP"
```

### Autosave

Use `softMotor_settings.req` with macros `P=$(P)`, `SM=$(SM)`. Saves all motor
fields, all calcout link/expression fields, the link-parser `AA` values, and
the `MoveLogic` setting.


## softMotorTf.db -- Transform Soft Motor

### Macros

| Macro | Default | Description |
|-------|---------|-------------|
| `P` | | PV prefix |
| `SM` | | Soft motor name |
| `DESC` | `$(P)$(SM)` | Motor description |

### Records

| Record | Type | Purpose |
|--------|------|---------|
| `$(P)$(SM)` | `motor` | The soft motor (DTYP: Soft Channel) |
| `$(P)$(SM)_able` | `bo` | Enable/disable switch |
| `$(P)$(SM)_ableput` | `transform` | Writes enable state to motor DISA/DISP |
| `$(P)$(SM)DriveTf` | `transform` | Drive output transform (16 channels) |
| `$(P)$(SM)CalcRdbk` | `calcout` | Readback calculation |
| `$(P)$(SM)CalcMove` | `calcout` | Done-moving calculation |
| `$(P)$(SM)StopTf` | `transform` | Stop command transform (16 channels) |

### Signal Flow

```
User command --> motor $(P)$(SM)
    |-- OUT  --> DriveTf.A --> [user OUTx links] --> real drive PV(s)
    |-- RDBL <-- CalcRdbk  <-- [user INPx links] <-- real readback PV(s) (CP)
    |-- DINP <-- CalcMove  <-- [user INPx links] <-- real done-moving PV(s) (CP)
    |-- STOO --> StopTf.PROC --> [user OUTx links] --> real stop PV(s)
```

### Configuration

Configure the transform and calcout records' expressions and links directly.
The motor's commanded position arrives in `DriveTf` channel A.

**Example: Simple 1:1 mapping**

```
# Drive: pass position to real motor
caput xxx:SM1DriveTf.CLCA  "a"
caput xxx:SM1DriveTf.OUTA  "xxx:m1.VAL PP MS"

# Readback: read from real motor
caput xxx:SM1CalcRdbk.INPA "xxx:m1.RBV CP MS"
caput xxx:SM1CalcRdbk.CALC "A"

# Done-moving: invert DMOV
caput xxx:SM1CalcMove.INPA "xxx:m1.DMOV CP MS"
caput xxx:SM1CalcMove.CALC "!A"

# Stop: send stop to real motor
caput xxx:SM1StopTf.CLCA   "1"
caput xxx:SM1StopTf.OUTA   "xxx:m1.STOP PP MS"
```

**Example: Unit conversion (mm to microns)**

```
caput xxx:SM1DriveTf.CLCA  "a*1000"
caput xxx:SM1DriveTf.OUTA  "xxx:piezo.VAL PP MS"
caput xxx:SM1CalcRdbk.INPA "xxx:piezo.RBV CP MS"
caput xxx:SM1CalcRdbk.CALC "A/1000"
```

**Example: Coupled two-motor drive**

```
# Drive both motors with a gap/center transform
caput xxx:SM1DriveTf.CLCA  "a+b"
caput xxx:SM1DriveTf.OUTA  "xxx:m1.VAL PP MS"
caput xxx:SM1DriveTf.INPB  "xxx:gapOffset PP MS"
caput xxx:SM1DriveTf.CLCB  "b"
caput xxx:SM1DriveTf.CLCC  "a-b"
caput xxx:SM1DriveTf.OUTC  "xxx:m2.VAL PP MS"
```

### Example

```
dbLoadRecords("$(STD)/stdApp/Db/softMotorTf.db", "P=xxx:,SM=SM1,DESC=Soft Axis 1")
```

### Autosave

Use `softMotorTf_settings.req` with macros `P=$(P)`, `SM=$(SM)`. Saves all
motor fields plus the complete state of both transform records (all 16 channels
of expressions, links, and comments) and both calcout records. References the
shared `basic_motor_settings.req` file which must be in the autosave search
path.


## Motor Record Defaults

Both databases create the motor record with these defaults:

| Field | Default | Description |
|-------|---------|-------------|
| `MRES` | 0.01 | Motor step size |
| `ERES` | 0.01 | Encoder step size |
| `RRES` | 1.0 | Readback step size |
| `VELO` | 100 | Velocity |
| `VBAS` | 25 | Base velocity |
| `ACCL` | 0.5 | Acceleration time |
| `PREC` | 4 | Display precision |
| `DHLM` | 1E10 | Dial high limit |
| `DLLM` | -1E10 | Dial low limit |
| `RTRY` | 0 | Retry count (disabled) |
| `TWV` | 1.0 | Tweak value |
| `URIP` | Yes | Use readback input for position |
