---
layout: default
title: Soft Motor
nav_order: 6
---


# Soft Motor Support
{: .no_toc}

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

The std module provides two databases that create EPICS motor records backed by
soft-channel device support. A soft motor presents any position-based PV (a
real motor, a DAC, a calculated pseudo-axis) through the standard motor record
interface, giving it all the motor record features: backlash compensation,
retries, tweaking, and integration with the scan system.

## Choosing the Right Variant

| | `softMotor.db` | `softMotorTf.db` |
|---|---|---|
| **Link setup** | Automated (link parsers assemble full link strings from bare PV names) | Manual (user writes full link strings including attributes) |
| **Post-iocInit init** | Self-initializing (dfanout fires once after iocInit) | None (relies on autosave or manual `caput`) |
| **Drive/stop mechanism** | `calcout` with 1 channel | `transform` with 16 channels (A-P) |
| **Readback mechanism** | `calcout` with 12 channels | `calcout` with 12 channels |
| **Coordinate transforms** | Simple (single `CALC` expression) | Complex (16-channel transform) |
| **Multi-actuator fan-out** | No (single output link) | Yes (up to 16 output links) |

Both variants configure their target PVs at runtime (not at `dbLoadRecords`
time). The difference is how: `softMotor.db` has a convenience layer that
assembles the link strings for you, while `softMotorTf.db` requires you to
write complete link strings (including `PP`, `CP`, `MS` attributes) directly
into the transform/calcout fields.

### When to use each

- **`softMotor.db`** -- Use when you need a simple 1:1 mapping between the soft
  motor and a single real PV, and you want the convenience of writing just the
  PV name (without link attributes) into a single field. Good for ad-hoc setups
  where the target may change frequently.

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

Load the database:

```
dbLoadRecords("$(STD)/stdApp/Db/softMotor.db", "P=xxx:,SM=SM1")
```

Then configure the target PVs at runtime (via the display screen, or autosave
will restore them on reboot). To connect soft motor `xxx:SM1` to real motor
`xxx:m1`:

| Field | Value |
|-------|-------|
| `xxx:SM1CalcFrwdOutput.AA` | `xxx:m1.VAL` |
| `xxx:SM1CalcRevsInput.AA` | `xxx:m1.RBV` |
| `xxx:SM1CalcMoveInput.AA` | `xxx:m1.DMOV` |
| `xxx:SM1CalcStopOutput.AA` | `xxx:m1.STOP` |

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

Configure the transform and calcout records' expressions and links directly
(via the display screen or autosave). The motor's commanded position arrives in
`DriveTf` channel A.

**Example: Simple 1:1 mapping** -- connect soft motor `xxx:SM1` to real motor
`xxx:m1`:

| Record | Field | Value |
|--------|-------|-------|
| `xxx:SM1DriveTf` | CLCA | `a` |
| | OUTA | `xxx:m1.VAL PP MS` |
| `xxx:SM1CalcRdbk` | INPA | `xxx:m1.RBV CP MS` |
| | CALC | `A` |
| `xxx:SM1CalcMove` | INPA | `xxx:m1.DMOV CP MS` |
| | CALC | `!A` |
| `xxx:SM1StopTf` | CLCA | `1` |
| | OUTA | `xxx:m1.STOP PP MS` |

**Example: Unit conversion (mm to microns)** -- soft motor in mm driving a
piezo controller in microns:

| Record | Field | Value |
|--------|-------|-------|
| `xxx:SM1DriveTf` | CLCA | `a*1000` |
| | OUTA | `xxx:piezo.VAL PP MS` |
| `xxx:SM1CalcRdbk` | INPA | `xxx:piezo.RBV CP MS` |
| | CALC | `A/1000` |

**Example: Coupled two-motor drive** -- a single soft motor driving two real
motors using a gap/center transform with an offset read from another PV:

| Record | Field | Value |
|--------|-------|-------|
| `xxx:SM1DriveTf` | INPB | `xxx:gapOffset PP MS` |
| | CLCA | `a+b` |
| | OUTA | `xxx:m1.VAL PP MS` |
| | CLCB | `b` |
| | CLCC | `a-b` |
| | OUTC | `xxx:m2.VAL PP MS` |

### Loading

```
dbLoadRecords("$(STD)/stdApp/Db/softMotorTf.db", "P=xxx:,SM=SM1,DESC=Soft Axis 1")
```

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
