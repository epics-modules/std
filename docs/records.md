---
layout: default
title: Record Support
nav_order: 3
has_children: true
---


# Record Support

The std module provides three custom EPICS record types.

## [EPID Record](epidRecord.md)

The EPID record is an enhanced PID (Proportional-Integral-Derivative) feedback
record. It maintains a setpoint and adjusts an output based on the difference
between a controlled value and that setpoint. Compared to the standard EPICS
base PID record, the EPID record adds:

- Separation of device support from the record, allowing "Soft Channel",
  "Async Soft Channel", and "Fast Epid" device types
- Absolute output computation (not differential), enabling direct limit
  checking on the output
- Integral term limiting to prevent windup
- Drive limit fields (`DRVH`, `DRVL`) and an output link (`OUTL`) built into
  the record
- Support for asynchronous readback devices via trigger/callback fields
  (`TRIG`, `TVAL`)

## [Throttle Record](throttleRecord.md)

The throttle record enforces a minimum delay between successive value changes
sent to an output PV. It was created to protect devices that need time to
stabilize after a change -- for example, a laser delay generator that requires
a settling period. If a value is changed faster than the configured delay, the
record holds the latest value and sends it when the delay period has elapsed.
Features include:

- Configurable minimum delay between output writes
- Optional drive limits with clipping
- Synchronization input for reading back the current target value
- Transparent operation when values change slower than the delay

## [Timestamp Record](timestampRecord.md)

The timestamp record captures the current time and formats it as a string. It
provides eleven selectable format styles via the `TST` menu field, ranging from
simple `HH:MM` to full date-time with nanosecond precision. Formats include
US, European, VMS, and ISO-like styles. The record stores both the formatted
string (`VAL`) and the raw seconds past EPICS epoch (`RVAL`).
