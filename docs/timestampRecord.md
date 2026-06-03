---
layout: default
title: Timestamp Record
nav_order: 6
---


# Timestamp Record

Author: Susumu Yoshida

## Introduction

The timestamp record captures the current time and formats it as a string. It
provides eleven selectable time format styles via the `TST` menu field, ranging
from simple `HH:MM` to full date-time with nanoseconds. This is useful for
embedding human-readable timestamps in databases, display screens, or log
entries.


## Record Fields

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|-------|---------|------|-----|---------|--------|--------|------------------|----|
| VAL | Formatted timestamp string | STRING [40] | No | | Yes | No | Yes | No |
| OVAL | Previous timestamp string | STRING [40] | No | | Yes | No | No | No |
| RVAL | Seconds past EPICS epoch | ULONG | No | 0 | Yes | No | Yes | No |
| TST | Timestamp format selection | MENU (timestampTST) | Yes | 0 | Yes | Yes | No | Yes |


## Timestamp Formats

The `TST` field selects one of eleven output formats:

| TST Value | Menu Label | Format | Example |
|-----------|-----------|--------|---------|
| 0 | `YY/MM/DD HH:MM:SS` | `%y/%m/%d %H:%M:%S` | `25/06/03 14:30:45` |
| 1 | `MM/DD/YY HH:MM:SS` | `%m/%d/%y %H:%M:%S` | `06/03/25 14:30:45` |
| 2 | `Mon DD HH:MM:SS YY` | `%b %d %H:%M:%S %y` | `Jun 03 14:30:45 25` |
| 3 | `Mon DD HH:MM:SS` | `%b %d %H:%M:%S` | `Jun 03 14:30:45` |
| 4 | `HH:MM:SS` | `%H:%M:%S` | `14:30:45` |
| 5 | `HH:MM` | `%H:%M` | `14:30` |
| 6 | `DD/MM/YY HH:MM:SS` | `%d/%m/%y %H:%M:%S` | `03/06/25 14:30:45` |
| 7 | `DD Mon HH:MM:SS YY` | `%d %b %H:%M:%S %y` | `03 Jun 14:30:45 25` |
| 8 | `DD-Mon-YYYY HH:MM:SS` | `%d-%b-%Y %H:%M:%S` | `03-Jun-2025 14:30:45` |
| 9 | `Mon DD, YYYY HH:MM:SS.ns` | `%b %d %Y %H:%M:%S.%03f` | `Jun 03 2025 14:30:45.123` |
| 10 | `MM/DD/YY HH:MM:SS.ns` | `%m/%d/%y %H:%M:%S.%03f` | `06/03/25 14:30:45.123` |

Formats 9 and 10 include millisecond precision (`.%03f`).


## Record Processing

When the record processes:

1. The current time is obtained. If `TSE` is set to `epicsTimeEventDeviceTime`,
   the OS time is used via `time()`; otherwise `recGblGetTimeStamp()` is used.
2. The `RVAL` field is set to `secPastEpoch` from the timestamp.
3. If `secPastEpoch` is zero (no valid time source), `VAL` is set to `"-NULL-"`.
4. Otherwise, the time is formatted according to the `TST` selection using
   `epicsTimeToStrftime()` and stored in `VAL`.
5. Monitors are posted (`DBE_VALUE|DBE_LOG`) only when `VAL` differs from `OVAL`
   (the previously posted value).


## Usage

Load the timestamp record from your IOC startup script:

```
dbLoadRecords("$(STD)/stdApp/Db/timestamp.db", "P=xxx:,R=timestamp1")
```

Or instantiate directly in a database:

```
record(timestamp, "$(P)myTimestamp") {
    field(SCAN, "1 second")
    field(TST,  "8")
}
```

In this example, TST=8 selects the VMS-style format `DD-Mon-YYYY HH:MM:SS`.


## Autosave

The timestamp record has no associated autosave request file, as the only
configurable field (`TST`) is typically set at load time.
