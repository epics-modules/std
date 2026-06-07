---
layout: default
title: Timers & Scheduling
nav_order: 7
---


# Timers & Scheduling
{: .no_toc}

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

The std module provides three databases for time-based triggering. They serve
different use cases:

## Choosing the Right Timer

| | `timer.db` | `countDownTimer.vdb` | `alarmClock.vdb` |
|---|---|---|---|
| **Trigger condition** | Elapsed time reaches preset | Count reaches target | Current time reaches set date/time |
| **Resolution** | 0.1 second | 1 second | 1 second |
| **Time reference** | Elapsed time (relative) | Count-based (relative) | Wall clock (absolute) |
| **Date awareness** | No | No | Yes (month, day, year, leap years) |
| **Busy record** | Yes | No | No |
| **Scan integration** | Yes (`ca_put_callback`) | No | No |
| **Preset unit** | Seconds (floating point) | Hours, minutes, seconds | Date and time fields |
| **Count direction** | Elapsed + remaining | Up or down (selectable) | N/A (comparison-based) |
| **Reset behavior** | Resets on Go | Reloads on Start | Re-arms on Enable toggle |

### When to use each

- **`timer.db`** -- General-purpose timer with scan system integration. Use when
  you need an action after a precise delay and want `ca_put_callback` support
  (e.g., timed delays in scan sequences). The `doStop` forward link is
  explicitly provided for user actions.

- **`countDownTimer.vdb`** -- Simple countdown (or count-up) timer. Use when you
  need a visual hours:minutes:seconds countdown display. Often used as a
  building block by other databases (e.g., `autoShutter.vdb` uses two
  instances).

- **`alarmClock.vdb`** -- Scheduled event trigger. Use when an action must occur
  at a specific date and time (e.g., "close the shutter at 6:00 AM on Monday").
  Includes full date/time entry with range validation and leap-year awareness.


## timer.db

A general-purpose elapsed/countdown timer with `busy` record support.

### Macros

| Macro | Description |
|-------|-------------|
| `P` | PV prefix |
| `N` | Timer instance number |

### Records

| Record | Type | Purpose |
|--------|------|---------|
| `$(P)timer$(N):Go` | `bo` | Start (1) / Stop (0) control |
| `$(P)timer$(N):preset` | `ao` | Timer duration in seconds |
| `$(P)timer$(N):elapsedSecs` | `ao` | Elapsed time readback |
| `$(P)timer$(N):remainingSecs` | `ao` | Remaining time readback |
| `$(P)timer$(N):currentTime` | `stringin` | Current date/time string |
| `$(P)timer$(N):startTime` | `stringin` | Captured start timestamp |
| `$(P)timer$(N):doStop` | `bo` | Stop trigger (**user should set FLNK**) |
| `$(P)timer$(N):busy` | `busy` | Busy record for scan synchronization |
| `$(P)timer$(N):resetOnGo` | `bo` | Resets elapsed time on start |
| `$(P)timer$(N):secsPastEpoch` | `ai` | Epoch time source |
| `$(P)timer$(N):calcElapsed` | `transform` | Core timing engine |
| `$(P)timer$(N):doneCalc` | `calcout` | Completion detector |
| `$(P)timer$(N):update` | `bo` | 0.1s scan driver (while running) |
| `$(P)timer$(N):updateCurrent` | `bo` | 1s scan driver (while stopped) |
| `$(P)timer$(N):scanFanout` | `fanout` | Triggers time reads |
| `$(P)timer$(N):setBusy` | `calcout` | Sets busy on Go |
| `$(P)timer$(N):clearBusy` | `bo` | Clears busy on done |
| `$(P)timer$(N):doStop1` | `bo` | Internal stop chaining |

### How It Works

1. Writing 1 to `Go` resets the timer, captures the start time, and sets the
   `busy` record.
2. The `update` record scans at 0.1 seconds (only while `Go`=1) and triggers
   the timing engine.
3. `calcElapsed` computes elapsed time (current epoch - start epoch) and
   remaining time (preset - elapsed).
4. When remaining time reaches 0.05 seconds or less, `doneCalc` fires:
   - Processes `doStop` (which the user can chain to their action via FLNK)
   - Clears the `busy` record
   - Sets `Go` back to 0
5. The `busy` record integrates with `sscan` and `ca_put_callback` for timed
   delays in scan sequences.

### User Action on Timer Expiry

The `doStop` record's FLNK is intentionally left empty. Set it to the PV you
want processed when the timer expires:

```
caput xxx:timer1:doStop.FLNK "xxx:myAction.PROC"
```

Or configure via autosave so it persists across IOC reboots.

### Example

```
dbLoadRecords("$(STD)/stdApp/Db/timer.db", "P=xxx:,N=1")
```

### Autosave

Use `timer_settings.req` (or `timer.req`) with macros `P=$(P)`, `N=$(N)`. Saves
preset, Go state, update scan rate, doStop FLNK, start time, and latched epoch.


## countDownTimer.vdb

A count-based timer that increments or decrements a counter every second.

### Macros

| Macro | Description |
|-------|-------------|
| `P` | PV prefix |
| `N` | Timer instance number |

### Records

| Record | Type | Purpose |
|--------|------|---------|
| `$(P)cdt$(N):start` | `bo` | Start (1) / Stop (0) |
| `$(P)cdt$(N):upDown` | `bo` | Direction: UP (0) / DOWN (1, default) |
| `$(P)cdt$(N):setTimeHrs` | `ao` | Preset hours |
| `$(P)cdt$(N):setTimeMin` | `ao` | Preset minutes |
| `$(P)cdt$(N):setTimeSec` | `ao` | Preset seconds |
| `$(P)cdt$(N):timeHrs` | `ao` | Current hours (display) |
| `$(P)cdt$(N):timeMin` | `ao` | Current minutes (display) |
| `$(P)cdt$(N):timeSec` | `ao` | Current seconds (display) |
| `$(P)cdt$(N):count` | `calcout` | Core counter (SCAN toggled dynamically) |
| `$(P)cdt$(N):countReset` | `calcout` | Loads preset on start |
| `$(P)cdt$(N):stopStart` | `calcout` | Start/stop logic (writes to count.SCAN) |
| `$(P)cdt$(N):trigger` | `calcout` | Detects stop transition |
| `$(P)cdt$(N):startReset` | `calcout` | Resets start flag when done |
| `$(P)cdt$(N):reset` | `calcout` | Converts H:M:S to total seconds |
| `$(P)cdt$(N):parse` | `transform` | Converts total seconds back to H:M:S |

### How It Works

1. The user sets hours, minutes, and seconds. The `reset` record converts these
   to total seconds.
2. Writing 1 to `start` triggers `countReset`, which loads the total seconds
   into `count` (for countdown) or 0 (for count-up).
3. `stopStart` sets `count.SCAN` to `"1 second"` (scan index 6), enabling
   periodic processing.
4. Each second, `count` adds -1 (down) or +1 (up) to its current value.
5. `parse` (linked via CP) converts the current count to hours, minutes, and
   seconds for display.
6. `stopStart` (linked via CP) monitors the count. When it reaches 0 (countdown)
   or the preset (count-up), it writes 0 to `count.SCAN`, stopping the timer.
7. `trigger` detects the stop transition and `startReset` clears the `start`
   flag.

### Example

```
dbLoadRecords("$(STD)/stdApp/Db/countDownTimer.vdb", "P=xxx:,N=1")
```


## alarmClock.vdb

A date/time alarm clock that triggers when the current time reaches a
user-specified date and time.

### Macros

| Macro | Description |
|-------|-------------|
| `P` | PV prefix |
| `N` | Alarm clock instance number |

### Key Records

| Record | Type | Purpose |
|--------|------|---------|
| `$(P)AClock$(N):Enable` | `bo` | Master enable (Disable/Enable) |
| `$(P)AClock$(N):Trigger` | `calcout` | Fires when current time >= set time |
| `$(P)AClock$(N):curTimeDate` | `stringin` | Current date/time (1-second scan) |
| `$(P)AClock$(N):Set:mm` | `ao` | Set month (1-12) |
| `$(P)AClock$(N):Set:dd` | `ao` | Set day (1-31, validated per month) |
| `$(P)AClock$(N):Set:yy` | `ao` | Set year (0-99, 2-digit) |
| `$(P)AClock$(N):Set:HH` | `ao` | Set hour (0-24) |
| `$(P)AClock$(N):Set:MM` | `ao` | Set minute (0-60) |
| `$(P)AClock$(N):Set:SS` | `ao` | Set second (0-60) |
| `$(P)AClock$(N):UD:mm` | `bo` | Month up/down button |
| `$(P)AClock$(N):UD:dd` | `bo` | Day up/down button |
| `$(P)AClock$(N):UD:yy` | `bo` | Year up/down button |
| `$(P)AClock$(N):UD:HH` | `bo` | Hour up/down button |
| `$(P)AClock$(N):UD:MM` | `bo` | Minute up/down button |
| `$(P)AClock$(N):UD:SS` | `bo` | Second up/down button |
| `$(P)AClock$(N):Chk:mm` ... `:SS` | `transform` | Range validation with wrap-around |
| `$(P)AClock$(N):TOD:yy` ... `:SS` | `scalcout` | Time-of-day field parsers/comparators |
| `$(P)AClock$(N):Set` | `scalcout` | Alarm time display string |
| `$(P)AClock$(N):SetNow` | `sseq` | Set alarm to current time |

The database contains approximately 33 records total.

### How It Works

1. The user sets a target date and time using the `Set:` records (directly or
   via the up/down buttons).

2. Range-check transforms (`Chk:`) validate each field. Values wrap around at
   boundaries (e.g., month 13 wraps to 1). The month check dynamically adjusts
   the day limit based on the month and leap year status.

3. Every second, `curTimeDate` reads the current time as a formatted string
   (`mm/dd/yy HH:MM:SS`).

4. Six `TOD:` scalcout records parse individual fields from the time string and
   compare them to the corresponding set values. Each produces a cascading
   comparison result:
   - `0` = current is before set (alarm not reached for this field)
   - `1` = current equals set (check the next finer field)
   - `2` = current is past set (alarm already passed)

   If a coarser field (e.g., month) returns `2`, all finer fields (day, hour,
   minute, second) automatically return `2`.

5. The `Trigger` calcout ANDs all comparison results with the Enable flag. It
   fires on "Transition To Non-zero" -- exactly once when the alarm time is
   reached.

6. Configure `Trigger.OUT` or `Trigger.FLNK` for the desired alarm action.

### Resetting the Alarm

The trigger fires on "Transition To Non-zero", so it will only fire once. To
re-arm:
- Toggle `Enable` off and back on
- Change the set time to a future value
- Use `SetNow` to reset to the current time, then adjust forward

### Example

```
dbLoadRecords("$(STD)/stdApp/Db/alarmClock.vdb", "P=xxx:,N=1")
```

Then configure the alarm action at runtime:

```
caput xxx:AClock1:Set:mm 12
caput xxx:AClock1:Set:dd 25
caput xxx:AClock1:Set:yy 25
caput xxx:AClock1:Set:HH 6
caput xxx:AClock1:Set:MM 0
caput xxx:AClock1:Set:SS 0
caput xxx:AClock1:Enable 1
```
