throttle - Record for Throttling Value Changes
==============================================

1. Introduction
===============

This record was created for solving the problem of an output floating-point value being changed too fast for a device that that is referencing that value. As an example, a laser system can have a time delay specified for a delay generator, and after changing the delay it takes some period of time for the laser system to stabilize; if the delay is changed again before the stabilization occurs, the laser could do something unintended. The throttle record enforces a delay.

 The sort of behavior that this record is really targetting is human behavior, where the user of a control might tweak things at a rate that the system doesn't want. Instead of trying to change human behavior, this record protects the controls in a minimally disruptive way.

2. Operation
============

If the value is naturally changed slower than the specified delay, the throttle record is completely transparent. The record only cares that the period is enforced, so if the time has already passed sinced the last change, the new change takes immediate effect.

 When a change has occurred within the period, the new value is stored and then used when the period is over. If multiple changes are made within the period, the __last__ change is used; changes are not accumulated in a queue and used sequentially over several periods.

 Limits can be specified for the record, by making the low and high limits have a positive difference. A status flag is set for when the record hits a limit, signifying which limit. If the clipped flag is set, then when a limit is hit, the limit value is used instead of not changing the value.

3. Record Fields
================

The __VAL__ field holds the current proposed value to send to the __OUT__ link. When this value is sent is determined by how long ago a value was previously sent, and the __DLY__ delay value. If a value was sent longer than the daly value ago, the value gets sent immediately, otherwise the latest __VAL__ value gets sent once the time since the last time a value was sent has been reached. The value that was last sent is shown in the __SENT__ field. If a value is currently being delayed, then __WAIT__ is "True".

The __VAL__ field can be limited, used when the __DRVLH__ high limit is set to a larger value than the __DRVLL__ low limit. If the value falls outside the limit range, it will be reset to the relevant limit, and the __DRVLS__ status will be set to either "Low Limit" or "High Limit". If __DRVLC__ is set to "On", then when a limit is hit, the limit value is used as the value in addition to the status changed.

As a convenience, the record has a method of synchronizing the __VAL__ value to a reference PV, without processing the record. The reference PV is stored in __SINP__, and its validity can be checked with __SIV__. To activate the synchronization, __SYNC__ needs to be set to "Process".

The __PREC__ field defines the resolution of __VAL__, while the __DPREC__ field defines the resolution of __DLY__.

The status of the record is held in __STS__, mainly relating to the link fields. The version of the record is kept in __VER__ as a string, in the example form of "0-9-1".

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| VAL | Value to Send | DOUBLE | No | 0.0 | Yes | Yes | Yes | Yes |
| OVAL | Previous Set Value | DOUBLE | No | 0.0 | Yes | No | No | No |
| DLY | Minimum Delay | DOUBLE | Yes | 0.0 | Yes | Yes | Yes | No |
| WAIT | Waiting Status | Menu: False/True | No | False | Yes | No | Yes | No |
| OUT | PV to Send Value | Link | Yes |  | Yes | Yes | Yes | No |
| OV | Output Link Validity | Menu: Ext PV NC/Ext PV OK/Local PV/Constant | No | Ext PV OK | Yes | No | No | No |
| SENT | Value Last Sent | DOUBLE | No | 0.0 | Yes | No | No | No |
| OSENT | Old Value Last Sent | DOUBLE | No | 0.0 | Yes | No | No | No |
| DRVLH | High Drive Limit | DOUBLE | Yes | 0.0 | Yes | Yes | Yes | No |
| DRVLL | Low Drive Limit | DOUBLE | Yes | 0.0 | Yes | Yes | Yes | No |
| DRVLS | Drive Limit Status | Menu: Normal/Low Limit/High Limit | No | Normal | Yes | No | No | No |
| DRVLC | Drive Limit Clipping | Menu: Off/On | Yes | Off | Yes | Yes | Yes | No |
| SINP | Sync Input | Link | Yes |  | Yes | Yes | Yes | No |
| SIV | Sync Input Validity | Menu: Ext PV NC/Ext PV OK/Local PV/Constant | No | Ext PV OK | Yes | No | No | No |
| SYNC | Synchronize Values | Menu: Idle/Process | No | Idle | Yes | Yes | Yes | No |
| PREC | Value Precision | SHORT | Yes | 6 | Yes | Yes | No | No |
| DPREC | Delay Precision | SHORT | Yes | 3 | Yes | Yes | No | No |
| STS | Record Status | Menu: Unknown/Error/Success | No | Unknown | Yes | No | No | No |
| HOPR | High Operating Range | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| LOPR | Low Operating Range | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| VER | Record Version | STRING | Yes |  | Yes | No | No | No |


4. Record Support Routines
==========================

### init\_record

The timing callbacks are configured, and the OUT link is checked. The DRVH and DRVL limits are checked to see if a positive range is defined.

### process

The validity of the OUT output link is checked to make sure it is a connected PV.

 The VAL field is checked against the DRVH and DRVL limits (if they are engaged). If the value is out of bounds, the VAL value is clipped back to the relevant limit value, and the DRVCS status is set to "Clipped".

 The timing system then runs. If there has been no value sent to the OUT link in DLY seconds, then the VAL value is sent to OUT immediately. Otherwise, an internal flag is set that triggers the value being sent to OUT when the delay has been met.

 The SENT field is whatever was last sent to OUT.

### special

Changing the DLY delay will cause the current timed callback (used for timing) to be canceled, and a new one with the current DLY value will be started to replace it. This is to make sure if someone changes the value to a very long value, then wants to make it much shorter, you don't have to wait that long delay.

 Changing the DRVH and DRVL limits will trigger a check of the range to make sure that the range is positive. If positive, a check is made to see if the current value is out of bounds or not. If it is, VAL is not immediately changed, but any further processing will cause it to be clipped back into range.

- - - - - -

 Dohn Arms  
 Advanced Photon Source
