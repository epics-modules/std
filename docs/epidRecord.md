---
layout: default
title: EPID Record
nav_order: 4
---


EPID Record
===========

Author: Mark Rivers
-------------------

Contents
--------

- [Introduction](#Introduction)
- [Scan Parameters](<#Scan Parameters>)
- [Controlled Variable Parameters](<#Controlled Variable Parameters>)
- [Setpoint Parameters](<#Setpoint Parameters>)
- [Output Parameters](<#Output parameters>)
- [Feedback Parameters](<#Feedback Parameters>)
- [Feedback Tuning](<#Feedback Tuning>)
- [Operator Display Parameters](<#Operator Display Parameters>)
- [Alarm Parameters](<#Alarm Parameters>)
- [Monitor Parameters](<#Monitor Parameters>)
- [Run-Time Parameters](<#Run-time Parameters>)
- [Record Support Routines](<#Record Support Routines>)
- [Record Processing](<#Record Processing>)
- [Device Support](<#Device Support>)
- [Problems with the Standard EPICS PID Record](#Problems)

Introduction
------------


The EPID record is used to maintain a setpoint that affects the output according to the difference between a controlled value and the setpoint. It provides EPICS with a very flexible and powerful tool for peforming feedback. The EPID record is an Enhanced version of the EPICS PID record. The major improvements in the EPID record compared to the PID record include:

- Separation of device support from the record. There is "Soft Channel" device support which uses EPICS database links, and this is very similar to the PID record. However, the EPID record can also be used with other device support, for example to communicate with faster feedback software, or with hardware controllers. Asyn device support is provided for fast feedback (&gt; 1 kHz) using an Acromag IP330 ADC and a Systran DAC128V DAC.
- Addition of many fields (OUTL, DRVH, DRVL) to simplify construction of databases
- The PID expression is computed as an absolute number, rather than a differential number to be added to the present output value. This simplifies database construction, and also permits the record itself to perform limit checking on the output.
- Limits are placed on the magnitude of the integral term (I) which are lacking in the PID record.
- Monitors are posted for the CVAL field, which simplifies construction of user-interface tools, such as plotting.
- The CVL field has been renamed INP. This field can now be modified (a feature of EPICS R3.12 and higher), so that a single EPID record can be used to control different processes at different times.
- Changed the time units of the KI and KD terms from minutes to seconds
- Addition of fields to better support the use of an asynchronous readback device.

Each time the record is processed the setpoint is read and then the do\_pid() function in device support is called. The "Soft Channel" device support reads the controlled value, computes the PID expression and, if feedback is enabled, writes its output to the output link.

There are three terms of the PID expression: the proportional, the integral, and the derivative. Using the gain that exists for each of these terms, the user can weight each contribution according to the characteristics of the controlled variable.

The source code is available in a [tar file.](pub/epidRecord.tar)

Scan Parameters
---------------

The PID record has the standard fields (SCAN, etc.) for specifying under what circumstances it will be processed. The minimum delta time field (MDT) is a processing field particular to the PID record. It can be configured by the user or modified at run-time. It contains a floating point value which represents the minimum amount of time between record processing. The actual meaning of the SCAN and MDT fields depends upon which device support is used:

- "Soft Channel" device support. The SCAN field controls the rate at which the feedback calculation is actually performed. The maximum rate is typically 10 Hz. If the amount of time between the last time the record was processed an the current time is less than MDT, than the record is not processed. If MDT is left at its default value (0), the minimum delta will be equal to one clock tick.

- "Fast Epid" device support. The SCAN field controls the rate at which new parameters (KP, KI, KD, VAL, etc.) are sent to the Ip330PID server, and the rate at which the current feedback parameters (P, I, D, CVAL, etc.) are read back from the server and displayed. Thus the SCAN field controls the rate at which the EPID record takes "snapshots" of the feedback processing running on the fastPIDServer server. The feedback itself can actually be running much faster, i.e. 1 kHz or more. The MDT field is ignorred by this device support. The DT field is used to control the time per feedback loop on the fastPIDServer server. The DT field is also modified by the device support to reflect the actual time per feedback loop, since this may differ from the requested time.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| MDT | Minimum Delta Time | DOUBLE | Yes | 0 | Yes | Yes | No | No |

Controlled Variable Parameters
------------------------------

The meaning of the INP field depends upon what device support is used:

1. "Soft Channel" device support. The input link field (INP) is used to obtain the value of the controlled process variable, i.e., the value read into the CVAL field. The link must be a database link. If it is not a database link a MAJOR alarm is triggered when the record is processed. The INP field *can* be modified at run-time.
1. "Fast Epid" device support. The input link field (INP) is used to specify the location of the "Fast Epid" server which actually performs the feedback. The fastPIDServer server is configured in a startup-script file to specify which Ip330 ADC channel is used as the input from which the value is read into the CVAL field. The link must be a VME link, and *cannot* be modified at run-time.
| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| INP | Controlled Value Location (an input link) | INLINK | Yes | 0 | Yes | Yes | N/A | No |
| CVAL | Value of controlled variable | DOUBLE | No | 0 | Yes | No | Yes | No |

If "Async Soft Channel" device support is selected, the following two fields are used to support an asynchronous readback device. The device support is expected to accomplish this by writing the value of the TVAL field to the PV specified by the TRIG link field, using dbCaPutLinkCallback(). This is expected to cause the readback device to process. EPICS will send a callback when that processing completes, at which time device support will read the readback value just as normal "Soft Channel" support would. | Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| TRIG | Readback Trigger (an output link) | OUTLINK | Yes | 0 | Yes | Yes | N/A | No |
| TVAL | Value written to TRIG link | DOUBLE | Yes | 0 | Yes | Yes | N/A | No |

Setpoint Parameters
-------------------

The setpoint mode select (SMSL) and the setpoint location (STPL) fields determine where the setpoint value is obtained, which is held in the VAL field. The SMSL and STPL fields work just like the OMSL and DOL fields found in many output records (analog output, binary output, etc.).

The SMSL field has two choices: `supervisory` and `closed_loop`. When `supervisory`, the setpoint, i.e., VAL, can be set via database access. When `closed_loop`, the setpoint value is retrieved from the link specified in STPL, which must be a database link. STPL can also be a constant in which case VAL is set equal to the constant value when the record is initialized, and the value can be modified at run-time via database access or channel access.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| STPL | Setpoint Location (an input link) | INLINK | Yes | 0 | Yes | Yes | N/A | No |
| SMSL | Setpoint Mode select. | GBLCHOICE | Yes | 0 | Yes | Yes | No | No |
| VAL | Setpoint value | DOUBLE | No | 0 | Yes | Yes | Yes | Yes |

Output parameters
-----------------

There are two fields which control the output of the EPID record, FBON (Feedback On) and OUTL (Output Link). The meanings of the these fields depends upon what device support is used:

1. "Soft Channel" device support. The output link field (OUTL) is used to control where the computed output (OVAL) is sent. If OUTL is a database link and if FBON=1 then the output value (OVAL) is written to this link each time the record processes. If FBON=0, or if OUTL is not a database link then the device support still computes the OVAL but does not write OVAL to the output link.
1. "Fast Epid" device support. The output link field (OUTL) is not used. Rather, the fastPIDServer server is configured in a vxWorks startup-script file to specify which DAC128V DAC channel is used as the output. If FBON=1 then the computed output will be written to the DAC channel each time the feedback loop executes. The execution of the feedback loop is independent of, and typically much faster than the rate at which the EPID record processes. If FBON=0 then the feedback loop still executes, but it does not write the output to the DAC channel.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| OUTL | Output Location (an outlink) | OUTLINK | Yes | 0 | Yes | Yes | N/A | No |
| FBON | Feedback On or Off | MENU | Yes | Off | Yes | Yes | No | No |

Feedback Parameters
-------------------

The discrete from of the PID expression is as follows:

M(n) = P + I + D

P = KP \* E(n)

I = KP \* KI \* SUMi ( E(i) \* dT(n) )

D = KP \* KD \* ( (E(n) - (E(n-1) ) / dT(n) )

where

 M(n) = value of manipulated variable at *n*th instant. P = Proportional term I = Integral term D = Derivative term KP = Proportional gain KI = Integral gain KD = Derivative gain E(n) = Error at nth sampling instant SUMi = Sum from i=0 to i=n dT(n) = Time difference between n-1 instance and nth instanceThe terms KP, KI, and KD are the only terms configured by the user. These terms represent the gain for the proportional (KP), integral (KI) and the derivative (KD) terms. A field exists for each of these terms (the KP, KI, and KD fields). KP should be configured according to the characteristics of the controlled variable; KI should be set equal to the number of times per second that the integral contribution repeats the proportional contribution; and KD should equal the number of seconds until the derivative contribution repeats the proportional contribution. See the[ guide to feedback tuning](<#Feedback Tuning>)below for more information on setting KP, KI and KD.

The PID device support calculates the other terms of the expression. The "Soft Channel" and "Fast Epid" device support both calculate these as follows:

 E(n) Error at nth sampling instant, where the Error equals the setpoint minus the value of the controlled variable (VAL - CVAL). The ERR field in the record holds this value. P Proportional term, calculated from the above equation. The P field in the record holds this value. I Integral term, calculated from the above equation. The I field in the record holds this value. Note that the integral term is a sum from time=0 to the present time, and hence can grow extremely large if not subject to some "sanity checks". The EPICS PID record does not perform any such checks, which is a serious limitation. The EPID record device support performs the following checks to prevent the integral term from growing too large when feedback is not working correctly:> - I is not allowed to increase if the computed output, M(n), is at the high limit, DRVH.
> - I is now allowed to decrease if the computed output, M(n), is at the low limit, DRVL.
> - I is not allowed to be less than DRVL or greater than DRVH.
> - If KI is zero then I is set to zero. This allows "clearing" the integral term by setting KI to zero.
> - I can be modified from database access or channel access. This allows the user to set I to a specific value to improve response time, rather than waiting for the normal time constant associated with this term.

 D Derivative term. The D field in the record holds this value. M(n) This is the end result of the PID expression--the change in the manipulated value, which is the desired output. The OVAL field holds this value. However, OVAL is first limited to be no smaller than DRVL and no larger than DRVH. dT(n) This is the time difference between n and n-1, between current and last samplings. It is stored in the DT field. For the "Soft Channel" device support this field is Read-Only. For the "Fast Epid" device support this field can be modified to control the time per feedback loop.
 
 | Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| KP | Proportional Gain | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| KI | Integral Gain, in repeats per second. | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| KD | Derivative Gain, in repeats per second | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| ERR | Current error (VAL - CVAL) | DOUBLE | No | 0 | Yes | No | Yes | No |
| P | Proportional contribution to OVAL | DOUBLE | No | 0 | Yes | No | Yes | No |
| I | Integral contribution to OVAL | DOUBLE | No | 0 | Yes | Yes | Yes | No |
| D | Derivative contribution to OVAL | DOUBLE | No | 0 | Yes | No | Yes | No |
| CT | Clock ticks when previous process occurred | ULONG | No | 0 | Yes | No | Yes | No |
| DT | Time difference in seconds between processing steps | DOUBLE | No | 0 | Yes | Yes | Yes | No |
| OVAL | Output value | DOUBLE | No | 0 | Yes | No | Yes | No |
| DRVL | Low limit on OVAL | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| DRVH | High limit on OVAL | DOUBLE | Yes | 0 | Yes | Yes | No | No |

Feedback Tuning
---------------

The following is intended to provide some guidance on selecting the optimal values for KP, KI and KD.

1. Turn off feedback (FBON=0).
2. Set KI and KD to zero initially in order to first determine the optimum value for KP.
3. If possible calculate theoretically or empirically the "correct" value of KP, e.g. the required change in OVAL to produce a unit change in CVAL. For example if controlling a heater power supply with a DAC which has 10 Watt/volt response, and the heater response is 10 degrees/Watt, then KP=.01 volts/degree.
4. Set KP to about 10%-25% of the "correct" value computed above.
5. Turn on feedback (FBON=1) and make changes in the setpoint (VAL) and observe the system response. Don't worry about system droop (CVAL != VAL), since this is unavoidable when KI=0. Rather look for oscillations and instability. Gradually increase KP while making changes in the setpoint. When KP is too large the system will begin to oscillate. Decrease KP until the oscillations just disappear.
6. Increase KI (units of Hz) to eliminate the system droop. The optimum value of KI depends upon the time constant of the system and the update rate of the feedback loop. Increase KI until the system responds as rapidly as possible to changes in the setpoint without overshoot or oscillation.
7. For systems with no significant inertia KD should be left at zero. For systems with large inertia increase KD (units of seconds) to minimize overshoot.

Operator Display Parameters
---------------------------


These parameters are used to present meaningful data to the operator. They display the setpoint (VAL), the controlled variable (CVAL), the output value (OVAL), and other fields of the EPID, either textually or graphically.

EGU is a string of up to 16 characters describing the units of PID's manipulated values measures. It is retrieved by the `get_units`record support routine. It must be configured by the user if at all.

The HOPR and LOPR fields set the upper and lower display limits for the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields.

The DRVH and DRVL fields set the upper and lower display limits for the OVAL, P, I, and D fields.

The PREC field determines the floating point precision with which to display VAL and CVAL. It is used whenever the `get_precision`record support routine is called.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| EGU | Engineering Units | STRING \[16\] | Yes | null | Yes | Yes | No | No |
| HOPR | High Operating Range | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| LOPR | Low Operating Range | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| PREC | Display Precision | SHORT | Yes | 0 | Yes | Yes | No | No |
| NAME | Record Name | STRING | Yes | 0 | Yes | No | No | No |
| DESC | Description | STRING | Yes | Null | Yes | Yes | No | No |

Alarm Parameters
----------------

The possible alarm conditions for EPID are the SCAN alarm, limit alarms, and an alarm that is triggered when CVL is not a database link. The SCAN and CVL alarms are called by the record routines and are always of MAJOR severity.

The limit alarms trigger alarms on the VAL field. They are configured by the user in the HIHI, LOLO, HIGH, and LOW fields using floating point values. For each of these fields, there is a corresponding severity field which can be either NO ALARM, MINOR, or MAJOR.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| HIHI | Hihi Alarm Limit | DOUBLE | Yes | 0 | Yes | Yes | No | Yes |
| LOLO | High Alarm Limit | DOUBLE | Yes | 0 | Yes | Yes | No | Yes |
| HIGH | Low Alarm Limit | DOUBLE | Yes | 0 | Yes | Yes | No | Yes |
| LOW | Lolo Alarm Limit | DOUBLE | Yes | 0 | Yes | Yes | No | Yes |
| HHSV | Hihi Alarm Severity | GBLCHOICE | Yes | 0 | Yes | Yes | No | Yes |
| LLSV | High Alarm Severity | GBLCHOICE | Yes | 0 | Yes | Yes | No | Yes |
| HSV | Low Alarm Severity | GBLCHOICE | Yes | 0 | Yes | Yes | No | Yes |
| LSV | Lolo Alarm Severity | GBLCHOICE | Yes | 0 | Yes | Yes | No | Yes |
| HYST | Alarm Deadband | DOUBLE | Yes | 0 | Yes | Yes | No | No |

Monitor Parameters
------------------


These parameters are used to determine when to send monitors placed on the VAL field. These fields contain values configured by the user. The monitors are sent when the VAL field exceeds the last monitored field (MLST, ALST, and LALM) by the appropriate delta. Otherwise, value change monitors are not called. If these fields have a value of zero, everytime the value changes, a monitor will be triggered; if they have a value of -1, everytime the record is processed, monitors are triggered. The ADEL field is the delta used for archive monitors, and the MDEL field is the delta for all other types of monitors.

The ODEL field specifies the hysteresis factor for the OVAL field, the field which holds the manipulated value. Unless the current value of OVAL is greater than the amount which the user specifies in this field, no monitors will be invoked. If zero, anytime OVAL is greater than zero, a monitor is triggered. If -1, each time the record is processed, a monitor is triggered. Note that when monitors are called for OVAL, they are also called for the following fields which comprise the PID expression: P, I, D, CT, DT, ERR, and DERR.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| ADEL | Archive Deadband | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| MDEL | Monitor, i.e. value change, Deadband | DOUBLE | Yes | 0 | Yes | Yes | No | No |
| ODEL | Output deadband | DOUBLE | Yes | 0 | Yes | Yes | No | No |

Run-time Parameters
-------------------

The LALM, ALST, and MLST fields are used by record processing to implement the monitors for this record. These fields hold the values for the VAL field from the last time the record was processed. When the record is processed again the difference between these fields and current value of VAL exceeds the appropriate delta (MDEL for instance), then the appropriate monitors are triggered.

| Field | Summary | Type | DCT | Initial | Access | Modify | Rec Proc Monitor | PP |
|---|---|---|---|---|---|---|---|---|
| LALM | Value when last monitors for alarm were triggered | DOUBLE | No | 0 | Yes | No | No | No |
| ALST | Value when last monitors for archiver were triggered | DOUBLE | No | 0 | Yes | No | No | No |
| MLST | Value when last monitors for value changes were triggered | DOUBLE | No | 0 | Yes | No | No | No |

Record Support Routines
-----------------------

### init\_record

If STPL is a constant link, initialize VAL with it's value and set UDF to false.

Call init\_record routine in device support. For "Soft Channel" device support this does nothing. For "Fast Epid" device support this establishes the connection to the fastPIDServer server.

### process

See next section.

### get\_value

Null.

### get\_units

Retrieves EGU.

### get\_precision

Retrieves PREC.

### get\_graphic\_double

Sets the following values:

For the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields sets:

1. upper\_disp\_limit = HOPR
2. lower\_disp\_limit = LOPR
For the OVAL, P, I, and D fields sets:

1. upper\_disp\_limit = DRVH
2. lower\_disp\_limit = DRVL

### get\_control\_double

Sets the following values

For the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields sets:

1. upper\_ctrl\_limit = HOPR
2. lower\_ctrl\_limit = LOPR
For the OVAL, P, I, and D fields sets:

1. upper\_ctrl\_limit = DRVH
2. lower\_ctrl\_limit = DRVL
### get\_alarm\_double

Sets the following values if the field is VAL:

upper\_alarm\_limit = hihi

upper\_warning\_limit = high

lower\_warning\_limit = low

lower\_alarm\_limit = lolo

Record Processing
-----------------

Routine process implements the following algorithm:

1. If STPL is a database link and SMSL is CLOSED\_LOOP then VAL is obtained from STPL and UDF is set to false.
2. The device support do\_pid routine is called. If device supports sets PACT=true (which the "Fast Epid" device support does when it needs to communicate with the fastPIDServer server) then return immediately.
3. Check alarms. This routine checks to see if the new VAL causes the alarm status and severity to change. If so NSEV and NSTA and LALM are set. It also honors the alarm hysteresis factor (HYST). Thus the value must change by at least HYST before the alarm status and severity changes.
4. Checks to see if monitors should be invoked:

> - Alarm Monitors are invoked if ADEL and MDEL conditions are met.
> - Archive and value change monitors are invoked if ODEL conditions are met. If monitors are triggered from OVAL, they are also triggered for P, I, D, CT, DT, E

Device Support
--------------


Device support for the EPID record should implement at least the init\_record and do\_pid functions. New device support for the EPID record could be written, for example, to communicate with hardware PID controllers.

There is presently device support for "Soft Channel" (which is very similar to the record support in the EPICS PID record) and for the "Fast Epid" server.

### "Soft Channel"

 init\_record() This does nothing in the current implementation.do\_pid()

1. If CVL is not a database link a major alarm is declared and the algorithm completes.
2. The current value of CVAL is obtained from CVL.
3. The time difference since the last time step is calculated. If it is less than MDT or if no ticks have occurred since the last time the algorithm was executed, process just completes without raising any alarms, checking monitors, or scanning the forward link.
4. The new values of P, I, D, OVAL, CT, DT, and ERR are computed.
5. If feedback is On (FBON=1) and OUTL is a valid output link then OVAL is written to the output link.

### "Fast Epid"

 init\_record() This calls the dev\_init routine, which creates the asyn objects and establishes communication with the fastPIDServer server.do\_pid()

1. Checks to see if any of the parameters KP, KI, KD, DT, DRVH, DRVL, VAL, or FBON have changed and if so sends the new values to the fastPIDServer server.
2. Reads the current values of CVAL, ERR, OVAL, P, I, D, and DT from the fastPIDServer server.


Problems with the Standard EPICS PID Record
-------------------------------------------

There are two serious problems with the standard PID record in EPICS.

1. The most serious problem concerns the difficulty of using the record to control a device within a "proportional band". Consider a hypothetical temperature control system using a digital-to-analog converter (DAC) to drive a power supply with the following parameters: 
    - DAC range is 0 to 10 volts
    - DAC drives a current-mode power supply. 0-10V control input equals 0-10A output.
    - The power supply drives a furnace.
    - The steady state temperature of the furnace is 100 times the power supply current, i.e. maximum output (10A) is 1000 degrees, while 50% output (5A) is 500 degrees.
    
    Conventional temperature controllers (e.g. Omega, Lakeshore, etc.) use the concept of a "proportional band", which might typically be 10%. This means that if the setpoint is 500 degrees, and the actual temperature is less than 450 degrees (setpoint minus proportional band) then the controller applies maximum power (10A in our example). Conversely if the actual temperature is more than 550 degrees then the controller applies minimum power (0A in our example).
    
    In order to implement a 10% deadband at 500 degrees in our example the proportional gain (KP) in the PID equation must be 10V/50 degrees = .2V/deg For the purposes of this discussion we will assume the integral and derivative gains (KI, KD) are zero. Setting KI=0 will cause a "droop" in the output, since the PID equation will only result in power to the controller when there is a non-zero error if only the proportional term is used. If the setpoint is 500 degrees and KP is 0.2V/deg then we can calculate the steady state temperature as:
    
    ```
         Current = Control voltage = Temperature/100 = KP*Error = 0.2V/deg*Error
    ```
    
    But `Error` is `(500 - Temperature)`, thus 
    ```
         Temperature/100 = 0.2V/deg * (500 - Temperature)
         0.21*Temperature = 100
    ```
    
    So Temperature = 476.2 degrees, or a droop of 23.8 degrees. In practice one would use a non-zero value for KI to correct for this, but for this discussion it is not necessary. For the purposes of this simulation consider the time response of the furnace to be:
    
    ```
    Actual temperature = 0.95 * previous temperature + 
                                            0.05 * (100 * control input)
    ```
    
    i.e. there is a delay in the system response, but at steady state the temperature is equal to 100 \* output current = 100 \* control input, as was supposed above. Consider the system to be currently at 0 degrees, and the setpoint is raised to 500 degrees at step N=1. Here are the first 20 steps in the absolute value of the output of the PID equation (M), and also the output of a DAC if it were set to the value of M but limited by DRVH=10, DRVL=0.
    
    ```
       N    Setpoint   Actual       Error     M      DM  Correct DAC
                        Temp                                Output
       0         0       0.000       0.000   0.000   0.000    0.000
       1       500       0.000     500.000 100.000 100.000   10.000
       2       500      50.000     450.000  90.000 -10.000   10.000
       3       500      97.500     402.500  80.500  -9.500   10.000
       4       500     142.625     357.375  71.475  -9.025   10.000
       5       500     185.494     314.506  62.901  -8.574   10.000
       6       500     226.219     273.781  54.756  -8.145   10.000
       7       500     264.908     235.092  47.018  -7.738   10.000
       8       500     301.663     198.337  39.667  -7.351   10.000
       9       500     336.580     163.420  32.684  -6.983   10.000
      10       500     369.751     130.249  26.050  -6.634   10.000
      11       500     401.263      98.737  19.747  -6.302   10.000
      12       500     431.200      68.800  13.760  -5.987   10.000
      13       500     459.640      40.360   8.072  -5.688    8.072
      14       500     477.018      22.982   4.596  -3.476    4.596
      15       500     476.149      23.851   4.770   0.174    4.770
      16       500     476.193      23.807   4.761  -0.009    4.761
      17       500     476.190      23.810   4.762   0.000    4.762
      18       500     476.190      23.810   4.762  -0.000    4.762
      19       500     476.190      23.810   4.762   0.000    4.762
      20       500     476.190      23.810   4.762   0.000    4.762
    ```
    
    This table lists the correct response of PID system with these parameters. Note that the steady state temperature (476.2) agrees with the previous calculation for the temperature droop. Now consider an actual standard EPICS PID record whose output (DM) is used directly as the input to an analog output (ao) record which drives the DAC. The ao record is used in incremental mode, as the documentation for the PID suggests. The DRHV value for the ao record is 10V, since this is the full scale range of the power supply.
    
    ```
       N    Setpoint   Actual       Error     M      DM    Actual DAC
                        Temp                (unused)         Output
       0         0       0.000       0.000   0.000   0.000    0.000
       1       500       0.000     500.000   0.000 100.000   10.000
       2       500      50.000     450.000   0.000 -10.000    0.000
       3       500      47.500     452.500   0.000   0.500    0.500
       4       500      47.625     452.375   0.000  -0.025    0.475
       5       500      47.619     452.381   0.000   0.001    0.476
       6       500      47.619     452.381   0.000  -0.000    0.476
       7       500      47.619     452.381   0.000   0.000    0.476
       8       500      47.619     452.381   0.000  -0.000    0.476
       9       500      47.619     452.381   0.000   0.000    0.476
      10       500      47.619     452.381   0.000   0.000    0.476
      11       500      47.619     452.381   0.000   0.000    0.476
      12       500      47.619     452.381   0.000   0.000    0.476
      13       500      47.619     452.381   0.000   0.000    0.476
      14       500      47.619     452.381   0.000   0.000    0.476
      15       500      47.619     452.381   0.000   0.000    0.476
      16       500      47.619     452.381   0.000   0.000    0.476
      17       500      47.619     452.381   0.000   0.000    0.476
      18       500      47.619     452.381   0.000   0.000    0.476
      19       500      47.619     452.381   0.000   0.000    0.476
      20       500      47.619     452.381   0.000   0.000    0.476
    ```
    
    NOTE: These DAC output values are completely incorrect!!!!! The reason for this is simple. The PID record is calculating a change in output (DM), but is writing this to an output location (the ao record) which __saturates__at the DRHV value. When N=1 above DM=100, so the ao record should go to 100. However, it is limited to 10. When N=2, DM=-10, so the ao should go to 90 (100-10), but it actually goes to 0 (10-10), because the ao clipped the previous value to 10. This will always happen when driving an output beyond the proportional band. Since the next change (DM) is applied to the CLIPPED value of the output, rather than the theoretical value of the output (see first table), all subsequent DAC values are incorrect. The bottom line is that the PID record can only work correctly when the DM value is used to drive a non-saturable value, i.e. not a real output. This means that it is necessary to create a database with another record in between the PID record and the actual DAC which is being controlled. This is a needless complication, and is not mentioned anywhere in the PID record documentation.
2. If one does put another soft ao record in between the PID record and the actual device being controlled, then one encounters the second serious problem with the PID record. This problem concerns the integral term in the PID equation. The problem is that the integral term continues to grow without bound whenever something is configured wrong (for example, a power supply is turned off overnight). When the device is turned back on the integral term will cause large compensating errors for a long time. It is desirable for the PID record to put some sanity checks on the integral term.

 Suggestions and Comments to:   
 [Mark Rivers ](mailto:rivers@cars.uchicago.edu): (rivers@cars.uchicago.edu)   
 Last modified: June 5, 2000
