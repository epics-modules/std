<html>

<head>
<meta http-equiv="Content-Type"
content="text/html; charset=iso-8859-1">
<meta name="GENERATOR" content="Microsoft FrontPage Express 2.0">
<title>EPID Record</title>
</head>

<body>

<h1 align="center">EPID Record</h1>

<h2 align="center">Author: Mark Rivers</h2>

<h2>Contents</h2>

<ul>
    <li><a href="#Introduction">Introduction</a></li>
    <li><a href="#Scan Parameters">Scan Parameters</a></li>
    <li><a href="#Controlled Variable Parameters">Controlled
        Variable Parameters</a></li>
    <li><a href="#Setpoint Parameters">Setpoint Parameters</a></li>
    <li><a href="#Output parameters">Output Parameters</a></li>
    <li><a href="#Feedback Parameters">Feedback Parameters</a></li>
    <li><a href="#Feedback Tuning">Feedback Tuning</a></li>
    <li><a href="#Operator Display Parameters">Operator Display
        Parameters</a></li>
    <li><a href="#Alarm Parameters">Alarm Parameters</a></li>
    <li><a href="#Monitor Parameters">Monitor Parameters</a></li>
    <li><a href="#Run-time Parameters">Run-Time Parameters</a></li>
    <li><a href="#Record Support Routines">Record Support
        Routines</a></li>
    <li><a href="#Record Processing">Record Processing</a></li>
    <li><a href="#Device Support">Device Support</a></li>
    <li><a href="#Problems">Problems with the Standard EPICS PID Record</a></li>
</ul>

<p>&nbsp;</p>

<h2 align="center"><a name="Introduction">Introduction</a></h2>

<hr>

<p>The EPID record is used to maintain a setpoint that affects
the output according to the difference between a controlled value
and the setpoint. It provides EPICS with a very flexible and
powerful tool for peforming feedback. The EPID record is an
Enhanced version of the EPICS PID record. The major improvements
in the EPID record compared to the PID record include:</p>

<ul>
    <li>Separation of device support from the record. There is
        "Soft Channel" device support which uses EPICS database
        links, and this is very similar to the PID record.
        However, the EPID record can also be used with other
        device support, for example to communicate with faster
        feedback software, or with hardware controllers. Asyn
        device support is provided for fast feedback (&gt; 1 kHz)
        using an Acromag IP330 ADC and a Systran DAC128V DAC.</li>
    <li>Addition of many fields (OUTL, DRVH, DRVL) to simplify
        construction of databases</li>
    <li>The PID expression is computed as an absolute number,
        rather than a differential number to be added to the
        present output value. This simplifies database
        construction, and also permits the record itself to
        perform limit checking on the output.</li>
    <li>Limits are placed on the magnitude of the integral term
        (I) which are lacking in the PID record.</li>
    <li>Monitors are posted for the CVAL field, which simplifies
        construction of user-interface tools, such as plotting.</li>
    <li>The CVL field has been renamed INP. This field can now be
        modified (a feature of EPICS R3.12 and higher), so that a
        single EPID record can be used to control different
        processes at different times.</li>
    <li>Changed the time units of the KI and KD terms from
        minutes to seconds</li>
	<li>Addition of fields to better support the use of an
        asynchronous readback device.
</ul>

<p>Each time the record is processed the setpoint is read and
then the <tt>do_pid()</tt> function in device support is called.
The "Soft Channel" device support reads the controlled value,
computes the PID expression and, if feedback is enabled, writes
its output to the output link.</p>

<p>There are three terms of the PID expression: the proportional,
the integral, and the derivative. Using the gain that exists for
each of these terms, the user can weight each contribution
according to the characteristics of the controlled variable.</p>
<P>
The source code is available in a
<a href="pub/epidRecord.tar">tar file.</a>

<p>&nbsp;</p>

<h2 align="center"><a name="Scan Parameters">Scan Parameters</a></h2>

<hr>

<p>The PID record has the standard fields (SCAN, etc.) for
specifying under what circumstances it will be processed. The
minimum delta time field (MDT) is a processing field particular
to the PID record. It can be configured by the user or modified
at run-time. It contains a floating point value which represents
the minimum amount of time between record processing. The actual
meaning of the SCAN and MDT fields depends upon which device
support is used:</p>

<ul>
    <li>"Soft Channel" device support. The SCAN field controls the
        rate at which the feedback calculation is actually
        performed. The maximum rate is typically 10 Hz. If the
        amount of time between the last time the record was
        processed an the current time is less than MDT, than the
        record is not processed. If MDT is left at its default
        value (0), the minimum delta will be equal to one clock
        tick. </li>
</ul>

<ul>
    <li>"Fast Epid" device support. The SCAN field controls the
        rate at which new parameters (KP, KI, KD, VAL, etc.) are
        sent to the Ip330PID server, and the rate at which the
        current feedback parameters (P, I, D, CVAL, etc.) are
        read back from the server and displayed. Thus the SCAN
        field controls the rate at which the EPID record takes
        &quot;snapshots&quot; of the feedback processing running
        on the fastPIDServer server. The feedback itself can actually be
        running much faster, i.e. 1 kHz or more. The MDT field is
        ignorred by this device support. The DT field is used to
        control the time per feedback loop on the fastPIDServer server. The
        DT field is also modified by the device support to
        reflect the actual time per feedback loop, since this may
        differ from the requested time.</li>
</ul>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>MDT</td>
        <td>Minimum Delta Time</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Controlled Variable Parameters">Controlled
Variable Parameters</a></h2>

<hr>

<p>The meaning of the INP field depends upon what device support
is used:</p>

<menu>
    <li>"Soft Channel" device support. The input link field (INP) is
        used to obtain the value of the controlled process
        variable, i.e., the value read into the CVAL field. The
        link must be a database link. If it is not a database
        link a MAJOR alarm is triggered when the record is
        processed. The INP field <em>can</em> be modified at
        run-time.</li>
</menu>

<menu>
    <li>"Fast Epid" device support. The input link field (INP)
        is used to specify the location of the "Fast Epid"
        server which actually performs the feedback. The fastPIDServer
        server is configured in a startup-script
        file to specify which Ip330 ADC channel is used as the
        input from which the value is read into the CVAL field.
        The link must be a VME link, and <em>cannot</em> be
        modified at run-time.</li>
</menu>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>INP</td>
        <td>Controlled Value Location (an input link)</td>
        <td>INLINK</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>N/A</td>
        <td>No</td>
    </tr>
    <tr>
        <td>CVAL</td>
        <td>Value of controlled variable</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
</table>

If "Async Soft Channel" device support is selected, the following two
fields are used to support an asynchronous readback device.  The
device support is expected to accomplish this by writing the value
of the TVAL field to the PV specified by the TRIG link field, using
dbCaPutLinkCallback().  This is expected to cause the readback device
to process.  EPICS will send a callback when that processing completes,
at which time device support will read the readback value just as normal
"Soft Channel" support would.

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>TRIG</td>
        <td>Readback Trigger (an output link)</td>
        <td>OUTLINK</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>N/A</td>
        <td>No</td>
    </tr>
    <tr>
        <td>TVAL</td>
        <td>Value written to TRIG link</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>N/A</td>
        <td>No</td>
    </tr>
</table>


<p>&nbsp;</p>

<h2 align="center"><a name="Setpoint Parameters">Setpoint
Parameters</a></h2>

<hr>

<p>The setpoint mode select (SMSL) and the setpoint location
(STPL) fields determine where the setpoint value is obtained,
which is held in the VAL field. The SMSL and STPL fields work
just like the OMSL and DOL fields found in many output records
(analog output, binary output, etc.).</p>

<p>The SMSL field has two choices: <code>supervisory</code> and 
<code>closed_loop</code>.
When <code>supervisory</code>, the setpoint, i.e., VAL, can be
set via database access. When <code>closed_loop</code>, the
setpoint value is retrieved from the link specified in STPL,
which must be a database link. STPL can also be a constant in
which case VAL is set equal to the constant value when the record
is initialized, and the value can be modified at run-time via
database access or channel access.</p>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>STPL</td>
        <td>Setpoint Location (an input link)</td>
        <td>INLINK</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>N/A</td>
        <td>No</td>
    </tr>
    <tr>
        <td>SMSL</td>
        <td>Setpoint Mode select.</td>
        <td>GBLCHOICE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>VAL</td>
        <td>Setpoint value</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>Yes</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Output parameters">Output parameters</a></h2>

<hr>

<p>There are two fields which control the output of the EPID
record, FBON (Feedback On) and OUTL (Output Link). The meanings
of the these fields depends upon what device support is used:</p>

<menu>
    <li>"Soft Channel" device support. The output link field (OUTL)
        is used to control where the computed output (OVAL) is
        sent. If OUTL is a database link and if FBON=1 then the
        output value (OVAL) is written to this link each time the
        record processes. If FBON=0, or if OUTL is not a database
        link then the device support still computes the OVAL but
        does not write OVAL to the output link.</li>
</menu>

<menu>
    <li>"Fast Epid" device support. The output link field (OUTL)
        is not used. Rather, the fastPIDServer server is configured in
        a vxWorks startup-script file to specify which
        DAC128V DAC channel is used as the output. If FBON=1 then
        the computed output will be written to the DAC channel
        each time the feedback loop executes. The execution of
        the feedback loop is independent of, and typically much
        faster than the rate at which the EPID record processes.
        If FBON=0 then the feedback loop still executes, but it
        does not write the output to the DAC channel.</li>
</menu>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>OUTL</td>
        <td>Output Location (an outlink)</td>
        <td>OUTLINK</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>N/A</td>
        <td>No</td>
    </tr>
    <tr>
        <td>FBON</td>
        <td>Feedback On or Off</td>
        <td>MENU</td>
        <td>Yes</td>
        <td>Off</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Feedback Parameters">Feedback
Parameters</a></h2>

<hr>

<p>The discrete from of the PID expression is as follows:</p>

<p>M(n) = P + I + D</p>

<p>P = KP * E(n)</p>

<p>I = KP * KI * SUMi ( E(i) * dT(n) )</p>

<p>D = KP * KD * ( (E(n) - (E(n-1) ) / dT(n) )</p>

<p>where</p>

<dl>
    <dd>M(n) = value of manipulated variable at <em>n</em>th
        instant.</dd>
    <dd>P = Proportional term</dd>
    <dd>I = Integral term</dd>
    <dd>D = Derivative term</dd>
    <dd>KP = Proportional gain</dd>
    <dd>KI = Integral gain</dd>
    <dd>KD = Derivative gain</dd>
    <dd>E(n) = Error at nth sampling instant</dd>
    <dd>SUMi = Sum from i=0 to i=n</dd>
    <dd>dT(n) = Time difference between n-1 instance and nth
        instance</dd>
</dl>

<p>The terms KP, KI, and KD are the only terms configured by the
user. These terms represent the gain for the proportional (KP),
integral (KI) and the derivative (KD) terms. A field exists for
each of these terms (the KP, KI, and KD fields). KP should be
configured according to the characteristics of the controlled
variable; KI should be set equal to the number of times per
second that the integral contribution repeats the proportional
contribution; and KD should equal the number of seconds until the
derivative contribution repeats the proportional contribution.
See the<a href="#Feedback Tuning"> guide to feedback tuning</a>
below for more information on setting KP, KI and KD.</p>

<p>The PID device support calculates the other terms of the
expression. The "Soft Channel" and "Fast Epid" device support both
calculate these as follows:</p>

<dl>
    <dt>E(n)</dt>
    <dd>Error at nth sampling instant, where the Error equals the
        setpoint minus the value of the controlled variable (VAL
        - CVAL). The ERR field in the record holds this value.</dd>
    <dt>P</dt>
    <dd>Proportional term, calculated from the above equation.
        The P field in the record holds this value.</dd>
    <dt>I</dt>
    <dd>Integral term, calculated from the above equation. The I
        field in the record holds this value. Note that the
        integral term is a sum from time=0 to the present time,
        and hence can grow extremely large if not subject to some
        &quot;sanity checks&quot;. The EPICS PID record does not
        perform any such checks, which is a serious limitation.
        The EPID record device support performs the following
        checks to prevent the integral term from growing too
        large when feedback is not working correctly:</dd>
</dl>

<blockquote>
    <ul>
        <li>I is not allowed to increase if the computed output, M(n), is at
            the high limit, DRVH.</li>
        <li>I is now allowed to decrease if the computed output, M(n), is at
            the low limit, DRVL.</li>
        <li>I is not allowed to be less than DRVL or greater than
            DRVH.</li>
        <li>If KI is zero then I is set to zero. This allows
            &quot;clearing&quot; the integral term by setting KI
            to zero.</li>
        <li>I can be modified from database access or channel
            access. This allows the user to set I to a specific
            value to improve response time, rather than waiting
            for the normal time constant associated with this
            term.</li>
    </ul>
</blockquote>

<dl>
    <dt>D</dt>
    <dd>Derivative term. The D field in the record holds this
        value.</dd>
    <dt>M(n)</dt>
    <dd>This is the end result of the PID expression--the change
        in the manipulated value, which is the desired output.
        The OVAL field holds this value. However, OVAL is first
        limited to be no smaller than DRVL and no larger than
        DRVH.</dd>
    <dt>dT(n)</dt>
    <dd>This is the time difference between n and n-1, between
        current and last samplings. It is stored in the DT field.
        For the "Soft Channel" device support this field is
        Read-Only. For the "Fast Epid" device support this field
        can be modified to control the time per feedback loop.</dd>
</dl>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>KP</td>
        <td>Proportional Gain</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>KI</td>
        <td>Integral Gain, in repeats per second.</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>KD</td>
        <td>Derivative Gain, in repeats per second</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>ERR</td>
        <td>Current error (VAL - CVAL)</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>P</td>
        <td>Proportional contribution to OVAL</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>I</td>
        <td>Integral contribution to OVAL</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>D</td>
        <td>Derivative contribution to OVAL</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>CT</td>
        <td>Clock ticks when previous process occurred</td>
        <td>ULONG</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>DT</td>
        <td>Time difference in seconds between processing steps</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>OVAL</td>
        <td>Output value</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
        <td>No</td>
    </tr>
    <tr>
        <td>DRVL</td>
        <td>Low limit on OVAL</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>DRVH</td>
        <td>High limit on OVAL</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Feedback Tuning">Feedback Tuning</a></h2>

<hr>

<p align="left">The following is intended to provide some
guidance on selecting the optimal values for KP, KI and KD.</p>

<ol>
    <li><p align="left">Turn off feedback (FBON=0).</p>
    </li>
    <li><p align="left">Set KI and KD to zero initially in order
        to first determine the optimum value for KP.</p>
    </li>
    <li><p align="left">If possible calculate theoretically or
        empirically the &quot;correct&quot; value of KP, e.g. the
        required change in OVAL to produce a unit change in CVAL.
        For example if controlling a heater power supply with a
        DAC which has 10 Watt/volt response, and the heater
        response is 10 degrees/Watt, then KP=.01 volts/degree.</p>
    </li>
    <li><p align="left">Set KP to about 10%-25% of the
        &quot;correct&quot; value computed above.</p>
    </li>
    <li><p align="left">Turn on feedback (FBON=1) and make
        changes in the setpoint (VAL) and observe the system
        response. Don't worry about system droop (CVAL != VAL),
        since this is unavoidable when KI=0. Rather look for
        oscillations and instability. Gradually increase KP while
        making changes in the setpoint. When KP is too large the
        system will begin to oscillate. Decrease KP until the
        oscillations just disappear.</p>
    </li>
    <li><p align="left">Increase KI (units of Hz) to eliminate
        the system droop. The optimum value of KI depends upon
        the time constant of the system and the update rate of
        the feedback loop. Increase KI until the system responds
        as rapidly as possible to changes in the setpoint without
        overshoot or oscillation.</p>
    </li>
    <li><p align="left">For systems with no significant inertia
        KD should be left at zero. For systems with large inertia
        increase KD (units of seconds) to minimize overshoot.</p>
    </li>
</ol>

<p>&nbsp;</p>

<h2 align="center"><a name="Operator Display Parameters">Operator
Display Parameters</a></h2>

<hr>

<p>These parameters are used to present meaningful data to the
operator. They display the setpoint (VAL), the controlled
variable (CVAL), the output value (OVAL), and other fields of the
EPID, either textually or graphically.</p>

<p>EGU is a string of up to 16 characters describing the units of
PID's manipulated values measures. It is retrieved by the 
<code>get_units</code>
record support routine. It must be configured by the user if at
all.</p>

<p>The HOPR and LOPR fields set the upper and lower display
limits for the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields.</p>

<p>The DRVH and DRVL fields set the upper and lower display
limits for the OVAL, P, I, and D fields.</p>

<p>The PREC field determines the floating point precision with
which to display VAL and CVAL. It is used whenever the 
<code>get_precision</code>
record support routine is called.</p>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>EGU</td>
        <td>Engineering Units</td>
        <td>STRING [16]</td>
        <td>Yes</td>
        <td>null</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>HOPR</td>
        <td>High Operating Range</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>LOPR</td>
        <td>Low Operating Range</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>PREC</td>
        <td>Display Precision</td>
        <td>SHORT</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>NAME</td>
        <td>Record Name</td>
        <td>STRING</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>DESC</td>
        <td>Description</td>
        <td>STRING</td>
        <td>Yes</td>
        <td>Null</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Alarm Parameters">Alarm Parameters</a></h2>

<hr>

<p>The possible alarm conditions for EPID are the SCAN alarm,
limit alarms, and an alarm that is triggered when CVL is not a
database link. The SCAN and CVL alarms are called by the record
routines and are always of MAJOR severity.</p>

<p>The limit alarms trigger alarms on the VAL field. They are
configured by the user in the HIHI, LOLO, HIGH, and LOW fields
using floating point values. For each of these fields, there is a
corresponding severity field which can be either NO ALARM, MINOR,
or MAJOR. </p>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>HIHI</td>
        <td>Hihi Alarm Limit</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>LOLO</td>
        <td>High Alarm Limit</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>HIGH</td>
        <td>Low Alarm Limit</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>LOW</td>
        <td>Lolo Alarm Limit</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>HHSV</td>
        <td>Hihi Alarm Severity</td>
        <td>GBLCHOICE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>LLSV</td>
        <td>High Alarm Severity</td>
        <td>GBLCHOICE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>HSV</td>
        <td>Low Alarm Severity</td>
        <td>GBLCHOICE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>LSV</td>
        <td>Lolo Alarm Severity</td>
        <td>GBLCHOICE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>Yes</td>
    </tr>
    <tr>
        <td>HYST</td>
        <td>Alarm Deadband</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Monitor Parameters">Monitor
Parameters</a></h2>

<hr>

<p>These parameters are used to determine when to send monitors
placed on the VAL field. These fields contain values configured
by the user. The monitors are sent when the VAL field exceeds the
last monitored field (MLST, ALST, and LALM) by the appropriate
delta. Otherwise, value change monitors are not called. If these
fields have a value of zero, everytime the value changes, a
monitor will be triggered; if they have a value of -1, everytime
the record is processed, monitors are triggered. The ADEL field
is the delta used for archive monitors, and the MDEL field is the
delta for all other types of monitors.</p>

<p>The ODEL field specifies the hysteresis factor for the OVAL
field, the field which holds the manipulated value. Unless the
current value of OVAL is greater than the amount which the user
specifies in this field, no monitors will be invoked. If zero,
anytime OVAL is greater than zero, a monitor is triggered. If -1,
each time the record is processed, a monitor is triggered. Note
that when monitors are called for OVAL, they are also called for
the following fields which comprise the PID expression: P, I, D,
CT, DT, ERR, and DERR.</p>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>ADEL</td>
        <td>Archive Deadband</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>MDEL</td>
        <td>Monitor, i.e. value change, Deadband</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>ODEL</td>
        <td>Output deadband</td>
        <td>DOUBLE</td>
        <td>Yes</td>
        <td>0</td>
        <td>Yes</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Run-time Parameters">Run-time
Parameters</a></h2>

<hr>

<p>The LALM, ALST, and MLST fields are used by record processing
to implement the monitors for this record. These fields hold the
values for the VAL field from the last time the record was
processed. When the record is processed again the difference
between these fields and current value of VAL exceeds the
appropriate delta (MDEL for instance), then the appropriate
monitors are triggered. </p>

<table border="1">
    <tr>
        <th>Field</th>
        <th>Summary</th>
        <th>Type</th>
        <th>DCT</th>
        <th>Initial</th>
        <th>Access</th>
        <th>Modify</th>
        <th>Rec Proc Monitor</th>
        <th>PP</th>
    </tr>
    <tr>
        <td>LALM</td>
        <td>Value when last monitors for alarm were triggered</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>ALST</td>
        <td>Value when last monitors for archiver were triggered</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
        <td>No</td>
    </tr>
    <tr>
        <td>MLST</td>
        <td>Value when last monitors for value changes were
        triggered</td>
        <td>DOUBLE</td>
        <td>No</td>
        <td>0</td>
        <td>Yes</td>
        <td>No</td>
        <td>No</td>
        <td>No</td>
    </tr>
</table>

<p>&nbsp;</p>

<h2 align="center"><a name="Record Support Routines">Record
Support Routines</a></h2>

<hr>

<h3>init_record</h3>

<p>If STPL is a constant link, initialize VAL with it's value and
set UDF to false.</p>

<p>Call init_record routine in device support. For "Soft Channel"
device support this does nothing. For "Fast Epid" device support
this establishes the connection to the fastPIDServer server.</p>

<h3>process</h3>

<p>See next section.</p>

<h3>get_value</h3>

<p>Null.</p>

<h3>get_units</h3>

<p>Retrieves EGU.</p>

<h3>get_precision</h3>

<p>Retrieves PREC.</p>

<h3>get_graphic_double</h3>

<p>Sets the following values:</p>

<p>For the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields sets:</p>

<menu>
    <li>upper_disp_limit = HOPR</li>
    <li>lower_disp_limit = LOPR</li>
</menu>

<p>For the OVAL, P, I, and D fields sets:</p>

<menu>
    <li>upper_disp_limit = DRVH</li>
    <li>lower_disp_limit = DRVL</li>
</menu>

<h3>get_control_double</h3>

<p>Sets the following values</p>

<p>For the VAL, CVAL, HIHI, HIGH, LOW, and LOLO fields sets:</p>

<menu>
    <li>upper_ctrl_limit = HOPR</li>
    <li>lower_ctrl_limit = LOPR</li>
</menu>

<p>For the OVAL, P, I, and D fields sets:</p>

<menu>
    <li>upper_ctrl_limit = DRVH</li>
    <li>lower_ctrl_limit = DRVL</li>
</menu>

<h3>get_alarm_double</h3>

<p>Sets the following values if the field is VAL:</p>

<p>upper_alarm_limit = hihi</p>

<p>upper_warning_limit = high</p>

<p>lower_warning_limit = low</p>

<p>lower_alarm_limit = lolo</p>

<p>&nbsp;</p>

<h2 align="center"><a name="Record Processing">Record Processing</a></h2>

<hr>

<p>Routine process implements the following algorithm:</p>

<ol>
    <li>If STPL is a database link and SMSL is CLOSED_LOOP then
        VAL is obtained from STPL and UDF is set to false.</li>
    <li>The device support <tt>do_pid</tt> routine is called. If device
        supports sets PACT=true (which the "Fast Epid" device support
        does when it needs to communicate with the fastPIDServer server)
        then return immediately.</li>
    <li>Check alarms. This routine checks to see if the new VAL
        causes the alarm status and severity to change. If so
        NSEV and NSTA and LALM are set. It also honors the alarm
        hysteresis factor (HYST). Thus the value must change by
        at least HYST before the alarm status and severity
        changes.</li>
    <li>Checks to see if monitors should be invoked:</li>
</ol>

<blockquote>
    <ul>
        <li>Alarm Monitors are invoked if ADEL and MDEL
            conditions are met.</li>
        <li>Archive and value change monitors are invoked if ODEL
            conditions are met. If monitors are triggered from
            OVAL, they are also triggered for P, I, D, CT, DT, E</li>
    </ul>
</blockquote>

<p>&nbsp;</p>

<h2 align="center"><a name="Device Support">Device Support</a></h2>

<hr>

<p>Device support for the EPID record should implement at least
the <tt>init_record</tt> and <tt>do_pid</tt> functions. 
New device support for the
EPID record could be written, for example, to communicate with
hardware PID controllers.</p>

<p>There is presently device support for "Soft Channel" (which is
very similar to the record support in the EPICS PID record) and
for the "Fast Epid" server.</p>

<h3>"Soft Channel"</h3>

<dl>
    <dt>init_record()</dt>
    <dd>This does nothing in the current implementation.</dd>
</dl>

<p>do_pid()</p>

<ol>
    <li>If CVL is not a database link a major alarm is declared
        and the algorithm completes.</li>
    <li>The current value of CVAL is obtained from CVL.</li>
    <li>The time difference since the last time step is
        calculated. If it is less than MDT or if no ticks have
        occurred since the last time the algorithm was executed,
        process just completes without raising any alarms,
        checking monitors, or scanning the forward link.</li>
    <li>The new values of P, I, D, OVAL, CT, DT, and ERR are
        computed.</li>
    <li>If feedback is On (FBON=1) and OUTL is a valid output
        link then OVAL is written to the output link.</li>
</ol>

<h3>"Fast Epid"</h3>

<dl>
    <dt>init_record()</dt>
    <dd>This calls the dev_init routine, which creates the asyn
        objects and establishes communication with the
        fastPIDServer server.</dd>
</dl>

<p>do_pid()</p>

<ol>
    <li>Checks to see if any of the parameters KP, KI, KD, DT,
        DRVH, DRVL, VAL, or FBON have changed and if so sends the
        new values to the fastPIDServer server.</li>
    <li>Reads the current values of CVAL, ERR, OVAL, P, I, D, and
        DT from the fastPIDServer server.</li>
</ol>

<hr>
<h2 align="center"><a name=Problems>
Problems with the Standard EPICS PID Record</a></h2>
<P>
There are two serious problems with the standard PID record in EPICS.
<OL>
<LI> The most serious problem concerns the difficulty of using the record to
control a device within a "proportional band".  Consider a hypothetical
temperature control system using a digital-to-analog converter (DAC) to drive a
power supply with the following parameters:
<UL>
<LI>DAC range is 0 to 10 volts
<LI>DAC drives a current-mode power supply.  0-10V control input equals
0-10A output.
<LI>The power supply drives a furnace.
<LI>The steady state temperature of the furnace is 100 times the power
supply current, i.e. maximum output (10A) is 1000 degrees, while 50%
output (5A) is 500 degrees.
</UL>
<P>
Conventional temperature controllers (e.g. Omega, Lakeshore, etc.) use the
concept of a "proportional band", which might typically be 10%.  This means
that if the setpoint is 500 degrees, and the actual temperature is less than
450 degrees (setpoint minus proportional band) then the controller applies
maximum power (10A in our example).  Conversely if the actual temperature is
more than 550 degrees then the controller applies minimum power (0A in our
example).
<P>
In order to implement a 10% deadband at 500 degrees in our example the 
proportional gain (KP) in the PID equation must be 10V/50 degrees = .2V/deg
For the purposes of this discussion we will assume the integral and
derivative gains (KI, KD) are zero.  Setting KI=0 will cause a "droop" in 
the output, since the PID equation will only result in power to the
controller when there is a non-zero error if only the proportional term is
used.  If the setpoint is 500 degrees and KP is 0.2V/deg then we can 
calculate the steady state temperature as:
<PRE>
     Current = Control voltage = Temperature/100 = KP*Error = 0.2V/deg*Error
</PRE>
But <CODE>Error</CODE> is <CODE>(500 - Temperature)</CODE>, thus
<PRE>
     Temperature/100 = 0.2V/deg * (500 - Temperature)
     0.21*Temperature = 100
</PRE>
So Temperature = 476.2 degrees, or a droop of 23.8 degrees.  In practice one 
would use a non-zero value for KI to correct for this, but for this
discussion it is not necessary.
<P>
For the purposes of this simulation consider the time response of the
furnace to be:
<PRE>
Actual temperature = 0.95 * previous temperature + 
                                        0.05 * (100 * control input)
</PRE>
i.e. there is a delay in the system response, but at steady state the
temperature is equal to 100 * output current = 100 * control input, as was
supposed above.
<P>
Consider the system to be currently at 0 degrees, and the setpoint is raised
to 500 degrees at step N=1.  Here are the first 20 steps in the absolute 
value of the output of the PID equation (M), and also the output of a DAC
if it were set to the value of M but limited by DRVH=10, DRVL=0.
<PRE>
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
</PRE>
This table lists the correct response of PID system with these parameters.
Note that the steady state temperature (476.2) agrees with the previous
calculation for the temperature droop.
<P>
Now consider an actual standard EPICS PID record whose output (DM) is used
directly as the input to an analog output (ao) record which drives the DAC. 
The ao record is used in incremental mode, as the documentation for the PID
suggests. The DRHV value for the ao record is 10V, since this is the full
scale range of the power supply.
<PRE>
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
</PRE>
NOTE:  These DAC output values are completely incorrect!!!!!  The reason for
this is simple.  The PID record is calculating a change in output (DM), but is
writing this to an output location (the ao record) which <b>saturates</b>
at the DRHV
value.  When N=1 above DM=100, so the ao record should go to 100.  However, it
is limited to 10.  When N=2, DM=-10, so the ao should go to 90 (100-10), but it
actually goes to 0 (10-10), because the ao clipped the previous value to 10.
This will always happen when driving an output beyond the proportional band. 
Since the next change (DM) is applied to the CLIPPED value of the output,
rather than the <it>theoretical</it> value of the output (see first table), all
subsequent DAC values are incorrect.
<P>
The bottom line is that the PID record can only work correctly when the DM
value is used to drive a non-saturable value, i.e. not a real output.  This
means that it is necessary to create a database with another record in between
the PID record and the actual DAC which is being controlled.  This is a
needless complication, and is not mentioned anywhere in the PID record
documentation.
<P>
<LI> If one does put another soft ao record in between the PID record and
the actual device being controlled, then one encounters the second serious
problem with the PID record. This problem concerns the integral term in the PID
equation.  The problem is that the integral term continues to grow without
bound whenever something is configured wrong (for example, a power supply is
turned off overnight).  When the device is turned back on the integral term
will cause large compensating errors for a long time.  It is desirable for the
PID record to put some sanity checks on the integral term.
</OL>


<hr>

<address>
    Suggestions and Comments to: <br>
    <a href="mailto:rivers@cars.uchicago.edu">Mark Rivers </a>:
    (rivers@cars.uchicago.edu) <br>
    Last modified: June 5, 2000
</address>
</body>
</html>
