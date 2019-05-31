<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=iso-8859-1">
  <title>throttleRecord</title>
<style>
  td, th
  {
  padding:5px;
  }
</style>
</head>
<body>

<h1>throttle - Record for Throttling Value Changes</h1>

<h1>1. Introduction</h1>

<p>
This record was created for solving the problem of an output
floating-point value being changed too fast for a device that that is
referencing that value.  As an example, a laser system can have a time
delay specified for a delay generator, and after changing the delay it
takes some period of time for the laser system to stabilize; if the
delay is changed again before the stabilization occurs, the laser
could do something unintended.  The throttle record enforces a delay.
</p><p> The sort of behavior that this record is really targetting is
human behavior, where the user of a control might tweak things at a
rate that the system doesn't want.  Instead of trying to change human
behavior, this record protects the controls in a minimally disruptive
way.
</p>
<p><hr>
<h1>2. Operation</h1>

<p>
If the value is naturally changed slower than the specified delay, the
throttle record is completely transparent.  The record only cares that
the period is enforced, so if the time has already passed sinced the
last change, the new change takes immediate effect.
</p><p> 
When a change has occurred within the period, the new value is stored
and then used when the period is over.  If multiple changes are made
within the period, the <b>last</b> change is used; changes are not
accumulated in a queue and used sequentially over several periods.
</p><p> 

Limits can be specified for the record, by making the low and high
limits have a positive difference.  A status flag is set for when the
record hits a limit, signifying which limit.  If the clipped flag is
set, then when a limit is hit, the limit value is used instead of not
changing the value.

</p>

<p><hr>
<h1>3. Record Fields</h1>

<p>
The <strong>VAL</strong> field holds the current proposed value to
send to the <strong>OUT</strong> link.  When this value is sent is
determined by how long ago a value was previously sent, and
the <strong>DLY</strong> delay value.  If a value was sent longer than
the daly value ago, the value gets sent immediately, otherwise the
latest <strong>VAL</strong> value gets sent once the time since the
last time a value was sent has been reached.  The value that was last
sent is shown in the <strong>SENT</strong> field.  If a value is
currently being delayed, then <strong>WAIT</strong> is "True".
</p>
<p>
The <strong>VAL</strong> field can be limited, used when the
<strong>DRVLH</strong> high limit is set to a larger value than
the <strong>DRVLL</strong> low limit.  If the value falls outside the
limit range, it will be reset to the relevant limit, and
the <strong>DRVLS</strong> status will be set to either "Low Limit" or
"High Limit".  If <strong>DRVLC</strong> is set to "On", then when a
limit is hit, the limit value is used as the value in addition to the
status changed.
</p>
<p>As a convenience, the record has a method of synchronizing
the <strong>VAL</strong> value to a reference PV, without processing
the record.  The reference PV is stored in <strong>SINP</strong>, and
its validity can be checked with <strong>SIV</strong>.  To activate the
synchronization, <strong>SYNC</strong> needs to be set to "Process".
</p>
<p>
The <strong>PREC</strong> field defines the resolution
of <strong>VAL</strong>, while the <strong>DPREC</strong> field
defines the resolution of <strong>DLY</strong>.
</p>
<p>
The status of the record is held in <strong>STS</strong>, mainly
relating to the link fields.  The version of the record is kept
in <strong>VER</strong> as a string, in the example form of "0-9-1".
</p>



<table border>
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
<td>VAL</td>
<td>Value to Send</td>
<td>DOUBLE</td>
<td>No</td>
<td>0.0</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
</tr>

<tr>
<td>OVAL</td>
<td>Previous Set Value</td>
<td>DOUBLE</td>
<td>No</td>
<td>0.0</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>


<tr>
<td>DLY</td>
<td>Minimum Delay</td>
<td>DOUBLE</td>
<td>Yes</td>
<td>0.0</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>WAIT</td>
<td>Waiting Status</td>
<td>Menu: False/True</td>
<td>No</td>
<td>False</td>
<td>Yes</td>
<td>No</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>OUT</td>
<td>PV to Send Value</td>
<td>Link</td>
<td>Yes</td>
<td>&nbsp;</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>OV</td>
<td>Output Link Validity</td>
<td>Menu: Ext PV NC/Ext PV OK/Local PV/Constant</td>
<td>No</td>
<td>Ext PV OK</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>SENT</td>
<td>Value Last Sent</td>
<td>DOUBLE</td>
<td>No</td>
<td>0.0</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>OSENT</td>
<td>Old Value Last Sent</td>
<td>DOUBLE</td>
<td>No</td>
<td>0.0</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>DRVLH</td>
<td>High Drive Limit</td>
<td>DOUBLE</td>
<td>Yes</td>
<td>0.0</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>DRVLL</td>
<td>Low Drive Limit</td>
<td>DOUBLE</td>
<td>Yes</td>
<td>0.0</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>DRVLS</td>
<td>Drive Limit Status</td>
<td>Menu: Normal/Low Limit/High Limit</td>
<td>No</td>
<td>Normal</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>DRVLC</td>
<td>Drive Limit Clipping</td>
<td>Menu: Off/On</td>
<td>Yes</td>
<td>Off</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>


<tr>
<td>SINP</td>
<td>Sync Input</td>
<td>Link</td>
<td>Yes</td>
<td>&nbsp;</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>SIV</td>
<td>Sync Input Validity</td>
<td>Menu: Ext PV NC/Ext PV OK/Local PV/Constant</td>
<td>No</td>
<td>Ext PV OK</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>SYNC</td>
<td>Synchronize Values</td>
<td>Menu: Idle/Process</td>
<td>No</td>
<td>Idle</td>
<td>Yes</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
</tr>

<tr>
<td>PREC</td>
<td>Value Precision</td>
<td>SHORT</td>
<td>Yes</td>
<td>6</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>DPREC</td>
<td>Delay Precision</td>
<td>SHORT</td>
<td>Yes</td>
<td>3</td>
<td>Yes</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
</tr>

<tr>
<td>STS</td>
<td>Record Status</td>
<td>Menu: Unknown/Error/Success</td>
<td>No</td>
<td>Unknown</td>
<td>Yes</td>
<td>No</td>
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
<td>VER</td>
<td>Record Version</td>
<td>STRING</td>
<td>Yes</td>
<td>&nbsp;</td>
<td>Yes</td>
<td>No</td>
<td>No</td>
<td>No</td>
</tr>


</table>


<hr>
<p><h1>4. Record Support Routines</h1>

<h3>init_record</h3>
<p>
The timing callbacks are configured, and the OUT link is checked.  The
DRVH and DRVL limits are checked to see if a positive range is
defined.
</p><p>
</p>

<h3>process</h3>

<p>
The validity of the OUT output link is checked to make sure it is a
connected PV.
</p><p> 
The VAL field is checked against the DRVH and DRVL limits (if they are
engaged).  If the value is out of bounds, the VAL value is clipped
back to the relevant limit value, and the DRVCS status is set to
"Clipped".
</p><p> 
The timing system then runs.  If there has been no value sent
to the OUT link in DLY seconds, then the VAL value is sent to OUT
immediately.  Otherwise, an internal flag is set that triggers the
value being sent to OUT when the delay has been met.
</p><p> 
The SENT field is whatever was last sent to OUT.
</p>

<h3>special</h3>

<p>
Changing the DLY delay will cause the current timed callback (used for
timing) to be canceled, and a new one with the current DLY value will
be started to replace it.  This is to make sure if someone changes the
value to a very long value, then wants to make it much shorter, you
don't have to wait that long delay.
</p><p> 
Changing the DRVH and DRVL limits will trigger a check of the range to
make sure that the range is positive.  If positive, a check is made to
see if the current value is out of bounds or not.  If it is, VAL is
not immediately changed, but any further processing will cause it to
be clipped back into range.
</p>

<hr>

<address>
    Dohn Arms<br>
    Advanced Photon Source
</address>

</body>
</html>
