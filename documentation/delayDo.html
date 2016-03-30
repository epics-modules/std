<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
<html>
<head>
<meta http-equiv="content-type" content=
"text/html; charset=us-ascii">
<title>delayDo</title>
<meta name="author" content="Kevin Peterson">
</head>
<body>
<img src="http://www.aps.anl.gov/bcda/Images/logo101.gif" alt=
"EPICS" hspace="5" height="101" width="101" align="right">
<h1>delayDo</h1>
For situations when something needs to be done after an intelligent delay.

<p>Please email any comments and bug reports to <a href="mailto:kmpeters_at_anl.gov">Kevin Peterson</a>

<h2>Motivation<hr></h2>
Many records can add a delay before writing to their output, but that is often not sophisticated enough for real applications.  There are times when the delay timer needs to be reset or temporarily disabled.  The delayDo support was written to provide this functionality.

<h2>Overview<hr></h2>
In the simplest mode of operating, delayDo behaves identically to a similar database with a fixed delay.  The delayDo support, however, is highly customizable.<br>

<h3>Records provided by delayDo.db:</h3>

<table summary="Records provided by delayDo.db" border="1" cellpadding="5">
<tbody>
<tr align="center">
<td><b>Record Name</b></td>
<td><b>Record Type</b></td>
<td><b>Purpose</b></td>
</tr>

<tr>
<td>$(P)$(R):activeCalc</td>
<td><a href="https://wiki-ext.aps.anl.gov/epics/index.php/RRM_3-14_Calcout">calcout</a></td>
<td>delayDo.st monitors activeCalc and starts the delay timer when activeCalc's output transitions to zero.</td>
</tr>

<tr>
<td>$(P)$(R):doSeq</td>
<td><a href="https://htmlpreview.github.io/?https://raw.githubusercontent.com/epics-modules/calc/master/documentation/sseqRecord.html">sseq</a></td>
<td>doSeq is the output of the delayDo support. It is processed by delayDo.st when the delay timer expires.</td>
</tr>

<tr>
<td>$(P)$(R):standbyCalc</td>
<td><a href="https://wiki-ext.aps.anl.gov/epics/index.php/RRM_3-14_Calcout">calcout</a></td>
<td>delayDo.st monitors standbyCalc and enters the standby state when standbyCalc's output transitions to a non-zero value.</td>
</tr>

<tr>
<td>$(P)$(R):enable</td>
<td>bo</td>
<td>Allows delayDo.st to be put into a disabled state.</td>
</tr>

<tr>
<td>$(P)$(R):delay</td>
<td>ao</td>
<td>The amount of time in seconds to wait from transition out of the active state until the doSeq is processed.</td>
</tr>

<tr>
<td>$(P)$(R):delayCalc</td>
<td><a href="https://wiki-ext.aps.anl.gov/epics/index.php/RRM_3-14_Calcout">calcout</a></td>
<td>(OPTIONAL) Allows the calculation of the delay period if the delay depends on other PVs</td>
</tr>

<tr>
<td>$(P)$(R):state</td>
<td>stringout</td>
<td>Contains the current state of delayDo.st</td>
</tr>

<tr>
<td>$(P)$(R):debug</td>
<td>longout</td>
<td>Allows enabling debug messages on the IOC console by setting to a non-zero value (higher numbers generate more messages)</td>
</tr>

</tbody>
</table>

<h3>Notes about behavior of delayDo.st:</h3>

<ul>
<li>The waiting state will be interrupted if the active calc outputs a non-zero value, which will result in further delaying the processing of the doSeq.</li>
<li>In the standby state, delayDo.st will detect active->inactive transitions and respond to them after returning from standby.</li>
<li>In the disabled state, delayDo.st will ignore any active->inactive transitions and clear any resume-delay flags.</li>
</ul>

<h3>MEDM</h3>
This is the MEDM screen for delayDo:<br><br>
<img src="delayDo.png" alt="delayDo MEDM screen"><br>

<h2>Use cases<hr></h2>

<h3>1. Disabling in-vacuum motors to prevent overheating</h3>

<h4>Goal</h4>
Disable (by stopping) six in-vacuum motors if they haven't moved in four seconds, but only if no scans (from multiple IOCs) are in progress and the motors aren't disabled.

<h4>Requirements</h4>
<ul>
<li>Standby if the FAZE fields of scan1 in the following IOCs are not "IDLE": aaa, bbb, ccc, ddd, eee, xxx</li>
<li>Standby if the motor is disabled</li>
<li>Active if motor is not done moving</li>
<li>Output "1" to motor STOP field after fixed delay of 4.0 seconds</li>
</ul>

<h4>delayDo.substitutions</h4>
<blockquote><pre>
file "$(TOP)/xxxApp/Db/delayDo.db"
{
pattern
{ P,    R,        SB_INPA,             SB_INPB,             SB_INPC,              SB_INPD,            SB_INPE,             SB_INPF,             SB_INPG,               SB_CALC,                A_INPA,         A_CALC, D_STR1,     D_LNK1,       DELAY}
{xxx:, m1, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m1_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m1.DMOV CP NMS",   "!A",     "1", "xxx:m1.STOP CA",   4.0}
{xxx:, m2, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m2_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m2.DMOV CP NMS",   "!A",     "1", "xxx:m2.STOP CA",   4.0}
{xxx:, m3, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m3_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m3.DMOV CP NMS",   "!A",     "1", "xxx:m3.STOP CA",   4.0}
{xxx:, m4, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m4_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m4.DMOV CP NMS",   "!A",     "1", "xxx:m4.STOP CA",   4.0}
{xxx:, m5, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m5_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m5.DMOV CP NMS",   "!A",     "1", "xxx:m5.STOP CA",   4.0}
{xxx:, m6, "aaa:scan1.FAZE CP", "bbb:scan1.FAZE CP", "ccc:scan1.FAZE CP", "ddd:scan1.FAZE CP", "eee:scan1.FAZE CP", "xxx:scan1.FAZE CP", "xxx:m6_able.VAL CP", "A||B||C||D||E||F||G",  "xxx:m6.DMOV CP NMS",   "!A",     "1", "xxx:m6.STOP CA",   4.0}
}
</pre></blockquote>

<h3>2. Unsticking areaDetector when detector is stuck acquiring</h3>

<h4>Goal</h4>
Stop acquisition if acquisition takes 3 seconds longer than the expected amount of time

<h4>Requirements</h4>
<ul>
<li>Standby when ImageMode is not "Multiple"</li>
<li>Active when DetectorState_RBV is not "Acquire"</li>
<li>Calculate delay based on number of images to be acquired: theoretical acquire time + 3 seconds</li>
<li>Output "Done" to Acquire PV after delay</li>
</ul>

<h4>delayDo.substitutions</h4>
<blockquote><pre>
file "$(TOP)/xxxApp/Db/delayDo.db"
{
pattern
{ P,       R,             SB_INPA,        SB_CALC,              A_INPA,             A_CALC, D_STR1,         D_LNK1,                 T_INPA,                     T_INPB,                     T_INPC,              T_CALC    }
{xxx:, delayDo1, "xxx:cam1:ImageMode CP",   "A=1", "xxx:cam1:DetectorState_RBV CP", "A!=1",    "0", "xxx:cam1:Acquire CA", "xxx:cam1:AcquireTime CP", "xxx:cam1:AcquirePeriod CP", "xxx:cam1:NumImages CP", "MAX(A,B)*C+3" }
}
</pre></blockquote>
<br>
</body>
</html>
