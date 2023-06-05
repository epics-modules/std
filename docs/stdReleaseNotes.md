---
layout: default
title: Release Notes
nav_order: 3
---


std Release Notes
=================

 Release 3-6-4 (Jun. 5, 2023)
--------------------------------

- Fix ODEL logic in devEpidSoftCallback.c. Same fix make for devEpidSoft.c in R3-6-1.
- Fix the display of hours and minutes in countDownTimer
- Documentation updated to github pages
 
 Release 3-6-3 (Jun. 10, 2021)
------------------------------

- Scaler code moved to separate SCALER module
 
 Release 3-6-2 (Oct. 5, 2020)
-----------------------------

- bob and edl files added
- iocsh files now installed to top-level file from stdApp/iocsh
 
 Release 3-6-1 (Sep. 17, 2019)
------------------------------

- Bugfix: ODEL==0 was causing OVAL not to change
 
 Release 3-6 (Jun. 14, 2019)
----------------------------

- Fixes to epid record device support 
    - devEpidFast.c Changed behavior when KI=0 to match slow feedback, i.e. set I=0 rather than DRVL or DRVH.
    - devEpidSlow.c and devEpidSoftCallback.c Fixed comment 5) describing behavior when KI=0 to match actual code since 2003, i.e I term is set to 0, not DRVH or DRVL. Added deadband implementation using ODEL.

 Release 3-5-1 (Apr. 3, 2019)
-----------------------------

- Top level db directory is now created on build.
- Fixes to build under base v7
- Fixes to build with sequencer 2.2.6
- Fixed typos in adl files

 Release 3-5 (Jul. 10, 2018)
----------------------------

- When counting is finished, post scaler values with DBE\_VALUE\|DBE\_LOG, after updating the time stamp.
- New iocsh top-level directory, contains scripts with the correct shell commands to add an individual device support to an IOC. For use with the iocshLoad command from EPICS base 3.15, further information can be found at [https://github.com/epics-modules/xxx/wiki/IOC-Shell-Scripts](https://github.com/epics-modules/xxx/wiki/IOC-Shell-Scripts)

  Release 3-4-1 (Nov. 4, 2015)
-----------------------------

- pid\_parameters.adl: added HOPR and LOPR
- delayCmd.cpp: added space for trailing null
- define SYM\_IS\_TEXT for vxWorks version &lt; 6 in vxcall.cpp (Thanks to Lewis Muir)

 Release 3-4 (Jan. 13, 2015)
----------------------------

- Added function "vxCall()", so that all built in vxWorks functions can be invoked from the IOC shell.

 Release 3-3 (Nov. 10, 2014)
----------------------------

- Added function "doAfterIocInit()", so that databases paired with SNL programs can be specified in the same place. (Thanks to Keenan Lang)
- scalerRecord.c: use "epicsUInt32" instead of "unsigned long" for Sn and PRn fields
- devScalerAsyn.c: Changed pointers from unsigned long to epicsUInt32; changes to avoid compiler warnings
- Added displays for the aSub, compress, and sub records
- translated .adl files for caQtDM and CSS-BOY
- drvScalerSoft.c: modified to zero scaler-count PVs when scaler is started.
- Added scalerSoftCtrl database and settings file; added softScaler.cmd example of loading soft scaler into ioc.
- Modified 4step.db, replacing hard-wired "4step:" with "$(Q)". Modified 4step\_settings.req, replacing "$(Q):" with "$(Q)".

 Release 3-2 (Apr. 17, 2013)
----------------------------

- The sseq record was moved from the std module to the calc module, along with all of its databases, request files, and display files.
- Added general purpose selector (e.g., for mirror stripe) selector.db, etc.
- misc.db - added two current time PVs, P:iso8601 and P:datetime
- pvHistory.db - use Soft Timestamp device support, because it's in base
- epid.req - add autosave/restore request file for epid record
- countDownTimer.vdb - New calc engine in base requires ?: expression to have both ? and :
- devTimeOfDay.c - Use epicsTimeToStrftime, instead of to-be-deleted tsStampToText.
- selector.db, selector\*.adl - general purpose selector
- Added display files for CSS-BOY and caQtDM

 Release 3-1 (Nov 14, 2011)
---------------------------

- Added databases and MEDM-display files for genericState database.

 Release 3-0 (Sept 15, 2011)
----------------------------

- The scaler record no longer makes two calls to disarm the hardware (dset-&gt;arm(0)) when the scaler completes counting normally. (Previously, the code supporting user abort of a count was also executed after normal completion.)
- The sseq record can now treat array PVs whose elements are of type DBF\_CHAR or DBF\_UCHAR, as strings. Only the first 39 characters are handled, because the sseq record uses PVs of type DBF\_STRING internally.
- Previously, sseq record links could wait or not wait for completion, before going on to the next action. Now links can wait for completion after a specified action. For example, you can execute the first two actions without waiting in between, and then wait for both to complete before processing the third action.
- The sseq record now displays the states of its DOL and LNK links.
- Previously the sseq record did not defend itself against the possibility that it would be executed by channel access and aborted by a PV link, which would cause the record to be executed again, immediately after the abort succeeded.
- Added support for femto\_DxPCA\_x00 Femto amplifier
- drvScalerSoft.c: The callback for done must pass the value 1, not 0
- Added userMbbo, a database of ten user-controlled menu choices.
- scaler16m.db: Added COPT=="Always" to $(P)$(S)\_calc1 (transform record)
- Modified RELEASE; deleted RELEASE.arch
- Added .opi display files for CSS-BOY

 Release 2-9 (Aug. 10, 2010)
----------------------------

- ramp\_tweak.db had an illegal link in it's Init record.

 Release 2-8 (Mar. 30, 2010)
----------------------------

- fast\_pid\_control.db, devEpidFast.c. Added new parameters to the INP field, because previously the DRV\_USER strings for the data and time interval were hardcoded. The length of the resulting string now is too large for INP in many cases, so put some of it in the DESC field, which is ugly.
- epidRecord.dbd - added fields TRIG and TVAL, which are used by devEpidSoftCallback.c to support an asynchronous readback device by waiting for completion before getting the readback value.
- async\_pid\_control.db, async\_pid\_control\_settings.db, async\_pid\_control.adl -- new files to support pid control with an asynchronous readback device.
- sseq\_settings.req - Added SCAN
- drvScaler974 - new file for Ortec 974 scaler
- devScalerAsyn.c - ignore callback when DONE==0, register callbacks on address 0.
- sseqRecord.c - fixed for 64-bit arch
- no longer build shareBase.dbd
- nameVal.adl, anyRecord.adl - new display files for any record type.
- deleted some unused .adl files

Release 2-7 (Sept 10, 2008)
---------------------------

- yySeq.adl, yySeq\_full.adl - new files
- genSub-record dependency replaced by aSub record in base 3.14.10

Release 2-6 (Sept 10, 2008)
---------------------------

- The sseq record was failing to process immediately after a reboot, when ordered to process because of a CP input link.
- The gpibRecord has been replaced by the asynRecord, which is loaded from the asyn module.
- New databases: alarmClock, autoShutter, countDownTimer, ramp\_tweak, scaler16m
- The sseqRecord (string Sequence, a variant of the seq record in EPICS base) can now be told to abort the sequence it's executing.
- The scaler record has a new field, COUTP, which is like the COUT field, but is not delayed by a nonzero setting of the DLY field.
- If the scaler-record DLY field was less that 1, it was not honored. (Thanks to Xuesong Jiao for the fix.)
- New link-help displays do a better job of describing the effects on link attributes, particularly as they bear on completion detection (i.e., the behavior of a data when written to by ca\_put\_callback). 

Release 2-5-5 (April, 2007)
---------------------------


- Added scaler16m.db database, and corresponding MEDM displays. These differ from scaler16.db and its display files only in the implementation of end calculations. The new support provides end calcs for all 16 signals.
- New link-help displays.
- Added pvHistoryRegister to stdSupport.dbd
- Added field COUTP, an output link, to the caler record. This is similar to the COUT link, which sends the value of the CNT field to its target whenever CNT changes. COUTP differs from COUT in that it doesn't wait for the delay specified in the DLY field, but sends promptly after CNT changes.
- Minor change to devScalerAsyn to support change in API for asyn callbacks.
- Minor change to devEpidFast to support change in API for asyn callbacks.
- Added ramp\_tweak database and assoc. support
- Minor change to devScalerAsyn to support change in API for asyn callbacks.
- Minor change to devEpidFast to support change in API for asyn callbacks.
- Added ramp\_tweak database and assoc. support

Release 2-5-4 (Dec. 6, 2006)
----------------------------


- Changes to scaler record:
    
    
    - PRn and Sn fields are now DBF\_ULONG rather than DBF\_LONG.
    - No longer hardcode VME\_IO device type in the record logic.
    - Removed .CARD record field.
    - Changed interface to device support so that all functions pass precord rather than card, and init\_record passes pointer to device callback structure.
    - Move callback structures from dpvt to rpvt so record does not access dpvt.
- Added asyn device support for scaler record. This is currently used by the SIS3820 device support in mcaApp/SISSrc, but all scaler device support will eventually be changed to use asyn.
- Changed stdApp/Db/scaler\*.db so that $(OUT) is a macro parameter, rather than assuming VME\_IO link type.
- Deleted CARD field from stdApp/op/adl/scaler\*.adl.
- Deleted stdApp/src/femto.dbd, not needed.
- Added timer.db (resettable software timer) and related software: timer.req, timer\*.adl

Release 2-5-3
-------------


- epid record still had some FLOAT stuff to convert to DOUBLE. Fixed record and docs.
- added userStringSeq10 database, autosave request, and medm display files.
- Added support for the Femto current amplifier ([www.femto.de](http://www.femto.de/)).

Release 2-5-2
-------------


- sseqRecord.html rewritten. The old version was a minimal modification of seqRecord.html, from the EPICS Record Reference Manual, and was not very clear.
- scaler record: v3.18: Don't post CNT field unless record-support changed its value. Modified debug macro.
- devEpidFast.c: changed strtok\_r to epicsStrtok\_r
- pvHistory: new software to maintain short-term history of a few PV's in the ioc. Software comprises pvHistory.c (genSub routines), devTimeOfDay.c, pvHistory.db, pvHistory.req (autosave-request file), and pvHistory\*.adl, and relies on autosave to save/restore the value arrays.
- softMotor.db: shortened some .DESC field values so they fit.
- all\_com\_4.db: new file
- softMotorConfig.adl: related-display callups were using sCalcout files for calcout records, because synApps did not contain calcout medm displays. Now the calc module does contain calcout displays, so we use them.

Release 2-4
-----------


- mpf-related stuff replaced by asyn-related stuff
- minor scaler-record changes to accommodate Joerger VS series. Existing support for other scaler devices should not require any modifications.
- BitBus is no longer supported
- ioc-status info now provided by vxStats module
- PID support rearranged around the asyn module
- SSEQ-record delays now snap to nearest clock period, and show user the delay that will actually be used. Fixed bug that was crashing on Linux and Windows.
- [cvs log](cvsLog_R2-4.txt)

Release 2-3
-----------

This is what's left of the synApps std module after the modules autosave, calc, optics, sscan, and vme were split out.

This version is intended to build with EPICS base 3.14.5. Differences from std 2.2:


- scaler record - device support that used "struct callback" defined in devScaler.h must now replace that with "CALLBACK", and must include epics/base/include/callback.h
- sseq record - Now allows user to specify whether or not the record should wait, after putting a link value, for the resulting processing to complete before processing the next link. Some code cleanup. Added seqRecDebug.
- topDAC8.adl - Deleted to avoid conflict with dac128V module.
- VXstats\_full.adl
- pid\_control\_settings.req - added delta time
- Added device-independent scaler database and save-restore settings files.
- timeString.db - new capability in 3.14.5: stringin record can get time date.

Suggestions and Comments to:   
[Tim Mooney ](mailto:mooney@aps.anl.gov): (mooney@aps.anl.gov)
