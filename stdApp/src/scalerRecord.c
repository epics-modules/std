/*******************************************************************************
scalerRecord.c
Record-support routines for <= 64-channel, 32-bit scaler


Original Author: Tim Mooney
Date: 1/16/95

Experimental Physics and Industrial Control System (EPICS)

Copyright 1995, the University of Chicago Board of Governors.

This software was produced under U.S. Government contract
W-31-109-ENG-38 at Argonne National Laboratory.

Initial development by:
	The X-ray Optics Group
	Experimental Facilities Division
	Advanced Photon Source
	Argonne National Laboratory

OSI by S. Kate Feng, NSLS, BNL 3/03

Modification Log:
-----------------
06/26/93    tmm     Lecroy-scaler record
01/16/95    tmm     Joerger-scaler
08/28/95    tmm     Added .vers (code version) and .card (VME-card number)
                    fields 
02/8/96     tmm     v1.7:  Fixed bug: was posting CNT field several times
                    when done.
02/21/96    tmm     v1.71:  precision of vers field is 2
06/5/96     tmm     v1.8: precision defaults to PREC field
08/16/96    tmm     v2.0: conversion to EPICS 3.13
08/16/96    tmm     v2.1: fixed off-by-one problem (in 3.13 version)
02/27/97    tmm     v2.11: fix TP/PR1 problem
03/03/97    tmm     v3.0: allow auto-count overridden by user count
07/09/97    tmm     v3.1: init_record wasn't posting TP, PR1, FREQ, or CARD
11/14/97    tmm     v3.2: fixed bug: if .cnt went true then false during the
                    .dly period, scaler would lock up.
04/24/98    tmm     v3.3 call recGblFwdLink only when user count completes.
10/2/98     tmm     v3.4 if dbPutNotify then delay longer before resuming
                    autocount.
03/24/99    tmm     v3.5 call recGblFwdLink whenever scaler is idle and
                    CNT = 0.
04/21/99    tmm     v3.6 call recGblFwdLink whenever USER_STATE_IDLE and
                    SCALER_STATE_IDLE and CNT makes a transition to 0.
07/14/99    tmm     v3.7 minor fixes
11/04/99    tmm     v3.8 added link to start and stop external process that
                    should coincide with scaler-integration period.
07/14/99    tmm     v3.9 hold time before autocount wipes scalers increased
                    to 5 sec for all count requests.
 ?          ?       v3.10 changed max number of signals from 16 to 32
11/12/01   tmm      v3.11 hold time before autocount wipes scalers is
                    volatile int
01/08/02   tmm     v3.12 Set VAL field to T and post after completed count
05/08/03   tmm     v3.13 Kate Feng's OSI version, with a new version number
10/22/03   tmm     v3.14 3.13 compatibility removed
09/22/03   tmm     v3.13 changed max number of signals from 32 to 64
09/26/03   tmm     v3.14 Make sure channel-1 preset count agrees with time
                   preset and freq.  (Required for VS64, because it changes
                   freq, and uses pr1/freq to infer count time.)
02/19/04   tmm     v3.15 Added semaphore to avoid contention for scanLock
                   between autocount-restart callback and periodic update
                   callback.  More mods for Joerger VS64
05/17/04   tmm     v3.16 merged VS64-capable 3.13 version with 3.14 version
11/17/04   tmm     v3.17 If device support changed PR1, recalc and post TP.
                   Autocount now calls write_preset again if device support
                   changed PR1, but doesn't let this change get out to user.
03/30/06   tmm     v3.18 Don't post CNT unless we changed it.
10/26/06   mlr     v3.19 Changed interface to device support 
                     All functions pass precord rather than card
                     init_record passes pointer to device callback structure
                   Don't assume VME_IO in record.
                   Move callback structures from dpvt to rpvt so record does not
                   access dpvt.
                   Change PRn and Sn fields from long to unsigned long.
*******************************************************************************/
#define VERSION 3.19

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include	<string.h>
#include <math.h>
#include <float.h>
#include <ctype.h>

#include	<epicsTimer.h>


#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbScan.h>
#include	<dbEvent.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<errlog.h>
#include	<recSup.h>
#include	<recGbl.h>
#include	<devSup.h>
#include	<special.h>
#include	<epicsMutex.h>
#define GEN_SIZE_OFFSET
#include	"scalerRecord.h"
#undef GEN_SIZE_OFFSET
#include	"devScaler.h"
#include	<epicsExport.h>

#include	<epicsVersion.h>
#ifndef EPICS_VERSION_INT
#define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#define EPICS_VERSION_INT VERSION_INT(EPICS_VERSION, EPICS_REVISION, EPICS_MODIFICATION, EPICS_PATCH_LEVEL)
#endif
#define LT_EPICSBASE(V,R,M,P) (EPICS_VERSION_INT < VERSION_INT((V),(R),(M),(P)))

#define SCALER_STATE_IDLE 0
#define SCALER_STATE_WAITING 1
#define SCALER_STATE_COUNTING 2

#define USER_STATE_IDLE 0
#define USER_STATE_WAITING 1
#define USER_STATE_REQSTART 2
#define USER_STATE_COUNTING 3

#ifdef NODEBUG
#define Debug(l,FMT,V) ;
#else
#define Debug(level)  if(level<=scalerRecordDebug) \
		{printf("%s(%d):",__FILE__,__LINE__); printf(
#endif

volatile int scalerRecordDebug = 0;
volatile int scaler_wait_time = 10;

#define MIN(a,b) (a)<(b)?(a):(b)
#define MAX(a,b) (a)>(b)?(a):(b)
#define NINT(f) (unsigned long)((f)>0 ? (f)+0.5 : (f)-0.5)

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record(dbCommon *pcommon, int pass);
static long process(dbCommon *pcommon);
static long special(const DBADDR *paddr, int after);
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
static long get_precision(const DBADDR *paddr, long *precision);
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL

struct rset scalerRSET = {
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};
epicsExportAddress(rset, scalerRSET);

struct rpvtStruct {
	epicsMutexId updateMutex;
	CALLBACK *pcallbacks;
};

static void do_alarm();
static void monitor();
static void updateCounts(scalerRecord *pscal);

static void deviceCallbackFunc(CALLBACK *pcb)
{
	scalerRecord *pscal;

	callbackGetUser(pscal, pcb);
	Debug(5) "scaler deviceCallbackFunc: entry for '%s'\n", pscal->name);}
	dbScanLock((struct dbCommon *)pscal);
	process((struct dbCommon *)pscal);
	dbScanUnlock((struct dbCommon *)pscal);
}


static void updateCallbackFunc(CALLBACK *pcb)
{
	scalerRecord *pscal;
	struct rpvtStruct *prpvt;

	callbackGetUser(pscal, pcb);
	Debug(5) "scaler updateCallbackFunc: entry for '%s'\n", pscal->name);}
	prpvt = (struct rpvtStruct *)pscal->rpvt;
	epicsMutexLock(prpvt->updateMutex);
	updateCounts(pscal);
	epicsMutexUnlock(prpvt->updateMutex);
}

static void delayCallbackFunc(CALLBACK *pcb)
{
	scalerRecord *pscal;

	/*
	 * User asked us to start counting after a delay that has now expired.
	 * If user has not rescinded that request in the meantime, tell
	 * process() to start counting.
	 */
	callbackGetUser(pscal, pcb);
	Debug(5) "scaler delayCallbackFunc: entry for '%s'\n", pscal->name);}
	if (pscal->us == USER_STATE_WAITING && pscal->cnt) {
		pscal->us = USER_STATE_REQSTART;
		(void)scanOnce((void *)pscal);
	}
}

static void autoCallbackFunc(CALLBACK *pcb)
{
	scalerRecord *pscal;

	callbackGetUser(pscal, pcb);
	Debug(5) "scaler autoCallbackFunc: entry for '%s'\n", pscal->name);}
	(void)scanOnce((void *)pscal);
}

static long init_record(dbCommon *pcommon, int pass)
{
	scalerRecord *pscal = (scalerRecord *) pcommon;
	long status;
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	CALLBACK *pcallbacks, *pupdateCallback, *pdelayCallback,
		*pautoCallback, *pdeviceCallback;
	struct rpvtStruct *prpvt;

	Debug(5) "scaler init_record: pass = %d\n", pass);}
	Debug(5) "init_record: .PR1 = %d\n", (epicsUInt32)pscal->pr1);}
	if (pass == 0) {
		pscal->vers = VERSION;
	pscal->rpvt = (void *)calloc(1, sizeof(struct rpvtStruct));
	prpvt = (struct rpvtStruct *)pscal->rpvt;
		if ((prpvt->updateMutex = epicsMutexCreate()) == 0) {
			epicsPrintf("scalerRecord:init_record: could not create mutex.\n");
			return(-1);
		}
		return (0);
	}
	prpvt = (struct rpvtStruct *)pscal->rpvt;

	/* Gotta have a .val field.  Make its value reproducible. */
	pscal->val = 0;

	/*** setup callback stuff (note: array of 4 callback structures) ***/
	pcallbacks = (CALLBACK *)(calloc(4,sizeof(CALLBACK)));
	prpvt->pcallbacks = pcallbacks;

	/* callback to implement periodic updates */
	pupdateCallback = (CALLBACK *)&(pcallbacks[0]);
	callbackSetCallback(updateCallbackFunc, pupdateCallback);
	callbackSetPriority(pscal->prio, pupdateCallback);
	callbackSetUser((void *)pscal, pupdateCallback);

	/* second callback to implement delay */
	pdelayCallback = (CALLBACK *)&(pcallbacks[1]);
	callbackSetCallback(delayCallbackFunc, pdelayCallback);
	callbackSetPriority(pscal->prio, pdelayCallback);
	callbackSetUser((void *)pscal, pdelayCallback);

	/* third callback to implement auto-count */
	pautoCallback = (CALLBACK *)&(pcallbacks[2]);
	callbackSetCallback(autoCallbackFunc, pautoCallback);
	callbackSetPriority(pscal->prio, pautoCallback);
	callbackSetUser((void *)pscal, pautoCallback);

	/* fourth callback for device support */
	pdeviceCallback = (CALLBACK *)&(pcallbacks[3]);
	callbackSetCallback(deviceCallbackFunc, pdeviceCallback);
	callbackSetPriority(pscal->prio, pdeviceCallback);
	callbackSetUser((void *)pscal, pdeviceCallback);

	/* Check that we have everything we need. */
	if (!(pdset = (SCALERDSET *)(pscal->dset)))
	{
		recGblRecordError(S_dev_noDSET,(void *)pscal, "scaler: init_record");
		return(S_dev_noDSET);
	}

	Debug(2) "init_record: calling dset->init_record\n");}
	if (pdset->init_record)
	{
		status=(*pdset->init_record)(pscal, pdeviceCallback);
		Debug(3) "init_record: dset->init_record returns %ld\n", status);}
		if (status) {
			return (status);
		}
	}

	/* default clock freq */
	if (pscal->freq == 0.0) {
		pscal->freq = 1.0e7;
		db_post_events(pscal,&(pscal->freq),DBE_VALUE);
	}

	/* default count time */
	if ((pscal->tp == 0.0) && (pscal->pr1 == 0)) {
		pscal->tp = 1.0;
		db_post_events(pscal,&(pscal->pr1),DBE_VALUE);
	}

	/* convert between time and clock ticks */
	if (pscal->tp) {
		/* convert time to clock ticks */
		pscal->pr1 = (epicsUInt32) (pscal->tp * pscal->freq);
		db_post_events(pscal,&(pscal->pr1),DBE_VALUE);
		Debug(3) "init_record: .TP != 0, so .PR1 set to %d\n", (epicsUInt32)pscal->pr1);}
	} else if (pscal->pr1 && pscal->freq) {
		/* convert clock ticks to time */
		pscal->tp = (double)(pscal->pr1 / pscal->freq);
		db_post_events(pscal,&(pscal->tp),DBE_VALUE);
		Debug(3) "init_record: .PR1/.FREQ != 0, so .TP set to %f\n", pscal->tp);}
	}
	return(0);
}


static long process(dbCommon *pcommon)
{
	scalerRecord *pscal = (scalerRecord *) pcommon;
	int i, status, prev_scaler_state, save_pr1, old_pr1;
	double old_freq;
	int justFinishedUserCount=0, justStartedUserCount=0, putNotifyOperation=0;
	epicsUInt32 *ppreset = (epicsUInt32 *)&(pscal->pr1);
	short *pdir = (short *)&pscal->d1;
	short *pgate = (short *)&pscal->g1;
	struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
	CALLBACK *pcallbacks = prpvt->pcallbacks;
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	CALLBACK *pupdateCallback = (CALLBACK *)&(pcallbacks[0]);
	/* CALLBACK *pdelayCallback = (CALLBACK *)&(pcallbacks[1]); */
	CALLBACK *pautoCallback = (CALLBACK *)&(pcallbacks[2]);

	
	Debug(5) "process: entry\n");}
	epicsMutexLock(prpvt->updateMutex);

	pscal->pact = TRUE;
	pscal->udf = FALSE;
	prev_scaler_state = pscal->ss;

	/* If we're being called as a result of a done-counting interrupt, */
	/* (*pdset->done)(pscal) will return TRUE */
	if ((*pdset->done)(pscal)) {
		pscal->ss = SCALER_STATE_IDLE;
		/* Auto-count cycle is not allowed to reset .CNT field. */
		if (pscal->us == USER_STATE_COUNTING) {
			pscal->cnt = 0;
			db_post_events(pscal,&(pscal->cnt),DBE_VALUE);
			pscal->us = USER_STATE_IDLE;
			justFinishedUserCount = 1;
			if (pscal->ppn) putNotifyOperation = 1;
		}
	}

	if (pscal->cnt != pscal->pcnt) {
		int handled = 0;
		if (pscal->cnt && ((pscal->us == USER_STATE_REQSTART) ||
							(pscal->us == USER_STATE_WAITING))) {
			/*** if we're already counting (auto-count), stop ***/
			if (pscal->ss == SCALER_STATE_COUNTING) {
				(*pdset->arm)(pscal, 0);
				pscal->ss = SCALER_STATE_IDLE;
			}

			if (pscal->us == USER_STATE_REQSTART) {
				/*** start counting ***/

				/* disarm, disable interrupt generation, reset disarm-on-cout, */
				/* clear mask register, clear direction register, clear counters */
				(*pdset->reset)(pscal);

				/*
				 * We tell device support how long to count by giving it a preset,
				 * because the Joerger VSCx device support doesn't know the clock
				 * frequency.  But the VS64 sets its own clock frequency, and adjusts
				 * the preset correspondingly.  We don't want to include the algorithm
				 * that calculates it here, but we do want to ensure that we've specified
				 * the count time with the best achievable precision, so if device support
				 * has changed the preset, we recalc the preset from tp, using the freq
				 * set by device support, and call write_preset again.
				 */
				old_pr1 = pscal->pr1;
				old_freq = pscal->freq;
				/* Make sure channel-1 preset count agrees with time preset and freq */
				if (pscal->pr1 != (epicsUInt32) NINT(pscal->tp * pscal->freq)) {
					pscal->pr1 = (epicsUInt32) NINT(pscal->tp * pscal->freq);
				}
				save_pr1 = pscal->pr1;
				for (i=0; i<pscal->nch; i++) {
					pdir[i] = pgate[i];
					if (pgate[i]) {
						Debug(5) "process: writing preset: %d.\n", ppreset[i]);}
						(*pdset->write_preset)(pscal, i, ppreset[i]);
					}
				}
				if (save_pr1 != pscal->pr1) {
					pscal->pr1 = (epicsUInt32) NINT(pscal->tp * pscal->freq);
					(*pdset->write_preset)(pscal, 0, pscal->pr1);
				}
				if (old_pr1 != pscal->pr1) {
					db_post_events(pscal,&(pscal->pr1),DBE_VALUE);
					pscal->tp = (double)(pscal->pr1 / pscal->freq);
					db_post_events(pscal,&(pscal->tp),DBE_VALUE);
				}
				if (old_freq != pscal->freq) {
					db_post_events(pscal,&(pscal->freq),DBE_VALUE);
				}

				(*pdset->arm)(pscal, 1);
				pscal->ss = SCALER_STATE_COUNTING;
				pscal->us = USER_STATE_COUNTING;
				handled = 1;
				justStartedUserCount = 1;
			}
		} else if (!pscal->cnt) {
			/*** stop counting ***/
			if (pscal->ss != SCALER_STATE_IDLE)	(*pdset->arm)(pscal, 0);
			pscal->ss = SCALER_STATE_IDLE;
			pscal->us = USER_STATE_IDLE;
			justFinishedUserCount = 1;
			handled = 1;
		}
		if (handled) {
			pscal->pcnt = pscal->cnt;
		}
	}

	/* read and display scalers */
	updateCounts(pscal);

	if (justStartedUserCount || justFinishedUserCount) {
		/* fire .cout link to trigger anything that should coincide with scaler integration */
		status = dbPutLink(&pscal->cout, DBR_SHORT, &pscal->cnt, 1);
		if (!RTN_SUCCESS(status)) {
			Debug(5) "scaler:process: ERROR %d PUTTING TO COUT LINK.\n", status);}
		}
		if (justFinishedUserCount) {
			/* fire .coutp link to trigger anything that should coincide with scaler integration */
			status = dbPutLink(&pscal->coutp, DBR_SHORT, &pscal->cnt, 1);
			if (!RTN_SUCCESS(status)) {
				Debug(5) "scaler:process: ERROR %d PUTTING TO COUTP LINK.\n", status);}
			}
		}
	}

	/* done counting? */
	if (pscal->ss == SCALER_STATE_IDLE) {
		recGblGetTimeStamp(pscal);
		do_alarm(pscal);
		monitor(pscal);
		if ((pscal->pcnt==0) && (pscal->us == USER_STATE_IDLE)) {
			if (prev_scaler_state == SCALER_STATE_COUNTING) {
				pscal->val = pscal->t;
				db_post_events(pscal,&(pscal->val),DBE_VALUE);
			}
			recGblFwdLink(pscal);
		}
	}

	/* Are we in auto-count mode and not already counting? */
	if (pscal->us == USER_STATE_IDLE && pscal->cont &&
		pscal->ss != SCALER_STATE_COUNTING) {
	double dly_sec=pscal->dly1;  /* seconds to delay */

		if (justFinishedUserCount) dly_sec = MAX(pscal->dly1, scaler_wait_time);
		if (putNotifyOperation) dly_sec = MAX(pscal->dly1, scaler_wait_time);
		/* if (we-have-to-wait && we-haven't-already-waited) { */
		if (dly_sec > 0 && pscal->ss != SCALER_STATE_WAITING) {
			Debug(5) "process: scheduling autocount restart\n");}
			/*
			 * Schedule a callback, and make a note that counting should start
			 * the next time we process (if pscal->ss is still SCALER_STATE_WAITING
			 * at that time).
			 */
			pscal->ss = SCALER_STATE_WAITING;  /* tell ourselves to start next time */
			callbackRequestDelayed(pautoCallback, dly_sec);
		} else {
			Debug(5) "process: restarting autocount\n");}
			/* Either the delay time is zero, or pscal->ss = SCALER_STATE_WAITING
			 * (we've already waited), so start auto-count counting.
			 * Different rules apply for auto-count counting:
			 * If (.TP1 >= .001 s) we count .TP1 seconds, regardless of any
			 * presets the user may have set.
			 */
			 old_freq = pscal->freq;
			 old_pr1 = pscal->pr1;
			(*pdset->reset)(pscal);
			if (pscal->tp1 >= 1.e-3) {
				save_pr1 = pscal->pr1;
				(*pdset->write_preset)(pscal, 0, (unsigned long)(pscal->tp1*pscal->freq));
				if (save_pr1 != pscal->pr1) {
					/*
					 * Device support wants to use a different clock freq.  We might
					 * get a more accurate counting time if we recalc the preset count
					 * from tp1 with the new clock frequency.
					 */
					(*pdset->write_preset)(pscal, 0, (unsigned long)(pscal->tp1*pscal->freq));
				}

			} else {
				for (i=0; i<pscal->nch; i++) {
					pdir[i] = pgate[i];
					if (pgate[i]) (*pdset->write_preset)(pscal, i, ppreset[i]);
				}
			}
			if (old_freq != pscal->freq) db_post_events(pscal,&(pscal->freq),DBE_VALUE);
			/* Don't let autocount disturb user's channel-1 preset */
			pscal->pr1 = old_pr1;
			(*pdset->arm)(pscal, 1);
			pscal->ss = SCALER_STATE_COUNTING;

			/* schedule first update callback */
			if (pscal->rat1 > .1) {
				callbackRequestDelayed(pupdateCallback, 1.0/pscal->rat1);
			}
		}
	}

	pscal->pact = FALSE;
	epicsMutexUnlock(prpvt->updateMutex);
	return(0);
}


static void updateCounts(scalerRecord *pscal)
{
	int i, called_by_process;
	float rate;
	epicsUInt32 *pscaler = (epicsUInt32 *)&(pscal->s1);
	epicsUInt32 counts[MAX_SCALER_CHANNELS];
	struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
	CALLBACK *pcallbacks = prpvt->pcallbacks;
	CALLBACK *pupdateCallback = (CALLBACK *)&(pcallbacks[0]);
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	double old_t;

	called_by_process = (pscal->pact == TRUE);
	Debug(5) "updateCounts: %s called by process()\n", called_by_process ? " " : "NOT");}
	if (!called_by_process) {
		if (pscal->ss != SCALER_STATE_IDLE)
			pscal->pact = TRUE;
		else
			return;
	}

	/* read scalers (get pointer to actual VME-resident scaler-data array) */
	if (pscal->us != USER_STATE_WAITING) {
		(*pdset->read)(pscal, counts);
	} else {
		for (i=0; i<pscal->nch; i++) {counts[i] = 0;}
	}

	Debug(5) "updateCounts: posting scaler values\n");}
	/* post scaler values */
	for (i=0; i<pscal->nch; i++) {
		if (counts[i] != pscaler[i]) {
			pscaler[i] = counts[i];
			db_post_events(pscal,&(pscaler[i]),DBE_VALUE);
		}
	}
	/* convert clock ticks to time. Note device support may have changed freq. */
	old_t = pscal->t;
	pscal->t = pscaler[0] / pscal->freq;
	if (pscal->t != old_t) db_post_events(pscal,&(pscal->t),DBE_VALUE);

	if (pscal->ss == SCALER_STATE_COUNTING) {
		/* arrange to call this routine again after user-specified time */
		Debug(5) "updateCounts: arranging for callback\n");}
		rate = ((pscal->us == USER_STATE_COUNTING) ? pscal->rate : pscal->rat1);
		if (rate > .1) {
			callbackRequestDelayed(pupdateCallback, 1.0/rate);
		}
	}

	if (!called_by_process) pscal->pact = FALSE;
	Debug(5) "updateCounts: exit\n");}
}


static long special(const DBADDR *paddr, int after)
{
	scalerRecord *pscal = (scalerRecord *)(paddr->precord);
	int i=0, status;
	unsigned short *pdir, *pgate;
	epicsUInt32 *ppreset;
	struct rpvtStruct *prpvt = (struct rpvtStruct *)pscal->rpvt;
	CALLBACK *pcallbacks = prpvt->pcallbacks;
	CALLBACK *pdelayCallback = (CALLBACK *)&(pcallbacks[1]);
	int fieldIndex = dbGetFieldIndex(paddr);
	double dly=0.0;

	Debug(5) "special: entry; after=%d\n", after);}
	if (!after) return (0);

	switch (fieldIndex) {
	case scalerRecordCNT:
		/* Ignore redundant (pscal->cnt == 1) commands */
		if (pscal->cnt && (pscal->us != USER_STATE_IDLE)) return(0);

		/* fire .coutp link to trigger anything that should coincide with scaler integration */
		status = dbPutLink(&pscal->coutp, DBR_SHORT, &pscal->cnt, 1);
		if (!RTN_SUCCESS(status)) {
			Debug(5) "scaler:special: ERROR %d PUTTING TO COUTP LINK.\n", status);}
		}

		/* Scan record if it's not Passive.  (If it's Passive, it'll get */
		/* scanned automatically, since .cnt is a Process-Passive field.) */
		/* Arrange to process after user-specified delay time */
		dly = pscal->dly;   /* seconds to delay */
		if (dly<0.0) dly = 0.0;
		if (dly == 0.0 || pscal->cnt == 0) {
			/*** handle request now ***/
			if (pscal->cnt) {
				/* start counting */
				pscal->us = USER_STATE_REQSTART;
			} else {
				/* abort any counting or request to start counting */
				switch (pscal->us) {
				case USER_STATE_WAITING:
					/* We may have a watchdog timer going.  Cancel it. */
					if (pdelayCallback->timer) epicsTimerCancel(pdelayCallback->timer);
					pscal->us = USER_STATE_IDLE;
					break;
				case USER_STATE_REQSTART:
					pscal->us = USER_STATE_IDLE;
					break;
				default:
					break;
				}
			}
			if (pscal->scan) scanOnce((void *)pscal);
		}
		else {
			/* schedule callback to handle request */
			pscal->us = USER_STATE_WAITING;
			callbackRequestDelayed(pdelayCallback, pscal->dly);
		}
		break;

	case scalerRecordCONT:
		/* Scan record if it's not Passive.  (If it's Passive, it'll get */
		/* scanned automatically, since .cont is a Process-Passive field.) */
		if (pscal->scan) scanOnce((void *)pscal);
		break;

	case scalerRecordTP:
		/* convert time to clock ticks */
		pscal->pr1 = (epicsUInt32) (pscal->tp * pscal->freq);
		db_post_events(pscal,&(pscal->pr1),DBE_VALUE);
		pscal->d1 = pscal->g1 = 1;
		db_post_events(pscal,&(pscal->d1),DBE_VALUE);
		db_post_events(pscal,&(pscal->g1),DBE_VALUE);
		break;

	case scalerRecordPR1:
		/* convert clock ticks to time */
		pscal->tp = (double)(pscal->pr1 / pscal->freq);
		db_post_events(pscal,&(pscal->tp),DBE_VALUE);
		if (pscal->tp > 0) {
			pscal->d1 = pscal->g1 = 1;
			db_post_events(pscal,&(pscal->d1),DBE_VALUE);
			db_post_events(pscal,&(pscal->g1),DBE_VALUE);
		}
		break;

	case scalerRecordRATE:
		pscal->rate = MIN(60.,MAX(0.,pscal->rate));
		db_post_events(pscal,&(pscal->tp),DBE_VALUE);
		break;

	default:
		if ((fieldIndex >= scalerRecordPR2) &&
				(fieldIndex <= scalerRecordPR64)) {
			i = ((char *)paddr->pfield - (char *)&(pscal->pr1)) / sizeof(epicsUInt32);
			Debug(4) "special: channel %d preset\n", i);}
			pdir = (unsigned short *) &(pscal->d1);
			pgate = (unsigned short *) &(pscal->g1);
			ppreset = (epicsUInt32 *) &(pscal->pr1);
			if (ppreset[i] > 0) {
				pdir[i] = pgate[i] = 1;
				db_post_events(pscal,&(pdir[i]),DBE_VALUE);
				db_post_events(pscal,&(pgate[i]),DBE_VALUE);
			}
		}
		else if ((fieldIndex >= scalerRecordG1) &&
				(fieldIndex <= scalerRecordG64)) {
			/* If user set gate field, make sure preset counter has some */
			/* reasonable value. */
			i = (int)(((char *)paddr->pfield - (char *)&(pscal->g1)) / sizeof(short));
			Debug(4) "special: channel %d gate\n", i);}
			ppreset = (epicsUInt32 *) &(pscal->pr1);
			pgate = (unsigned short *) &(pscal->g1);
			if (pgate[i] && (ppreset[i] == 0)) {
				ppreset[i] = 1000;
				db_post_events(pscal,&(ppreset[i]),DBE_VALUE);
			}
		}
		break;
	} /* switch (fieldIndex) */

	return(0);
}

static long get_precision(const DBADDR *paddr, long *precision)
{
	scalerRecord *pscal = (scalerRecord *) paddr->precord;
	int fieldIndex = dbGetFieldIndex(paddr);

	*precision = pscal->prec;
	if (fieldIndex == scalerRecordVERS) {
		*precision = 2;
	} else if (fieldIndex >= scalerRecordVAL) {
		*precision = pscal->prec;
	} else {
		recGblGetPrec(paddr, precision);	/* Field is in dbCommon */
	}
	return (0);
}


static void do_alarm(scalerRecord *pscal)
{
	if(pscal->udf == TRUE ){
#if LT_EPICSBASE(3,15,0,2)
		recGblSetSevr(pscal,UDF_ALARM,INVALID_ALARM);
#else
		recGblSetSevr(pscal,UDF_ALARM,pscal->udfs);
#endif
		return;
	}
	return;
}

static void monitor(scalerRecord *pscal)
{
	unsigned short monitor_mask;
	epicsUInt32 *pscaler = (epicsUInt32 *)&(pscal->s1);
	int i;

	monitor_mask = recGblResetAlarms(pscal);

	monitor_mask|=(DBE_VALUE|DBE_LOG);

	/* check all value fields for changes */
	/* post scaler values */
	for (i=0; i<pscal->nch; i++) {
		db_post_events(pscal,&(pscaler[i]),DBE_LOG);
	}
	return;
}


#include <epicsExport.h>

epicsExportAddress(int, scalerRecordDebug);
epicsExportAddress(int, scaler_wait_time);
