/*******************************************************************************
scalerRecord.c
Record-support routines for <= 32-channel, 32-bit scaler

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
.01  6/26/93	tmm     Lecroy-scaler record
.02  1/16/95    tmm     Joerger-scaler
.03  8/28/95    tmm     Added .vers (code version) and .card (VME-card number)
                        fields 
.04  2/8/96     tmm     v1.7:  Fixed bug: was posting CNT field several times
                        when done.
.05  2/21/96    tmm     v1.71:  precision of vers field is 2
.06  6/5/96     tmm     v1.8: precision defaults to PREC field
.07  8/16/96    tmm     v2.0: conversion to EPICS 3.13
.07  8/16/96    tmm     v2.1: fixed off-by-one problem (in 3.13 version)
.09  2/27/97    tmm     v2.11: fix TP/PR1 problem
.10  3/03/97    tmm     v3.0: allow auto-count overridden by user count
.11  7/09/97    tmm     v3.1: init_record wasn't posting TP, PR1, FREQ, or CARD
.12 11/14/97    tmm     v3.2: fixed bug: if .cnt went true then false during the
                        .dly period, scaler would lock up.
.13  4/24/98    tmm     v3.3 call recGblFwdLink only when user count completes.
.14  10/2/98    tmm     v3.4 if dbPutNotify then delay longer before resuming
                        autocount.
.15  3/24/99    tmm     v3.5 call recGblFwdLink whenever scaler is idle and
						CNT = 0.
.16  4/21/99    tmm     v3.6 call recGblFwdLink whenever USER_STATE_IDLE and
						SCALER_STATE_IDLE and CNT makes a transition to 0.
.17  7/14/99    tmm     v3.7 minor fixes
.18 11/04/99    tmm     v3.8 added link to start and stop external process that
                        should coincide with scaler-integration period.
.19  7/14/99    tmm     v3.9 hold time before autocount wipes scalers increased
                        to 5 sec for all count requests.
.20  ?          ?       v3.10 changed max number of signals from 16 to 32
.21  11/12/01   tmm     v3.11 hold time before autocount wipes scalers is
                        volatile int
.22  01/08/02   tmm     v3.12 Set VAL field to T and post after completed count
*******************************************************************************/
#define VERSION 3.12

#include        <epicsVersion.h>

#if EPICS_VERSION < 3 || (EPICS_VERSION==3 && EPICS_REVISION < 14)
#define NOT_YET_OSI
#endif

#if defined(NOT_YET_OSI) || defined(VXWORKSTARGET)
#include	<vxWorks.h>
#ifndef __vxworks
#define __vxworks 1
#endif
#include	<types.h>
#include	<stdioLib.h>
#include	<lstLib.h>
#include	<string.h>
#include	<wdLib.h>
#else
#include        <epicsTimer.h>

extern epicsTimerQueueId	scalerWdTimerQ;

#endif

#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <dbScan.h>
#include        <dbEvent.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<recSup.h>
#include        <recGbl.h>
#include	<devSup.h>
#include	<special.h>
#define GEN_SIZE_OFFSET
#include	"scalerRecord.h"
#undef GEN_SIZE_OFFSET
#include	"devScaler.h"
#include        <epicsExport.h>

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
#define Debug(l,FMT,V) {  if(l <= scalerRecordDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif
volatile int scalerRecordDebug = 0;
volatile int scaler_wait_time = 10;

#define MIN(a,b) (a)<(b)?(a):(b)
#define MAX(a,b) (a)>(b)?(a):(b)


/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
static long special();
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
static long get_precision();
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

static void do_alarm();
static void monitor();
static void updateCounts(scalerRecord *pscal);

static void deviceCallbackFunc(struct callback *pcb)
{
    struct dbCommon *precord=pcb->precord;

    dbScanLock(precord);
    process(precord);
    dbScanUnlock(precord);
}


static void updateCallbackFunc(struct callback *pcb)
{
    struct dbCommon *precord=pcb->precord;

    dbScanLock(precord);
    updateCounts((scalerRecord *)precord);
    dbScanUnlock(precord);
}

static void userCallbackFunc(struct callback *pcb)
{
    scalerRecord *pscal = (scalerRecord *)pcb->precord;

	/*
	 * User asked us to start counting after a delay that has now expired.
	 * If user has not rescinded that request in the meantime, tell
	 * process() to start counting.
	 */
	if (pscal->us == USER_STATE_WAITING && pscal->cnt) {
		pscal->us = USER_STATE_REQSTART;
		(void)scanOnce((void *)pscal);
	}
}

static void autoCallbackFunc(struct callback *pcb)
{
    scalerRecord *pscal = (scalerRecord *)pcb->precord;

	(void)scanOnce((void *)pscal);
}

static long init_record(pscal,pass)
scalerRecord *pscal;
int pass;
{
	long status;
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	struct callback *pcallbacks, *pupdateCallback, *puserCallback, *pautoCallback, *pdeviceCallback;

	Debug(5, "scaler init_record: pass = %d\n", pass);
	Debug(5, "init_record: .PR1 = %ld\n", pscal->pr1);
	if (pass == 0) {
		pscal->vers = VERSION;
		return (0);
	}

	/* Gotta have a .val field.  Make its value reproducible. */
	pscal->val = 0;

	/*** setup callback stuff (note: array of 4 callback structures) ***/
	pcallbacks = (struct callback *)(calloc(4,sizeof(struct callback)));
	pscal->dpvt = (void *)pcallbacks;

	/* callback to implement periodic updates */
	pupdateCallback = (struct callback *)&(pcallbacks[0]);
	callbackSetCallback(updateCallbackFunc,&pupdateCallback->callback);
	callbackSetPriority(pscal->prio,&pupdateCallback->callback);
	pupdateCallback->precord = (struct dbCommon *)pscal;

#ifdef NOT_YET_OSI
	pupdateCallback->wd_id = wdCreate();
#else
 	pupdateCallback->wd_id = epicsTimerQueueCreateTimer( scalerWdTimerQ, (void(*)())callbackRequest,pupdateCallback );
#endif

	/* second callback to implement delay */
	puserCallback = (struct callback *)&(pcallbacks[1]);
	callbackSetCallback(userCallbackFunc,&puserCallback->callback);
	callbackSetPriority(pscal->prio,&puserCallback->callback);
	puserCallback->precord = (struct dbCommon *)pscal;
#ifdef NOT_YET_OSI
	puserCallback->wd_id = wdCreate();
#else
 	puserCallback->wd_id = epicsTimerQueueCreateTimer( scalerWdTimerQ, (void(*)())callbackRequest,puserCallback );
#endif
	/* third callback to implement auto-count */
	pautoCallback = (struct callback *)&(pcallbacks[2]);
	callbackSetCallback(autoCallbackFunc,&pautoCallback->callback);
	callbackSetPriority(pscal->prio,&pautoCallback->callback);
	pautoCallback->precord = (struct dbCommon *)pscal;
#ifdef NOT_YET_OSI
	pautoCallback->wd_id = wdCreate();
#else
 	pautoCallback->wd_id = epicsTimerQueueCreateTimer( scalerWdTimerQ, (void(*)())callbackRequest,pautoCallback );
#endif

	/* fourth callback for device support */
	pdeviceCallback = (struct callback *)&(pcallbacks[3]);
	callbackSetCallback(deviceCallbackFunc,&pdeviceCallback->callback);
	callbackSetPriority(pscal->prio,&pdeviceCallback->callback);
	callbackSetUser((struct dbCommon *)pscal,&pdeviceCallback->callback);
	pdeviceCallback->precord = (struct dbCommon *)pscal;

	/* Check that we have everything we need. */
	if (!(pdset = (SCALERDSET *)(pscal->dset)))
	{
		recGblRecordError(S_dev_noDSET,(void *)pscal, "scaler: init_record");
		return(S_dev_noDSET);
	}

	Debug(2, "init_record: calling dset->init_record\n", 0);
	if (pdset->init_record)
	{
		status=(*pdset->init_record)(pscal);
		Debug(3, "init_record: dset->init_record returns %d\n", status);
		if (status) {
			pscal->card = -1;
			return (status);
		}
		pscal->card = pscal->out.value.vmeio.card;
		db_post_events(pscal,&(pscal->card),DBE_VALUE);
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
		pscal->pr1 = (long) (pscal->tp * pscal->freq);
		db_post_events(pscal,&(pscal->pr1),DBE_VALUE);
		Debug(3, "init_record: .TP != 0, so .PR1 set to %ld\n", pscal->pr1);
	} else if (pscal->pr1 && pscal->freq) {
		/* convert clock ticks to time */
		pscal->tp = (double)(pscal->pr1 / pscal->freq);
		db_post_events(pscal,&(pscal->tp),DBE_VALUE);
		Debug(3, "init_record: .PR1/.FREQ != 0, so .TP set to %f\n", pscal->tp);
	}
	return(0);
}


static long process(pscal)
scalerRecord *pscal;
{
	int i, status, prev_scaler_state;
	int card = pscal->out.value.vmeio.card;
	int justFinishedUserCount=0, justStartedUserCount=0, putNotifyOperation=0;
	long *ppreset = &(pscal->pr1);
	short *pdir = &pscal->d1;
	short *pgate = &pscal->g1;
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	struct callback *pcallbacks = (struct callback *)pscal->dpvt;
	struct callback *pupdateCallback = (struct callback *)&(pcallbacks[0]);
	struct callback *puserCallback = (struct callback *)&(pcallbacks[1]);
	struct callback *pautoCallback = (struct callback *)&(pcallbacks[2]);

	Debug(5, "process: entry\n", 0);
	pscal->pact = TRUE;
	pscal->udf = FALSE;
	prev_scaler_state = pscal->ss;

	/* If we're being called as a result of a done-counting interrupt, */
	/* (*pdset->done)(card) will return TRUE */
	if ((*pdset->done)(card)) {
		pscal->ss = SCALER_STATE_IDLE;
		/* Auto-count cycle is not allowed to reset .CNT field. */
		if (pscal->us == USER_STATE_COUNTING) {
			pscal->cnt = 0;
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
				(*pdset->arm)(card, 0);
				pscal->ss = SCALER_STATE_IDLE;
			}

			if (pscal->us == USER_STATE_REQSTART) {
				/*** start counting ***/
				/* disarm, disable interrupt generation, reset disarm-on-cout, */
				/* clear mask register, clear direction register, clear counters */
				(*pdset->reset)(card);
				for (i=0; i<pscal->nch; i++) {
					pdir[i] = pgate[i];
					if (pgate[i]) {
						Debug(5, "process: writing preset: %d.\n", ppreset[i]);
						(*pdset->write_preset)(card, i, ppreset[i]);
					}
				}
				(*pdset->arm)(card, 1);
				pscal->ss = SCALER_STATE_COUNTING;
				pscal->us = USER_STATE_COUNTING;
				handled = 1;
				justStartedUserCount = 1;
			}
		} else if (!pscal->cnt) {
			/*** stop counting ***/
			(*pdset->arm)(card, 0);
			pscal->ss = SCALER_STATE_IDLE;
			pscal->us = USER_STATE_IDLE;
			justFinishedUserCount = 1;
			handled = 1;
		}
		if (handled) {
			pscal->pcnt = pscal->cnt;
			Debug(2, "process: posting done flag (%d)\n", pscal->cnt);
			db_post_events(pscal,&(pscal->cnt),DBE_VALUE);
		}
	}

	/* read and display scalers */
	updateCounts(pscal);

	if (justStartedUserCount || justFinishedUserCount) {
		/* fire .cout link to trigger anything that should coincide with scaler integration */
		status = dbPutLink(&pscal->cout, DBR_SHORT, &pscal->cnt, 1);
		if (!RTN_SUCCESS(status)) {
			Debug(5, "scaler:process: ERROR %d PUTTING TO COUT LINK.\n", status);
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
#ifdef NOT_YET_OSI
		int ticks; /* ticks to delay */

		/* If we just finished normal counting, display the result for at
		 * least five seconds before starting auto-count counting.  Otherwise,
		 * use the auto-count delay time.
		 */
		ticks = vxTicksPerSecond * pscal->dly1;
		if (justFinishedUserCount) ticks = MAX(ticks, scaler_wait_time*vxTicksPerSecond);
		if (putNotifyOperation) ticks = MAX(ticks, scaler_wait_time*vxTicksPerSecond);

		/* if (we-have-to-wait && we-haven't-already-waited) { */
		if (ticks > 0 && pscal->ss != SCALER_STATE_WAITING) {
			Debug(5, "process: scheduling autocount restart\n", 0);
			/* Schedule a callback, and make a note that counting should start
			 * the next time we process (if pscal->ss is still 1 at that time).
			 */
			pscal->ss = SCALER_STATE_WAITING;  /* tell ourselves to start next time */
			status = wdStart(pautoCallback->wd_id, ticks, (FUNCPTR)callbackRequest,
			if (status != OK) printf("process: wdStart(autoCallback) returns ERROR\n");
					(int)pautoCallback);
		}
#else
	        double dly_sec=0.0;  /* seconds to delay */

		if (justFinishedUserCount) dly_sec = MAX(pscal->dly1, scaler_wait_time);
		if (putNotifyOperation) dly_sec = MAX(pscal->dly1, scaler_wait_time);
		/* if (we-have-to-wait && we-haven't-already-waited) { */
		if ( dly_sec > 0 && pscal->ss != SCALER_STATE_WAITING) {
			Debug(5, "process: scheduling autocount restart\n", 0);
			/* Schedule a callback, and make a note that counting should start
			 * the next time we process (if pscal->ss is still 1 at that time).
			 */
			pscal->ss = SCALER_STATE_WAITING;  /* tell ourselves to start next time */
			epicsTimerStartDelay(pautoCallback->wd_id, dly_sec);
		}
#endif
	       else {
			Debug(5, "process: restarting autocount\n", 0);
			/* Either the delay time is zero, or pscal->ss = SCALER_STATE_WAITING
			 * (we've already waited), so start auto-count counting.
			 * Different rules apply for auto-count counting:
			 * If (.TP1 >= .001 s) we count .TP1 seconds, regardless of any
			 * presets the user may have set.
			 */
			(*pdset->reset)(card);
			if (pscal->tp1 >= 1.e-3) {
				(*pdset->write_preset)(card, 0, (long)(pscal->tp1*pscal->freq));
			} else {
				for (i=0; i<pscal->nch; i++) {
					pdir[i] = pgate[i];
					if (pgate[i]) (*pdset->write_preset)(card, i, ppreset[i]);
				}
			}
			(*pdset->arm)(card, 1);
			pscal->ss = SCALER_STATE_COUNTING;

			/* schedule first update callback */
			callbackSetPriority(pscal->prio,&pupdateCallback->callback);
			if (pscal->rat1 > .1) {
#ifdef NOT_YET_OSI
				i = MAX(1, vxTicksPerSecond/pscal->rat1); /* ticks between updates */
				status = wdStart(pupdateCallback->wd_id,i,(FUNCPTR)callbackRequest,(int)pupdateCallback);
				if (status != OK) printf("process: wdStart(updateCallback) returns ERROR\n");
#else
				double dly_sec;

				dly_sec = 1.0/pscal->rat1; /* delay in seconds between updates */
			        epicsTimerStartDelay(pupdateCallback->wd_id, dly_sec);
#endif
			}
		}
	}

	pscal->pact = FALSE;
	return(0);
}


static void updateCounts(scalerRecord *pscal)
{
	int i, called_by_process;
	float rate;
	int card = pscal->out.value.vmeio.card;
	long *pscaler = &(pscal->s1);
	long counts[MAX_SCALER_CHANNELS];
	SCALERDSET *pdset = (SCALERDSET *)(pscal->dset);
	struct callback *pupdateCallback = (struct callback *)pscal->dpvt;

	called_by_process = (pscal->pact == TRUE);
	Debug(5, "updateCounts: %s called by process()\n", called_by_process ? " " : "NOT");
	if (!called_by_process) {
		if (pscal->ss != SCALER_STATE_IDLE)
			pscal->pact = TRUE;
		else
			return;
	}

	/* read scalers (get pointer to actual VME-resident scaler-data array) */
	if (pscal->us != USER_STATE_WAITING) {
		(*pdset->read)(card, counts);
	} else {
		for (i=0; i<pscal->nch; i++) {counts[i] = 0;}
	}

	Debug(5, "updateCounts: posting scaler values\n", 0);
	/* post scaler values */
	for (i=0; i<pscal->nch; i++) {
		if (counts[i] != pscaler[i]) {
			pscaler[i] = counts[i];
			db_post_events(pscal,&(pscaler[i]),DBE_VALUE);
			if (i==0) {
				/* convert clock ticks to time */
				pscal->t = pscaler[i] / pscal->freq;
				db_post_events(pscal,&(pscal->t),DBE_VALUE);
			}
		}
	}

	if (pscal->ss == SCALER_STATE_COUNTING) {
		/* arrange to call this routine again after user-specified time */
		Debug(5, "updateCounts: arranging for callback\n", 0);
		callbackSetPriority(pscal->prio,&pupdateCallback->callback);
		rate = ((pscal->us == USER_STATE_COUNTING) ? pscal->rate : pscal->rat1);
		if (rate > .1) {
#ifdef NOT_YET_OSI
			i = MAX(1, vxTicksPerSecond/rate); /* ticks between updates */
			wdStart(pupdateCallback->wd_id,i,(FUNCPTR)callbackRequest,(int)pupdateCallback);
#else
			{
			  double dly_sec;

			 dly_sec = 1.0/rate; /* delay in seconds between updates */
			epicsTimerStartDelay(pupdateCallback->wd_id, dly_sec);
			}
#endif
		}
	}

	if (!called_by_process) pscal->pact = FALSE;
}


static long special(paddr,after)
struct dbAddr *paddr;
int	after;
{
	scalerRecord *pscal = (scalerRecord *)(paddr->precord);
	int special_type = paddr->special;
	int i=0;
	unsigned short *pdir, *pgate;
	long *ppreset;
	struct callback *pcallbacks = (struct callback *)pscal->dpvt;
	struct callback *puserCallback = (struct callback *)&(pcallbacks[1]);
    int fieldIndex = dbGetFieldIndex(paddr);

	Debug(5, "special: entry; after=%d\n", after);
	if (!after) return (0);

	switch (fieldIndex) {
	case scalerRecordCNT:
		/* Ignore redundant (pscal->cnt == 1) commands */
		if (pscal->cnt && (pscal->us != USER_STATE_IDLE)) return(0);

		/* Scan record if it's not Passive.  (If it's Passive, it'll get */
		/* scanned automatically, since .cnt is a Process-Passive field.) */
		/* Arrange to process after user-specified delay time */
		callbackSetPriority(pscal->prio,&puserCallback->callback);
#ifdef NOT_YET_OSI
		i = vxTicksPerSecond * pscal->dly; /* ticks to delay */
#else
                i = pscal->dly;   /* seconds to delay */
#endif
		if (i<0) i = 0;
		if (i == 0 || pscal->cnt == 0) {
			/*** handle request now ***/
			if (pscal->cnt) {
				/* start counting */
				pscal->us = USER_STATE_REQSTART;
			} else {
				/* abort any counting or request to start counting */
				switch (pscal->us) {
				case USER_STATE_WAITING:
					/* We may have a watchdog timer going.  Cancel it. */
#ifdef NOT_YET_OSI
					(void)wdCancel(puserCallback->wd_id);
#else
					epicsTimerCancel(puserCallback->wd_id);
#endif
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

#ifdef NOT_YET_OSI
			wdStart(puserCallback->wd_id,i,(FUNCPTR)callbackRequest,
				(int)puserCallback);
#else
			epicsTimerStartDelay(puserCallback->wd_id, pscal->dly);
#endif
		}
		break;

	case scalerRecordCONT:
		/* Scan record if it's not Passive.  (If it's Passive, it'll get */
		/* scanned automatically, since .cont is a Process-Passive field.) */
		if (pscal->scan) scanOnce((void *)pscal);
		break;

	case scalerRecordTP:
		/* convert time to clock ticks */
		pscal->pr1 = (long) (pscal->tp * pscal->freq);
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
				(fieldIndex <= scalerRecordPR32)) {
			i = (paddr->pfield - (void *)&(pscal->pr1)) / sizeof(long);
			Debug(4, "special: channel %d preset\n", i);
			pdir = (unsigned short *) &(pscal->d1);
			pgate = (unsigned short *) &(pscal->g1);
			ppreset = (long *) &(pscal->pr1);
			if (ppreset[i] > 0) {
				pdir[i] = pgate[i] = 1;
				db_post_events(pscal,&(pdir[i]),DBE_VALUE);
				db_post_events(pscal,&(pgate[i]),DBE_VALUE);
			}
		}
		else if ((fieldIndex >= scalerRecordG1) &&
				(fieldIndex <= scalerRecordG32)) {
			/* If user set gate field, make sure preset counter has some */
			/* reasonable value. */
			i = (int)((paddr->pfield - (void *)&(pscal->g1)) / sizeof(short));
			Debug(4, "special: channel %d gate\n", i);
			ppreset = (long *) &(pscal->pr1);
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

static long get_precision(paddr, precision)
struct dbAddr *paddr;
long          *precision;
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


static void do_alarm(pscal)
scalerRecord *pscal;
{
	if(pscal->udf == TRUE ){
		recGblSetSevr(pscal,UDF_ALARM,INVALID_ALARM);
		return;
	}
	return;
}

static void monitor(pscal)
scalerRecord *pscal;
{
	unsigned short monitor_mask;

	monitor_mask = recGblResetAlarms(pscal);

	monitor_mask|=(DBE_VALUE|DBE_LOG);

	/* check all value fields for changes */
	return;
}
