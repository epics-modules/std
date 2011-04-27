/*
 *
 *      Author:	Tim Mooney
 *      Date:	05-17-99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1999, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Beamline Controls & Data Acquisition Group,
 *                 Experimental Facilities Division; and
 *              The Controls and Computing Group,
 *                 Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 *  .01 05-17-99  tmm  Created from seq record (by John Winans)
 */
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>

#include	<dbDefs.h>
#include	<epicsPrint.h>
#include	<alarm.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<dbScan.h>
#include	<dbFldTypes.h>
#include	<devSup.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<recGbl.h>
#include	<special.h>
#include	<callback.h>
#include	<cvtFast.h>
#include	<dbCa.h>
#include	<epicsThread.h>
#include	<epicsTimer.h>

#define GEN_SIZE_OFFSET
#include	"sseqRecord.h"
#undef  GEN_SIZE_OFFSET
#include        <epicsExport.h>

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

volatile int sseqRecDebug = 0;
epicsExportAddress(int, sseqRecDebug);

/* Total number of link-groups in a sequence record */
#define NUM_LINKS	10
#define SELN_BIT_MASK	~(0xffff << NUM_LINKS)

#define DBF_unknown -1
/* This is what a link-group looks like in a string-sequence record.  Note
 * that this must match the .dbd file.
 */
struct	linkGroup {
	epicsFloat64    dly;	/* Delay value (in seconds) */
	DBLINK          dol;	/* Where to fetch the input value from */
	epicsFloat64    dov;	/* If dol is CONSTANT, this is the CONSTANT value */
	DBLINK          lnk;	/* Where to put the value from dol */
	char            s[40]; /* string value */
	epicsInt16		dol_field_type;
	epicsInt16		lnk_field_type;
	epicsEnum16		usePutCallback;
	epicsInt16		waiting;
	epicsInt16		index;
	epicsInt16		dol_status;
	epicsInt16		lnk_status;
};

/* Per-record-instance structure used to hold callback structures and callback-related
 * information.
 */
#define LINKS_ALL_OK	0
#define LINKS_NOT_OK	1
struct callbackSeq {
	/* the following are for processing link groups */
	CALLBACK			callback;	/* used for the callback task */
	struct linkGroup	*plinkGroups[NUM_LINKS+1]; /* Pointers to links to process */
	int					index;
	/* the following are for maintaining links */
	CALLBACK			checkLinksCB;
	short				pending_checkLinksCB;
	short				linkStat; /* LINKS_ALL_OK, LINKS_NOT_OK */
};

static long init_record(sseqRecord *pR, int pass);
static long process(sseqRecord *pR);
static int processNextLink(sseqRecord *pR);
static long asyncFinish(sseqRecord *pR);
static void processCallback(CALLBACK *pCallback);
static long get_precision(struct dbAddr *paddr, long *precision);
static void checkLinksCallback(CALLBACK *pCallback);
static void checkLinks(sseqRecord *pR);
static long special(struct dbAddr *paddr, int after);

/* Create RSET - Record Support Entry Table*/
struct rset sseqRSET={
	RSETNUMBER,
	NULL,			/* report */
	NULL,			/* initialize */
	init_record,	/* init_record */
	process,		/* process */
	special,		/* special */
	NULL,			/* get_value */
	NULL,			/* cvt_dbaddr */
	NULL,			/* get_array_info */
	NULL,			/* put_array_info */
	NULL,			/* get_units */
	get_precision,	/* get_precision */
	NULL,			/* get_enum_str */
	NULL,			/* get_enum_strs */
	NULL,			/* put_enum_str */
	NULL,			/* get_graphic_double */
	NULL,			/* get_control_double */
	NULL 			/* get_alarm_double */

};
epicsExportAddress(rset, sseqRSET);
/*****************************************************************************
 *
 * Initialize a sequence record.
 *
 * Allocate the callback request structure (tacked on to dpvt.)
 * Initialize watch-dog ID
 * Initialize SELN based on the link-type of SELL
 * If SELL is a CA_LINK, inform CA about it
 * For each constant input link, fill in the DOV field
 *
 ******************************************************************************/
static long 
init_record(sseqRecord *pR, int pass)
{
	int					index;
	struct linkGroup	*plinkGroup;
	struct callbackSeq	*pcb;
	struct dbAddr       dbAddr;
	struct dbAddr       *pAddr = &dbAddr;

	if (pass==0) return(0);

	if (sseqRecDebug > 5) {
		printf("sseq:init_record(%s) entered\n", pR->name);
	}

	/* Allocate a callback structure for use in processing */
	pR->dpvt = (void *)calloc(1,sizeof(struct  callbackSeq));
	pcb = (struct callbackSeq *)pR->dpvt;

	callbackSetCallback(processCallback, &pcb->callback);
	callbackSetPriority(pR->prio, &pcb->callback);
	callbackSetUser(pR, &pcb->callback);

	callbackSetCallback(checkLinksCallback, &pcb->checkLinksCB);
	callbackSetPriority(pR->prio, &pcb->checkLinksCB);
	callbackSetUser(pR, &pcb->checkLinksCB);
	pcb->pending_checkLinksCB = 0;

	/* Get link selection if sell is a constant and nonzero */
	if (pR->sell.type==CONSTANT) {
		if (sseqRecDebug > 5) {
			printf("sseq:init_record(%s) SELL is a constant\n", pR->name);
		}
		recGblInitConstantLink(&pR->sell,DBF_USHORT,&pR->seln);
	}

	/*** init links, get initial values, field types ***/
	plinkGroup = (struct linkGroup *)(&(pR->dly1));
	for (index = 0; index < NUM_LINKS; index++, plinkGroup++) {
		/* set delays to nearest multiple of clock period */
		plinkGroup->dly = epicsThreadSleepQuantum() *
			NINT(plinkGroup->dly/epicsThreadSleepQuantum());
			db_post_events(pR, &plinkGroup->dly, DBE_VALUE);

		/* init DOL*-related stuff (input links) */
		if (plinkGroup->dol.type == CONSTANT) {
			recGblInitConstantLink(&plinkGroup->dol, DBF_DOUBLE, &plinkGroup->dov);
			recGblInitConstantLink(&plinkGroup->dol, DBF_STRING, plinkGroup->s);
			plinkGroup->dol_field_type = DBF_NOACCESS;
			plinkGroup->dol_status = sseqLNKV_CON;
        } else if (!dbNameToAddr(plinkGroup->dol.value.pv_link.pvname, pAddr)) {
			plinkGroup->dol_field_type = pAddr->field_type;
			plinkGroup->dol_status = sseqLNKV_LOC;
			if (sseqRecDebug > 5) printf("sseq:init:dol_field_type=%d (%s)\n",
				plinkGroup->dol_field_type, plinkGroup->dol_field_type>=0 ?
					pamapdbfType[plinkGroup->dol_field_type].strvalue : "");
		} else {
			/* pv is not on this ioc. Callback later for connection stat */
			plinkGroup->dol_status = sseqLNKV_EXT_NC;
			pcb->linkStat = LINKS_NOT_OK;
			plinkGroup->dol_field_type = DBF_unknown; /* don't know field type */
		}
		db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);


		/* same for LNK* stuff (output links) */
		if (plinkGroup->lnk.type == CONSTANT) {
			plinkGroup->lnk_field_type = DBF_unknown;
			plinkGroup->lnk_status = sseqLNKV_CON;
        } else if (!dbNameToAddr(plinkGroup->lnk.value.pv_link.pvname, pAddr)) {
			plinkGroup->lnk_field_type = pAddr->field_type;
			plinkGroup->lnk_status = sseqLNKV_LOC;
			if (sseqRecDebug > 5) printf("sseq:init:lnk_field_type=%d (%s)\n",
				plinkGroup->lnk_field_type, plinkGroup->lnk_field_type>=0 ?
					pamapdbfType[plinkGroup->lnk_field_type].strvalue : "");
		} else {
			/* pv is not on this ioc. Callback later for connection stat */
			plinkGroup->lnk_status = sseqLNKV_EXT_NC;
			pcb->linkStat = LINKS_NOT_OK;
			plinkGroup->lnk_field_type = DBF_unknown; /* don't know field type */
		}
		db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);

		/* convert between value types */
		if (plinkGroup->s[0]) {
			plinkGroup->dov = atof(plinkGroup->s);
			db_post_events(pR, &plinkGroup->dov, DBE_VALUE);
		} else {
			cvtDoubleToString(plinkGroup->dov, plinkGroup->s, pR->prec);
			db_post_events(pR, &plinkGroup->s, DBE_VALUE);
		}
	}

	if (pcb->linkStat == LINKS_NOT_OK) {
		callbackRequestDelayed(&pcb->checkLinksCB, 1.0);
		pcb->pending_checkLinksCB = 1;
	}

	return(0);
}
/*****************************************************************************
 *
 * Process a sequence record.
 *
 * If is async completion phase
 *   call asyncFinish() to finish things up
 * else
 *   figure out selection mechanism
 *   build the correct mask value using the mode and the selection value
 *   build a list of pointers to the selected link-group structures
 *   If there are no links to process
 *     call asyncFinish() to finish things up
 *   else
 *     call processNextLink() to schecule a delay for the first link-group
 *
 *
 * NOTE:
 *   dbScanLock is already held for pR before this function is called.
 *
 *   We do NOT call processNextLink() if there is nothing to do, this prevents
 *   us from calling dbProcess() recursively.
 *
 ******************************************************************************/
static long 
process(sseqRecord *pR)
{
	struct callbackSeq	*pcb = (struct callbackSeq *) (pR->dpvt);
	struct linkGroup	*plinkGroup;
	unsigned short		lmask;
	int					i;
	int					index;

	if (sseqRecDebug) {
		printf("sseq: process(%s) pact = %d\n", pR->name, pR->pact);
	}

	if (pR->pact) {
		/* In async completion phase */
		asyncFinish(pR);
		return(0);
	}
	pR->pact = TRUE;
	if ((pR->abort == 0) && (pR->busy == 0)) {
		pR->busy = 1;
		db_post_events(pR, &pR->busy, DBE_VALUE);
	}

	/* Reset the PRIO in case it was changed */
	pcb->callback.priority = pR->prio;

	/* Build the selection mask */
	if (pR->selm == sseqSELM_All) {
		lmask = (unsigned short) SELN_BIT_MASK;
	} else { 
		/* Fill in the SELN field */
		if (pR->sell.type != CONSTANT) {
			dbGetLink(&(pR->sell), DBR_USHORT, &(pR->seln), 0,0);
		}
		if (pR->selm == sseqSELM_Specified) {
			if (pR->seln>10) {
				/* Invalid selection number */
				recGblSetSevr(pR,SOFT_ALARM,INVALID_ALARM);
				return(asyncFinish(pR));
			}
			if (pR->seln == 0) {
				return(asyncFinish(pR));	/* Nothing selected */
			}
			lmask = 1;
			lmask <<= pR->seln - 1;
		} else if (pR->selm == sseqSELM_Mask) {
			lmask = (pR->seln) & SELN_BIT_MASK;
		} else {
			/* Invalid selection option */
			recGblSetSevr(pR,SOFT_ALARM,INVALID_ALARM);
			return(asyncFinish(pR));
		}
	}

	/* Clear all 'waiting' fields */
	plinkGroup = (struct linkGroup *)(&(pR->dly1));
	for (i=0; i<10; i++, plinkGroup++) {
		plinkGroup->waiting = 0;
		db_post_events(pR, &plinkGroup->waiting, DBE_VALUE);
	}

	/* Figure out which links are going to be processed */
	index = 0;
	plinkGroup = (struct linkGroup *)(&(pR->dly1));
	for (i = 1; lmask; lmask >>= 1, plinkGroup++, i++) {
		if (sseqRecDebug > 10) {
			printf("sseq:process: link %d - lnk.type=%d dol.type=%d\n",
				i, plinkGroup->lnk.type, plinkGroup->dol.type);
		}

		if ((lmask & 1) && ((plinkGroup->lnk.type != CONSTANT) ||
				(plinkGroup->dol.type != CONSTANT))) {
			if (sseqRecDebug > 10) {
				printf("  sseq:process: Adding link %d at index %d\n",
					i, index);
			}
			pcb->plinkGroups[index] = plinkGroup;
			index++;
		}
	}
	pcb->plinkGroups[index] = NULL;	/* mark the bottom of the list */

	if (!index) {
		/* There was nothing to do, finish record processing here */
		return(asyncFinish(pR));
	}

	pcb->index = 0;
	/* Start doing the first forward link (We have at least one for sure) */
	processNextLink(pR);

	return(0);
}
/*****************************************************************************
 *
 * Find the next link-group that needs processing.
 *
 * If there are no link groups left to process
 *   call dbProcess() to complete the async record processing.
 * else
 *   if the delay is > 0 seconds
 *     schedule the watch dog task to wake us up later
 *   else
 *     invoke the watch-dog wakeup routine now
 *
 *
 * NOTE:
 *   dbScanLock is already held for pR before this function is called.
 *
 ******************************************************************************/
static int processNextLink(sseqRecord *pR)
{
	struct callbackSeq	*pcb = (struct callbackSeq *) (pR->dpvt);
	struct linkGroup	*plinkGroupCurrent = pcb->plinkGroups[pcb->index];
	struct linkGroup	*plinkGroup = NULL;
	int ix;

	if (sseqRecDebug >= 2) {
		printf("sseq:processNextLink(%s) looking for work to do, index = %d, abort=%d\n",
			pR->name, pcb->index, pR->abort);
	}

	if (plinkGroupCurrent == NULL) {
		/* Nothing left to do.  If any links are still waiting, return and let the callback wake us up. */
		for (ix=0; ix<pcb->index; ix++) {
			plinkGroup = pcb->plinkGroups[ix];
			if (plinkGroup->waiting) {
				return(0);
			}
		}
		/* no outstanding callbacks.  finish up */
		(*(struct rset *)(pR->rset)).process(pR);
		return(0);
	}

	/* See if all completions needed before firing this action have occurred. */	
	for (ix=0; ix<pcb->index; ix++) {
		plinkGroup = pcb->plinkGroups[ix];
		if (plinkGroup->waiting) {
			if (plinkGroup->usePutCallback == sseqWAIT_Wait) {
				if (sseqRecDebug >= 2)
					printf("sseq:processNextLink: waiting for link index %d (waitIx='next')\n",
						plinkGroup->index);
				return(0);
			}
			if ((plinkGroup->usePutCallback-2) < plinkGroupCurrent->index) {
				if (sseqRecDebug >= 2)
					printf("sseq:processNextLink: waiting for link index %d (waitIx=%d)\n",
						plinkGroup->index, plinkGroup->usePutCallback-2);
				return(0);
			}
		}
	}

	if ((plinkGroupCurrent->dly > 0.0) && !pR->abort) {
		/* Request callback after a delay */
		callbackRequestDelayed(&pcb->callback, plinkGroupCurrent->dly);
	} else {
		/* No delay, do it now.  Avoid recursion;  use callback task */
		callbackRequest(&pcb->callback);
	}
	return(0);
}
/*****************************************************************************
 *
 * Finish record processing by posting any events and processing forward links.
 *
 * NOTE:
 *   dbScanLock is already held for pR before this function is called.
 *
 ******************************************************************************/
static long
asyncFinish(sseqRecord *pR)
{
	unsigned short MonitorMask;

	if (sseqRecDebug > 5) {
		printf("sseq:asyncFinish(%s) completing processing\n", pR->name);
	}
	pR->udf = FALSE;
 

	MonitorMask = DBE_VALUE | recGblResetAlarms(pR);

	if (MonitorMask) {
		db_post_events(pR, &pR->val, MonitorMask);
	}

	if (pR->abort) {
		if (sseqRecDebug > 5) printf("sseq:asyncFinish(%s) abort completed.\n", pR->name);
		pR->abort = 0;
		pR->aborting = 0;
		db_post_events(pR, &pR->abort, MonitorMask);
		db_post_events(pR, &pR->aborting, MonitorMask);
		if (pR->rpro) {
			/* EPICS wants to process the record again as soon as it's done, because a PP link
			 * found the record already processing as the result of a caput.  Because abort is
			 * true, we think that PP link was trying to abort an executing sequence, not start
			 * a new one, so we clear rpro.
			 */ 
			if (sseqRecDebug >= 2) printf("sseq:asyncFinish(%s) rpro changed to 0.\n", pR->name);
			pR->rpro = 0;
		}
	}

	/*
	 * Process the forward link.  Note that we have to do this, even if
	 * we're aborting, because this signals EPICS (putNotify) that we are done.
	 */
	if (sseqRecDebug>=2) printf("sseq:asyncFinish: calling recGblFwdLink\n");
	recGblFwdLink(pR);

	recGblGetTimeStamp(pR);
	/* tsLocalTime(&pR->time); */
	pR->pact = FALSE;
	pR->busy = 0;
	db_post_events(pR, &pR->busy, MonitorMask);

	return(0);
}

/* This is the function that will be called when processing started by a dbCaPutLinkCallback completes */
void epicsShareAPI putCallbackCB(void *arg)
{
	struct linkGroup	*plinkGroupThis = (struct linkGroup *)arg;
	struct link 		*plink = &(plinkGroupThis->lnk);
	sseqRecord			*pR;
	struct linkGroup	*plinkGroup;
	int 				ix, numWaiting, linkIsOK;
	dbAddr				Addr;
	dbAddr				*pAddr = &Addr;

	if (sseqRecDebug>=2) printf("sseq:putCallbackCB: entry\n");

	/* Check that link is valid */
	linkIsOK = 0;
	if (!dbNameToAddr(plink->value.pv_link.pvname, pAddr)) {
		linkIsOK = 1;
	} else if ((plink->type == CA_LINK) && dbCaIsLinkConnected(plink)) {
		linkIsOK = 1;
	}
	if (!linkIsOK) {
		printf("sseq:putCallbackCB: Bad link at index %d\n", plinkGroupThis->index);
		/* plinkGroupThis->waiting = 0; */
		return;
	}


	pR = (sseqRecord *)(plink->value.pv_link.precord);
	/* If sequence was aborted, waitinf fields may have been cleared. */
	if (plinkGroupThis->waiting == 0) {
		if (sseqRecDebug)
			printf("sseq(%s):putCallbackCB: ignoring abandoned callback from link %d (0..9)\n", 
				pR->name, plinkGroupThis->index);
		return;
	}

	dbScanLock((struct dbCommon *)pR);

	/* Clear the 'waiting' field for the linkGroup whose callback we received. */
	if (sseqRecDebug>=2) printf("sseq:putCallbackCB: Got callback for link %d (0..9)\n", plinkGroupThis->index);
	plinkGroupThis->waiting = 0;
	db_post_events(pR, &plinkGroupThis->waiting, DBE_VALUE);

	if (pR->abort) {
		plinkGroup = (struct linkGroup *)(&(pR->dly1));
		for (ix=0, numWaiting=0; ix<10; ix++, plinkGroup++) {
			numWaiting += plinkGroup->waiting;
		}
		/* If all links are done, call process to finish up. */
		if (numWaiting == 0) {
			if (sseqRecDebug > 5) printf("sseq:putCallbackCB(%s) aborting\n", pR->name);
			(*(struct rset *)(pR->rset)).process(pR);
		}
		dbScanUnlock((struct dbCommon *)pR);
		return;
	}

	/* Find the 'next' link-seq that is ready for processing. */
	processNextLink(pR);

	dbScanUnlock((struct dbCommon *)pR);
	return;

}
/*****************************************************************************
 *
 * Link-group processing function.
 * This routine runs only as the result of a callbackRequest() or a
 * callbackRequestDelayed().  Because the sseq record currently does not process
 * a link group until all previous link groups are done, when this routine runs,
 * there are no outstanding delays or dbCaPutLinkCallbacks.  Thus, abort is simple.
 * 
 * if the input link is not a constant
 *   call dbGetLink() to get the link value
 * else
 *   get the value from the DOV field
 * call dbPutLink() to forward the value to destination location
 * call processNextLink() to schedule the processing of the next link-group
 *
 * NOTE:
 *   dbScanLock is NOT held for pR when this function is called!!
 *
 ******************************************************************************/
static void
processCallback(CALLBACK *pCallback)
{
	sseqRecord			*pR = (sseqRecord *)(pCallback->user);
	struct callbackSeq	*pcb = (struct callbackSeq *) (pR->dpvt);
	struct linkGroup	*plinkGroup =
		(struct linkGroup *)(pcb->plinkGroups[pcb->index]);
	int					status, did_putCallback=0;
    /*epicsInt32				n_elements=1; */
    long				n_elements=1;
	double				d;
	char				str[40];


	if (sseqRecDebug >= 5) printf("sseq:processCallback(%s) entry\n", pR->name);

	dbScanLock((struct dbCommon *)pR);

	if (pR->abort) {
		if (sseqRecDebug >= 5)
			printf("sseq:processCallback(%s) aborting at field index %d\n", pR->name, pcb->index);
		/* Finish up. */
		(*(struct rset *)(pR->rset)).process(pR);
		dbScanUnlock((struct dbCommon *)pR);
		return;
	}

	if (sseqRecDebug >= 5) {
		printf("sseq:processCallback(%s) processing field index %d\n",
			pR->name, pcb->index);
	}

	/* get the value */
	if (sseqRecDebug > 10) printf("sseq:processCallback:dol_field_type=%d (%s)\n",
			plinkGroup->dol_field_type, plinkGroup->dol_field_type>=0 ?
				pamapdbfType[plinkGroup->dol_field_type].strvalue : "");

	if (plinkGroup->dol_field_type == DBF_unknown)
		plinkGroup->dol_field_type = dbGetLinkDBFtype(&plinkGroup->dol);

	switch (plinkGroup->dol_field_type) {
	case DBF_STRING: case DBF_ENUM: case DBF_MENU:
	case DBF_DEVICE: case DBF_INLINK: case DBF_OUTLINK: case DBF_FWDLINK:
		strcpy(str, plinkGroup->s);
		status = dbGetLink(&(plinkGroup->dol), DBR_STRING, &(plinkGroup->s),0,0);
		/* post string if it changed */
		if (strcmp(str, plinkGroup->s)) {
			if (sseqRecDebug > 10) {
				printf("sseq:processCallback: link %d changed from '%s' to '%s'\n", pcb->index,
					str, plinkGroup->s);
			}
			db_post_events(pR, &plinkGroup->s, DBE_VALUE|DBE_LOG);
		}
		/* make dov agree with s, post dov if it changed */
		d = atof(plinkGroup->s);
		if (d != plinkGroup->dov) {
			plinkGroup->dov = d;
			db_post_events(pR, &plinkGroup->dov, DBE_VALUE);
		}
		break;
	case DBF_SHORT: case DBF_USHORT: case DBF_LONG:
	case DBF_ULONG: case DBF_FLOAT: case DBF_DOUBLE:
		d = plinkGroup->dov;
		status = dbGetLink(&(plinkGroup->dol), DBR_DOUBLE, &(plinkGroup->dov),0,0);
		/* post value if it changed */
		if (d != plinkGroup->dov) {
			if (sseqRecDebug > 10) {
				printf("sseq:processCallback: link %d changed from %f to %f\n", pcb->index, d,
					plinkGroup->dov);
			}
			db_post_events(pR, &plinkGroup->dov, DBE_VALUE|DBE_LOG);
		}
		/* make s agree with dov, post s if it changed */
		cvtDoubleToString(plinkGroup->dov, str, pR->prec);
		if (strcmp(str, plinkGroup->s)) {
			strcpy(plinkGroup->s, str);
			db_post_events(pR, &plinkGroup->s, DBE_VALUE);
		}
		break;
	case DBF_CHAR: case DBF_UCHAR:
		dbGetNelements(&plinkGroup->dol, &n_elements);
		if (n_elements>40) n_elements=40;
		strcpy(str, plinkGroup->s);
		status = dbGetLink(&(plinkGroup->dol), plinkGroup->dol_field_type, &(plinkGroup->s),0,&n_elements);
		/* post string if it changed */
		if (strcmp(str, plinkGroup->s)) {
			if (sseqRecDebug > 10) {
				printf("sseq:processCallback: link %d changed from '%s' to '%s'\n", pcb->index,
					str, plinkGroup->s);
			}
			db_post_events(pR, &plinkGroup->s, DBE_VALUE|DBE_LOG);
		}
		/* make dov agree with s, post dov if it changed */
		d = atof(plinkGroup->s);
		if (d != plinkGroup->dov) {
			plinkGroup->dov = d;
			db_post_events(pR, &plinkGroup->dov, DBE_VALUE);
		}
		break;

	default:
		break;
	}

	/* Dump the value to the destination field */
	if (plinkGroup->lnk_field_type == DBF_unknown)
		plinkGroup->lnk_field_type = dbGetLinkDBFtype(&plinkGroup->lnk);
	if (sseqRecDebug >= 5) {
		printf("sseq:processCallback: lnk_field_type = %d (%s)\n", plinkGroup->lnk_field_type,
			plinkGroup->lnk_field_type>=0 ?	pamapdbfType[plinkGroup->lnk_field_type].strvalue : "");
	}
	switch (plinkGroup->lnk_field_type) {
	case DBF_STRING: case DBF_ENUM: case DBF_MENU:
	case DBF_DEVICE: case DBF_INLINK: case DBF_OUTLINK: case DBF_FWDLINK:
		if (plinkGroup->usePutCallback && (plinkGroup->lnk.type == CA_LINK)) {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbCaPutLinkCallback\n");
			status = dbCaPutLinkCallback(&(plinkGroup->lnk), DBR_STRING,
				&(plinkGroup->s), 1, (dbCaCallback) putCallbackCB, (void *)plinkGroup);
			plinkGroup->waiting = 1;
			db_post_events(pR, &plinkGroup->waiting, DBE_VALUE);
			did_putCallback = 1;
		} else {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbPutLink\n");
			status = dbPutLink(&(plinkGroup->lnk), DBR_STRING, &(plinkGroup->s),1);
		}
		break;
	case DBF_SHORT: case DBF_USHORT: case DBF_LONG:
	case DBF_ULONG: case DBF_FLOAT: case DBF_DOUBLE:
		if (plinkGroup->usePutCallback && (plinkGroup->lnk.type == CA_LINK)) {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbCaPutLinkCallback\n");
			status = dbCaPutLinkCallback(&(plinkGroup->lnk), DBR_DOUBLE,
				&(plinkGroup->dov), 1, (dbCaCallback) putCallbackCB, (void *)plinkGroup);
			plinkGroup->waiting = 1;
			db_post_events(pR, &plinkGroup->waiting, DBE_VALUE);
			did_putCallback = 1;
		} else {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbPutLink\n");
			status = dbPutLink(&(plinkGroup->lnk), DBR_DOUBLE, &(plinkGroup->dov),1);
		}
		break;
	case DBF_CHAR: case DBF_UCHAR:
		dbGetNelements(&plinkGroup->lnk, &n_elements);
		if (n_elements>40) n_elements = 40;
		if (sseqRecDebug >= 5) printf("sseq:processCallback: n_elements=%ld\n", n_elements); 
		if (plinkGroup->usePutCallback && (plinkGroup->lnk.type == CA_LINK)) {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbCaPutLinkCallback for %s\n",
					plinkGroup->lnk_field_type==DBF_CHAR?"DBF_CHAR":"DBF_UCHAR");
			if (n_elements>1) {
				status = dbCaPutLinkCallback(&(plinkGroup->lnk), plinkGroup->lnk_field_type,
					&(plinkGroup->s), n_elements, (dbCaCallback) putCallbackCB, (void *)plinkGroup);
			} else {
				status = dbCaPutLinkCallback(&(plinkGroup->lnk), DBR_DOUBLE,
					&(plinkGroup->dov), 1, (dbCaCallback) putCallbackCB, (void *)plinkGroup);
			}
			plinkGroup->waiting = 1;
			db_post_events(pR, &plinkGroup->waiting, DBE_VALUE);
			did_putCallback = 1;
		} else {
			if (sseqRecDebug >= 5)
				printf("sseq:processCallback: calling dbPutLink\n");
			if (n_elements>1) {
				status = dbPutLink(&(plinkGroup->lnk), plinkGroup->lnk_field_type, &(plinkGroup->s),n_elements);
			} else {
				status = dbPutLink(&(plinkGroup->lnk), DBR_DOUBLE, &(plinkGroup->dov),1);
			}
		}
		break;
	default:
		break;
	}

	/* Find the 'next' link-seq that is ready for processing. */
	pcb->index++;
	processNextLink(pR);

	dbScanUnlock((struct dbCommon *)pR);
	return;
}
/*****************************************************************************
 *
 * Return the precision value from PREC
 *
 *****************************************************************************/
static long
get_precision(struct dbAddr *paddr, long *precision)
{
	sseqRecord	*pR = (struct sseqRecord *) paddr->precord;

	*precision = pR->prec;

	if (paddr->pfield < (void *)&pR->val) {
		return(0);						/* Field is NOT in dbCommon */
	}

	recGblGetPrec(paddr, precision);	/* Field is in dbCommon */
	return(0);
}


static void checkLinksCallback(CALLBACK *pCallback)
{
    sseqRecord			*pR;
	struct callbackSeq	*pcb;

    callbackGetUser(pR, pCallback);
    pcb = (struct callbackSeq	*)pR->dpvt;

	if (!interruptAccept) {
		if (sseqRecDebug >= 10) printf("sseq:checkLinksCB(%s), before interruptAccept\n",
			pR->name);
		/* Can't call dbScanLock yet.  Schedule another CALLBACK */
		pcb->pending_checkLinksCB = 1;  /* make sure */
		callbackRequestDelayed(&pcb->checkLinksCB, 0.5);
	} else {
	    dbScanLock((struct dbCommon *)pR);
	    pcb->pending_checkLinksCB = 0;
	    checkLinks(pR);
	    dbScanUnlock((struct dbCommon *)pR);
	}
}


static void checkLinks(sseqRecord *pR)
{
	struct linkGroup *plinkGroup = (struct linkGroup *)(&(pR->dly1));
	struct callbackSeq	*pcb = (struct callbackSeq *)pR->dpvt;
	int i;

	if (sseqRecDebug > 10) printf("sseq:checkLinks(%s)\n", pR->name);

	pcb->linkStat = LINKS_ALL_OK;
	for (i = 0; i < NUM_LINKS; i++, plinkGroup++) {
		if (sseqRecDebug > 10)
			printf("sseq:checkLinks(%s): checking link %d\n", pR->name, i);

		if (plinkGroup->dol.type == CA_LINK) {
			if (dbCaIsLinkConnected(&(plinkGroup->dol))) {
				if (plinkGroup->dol_status == sseqLNKV_EXT_NC) {
					plinkGroup->dol_status = sseqLNKV_EXT;
					db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);
				}
			} else {
				if (plinkGroup->dol_status == sseqLNKV_EXT) {
					plinkGroup->dol_status = sseqLNKV_EXT_NC;
					db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);
				}
			}
		}
		plinkGroup->dol_field_type = DBF_unknown;
		if (plinkGroup->dol.value.pv_link.pvname &&
		    plinkGroup->dol.value.pv_link.pvname[0]) {
			plinkGroup->dol_field_type = dbGetLinkDBFtype(&plinkGroup->dol);
			if (plinkGroup->dol_field_type < 0) pcb->linkStat = LINKS_NOT_OK;
			if (sseqRecDebug > 10) {
				printf("sseq:checkLinks:dol_field_type=%d (%s), linked to %s\n",
					plinkGroup->dol_field_type,
					plinkGroup->dol_field_type>=0 ?
						pamapdbfType[plinkGroup->dol_field_type].strvalue : "???",
					plinkGroup->dol.value.pv_link.pvname);
			}
		}

		if (plinkGroup->lnk.type == CA_LINK) {
			if (dbCaIsLinkConnected(&(plinkGroup->lnk))) {
				if (plinkGroup->lnk_status == sseqLNKV_EXT_NC) {
					plinkGroup->lnk_status = sseqLNKV_EXT;
					db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);
				}
			} else {
				if (plinkGroup->lnk_status == sseqLNKV_EXT) {
					plinkGroup->lnk_status = sseqLNKV_EXT_NC;
					db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);
				}
			}
		}
		plinkGroup->lnk_field_type = DBF_unknown;
		if (plinkGroup->lnk.value.pv_link.pvname &&
		    plinkGroup->lnk.value.pv_link.pvname[0]) {
			plinkGroup->lnk_field_type = dbGetLinkDBFtype(&plinkGroup->lnk);
			if (plinkGroup->lnk_field_type < 0) pcb->linkStat = LINKS_NOT_OK;
			if (plinkGroup->usePutCallback && (plinkGroup->lnk.type != CA_LINK))
				pcb->linkStat = LINKS_NOT_OK;
			if (sseqRecDebug > 10) {
				printf("sseq:checkLinks:lnk_field_type=%d (%s), linked to %s\n",
					plinkGroup->lnk_field_type,
					plinkGroup->lnk_field_type>=0 ?
						pamapdbfType[plinkGroup->lnk_field_type].strvalue : "???",
					plinkGroup->lnk.value.pv_link.pvname);
			}
		}
	}
	if (pcb->linkStat == LINKS_NOT_OK) {
		if (!pcb->pending_checkLinksCB) {
			/* Schedule another callback */
			if (sseqRecDebug > 10)
				printf("sseq:checkLinks(%s): scheduling another callback\n", pR->name);
			pcb->pending_checkLinksCB = 1;
			callbackRequestDelayed(&pcb->checkLinksCB, 0.5);
		} else {
			/* We need another callback, but one has already been scheduled */
			if (sseqRecDebug > 10)
				printf("sseq:checkLinks(%s): callback already pending\n", pR->name);
		}
	} else {
		if (sseqRecDebug > 10) printf("sseq:checkLinks(%s): links ok\n", pR->name);
	}
}


static long special(struct dbAddr *paddr, int after)
{
	sseqRecord			*pR = (sseqRecord *)(paddr->precord);
	struct callbackSeq	*pcb = (struct callbackSeq *)pR->dpvt;
	int                 fieldIndex = dbGetFieldIndex(paddr);
	int                 lnkIndex;
	struct linkGroup	*plinkGroup;
	char				str[40];
	double				d;
	dbAddr				Addr;
	dbAddr				*pAddr = &Addr;

	if (sseqRecDebug > 5) printf("sseq:special(%s)\n", pR->name);
	if (!after) return(0);
	switch (fieldIndex) {
	case(sseqRecordDOL1):
	case(sseqRecordDOL2):
	case(sseqRecordDOL3):
	case(sseqRecordDOL4):
	case(sseqRecordDOL5):
	case(sseqRecordDOL6):
	case(sseqRecordDOL7):
	case(sseqRecordDOL8):
	case(sseqRecordDOL9):
	case(sseqRecordDOLA):
		lnkIndex = ((char *)paddr->pfield - (char *)&pR->dly1) /
			sizeof(struct linkGroup);
		plinkGroup = (struct linkGroup *)&pR->dly1;
		plinkGroup += lnkIndex;

		/* Get link status */
		if (plinkGroup->dol.type == CONSTANT) {
			plinkGroup->dol_status = sseqLNKV_CON;
			db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);
		} else if (!dbNameToAddr(plinkGroup->dol.value.pv_link.pvname, pAddr)) {
			plinkGroup->dol_status = sseqLNKV_LOC;
			db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);
		} else {
			plinkGroup->dol_status = sseqLNKV_EXT_NC;
			db_post_events(pR, &plinkGroup->dol_status, DBE_VALUE);
		}

		plinkGroup->dol_field_type = DBF_unknown;
		if (plinkGroup->dol.value.pv_link.pvname && plinkGroup->dol.value.pv_link.pvname[0]) {
			plinkGroup->dol_field_type = dbGetLinkDBFtype(&plinkGroup->dol);
			if (plinkGroup->dol_field_type < 0) pcb->linkStat = LINKS_NOT_OK;
		}
		if (!pcb->pending_checkLinksCB && (pcb->linkStat == LINKS_NOT_OK)) {
			pcb->pending_checkLinksCB = 1;
			callbackRequestDelayed(&pcb->checkLinksCB, 0.5);
		}
		if (sseqRecDebug > 5) printf("sseq:special:dol_field_type=%d (%s)\n",
			plinkGroup->dol_field_type, plinkGroup->dol_field_type>=0 ?
				pamapdbfType[plinkGroup->dol_field_type].strvalue : "");
		return(0);

	case(sseqRecordLNK1):
	case(sseqRecordLNK2):
	case(sseqRecordLNK3):
	case(sseqRecordLNK4):
	case(sseqRecordLNK5):
	case(sseqRecordLNK6):
	case(sseqRecordLNK7):
	case(sseqRecordLNK8):
	case(sseqRecordLNK9):
	case(sseqRecordLNKA):
		lnkIndex = ((char *)paddr->pfield - (char *)&pR->dly1) /
			sizeof(struct linkGroup);
		plinkGroup = (struct linkGroup *)&pR->dly1;
		plinkGroup += lnkIndex;

		/* Get link status */
		if (plinkGroup->lnk.type == CONSTANT) {
			if (sseqRecDebug > 5) printf("sseq:special:lnk_status = %d\n", sseqLNKV_CON);
			plinkGroup->lnk_status = sseqLNKV_CON;
			db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);
		}
		else if (!dbNameToAddr(plinkGroup->lnk.value.pv_link.pvname, pAddr)) {
			if (sseqRecDebug > 5) printf("sseq:special:lnk_status = %d\n", sseqLNKV_LOC);
			plinkGroup->lnk_status = sseqLNKV_LOC;
			db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);
		}
		else {
			if (sseqRecDebug > 5) printf("sseq:special:lnk_status = %d\n", sseqLNKV_EXT_NC);
			plinkGroup->lnk_status = sseqLNKV_EXT_NC;
			db_post_events(pR, &plinkGroup->lnk_status, DBE_VALUE);
		}


		if (sseqRecDebug > 5) {
			printf("sseq:special:lnkIndex=%d\n", lnkIndex);
			printf("sseq:special: &lnk1=%p, &plinkGroup->lnk=%p\n",
				&pR->lnk1, &plinkGroup->lnk);
		}
		plinkGroup->lnk_field_type = DBF_unknown;

		if (plinkGroup->lnk.value.pv_link.pvname && plinkGroup->lnk.value.pv_link.pvname[0]) {
			plinkGroup->lnk_field_type = dbGetLinkDBFtype(&plinkGroup->lnk);
			if (plinkGroup->lnk_field_type < 0) pcb->linkStat = LINKS_NOT_OK;
		}
		if (!pcb->pending_checkLinksCB && (pcb->linkStat == LINKS_NOT_OK)) {
			pcb->pending_checkLinksCB = 1;
			callbackRequestDelayed(&pcb->checkLinksCB, 0.5);
		}
		if (sseqRecDebug > 5) printf("sseq:special:lnk_field_type=%d (%s)\n",
			plinkGroup->lnk_field_type, plinkGroup->lnk_field_type>=0 ?
				pamapdbfType[plinkGroup->lnk_field_type].strvalue : "");
		return(0);

	case(sseqRecordDO1):
	case(sseqRecordDO2):
	case(sseqRecordDO3):
	case(sseqRecordDO4):
	case(sseqRecordDO5):
	case(sseqRecordDO6):
	case(sseqRecordDO7):
	case(sseqRecordDO8):
	case(sseqRecordDO9):
	case(sseqRecordDOA):
		lnkIndex = ((char *)paddr->pfield - (char *)&pR->dly1) /
			sizeof(struct linkGroup);
		plinkGroup = (struct linkGroup *)&pR->dly1;
		plinkGroup += lnkIndex;
		cvtDoubleToString(plinkGroup->dov, str, pR->prec);
		if (strcmp(str, plinkGroup->s)) {
			strcpy(plinkGroup->s, str);
			db_post_events(pR, &plinkGroup->s, DBE_VALUE);
		}
		break;

	case(sseqRecordSTR1):
	case(sseqRecordSTR2):
	case(sseqRecordSTR3):
	case(sseqRecordSTR4):
	case(sseqRecordSTR5):
	case(sseqRecordSTR6):
	case(sseqRecordSTR7):
	case(sseqRecordSTR8):
	case(sseqRecordSTR9):
	case(sseqRecordSTRA):
		lnkIndex = ((char *)paddr->pfield - (char *)&pR->dly1) /
			sizeof(struct linkGroup);
		plinkGroup = (struct linkGroup *)&pR->dly1;
		plinkGroup += lnkIndex;
		d = atof(plinkGroup->s);
		if (d != plinkGroup->dov) {
			plinkGroup->dov = d;
			db_post_events(pR, &plinkGroup->dov, DBE_VALUE);
		}
		break;

	case(sseqRecordDLY1):
	case(sseqRecordDLY2):
	case(sseqRecordDLY3):
	case(sseqRecordDLY4):
	case(sseqRecordDLY5):
	case(sseqRecordDLY6):
	case(sseqRecordDLY7):
	case(sseqRecordDLY8):
	case(sseqRecordDLY9):
	case(sseqRecordDLYA):
		lnkIndex = ((char *)paddr->pfield - (char *)&pR->dly1) /
			sizeof(struct linkGroup);
		plinkGroup = (struct linkGroup *)&pR->dly1;
		plinkGroup->dly = epicsThreadSleepQuantum() *
			NINT(plinkGroup->dly/epicsThreadSleepQuantum());
			db_post_events(pR, &plinkGroup->dly, DBE_VALUE);
		break;

	case(sseqRecordABORT):
		/*
		 * If there is an outstanding delay timer, cancel it.
		 * If there is an outstanding dbCaPutLinkCallback, we'd like to cancel it, but that would require
		 * clearing and renewing the link, which we don't have code in place to do.  Instead, we wait for
		 * the callback.  If we get a second 'abort' command, while waiting, we clear all 'waiting' flags,
		 * which returns the record to the idle state while any callbacks are still outstanding.
		 * This might result in callbacks arriving while the record is idle (in which case we ignore them),
		 * or after a fresh sequence has been started (in which case we incorrectly treat them as the result
		 * of that fresh sequence's dbCaPutLinkCallback calls.  That's the best we can do without cancelling
		 * dbCaPutLinkCallbacks.
		 */
		if (sseqRecDebug>=2)
			printf("sseq:special: abort\n");

		if (!pR->busy) {
			pR->abort = 0;
			printf("sseq:special: no activity to abort\n");
			db_post_events(pR, &pR->busy, DBE_VALUE);
			return(-1);
		}
		if (pR->aborting) {
			/* We're already tryng to abort.  Maybe something's hung up. */
			/* Clear all 'waiting' fields. */
			plinkGroup = (struct linkGroup *)(&(pR->dly1));
			for (lnkIndex=0; lnkIndex<10; lnkIndex++, plinkGroup++) {
				plinkGroup->waiting = 0;
				db_post_events(pR, &plinkGroup->waiting, DBE_VALUE);
			}
			pcb->index = 0;
			callbackRequest(&pcb->callback);
			return(0);
		}
		pR->aborting = 1;
		db_post_events(pR, &pR->aborting, DBE_VALUE);

		plinkGroup = pcb->plinkGroups[pcb->index];
		if (plinkGroup && (plinkGroup->dly > 0.0)) {
			/* There is a current link group.  If it started a delay timer, cancel the timer. */
			CALLBACK *pcallback = &(pcb->callback);
			epicsTimerId timer = (epicsTimerId)(pcallback->timer);
			if (sseqRecDebug>=2) printf("sseq:special: timer=%p\n", timer);
			if (timer) {
				double expire = epicsTimerGetExpireDelay(timer);
				if ((sseqRecDebug>=2) && (expire > 0.) && (expire < DBL_MAX))
					printf("sseq:special: expire=%f\n", expire);
				if ((expire > 0.1) && (expire < DBL_MAX)) {
					if (sseqRecDebug>=2) printf("sseq:special: calling epicsTimerCancel\n");
					epicsTimerCancel(timer);
					/* We should not get the timer's callback, so we have to
					 * complete the abort from here.
					 */
					if (sseqRecDebug>=2)
						printf("sseq:special: calling callbackRequest() to abort\n");
					callbackRequest(&pcb->callback);
				}
			}
		}
		break;

	default:
		recGblDbaddrError(S_db_badChoice,paddr,"sseq: special");
		return(S_db_badChoice);
	}
	return(0);
}
