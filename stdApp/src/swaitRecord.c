/* swaitRecord.c */   
/*
 *      Original Author: Ned Arnold 
 *      Date:            05-31-94
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *	Copyright 1991, the Regents of the University of California,
 *	and the University of Chicago Board of Governors.
 *
 *	This software was produced under  U.S. Government contracts:
 *	(W-7405-ENG-36) at the Los Alamos National Laboratory,
 *	and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *	Initial development by:
 *		The Controls and Automation Group (AT-8)
 *		Ground Test Accelerator
 *		Accelerator Technology Division
 *		Los Alamos National Laboratory
 *
 *	Co-developed with
 *		The Controls and Computing Group
 *		Accelerator Systems Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 *
 * Modification Log:
 * -----------------
 * 1.01  05-31-94  nda  initial try            
 * 1.02  07-11-94  mrk/nda  added "process on input change" feature
 * 1.03  08-16-94  mrk/nda  continuing "process on input change" feature
 * 1.04  08-16-94  nda  record does not get notified when a SCAN related field
 *                      changes,so for now we have to always add Monitors
 * 1.05  08-18-94  nda  Starting with R3.11.6, dbGetField locks the record
 *                      before fetching the data. This can cause deadlocks
 *                      within a database. Change all dbGetField() to dbGet()
 * 1.06  08-19-94  nda  added Output data option of VAL or DOL
 * 1.07  09-14-94  nda  corrected bug that caused SCAN_DISABLE to lock up the
 *                      record forever
 * 1.08  02-01-95  nda  added VERS and ODLY (output execution delay)
 * 1.09  02-15-95  nda  added INxP to specify which inputs should cause the
 *                      record to process when in I/O INTR
 * 2.00  02-20-95  nda  added queuing to SCAN_IO_EVENT mode so no transitions
 *                      of data would be missed.
 * 2.01  08-07-95  nda  Multiple records with DOLN's didn't work, added calloc
 *                      for dola structure.
 * 3.00  08-28-95  nda  Significant rewrite to add Channel Access for dynamic
 *                      links using recDynLink.c . All inputs are now
 *                      "monitored" via Channel Access.
 *                       Removed some "callbacks" because recDynLink lib uses
 *                       it's own task context.
 *                       INxV field is used to keep track of PV connection
 *                       status: 0-PV_OK, 1-NotConnected, 2-NO_PV
 * 3.01  10-03-95  nda   Also post monitors on .la, .lb, .lc etc when new
 *                       values are written
 * 4.00  09-23-98  tmm   Use ca_put_callback() for output links, don't call
 *                       recGblFwdLink until we get called back
 * 4.01  11-04-98  tmm   Improve debugging statements
 * 4.02  01-26-99  tmm   If output link will not be fired, call recGblFwdLink.
 * 4.03  03-24-99  tmm   If recDynLinkPutCallback fails because a callback is
 *                       already in progress, do a recDynLinkPut.  Set pact and
 *                       outputWait to zero, and post output event, as soon as
 *                       we've called recDynLinkPut*.  Rename as swaitRecord.
 * 4.04  08-24-99  tmm   Fix special treatment of link fields for new dbd file
 * 4.05  10-04-01  tmm   In earlier versions, watchdog interrupt was running
 *                       code directly that should not run at interrupt level.
 *                       Changed to use callback task for delayed output.
 * 4.06  11-30-01  tmm   If PV name is all blank, reset to empty string.  Fix
 *                       ppvn pointer problem.
 */

#define VERSION 4.6



#ifdef vxWorks
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#endif
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <epicsRingBytes.h>

#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <dbEvent.h>
#include <dbScan.h>
#include <dbDefs.h>
#include <dbFldTypes.h>
#include <devSup.h>
#include <errMdef.h>
#include <recSup.h>
#include <recGbl.h>
#include <special.h>
#include <callback.h>
#include <taskwd.h>
#include <postfix.h>

#define GEN_SIZE_OFFSET
#include "swaitRecord.h"
#undef  GEN_SIZE_OFFSET
#include "recDynLink.h"


#define PRIVATE_FUNCTIONS 1	/* normal:1, debug:0 */
#if PRIVATE_FUNCTIONS
#define STATIC static
#else
#define STATIC
#endif

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
STATIC long init_record();
STATIC long process();
STATIC long special();
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL 
#define put_array_info NULL 
#define get_units NULL 
STATIC long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL 
#define put_enum_str NULL
STATIC long get_graphic_double();
#define get_control_double NULL 
STATIC long get_alarm_double(); 
 
struct rset swaitRSET={
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

/* Create DSET for "soft channel" to allow for IO Event (this is to implement
   the feature of processing the record when an input changes) */

static long get_ioint_info();
struct {
        long            number;
        DEVSUPFUN       dev_report;
        DEVSUPFUN       init_dev;
        DEVSUPFUN       dev_init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_event;
} devSWaitIoEvent = {
        5,
        NULL,
        NULL,
        NULL,
        get_ioint_info,
        NULL
};


/* DEFINES */
#define   ARG_MAX   12  /* Number of input arguments of the record */
#define   IN_PVS     1  /* Number of other input dynamic links(DOLN) */
#define   OUT_PVS    1  /* Number of "non-input" dynamic links(OUTN) */
#define   DOL_INDEX  ARG_MAX 
#define   OUT_INDEX  (ARG_MAX + IN_PVS)
#define   NUM_LINKS  (ARG_MAX + IN_PVS + OUT_PVS)
#define   PVN_SIZE  40  /*must match the length defined in swaitRecord.db*/
#define   Q_SIZE    50
#define   PV_OK     0
#define   PV_NC     1
#define   NO_PV     2

/**********************************************
  Declare constants and structures 
***********************************************/

/* callback structures and record private data */
struct cbStruct {
    CALLBACK           doOutCb;   /* cback struct for doing the OUT link*/
    CALLBACK           ioProcCb;  /* cback struct for io_event scanning */
    swaitRecord *pwait;     /* pointer to wait record */
    recDynLink         caLinkStruct[NUM_LINKS]; /* req'd for recDynLink*/
    epicsRingBytesId   monitorQ;  /* queue to store ca callback data */
    IOSCANPVT          ioscanpvt; /* used for IO_EVENT scanning */
    int                outputWait;/* waiting to do output */
    int                procPending;/*record processing is pending */
    unsigned long      tickStart; /* used for timing  */
};


typedef struct recDynLinkPvt {
        swaitRecord *pwait;     /* pointer to wait record */
        unsigned short     linkIndex; /* specifies which dynamic link */
} recDynLinkPvt;


STATIC long get_ioint_info(cmd,pwait,ppvt)
    int                     cmd;
    swaitRecord       *pwait;
    IOSCANPVT               *ppvt;
{
    *ppvt = (((struct cbStruct *)pwait->cbst)->ioscanpvt);
    return(0);
}


/* This is the data that will be put on the work queue ring buffer */
struct qStruct {
    char               inputIndex;
    double             monData;
};


int    swaitRecordDebug=0;
int    swaitRecordCacheMode=0;
STATIC void schedOutput(swaitRecord *pwait);
static void doOutputCallback(CALLBACK *pcallback);
STATIC void execOutput(swaitRecord *pwait);
STATIC int fetch_values(swaitRecord *pwait);
STATIC void monitor(swaitRecord *pwait);
STATIC long initSiml();
STATIC void ioIntProcess(CALLBACK *pioProcCb);

STATIC void pvSearchCallback(recDynLink *precDynLink);
STATIC void inputChanged(recDynLink *precDynLink);


static int isBlank(char *name)
{
	int i;

	for(i=0; name[i]; i++) {
		if (!(isspace(name[i]))) return(0);
	}
	return((i>0));
}

STATIC long init_record(pwait,pass)
    swaitRecord	*pwait;
    int pass;
{
    struct cbStruct *pcbst;
    long    status = 0;
    int i;

    char            *ppvn;
    unsigned short  *pPvStat;

    recDynLinkPvt   *puserPvt;

    short error_number;

    if (pass==0) {
      pwait->vers = VERSION;
 
      pwait->cbst = calloc(1,sizeof(struct cbStruct));

      /* init the private area of the caLinkStruct's   */
      for(i=0;i<NUM_LINKS; i++) {
        ((struct cbStruct *)pwait->cbst)->caLinkStruct[i].puserPvt
                 = calloc(1,sizeof(struct recDynLinkPvt));
        puserPvt = ((struct cbStruct *)pwait->cbst)->caLinkStruct[i].puserPvt;
        puserPvt->pwait = pwait;
        puserPvt->linkIndex = i;
      }

    /* do scanIoInit here because init_dev doesn't know which record */
    scanIoInit(&(((struct cbStruct *)pwait->cbst)->ioscanpvt));

    return(0);
    }

    /* This is pass == 1, so pwait->cbst is valid */

    pcbst = (struct cbStruct *)pwait->cbst;

    pwait->clcv=postfix(pwait->calc,pwait->rpcl,&error_number);
    if (pwait->clcv){
        recGblRecordError(S_db_badField,(void *)pwait,
                          "swait:init_record: Illegal CALC field");
    }
    db_post_events(pwait,&pwait->clcv,DBE_VALUE);

    callbackSetCallback(doOutputCallback, &pcbst->doOutCb);
    callbackSetPriority(pwait->prio, &pcbst->doOutCb);
    callbackSetUser(pwait, &pcbst->doOutCb);
    callbackSetCallback(ioIntProcess, &pcbst->ioProcCb);
    callbackSetPriority(pwait->prio, &pcbst->ioProcCb);
    callbackSetUser(pwait, &pcbst->ioProcCb);
    pcbst->pwait = pwait;
    if ((pcbst->monitorQ=epicsRingBytesCreate(sizeof(struct qStruct)*Q_SIZE)) == NULL) {
        errMessage(0,"recWait can't create ring buffer");
        exit(1);
    }

    if ((status=initSiml(pwait))) return(status);

    /* reset miscellaneous flags */
    pcbst->outputWait = 0;
    pcbst->procPending = 0;

    pwait->init = TRUE;

    /* Do initial lookup of PV Names using recDynLink lib */

    ppvn = &pwait->inan[0];
    pPvStat = &pwait->inav;

    /* check all dynLinks for non-NULL  */
    for(i=0;i<NUM_LINKS; i++, pPvStat++, ppvn += PVN_SIZE) {
		if (isBlank(ppvn)) {
			ppvn[0] = '\0';
			db_post_events(pwait, ppvn, DBE_VALUE);
			*pPvStat = NO_PV;
		} else if (ppvn[0] != 0) {
            *pPvStat = PV_NC;
            if (i<OUT_INDEX) {
                recDynLinkAddInput(&pcbst->caLinkStruct[i], ppvn, 
                 DBR_DOUBLE, rdlSCALAR, pvSearchCallback, inputChanged);
            }
            else {
                recDynLinkAddOutput(&pcbst->caLinkStruct[i], ppvn,
                DBR_DOUBLE, rdlSCALAR, pvSearchCallback);
            }
            if (swaitRecordDebug > 5) errlogPrintf("%s:Search during init\n", pwait->name);
        }
        else {
            *pPvStat = NO_PV;
        }
    } 
    pwait->init = TRUE;
    return(0);
}

LOCAL void 
notifyCallback(recDynLink * precDynLink)
{
	recDynLinkPvt  *puserPvt = (recDynLinkPvt *) precDynLink->puserPvt;
	swaitRecord *pwait = puserPvt->pwait;

	if (swaitRecordDebug >= 10) errlogPrintf("swaitRecord:notifyCallback: entry\n");
    recGblFwdLink(pwait);
}
STATIC long process(swaitRecord	*pwait)
{
	struct cbStruct *pcbst = (struct cbStruct *)pwait->cbst;
	short async    = FALSE;
	long  status;

	if (pwait->pact &&	pcbst->outputWait) {
		execOutput(pwait);
		return(0);
	}

	pwait->pact = TRUE;

	/* Check for simulation mode */
	status=dbGetLink(&(pwait->siml),DBR_ENUM,&(pwait->simm),0,0);

	/* reset procPending before getting values */
	((struct cbStruct *)pwait->cbst)->procPending = 0;

	if (pwait->simm == NO) {
		if (fetch_values(pwait)==0) {
			if (calcPerform(&pwait->a,&pwait->val,pwait->rpcl)) {
				recGblSetSevr(pwait,CALC_ALARM,INVALID_ALARM);
			} else pwait->udf = FALSE;
		} else {
			recGblSetSevr(pwait,READ_ALARM,INVALID_ALARM);
		}
	} else {      /* SIMULATION MODE */
		status = dbGetLink(&(pwait->siol),DBR_DOUBLE,&(pwait->sval),0,0);
		if (status==0) {
			pwait->val=pwait->sval;
			pwait->udf=FALSE;
		}
		recGblSetSevr(pwait,SIMM_ALARM,pwait->sims);
	}

	/* decide whether to write Output PV  */
	switch(pwait->oopt) {
		case swaitOOPT_Every_Time:
			schedOutput(pwait);
			async = TRUE;
			break;

		case swaitOOPT_On_Change:
			if (fabs(pwait->oval - pwait->val) > pwait->mdel)  {
				schedOutput(pwait);
				async = TRUE;
			}
			break;

		case swaitOOPT_Transition_To_Zero:
			if ((pwait->oval != 0) && (pwait->val == 0)) {
				schedOutput(pwait);
				async = TRUE;
			}
			break;

		case swaitOOPT_Transition_To_Non_zero:
			if ((pwait->oval == 0) && (pwait->val != 0)) {
				schedOutput(pwait);
				async = TRUE;
			}
			break;

		case swaitOOPT_When_Zero:
			if (!pwait->val) {
				schedOutput(pwait);
				async = TRUE;
			}
			break;

		case swaitOOPT_When_Non_zero:
			if (pwait->val) {
				schedOutput(pwait);
				async = TRUE;
			}
			break;

		case swaitOOPT_Never:
		default:
			break;
	}

	pwait->oval = pwait->val;

	recGblGetTimeStamp(pwait); 
	/* check event list */
	monitor(pwait);

	if (!async) {
		recGblFwdLink(pwait);
		pwait->pact = FALSE;
	}
	return(0);
}

STATIC long special(paddr,after)
    struct dbAddr *paddr;
    int	   	  after;
{
    swaitRecord  	*pwait = (swaitRecord *)(paddr->precord);
    struct cbStruct     *pcbst = (struct cbStruct *)pwait->cbst;
    int           	special_type = paddr->special;
    char                *ppvn;
    unsigned short      *pPvStat;
    unsigned short       oldStat;
    int  index;
    int				fieldIndex = dbGetFieldIndex(paddr);
    short error_number;

    if (swaitRecordDebug) errlogPrintf("entering special %d \n",after);

    if (!after) {  /* this is called before ca changes the field */
        
        /* check if changing any dynamic link names  */
        /* This is where one would do a recDynLinkClear, but it is
           not required prior to a new search */
     return(0);
     }

    /* this is executed after ca changed the field */
    if ((fieldIndex >= swaitRecordINAN) && 
       (fieldIndex < (swaitRecordINAN + NUM_LINKS))) {
        index = fieldIndex - swaitRecordINAN; /* array index of input */
        pPvStat = &pwait->inav + index; /* pointer arithmetic */
        oldStat = *pPvStat;
        ppvn = &pwait->inan[0] + (index*PVN_SIZE); 
		if (isBlank(ppvn)) {
			ppvn[0] = '\0';
			db_post_events(pwait, ppvn, DBE_VALUE);
		} 
        if (ppvn[0] != 0) {
            if (swaitRecordDebug > 5) errlogPrintf("Search during special \n");
            *pPvStat = PV_NC;
            /* need to post_event before recDynLinkAddXxx because
               SearchCallback could happen immediatley */
            if (*pPvStat != oldStat) {
                db_post_events(pwait,pPvStat,DBE_VALUE);
            }
            if (index<OUT_INDEX) {
                recDynLinkAddInput(&pcbst->caLinkStruct[index], ppvn, 
                DBR_DOUBLE, rdlSCALAR, pvSearchCallback, inputChanged);
            }
            else {
                recDynLinkAddOutput(&pcbst->caLinkStruct[index], ppvn,
                DBR_DOUBLE, rdlSCALAR, pvSearchCallback);
            }
        }
        else if (*pPvStat != NO_PV) {
            /* PV is now NULL but didn't used to be */
            *pPvStat = NO_PV;              /* PV just cleared */
            if (*pPvStat != oldStat) {
                db_post_events(pwait,pPvStat,DBE_VALUE);
            }
            recDynLinkClear(&pcbst->caLinkStruct[index]);
        }
        return(0);
    }
    else if (special_type == SPC_CALC) {
        pwait->clcv=postfix(pwait->calc,pwait->rpcl,&error_number);
        if (pwait->clcv){
                recGblRecordError(S_db_badField,(void *)pwait,
                        "swaitRecord:special: Illegal CALC field");
        }
        db_post_events(pwait,&pwait->clcv,DBE_VALUE);
        db_post_events(pwait,pwait->calc,DBE_VALUE);
        db_post_events(pwait,&pwait->clcv,DBE_VALUE);
        return(0);
    }
    else if (paddr->pfield==(void *)&pwait->prio) {
        callbackSetPriority(pwait->prio, &pcbst->doOutCb);
        callbackSetPriority(pwait->prio, &pcbst->ioProcCb);
        return(0);
    }
    else {
	recGblDbaddrError(S_db_badChoice,paddr,"swait:special");
	return(S_db_badChoice);
        return(0);
    }
}

STATIC long get_precision(paddr,precision)
    struct dbAddr *paddr;
    long	  *precision;
{
    swaitRecord	*pwait=(swaitRecord *)paddr->precord;

    *precision = pwait->prec;
    if (paddr->pfield == (void *)&pwait->val) {
        *precision = pwait->prec;
    }
    else if (paddr->pfield == (void *)&pwait->odly) {
        *precision = 3;
    }
    return(0);
}

STATIC long get_graphic_double(paddr,pgd)
    struct dbAddr *paddr;
    struct dbr_grDouble	*pgd;
{
    swaitRecord *pwait=(swaitRecord *)paddr->precord;

    if (paddr->pfield==(void *)&pwait->val) { 
          pgd->upper_disp_limit = pwait->hopr;
          pgd->lower_disp_limit = pwait->lopr;
    } else recGblGetGraphicDouble(paddr,pgd);
    return(0);
}

STATIC long get_alarm_double(paddr,pad)
    struct dbAddr *paddr;
    struct dbr_alDouble *pad;
{
    recGblGetAlarmDouble(paddr,pad);
    return(0);
}

STATIC void monitor(pwait)
    swaitRecord   *pwait;
{
        unsigned short  monitor_mask;
        double          delta;
        double          *pnew;
        double          *pprev;
        int             i;

        monitor_mask = recGblResetAlarms(pwait);
        /* check for value change */
        delta = pwait->mlst - pwait->val;
        if (delta<0.0) delta = -delta;
        if (delta > pwait->mdel) {
                /* post events for value change */
                monitor_mask |= DBE_VALUE;
                /* update last value monitored */
                pwait->mlst = pwait->val;
        }
        /* check for archive change */
        delta = pwait->alst - pwait->val;
        if (delta<0.0) delta = -delta;
        if (delta > pwait->adel) {
                /* post events on value field for archive change */
                monitor_mask |= DBE_LOG;
                /* update last archive value monitored */
                pwait->alst = pwait->val;
        }

        /* send out monitors connected to the value field */
        if (monitor_mask){
                db_post_events(pwait,&pwait->val,monitor_mask);
        }
        /* check all input fields for changes */
        for(i=0, pnew=&pwait->a, pprev=&pwait->la; i<ARG_MAX;
            i++, pnew++, pprev++) {
            if (*pnew != *pprev) {
                 db_post_events(pwait,pnew,monitor_mask|DBE_VALUE);
                 *pprev = *pnew;
                 db_post_events(pwait,pprev,monitor_mask|DBE_VALUE);
            }
        }
        return;
}


STATIC long initSiml(pwait)
swaitRecord   *pwait;
{ 

    /* swait.siml must be a CONSTANT or a PV_LINK or a DB_LINK */
    if (pwait->siml.type == CONSTANT) {
	recGblInitConstantLink(&pwait->siml,DBF_USHORT,&pwait->simm);
    }

    /* swait.siol must be a CONSTANT or a PV_LINK or a DB_LINK */
    if (pwait->siol.type == CONSTANT) {
	recGblInitConstantLink(&pwait->siol,DBF_DOUBLE,&pwait->sval);
    }

    return(0);
}

STATIC int fetch_values(pwait)
swaitRecord *pwait;
{
        struct cbStruct *pcbst = (struct cbStruct *)pwait->cbst;
        double          *pvalue;
        unsigned short  *pPvStat;
        unsigned short  *piointInc;   /* include for IO_INT ? */
        long            status=0;
	size_t		nRequest=1;
        int             i;

        piointInc  = &pwait->inap;
        for(i=0,  pvalue=&pwait->a, pPvStat = &pwait->inav;
            i<ARG_MAX; i++, pvalue++, pPvStat++, piointInc++) {

            /* if any input should be connected, but is not, return */
            if (*pPvStat == PV_NC) {
                 status = -1;
            }

            /* only fetch a value if the connection is valid */
            /* if not in SCAN_IO_EVENT, fetch all valid inputs */
            /* if in SCAN_IO_EVENT, only fetch inputs if INxP ==  0 */
            /* The data from those with INxP=1 comes from the ring buffer */
            else if ((*pPvStat == PV_OK)&&
                    ((pwait->scan != SCAN_IO_EVENT) || 
                     ((pwait->scan == SCAN_IO_EVENT) && !*piointInc))) {
               if (swaitRecordDebug > 5) errlogPrintf("Fetching input %d \n",i);
               status = recDynLinkGet(&pcbst->caLinkStruct[i], pvalue,
                                      &nRequest, 0, 0, 0);
            }
            if (!RTN_SUCCESS(status)) return(status);
        }
        return(0);
}

/***************************************************************************
 *
 * The following functions schedule and/or request the execution of the
 * output PV and output event based on the Output Execution Delay (ODLY).
 * If .odly > 0, a watchdog is scheduled; if 0, execOutput() is called 
 * immediately.
 * NOTE: THE RECORD REMAINS "ACTIVE" WHILE WAITING ON THE WATCHDOG
 *
 **************************************************************************/
STATIC void schedOutput(swaitRecord *pwait)
{
	struct cbStruct *pcbst = (struct cbStruct *)pwait->cbst;

	if (pwait->odly > 0.0) {
		/* Use the watch-dog as a delay mechanism */
		pcbst->outputWait = 1;
        callbackRequestDelayed(&pcbst->doOutCb, pwait->odly);
	} else {
		execOutput(pwait);
	}
}


static void doOutputCallback(CALLBACK *pcallback)
{
	dbCommon    *pwait;
	struct rset *prset;

	callbackGetUser(pwait, pcallback);
	prset = (struct rset *)pwait->rset;

	dbScanLock((struct dbCommon *)pwait);
	(*prset->process)(pwait);
	dbScanUnlock((struct dbCommon *)pwait);
}

/***************************************************************************
 *
 * This code calls recDynLinkPut to execute the output link. Since requests   
 * recDynLinkPut are done via another task, one need not worry about
 * lock sets.
 *
 ***************************************************************************/
STATIC void execOutput(swaitRecord *pwait)
{
	long status;
	size_t nRequest = 1;
	double oldDold, outValue=0;
	struct cbStruct *pcbst = pwait->cbst;

	if (swaitRecordDebug >= 10)
		errlogPrintf("swaitRecord(%s)execOutput: entry\n", pwait->name);
	/* if output link is valid , decide between VAL and DOL */
	if (!pwait->outv) {
		if (pwait->dopt) {
			if (!pwait->dolv) {
				oldDold = pwait->dold;
				status = recDynLinkGet(&pcbst->caLinkStruct[DOL_INDEX],
						&(pwait->dold), &nRequest, 0, 0, 0);
				if (pwait->dold != oldDold)
					db_post_events(pwait,&pcbst->pwait->dold,DBE_VALUE);
			}
			outValue = pwait->dold;
		} else {
			outValue = pwait->val;
		}
		if (swaitRecordDebug >= 10)
			errlogPrintf("swaitRecord(%s)execOutput: calling recDynLinkPutCallback()\n",
				pwait->name);
		status = recDynLinkPutCallback(&pcbst->caLinkStruct[OUT_INDEX],
			&outValue, 1, notifyCallback);
		if (status == NOTIFY_IN_PROGRESS) {
			/*
			 * The record we want to Put to is busy.  This might indicate
			 * a logical error in the sequence of events the user has
			 * programmed, or it may simply mean he wants to abort or modify
			 * the sequence.  In any case, we can't do a callback put, so the
			 * only hope of success is to do a non-callback put. 
			 */
			status = recDynLinkPut(&pcbst->caLinkStruct[OUT_INDEX], &outValue, 1);
		}
	} else {
		if (swaitRecordDebug >= 10)
			errlogPrintf("swaitRecord(%s)execOutput: calling recGblFwdLink()\n", pwait->name);
		recGblFwdLink(pwait);
	}

	if (pwait->oevt > 0) {post_event((int)pwait->oevt);}
	pcbst->outputWait = 0;
	pwait->pact = FALSE;

	/* If I/O Interrupt scanned, see if any inputs changed during delay */
	if ((pwait->scan == SCAN_IO_EVENT) && (pcbst->procPending == 1)) {
		if (swaitRecordDebug >= 10)
			errlogPrintf("swaitRecord(%s)execOutput: calling scanOnce()\n", pwait->name);
		scanOnce(pwait);
	}
	return;
}



/* This routine is called by the recDynLink task whenver a monitored input
 * changes. If the particular input is flagged to cause record processing, 
 * The input index and new data are put on a work queue, and a callback
 * request is issued to the routine ioIntProcess
 */

STATIC void inputChanged(recDynLink *precDynLink)
{
  swaitRecord *pwait = ((recDynLinkPvt *)precDynLink->puserPvt)->pwait;
  struct cbStruct   *pcbst = (struct cbStruct   *)pwait->cbst;
  double             monData;
  size_t      	     nRequest;
  char               index;
  unsigned short    *piointInc;

    if (pwait->scan != SCAN_IO_EVENT) return; 

    index = (char)((recDynLinkPvt *)precDynLink->puserPvt)->linkIndex;

    piointInc = &pwait->inap + index;    /* pointer arithmetic */
    if (*piointInc == 0) return;     /* input cause processing ???*/

    /* put input index and monitored data on processing queue */
    recDynLinkGet(precDynLink, &monData, &nRequest, 0, 0, 0); 
    if (swaitRecordDebug>5)
        errlogPrintf("swaitRecord(%s)inputChanged: queuing monitor on %d = %f\n",pwait->name,index,monData);
    if (epicsRingBytesPut(pcbst->monitorQ, (char *)&index, sizeof(char))
        != sizeof(char)) errMessage(0,"recWait rngBufPut error");
    if (epicsRingBytesPut(pcbst->monitorQ, (char *)&monData, sizeof(double))
        != sizeof(double)) errMessage(0,"recWait rngBufPut error");
	if (swaitRecordDebug>5) {
		errlogPrintf("swaitRecord(%s)inputChanged: %d entries left in monitorQ\n", pwait->name,
			(int)(epicsRingBytesFreeBytes(pcbst->monitorQ)/sizeof(struct qStruct)));
	}
    callbackRequest(&pcbst->ioProcCb);
}


/* This routine performs the record processing when in SCAN_IO_EVENT. An
   event queue is built by inputChanged() and emptied here so each change
   of an input causes the record to process.
*/
STATIC void ioIntProcess(CALLBACK *pioProcCb)
{
	swaitRecord *pwait;
	struct cbStruct   *pcbst;

	char     inputIndex;
	double   monData;
	double   *pInput; 

	callbackGetUser(pwait, pioProcCb);
	pcbst = (struct cbStruct *)pwait->cbst;
	pInput = &pwait->a;  /* a pointer to the first input field */

	if (pwait->scan != SCAN_IO_EVENT) return;

	if (!swaitRecordCacheMode) {
		if (epicsRingBytesGet(pcbst->monitorQ, (char *)&inputIndex, sizeof(char))
			!= sizeof(char)) errMessage(0, "recWait: rngBufGet error");
		if (epicsRingBytesGet(pcbst->monitorQ, (char *)&monData, sizeof(double))
			!= sizeof(double)) errMessage(0, "recWait: rngBufGet error");

		if (swaitRecordDebug>=5)
			errlogPrintf("swaitRecord(%s)ioIntProcess: processing on %d = %f  (%f)\n",
				pwait->name, inputIndex, monData,pwait->val);
		pInput += inputIndex;   /* pointer arithmetic for appropriate input */ 
		dbScanLock((struct dbCommon *)pwait);
		*pInput = monData;      /* put data in input data field */
      
		/* Process the record, unless busy waiting to do the output link */
		if (pcbst->outputWait) {
			pcbst->procPending = 1;
			if (swaitRecordDebug)
				errlogPrintf("swaitRecord(%s)ioIntProcess:record busy, setting procPending\n", 
					pwait->name);
		} else {
			dbProcess((struct dbCommon *)pwait);     /* process the record */
		}
		dbScanUnlock((struct dbCommon *)pwait);
	} else {
		if (swaitRecordDebug>=5) errlogPrintf("%s:processing (cached)\n", pwait->name);
		dbScanLock((struct dbCommon *)pwait);
		dbProcess((struct dbCommon *)pwait);     /* process the record */
		dbScanUnlock((struct dbCommon *)pwait);
	} 
}
    

STATIC void pvSearchCallback(recDynLink *precDynLink)
{

    recDynLinkPvt     *puserPvt = (recDynLinkPvt *)precDynLink->puserPvt;
    swaitRecord *pwait    = puserPvt->pwait;
    unsigned short     index    = puserPvt->linkIndex;
    unsigned short    *pPvStat;
    unsigned short     oldValid;

    pPvStat = &pwait->inav + index;    /* pointer arithmetic */
    puserPvt = (recDynLinkPvt *)precDynLink->puserPvt;

    oldValid = *pPvStat;
    if (recDynLinkConnectionStatus(precDynLink)) {
        *pPvStat = PV_NC;
        if (swaitRecordDebug) errlogPrintf("%s:Search Callback: No Connection\n", pwait->name);
    }
    else {
        *pPvStat = PV_OK;
        if (swaitRecordDebug) errlogPrintf("%s:Search Callback: Success\n", pwait->name);
    }
    if (*pPvStat != oldValid) {
        db_post_events(pwait, pPvStat, DBE_VALUE);
    }
        
}


