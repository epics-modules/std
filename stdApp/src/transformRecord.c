/* transformRecord.c - Record Support Routines for Transform records */
/*
 *      Original Author: Julie Sander and Bob Dalesio (as Calc record)
 *      Current  Author: Tim Mooney
 *      Date:            3/12/93
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
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
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 *      and
 *              Beamlines
 *              Experimental Facilities Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  3-12-93   tmm  Adapted from EPICS V3.8 Calc record
 * .02  6-8-93    tmm  Convert infix buffers to pointers for DCT
 * .03  2-22-96   tmm  v1.1: Added version field (VERS)
 * .04  2-22-96   tmm  v2.0: Added "Process on Put" fields for A-L
 * .05  06-06-96  tmm  v2.01: default get_precision to PREC field
 * .06  08-16-96  tmm  v3.0: conversion to EPICS 3.13
 * .07  06-19-97  tmm  v3.1: added four more sets of calc/i/o fields,
 *                     detect new value from bitmap marked in special()
 *                     instead of by comparing with previous value.
 * .08  06-20-97  tmm  v3.2: check for valid links (using code stolen
 *                     from Ned Arnold's calcAo record).  Added IxV and
 *                     OxV fields to indicate valid links.
 * .09  07-07-97  tmm  v3.3: mark bad infix expression in the postfix array
 *                     with BAD_EXPRESSION
 * .10  07-10-97  tmm  v3.4: back off from detecting new value from bitmap
 *                     marked in special()
 * .11  08-08-97  tmm  v3.5: fixed link-checking code for output links (this
 *                     required rearranging fields in transformRecord.dbd)
 * .12  12-16-97  tmm  v3.6: unreachable links no longer stop processing
 * .13  01-05-98  tmm  v3.61: use interim version of calcPostfix
 * .14  01-14-98  tmm  v4.0: new calcPostfix allocates postfix buffer; added
 *                     C*V fields to mark invalid infix expressions
 * .15  02-06-98  tmm  v4.1: uses new sCalcPostfix() and sCalcPerform().
 * .16  05-20-98  tmm  v5.0: get scanOnce out of special() (kills dbNotify())
 * .17  02-08-99  tmm  v5.1: delete test code re SPC_MOD treatment of APP, etc.
 * .18  01-21-02  tmm  v5.2: test for old value (e.g., A == LA) failed (always
 *                     returned false) if A and LA were NaN.  This meant
 *                     you could never get rid of a NaN result: the expression
 *                     would never be evaluated, because the value always looked
 *                     new.)  Now we test bit pattern directly to see if a value
 *                     is new.
 * .19  04-25-02  tmm  v5.3: debugging test for old value
 * .20  04-29-02  tmm  v5.4: Debug allows to single out a particular record using TPRO:
 *                     if TPRO then transformRecordDebug effectively gets increased by 10
 *                     Try again to use bitmap to help check for new values: If the value
 *                     of a field is different from it's previous value, or if the field's
 *                     bit has been marked, then treat the value as new (don't do the calc)
 *                     In special, if the value is being written to as a direct result of
 *                     our processing (i.e., if PACT != 0), then don't mark the bitmap.
 */
#define VERSION 5.4

#include	<vxWorks.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<stdlib.h>
#include	<lstLib.h>
#include	<string.h>
#include	<tickLib.h>
#include	<wdLib.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbEvent.h>
#include	<dbAccess.h>
#include	<dbFldTypes.h>
#include	<dbStaticLib.h>
#include	<dbScan.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<special.h>
#include	<callback.h>
#include	<taskwd.h>
#include	"sCalcPostfix.h"
#include	"sCalcPostfixPvt.h"	/* define BAD_EXPRESSION, END_STACK */

#define GEN_SIZE_OFFSET
#include	"transformRecord.h"
#undef GEN_SIZE_OFFSET

#ifdef NODEBUG
#define Debug(l,FMT,V) ;
#else
#define Debug(l,FMT,V) {  if (l <= transformRecordDebug+10*ptran->tpro) \
			{ printf("transform(%s):", ptran->name); \
			  printf(FMT,V); } }
#endif
volatile int    transformRecordDebug = 0;

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long     init_record();
static long     process();
static long     special();
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
static long     get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL

struct rset     transformRSET = {
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

static void     alarm();
static void     monitor();

/* To provide feedback to the user as to the connection status of the 
 * links (.IxV and .OxV), the following algorithm has been implemented ...
 *
 * A new PV_LINK [either in init() or special()] is searched for using
 * dbNameToAddr. If local, it is so indicated. If not, a checkLinkCb
 * callback is scheduled to check the connectivity later using 
 * dbCaIsLinkConnected(). Anytime there are unconnected CA_LINKs, another
 * callback is scheduled. Once all connections are established, the CA_LINKs
 * are checked whenever the record processes. 
 * (This code stolen from Ned Arnold's calcAo record.)
 */

static void checkLinksCallback();
static void checkLinks();
#define NO_CA_LINKS     0
#define CA_LINKS_ALL_OK 1
#define CA_LINKS_NOT_OK 2

struct rpvtStruct {
	CALLBACK	checkLinkCb;
	WDOG_ID		wd_id;
	short		wd_id_LOCK;
	short		caLinkStat; /* NO_CA_LINKS,CA_LINKS_ALL_OK,CA_LINKS_NOT_OK */
};

/* If the .dbd file changes, these must also change. */
#define INFIX_SIZE 40
#define ARG_MAX 16
/* Fldnames should have ARG_MAX elements */
static char Fldnames[ARG_MAX][FLDNAME_SZ] =
{"A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P"};


static long 
init_record(transformRecord *ptran, int pass)
{
	int				i, j;
	long			*pcalcInvalid;
	struct link		*pinlink, *poutlink;
	double			*pvalue, *plvalue;
	short			error_number;
	/* ptr to arrays of ptrs to buffers holding infix, postfix expressions */
	char			*pclcbuf, **pprpcbuf;	
	/* for checking links */
    unsigned short	*pInLinkValid, *pOutLinkValid;
    struct dbAddr	dbAddr;
    struct rpvtStruct	*prpvt;

	Debug(15, "init_record: pass = %d\n", pass);

	if (pass == 0) {
		ptran->vers = VERSION;
        ptran->rpvt = (void *)calloc(1, sizeof(struct rpvtStruct));
		return (0);
	}
    prpvt = (struct rpvtStruct *)ptran->rpvt;

	/* Gotta have a .val field.  Make its value reproducible. */
	ptran->val = 0;
	ptran->map = 0; /* init "marked" bitmap to all unmarked */

	pinlink = &ptran->inpa;
	poutlink = &ptran->outa;
    pInLinkValid = &ptran->iav;
    pOutLinkValid = &ptran->oav;
	pvalue = &ptran->a;
	plvalue = &ptran->la;
	pclcbuf = ptran->clca;	/* infix expressions */
	pprpcbuf = (char **)&(ptran->rpca);	/* postfix expressions */
	pcalcInvalid = &ptran->cav;
	for (i = 0; i < ARG_MAX;
	     i++, pinlink++, poutlink++, pvalue++, plvalue++, pInLinkValid++,
		pOutLinkValid++, pclcbuf += INFIX_SIZE, pprpcbuf++, pcalcInvalid++) {

		Debug(25, "init_record: ...field %s\n", Fldnames[i]);
		*pprpcbuf = NULL;

		/*** check input links ***/
		if (pinlink->type == CONSTANT) {
			recGblInitConstantLink(pinlink,DBF_DOUBLE,pvalue);
			db_post_events(ptran, pvalue, DBE_VALUE);
            *pInLinkValid = transformIAV_CON;
		}
        /* see if the PV resides on this ioc */
        else if (!dbNameToAddr(pinlink->value.pv_link.pvname, &dbAddr)) {
            *pInLinkValid = transformIAV_LOC;
        }
        /* pv is not on this ioc. Callback later for connection stat */
        else {
            *pInLinkValid = transformIAV_EXT_NC;
             prpvt->caLinkStat = CA_LINKS_NOT_OK;
        }
        db_post_events(ptran,pInLinkValid,DBE_VALUE);

		/*** check output links ***/
		if (poutlink->type == CONSTANT) {
            *pOutLinkValid = transformIAV_CON;
		}
        /* see if the PV resides on this ioc */
        else if (!dbNameToAddr(poutlink->value.pv_link.pvname, &dbAddr)) {
            *pOutLinkValid = transformIAV_LOC;
        }
        /* pv is not on this ioc. Callback later for connection stat */
        else {
            *pOutLinkValid = transformIAV_EXT_NC;
             prpvt->caLinkStat = CA_LINKS_NOT_OK;
        }
        db_post_events(ptran,pOutLinkValid,DBE_VALUE);

		/*** check and convert calc expressions ***/
		*pcalcInvalid = 0; /* empty expression is valid */
		if (*pclcbuf) {
			/* make sure it's no longer than INFIX_SIZE characters */
			pclcbuf[INFIX_SIZE - 1] = (char) 0;
			Debug(19, "init_record: infix expression: '%s'\n", pclcbuf);
			*pcalcInvalid = sCalcPostfix(pclcbuf, pprpcbuf, &error_number);
			if (*pcalcInvalid) {
				recGblRecordError(S_db_badField,(void *)ptran,
					"transform: init_record: Illegal CALC field");
			}
			db_post_events(ptran,pcalcInvalid,DBE_VALUE);
			if (transformRecordDebug+10*ptran->tpro >= 19) {
				printf("transform(%s):init_record: postfix expression:", ptran->name);
				for (j = 0; (*pprpcbuf)[j] != BAD_EXPRESSION; j++) {
					printf(" %2d", (*pprpcbuf)[j]);
					if ((*pprpcbuf)[j] == END_STACK) break;
				}
				printf("\n");
			}
		}
		*plvalue = *pvalue;
	}

    callbackSetCallback(checkLinksCallback, &prpvt->checkLinkCb);
    callbackSetPriority(0, &prpvt->checkLinkCb);
    callbackSetUser(ptran, &prpvt->checkLinkCb);
    prpvt->wd_id = wdCreate();
    prpvt->wd_id_LOCK = 0;

    if (prpvt->caLinkStat == CA_LINKS_NOT_OK) {
        prpvt->wd_id_LOCK = 1;
        wdStart(prpvt->wd_id, 60, (FUNCPTR)callbackRequest,
                (int)(&prpvt->checkLinkCb));
    }

	return (0);
}

static long 
process(transformRecord *ptran)
{
	int				i, no_inlink, new_value, postfix_ok, same;
	long			status;
	struct link		*plink;
	double			*pval, *plval;
	char			**pprpcbuf, *pclcbuf;
    struct rpvtStruct	*prpvt = (struct rpvtStruct *)ptran->rpvt;
	int				*pu, *plu;

	Debug(15, "process: entry%s\n", ".");
	ptran->pact = TRUE;
	ptran->udf = FALSE;

	/* if some links are CA, check connections */
	if (prpvt->caLinkStat != NO_CA_LINKS) {
		checkLinks(ptran);
	}

	/* Process input links. */
	plink = &ptran->inpa;
	pval = &ptran->a;
	for (i = 0; i < ARG_MAX; i++, plink++, pval++) {
		if (plink->type != CONSTANT) {
			Debug(15, "process: field %s has an input link.\n", Fldnames[i]);
			status = dbGetLink(plink, DBR_DOUBLE, pval, NULL, NULL);
			if (!RTN_SUCCESS(status)) {
				Debug(15, "process: dbGetLink() failed for field %s.\n", Fldnames[i]);
				*pval = 0.;
			}
			Debug(15, " ...process: Value is %f.\n", *pval);
		}
	}

	/* Do calculations. */
	plink = &ptran->inpa;
	pval = &ptran->a;
	plval = &ptran->la;
	pprpcbuf = (char **) &(ptran->rpca);
	pclcbuf = (char *)ptran->clca;
	for (i=0; i < ARG_MAX;
			i++, plink++, pval++, plval++, pprpcbuf++, pclcbuf+=INFIX_SIZE) {
		no_inlink = plink->type == CONSTANT;
		/* if value is same as last time, and bitmap is unmarked, don't calc */
		pu = (int *)pval;
		plu = (int *)plval;
		same = (*pval==0. && *pval==0.) || ((pu[0] == plu[0]) && (pu[1] == plu[1]));
		if (transformRecordDebug+10*ptran->tpro >= 15) {
			printf("transform(%s.%1s): same=%d, (*pval==*plval) = %d, map=0x%x\n", ptran->name,
				Fldnames[i], same, *pval == *plval, ptran->map);
		}
		if (transformRecordDebug+10*ptran->tpro >= 19) {
			printf("   *pval=%f, *plval=%f, pu=%x,%x, plu=%x,%x\n",
					*pval,*plval,pu[0],pu[1],plu[0],plu[1]);
		}
		new_value = (!same || ((ptran->map&(1<<i)) != 0));
		postfix_ok = *pclcbuf && (**pprpcbuf != BAD_EXPRESSION);
		Debug(15, "process: %s input link; \n", no_inlink ? "NO" : "");
		Debug(15, "process: value is %s\n", new_value ? "NEW" : "OLD");
		Debug(15, "process: expression is%s ok\n", postfix_ok ? " " : " NOT");
		if (no_inlink && !new_value && postfix_ok) {
			Debug(15, "process: calculating for field %s\n", Fldnames[i]);
			if (sCalcPerform(&ptran->a, 16, NULL,0, pval, NULL,0, *pprpcbuf)) {
				recGblSetSevr(ptran, CALC_ALARM, INVALID_ALARM);
				ptran->udf = TRUE;
			}
			Debug(15, "process: calculation yields %f\n", *pval);
		}
	}
	ptran->map = 0;

	/* Process output links. */
	plink = &(ptran->outa);
	pval = &ptran->a;
	for (i = 0; i < ARG_MAX; i++, plink++, pval++) {
		if (plink->type != CONSTANT) {
			Debug(15, "process: field %s has an output link.\n", Fldnames[i]);
			status = dbPutLink(plink, DBR_DOUBLE, pval, 1);
			if (!RTN_SUCCESS(status)) {
				Debug(15, "process: ERROR %d PUTTING TO OUTPUT LINK.\n", status);
			}
		}
	}

	recGblGetTimeStamp(ptran);
	/* check for alarms */
	alarm(ptran);
	/* check event list */
	monitor(ptran);
	/* process the forward scan link record */
	recGblFwdLink(ptran);
	ptran->pact = FALSE;
	return (0);
}


static long 
special(struct dbAddr *paddr, int after)
{
	int				i, j;
	transformRecord	*ptran = (transformRecord *) (paddr->precord);
	int				special_type = paddr->special;
	short			error_number;
	char			*pclcbuf, **pprpcbuf;
	struct link		*plink = &ptran->inpa;
    int				fieldIndex = dbGetFieldIndex(paddr);
	/* link-check stuff */
    struct rpvtStruct   *prpvt = (struct rpvtStruct *)ptran->rpvt;
    struct dbAddr	dbAddr;
    unsigned short	*plinkValid;
    double			*pvalue;
	long			status, *pcalcInvalid;

	Debug(15, "special: after = %d\n", after);

	/*
	 * Don't accept changes to a value field if its input link exists
	 * (whether or not the link is valid).
	 */
	if (!after) {
		plink = &ptran->inpa;
		i = fieldIndex - transformRecordA;
		if ((i >= 0) && (i < ARG_MAX)) {
			/* user is attempting to change a value field */
			plink += i;
			if (plink->type != CONSTANT) return(1);
		}
		return (0);
	}

	Debug(15, "special: special_type = %d\n", special_type);
	switch (special_type) {
	case (SPC_CALC):
		pclcbuf = ptran->clca;
		pprpcbuf = (char **)&(ptran->rpca);
		pcalcInvalid = &ptran->cav;
		for (i = 0;
		     i < ARG_MAX && paddr->pfield != (void *) pclcbuf;
		     i++, pclcbuf += INFIX_SIZE, pprpcbuf++, pcalcInvalid++);
		if (i < ARG_MAX) {
			status = 0; /* empty expression is valid */
			if (*pclcbuf) {
				/* make sure it's no longer than INFIX_SIZE chars */
				pclcbuf[INFIX_SIZE - 1] = (char) 0;
				Debug(15, "special: infix expression: '%s'\n", pclcbuf);
				status = sCalcPostfix(pclcbuf, pprpcbuf, &error_number);
				if (status) {
					recGblRecordError(S_db_badField,(void *)ptran,
						"transform:special: Illegal CALC field");
				}
				if (transformRecordDebug+10*ptran->tpro >= 5) {
					printf("special: postfix expression:");
					for (j = 0; ((char *)(*pprpcbuf))[j] != BAD_EXPRESSION; j++) {
						printf(" %2d", ((unsigned char *)(*pprpcbuf))[j]);
						if (((char *)(*pprpcbuf))[j] == END_STACK) break;
					}
					printf("\n");
				}
			}
			if (*pcalcInvalid != status) {
				*pcalcInvalid = status;
				db_post_events(ptran, pcalcInvalid, DBE_VALUE);
			}
		}
		return (0);

	case (SPC_MOD):
#if 1
		/* Mark value field as "new", unless we caused the field to be written */
		if (ptran->pact == 0) {
			i = fieldIndex - transformRecordA;
			if ((i >= 0) && (i < ARG_MAX)) {
				/* user is changing a value field */
				ptran->map |= (1<<i);	/* note new value (don't do calc) */
			}
		}
#endif

		/* If user has changed a link, check it */
		i = fieldIndex - transformRecordINPA;
		if ((i >= 0) && (i < 2*ARG_MAX)) {
			Debug(15, "special: checking link, i=%d\n", i);
			plink   = &ptran->inpa + i;
			pvalue  = &ptran->a    + i;
			plinkValid = &ptran->iav + i;

	        if (plink->type == CONSTANT) {
				/* get initial value if this is an input link */
	            if (fieldIndex < transformRecordOUTA) {
	                recGblInitConstantLink(plink,DBF_DOUBLE,pvalue);
	                db_post_events(ptran,pvalue,DBE_VALUE);
	            }
				Debug(15, "special: ...constant link, i=%d\n", i);
	            *plinkValid = transformIAV_CON;
	        }
	        /* see if the PV resides on this ioc */
	        else if (!dbNameToAddr(plink->value.pv_link.pvname, &dbAddr)) {
	            *plinkValid = transformIAV_LOC;
				Debug(15, "special: ...local link, i=%d\n", i);
	        }
	        /* pv is not on this ioc. Callback later for connection stat */
	        else {
	            *plinkValid = transformIAV_EXT_NC;
	            /* DO_CALLBACK, if not already scheduled */
				Debug(15, "special: ...CA link, wd_id_LOCK=%d\n", prpvt->wd_id_LOCK);
	            if (!prpvt->wd_id_LOCK) {
	                prpvt->wd_id_LOCK = 1;
	                wdStart(prpvt->wd_id, 30, (FUNCPTR)callbackRequest,
	                    (int)(&prpvt->checkLinkCb));
	                prpvt->caLinkStat = CA_LINKS_NOT_OK;
					Debug(15, "special: ...CA link, i=%d, call wdStart()\n", i);
	            }
	        }
	        db_post_events(ptran,plinkValid,DBE_VALUE);
		}

		return(0);

	default:
		recGblDbaddrError(S_db_badChoice, paddr, "transform: special");
		return (S_db_badChoice);
	}
}

static long 
get_precision(struct dbAddr *paddr, long *precision)
{
	transformRecord *ptran = (transformRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

	*precision = ptran->prec;
	if (fieldIndex == transformRecordVERS) {
		*precision = 2;
	} else if (fieldIndex >= transformRecordVAL) {
		*precision = ptran->prec;
	} else {
		recGblGetPrec(paddr, precision);	/* Field is in dbCommon */
	}
	return (0);
}


static void 
alarm(transformRecord *ptran)
{
	if (ptran->udf == TRUE) {
		recGblSetSevr(ptran, UDF_ALARM, INVALID_ALARM);
		return;
	}
	return;
}


static void 
monitor(transformRecord *ptran)
{
	unsigned short  monitor_mask;
	double         *pnew;
	double         *pprev;
	int             i;

	monitor_mask = recGblResetAlarms(ptran);
	monitor_mask = DBE_VALUE;

	/* check all value fields for changes */
	for (i = 0, pnew = &ptran->a, pprev = &ptran->la; i < ARG_MAX; i++, pnew++, pprev++) {
		if (*pnew != *pprev) {
			db_post_events(ptran, pnew, monitor_mask);
			*pprev = *pnew;
		}
	}
	return;
}


static void checkLinksCallback(CALLBACK *pcallback)
{
    struct transformRecord	*ptran;
    struct rpvtStruct		*prpvt;

    callbackGetUser(ptran, pcallback);
    prpvt = (struct rpvtStruct *)ptran->rpvt;
	Debug(15, "checkLinksCallback() for %s\n", ptran->name);

	if (!interruptAccept) {
		/* Can't call dbScanLock yet.  Schedule another CALLBACK */
		prpvt->wd_id_LOCK = 1;  /* make sure */
		wdStart(prpvt->wd_id, 30, (FUNCPTR)callbackRequest,
			(int)(&prpvt->checkLinkCb));
	} else {
		dbScanLock((struct dbCommon *)ptran);
		prpvt->wd_id_LOCK = 0;
		checkLinks(ptran);
		dbScanUnlock((struct dbCommon *)ptran);
	}
}


static void checkLinks(struct transformRecord *ptran)
{
    struct link *plink;
    struct rpvtStruct   *prpvt = (struct rpvtStruct *)ptran->rpvt;
    int i;
    int stat;
    int caLink   = 0;
    int caLinkNc = 0;
    unsigned short *plinkValid;

    Debug(15, "checkLinks() for %p\n", ptran);

    plink   = &ptran->inpa;
    plinkValid = &ptran->iav;

    for (i=0; i<2*ARG_MAX; i++, plink++, plinkValid++) {
        if (plink->type == CA_LINK) {
            caLink = 1;
            stat = dbCaIsLinkConnected(plink);
            if (!stat && (*plinkValid == transformIAV_EXT_NC)) {
                caLinkNc = 1;
            }
            else if (!stat && (*plinkValid == transformIAV_EXT)) {
                *plinkValid = transformIAV_EXT_NC;
                db_post_events(ptran,plinkValid,DBE_VALUE);
                caLinkNc = 1;
            } 
            else if (stat && (*plinkValid == transformIAV_EXT_NC)) {
                *plinkValid = transformIAV_EXT;
                db_post_events(ptran,plinkValid,DBE_VALUE);
            } 
        }
        
    }
    if (caLinkNc)
        prpvt->caLinkStat = CA_LINKS_NOT_OK;
    else if (caLink)
        prpvt->caLinkStat = CA_LINKS_ALL_OK;
    else
        prpvt->caLinkStat = NO_CA_LINKS;

    if (!prpvt->wd_id_LOCK && caLinkNc) {
        /* Schedule another CALLBACK */
        prpvt->wd_id_LOCK = 1;
        wdStart(prpvt->wd_id, 30, (FUNCPTR)callbackRequest,
               (int)(&prpvt->checkLinkCb));
    }
}
