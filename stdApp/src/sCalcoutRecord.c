/* scalcout.c - Record Support Routines for calc with output records */
/*
 *   Author : Ned Arnold
 *   Based on recCalc.c, by ...
 *      Original Author: Julie Sander and Bob Dalesio
 *      Current  Author: Marty Kraimer
 *      Date:            7-27-87
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
 * Modification Log:
 * -----------------
 * .01  08-29-96	nda    Created from scalcoutRecord.c for EPICS R3.13 
 * .02  09-13-96	nda    Original release for EPICS R3.13beta3
 * .03  03-24-98    tmm    v2.0: created from Ned Arnold's calcout record
 * .04  07-06-98    tmm    v3.1: prototype field-type sensitive output link
 * .05  08-25-98    tmm    v3.2: use new support in EPICS 3.13.0.beta12 for
 *                         field-type sensitive link.
 * .06  11-04-98    tmm    v3.3: Don't call dbScanLock until interruptAccept
 *                         removed some dead code re. type-sensitive links
 * .07  11-11-98    tmm    v3.4: Support 12 strings settable at DCT time
 * .08  03-23-99    tmm    v3.5: time stamp support
 * .09  01-18-00    tmm    v3.6: special() did not list INGG...INLL
 * .10  05-08-00    tmm    v3.61: changed some status messages to debug messages.
 * .11  08-22-00    tmm    v3.62: changed message text.
 * .12  04-22-03    tmm    v3.7: RPC fields now allocated in dbd file, since
 *                         sCalcPostfix doesn't allocate them anymore
 *
 */

#define VERSION 3.7


#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdarg.h>
#include	<stdio.h>
#include	<string.h>
#include	<math.h>

#include	<tickLib.h>
#include	<wdLib.h>
#include	<sysLib.h>

#include	<epicsVersion.h>
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<dbScan.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<special.h>
#include	<callback.h>
#include	<taskwd.h>
#include	<drvTS.h>	/* also includes timers.h and tsDefs.h */
#include "sCalcPostfix.h"

#define GEN_SIZE_OFFSET
#include	"sCalcoutRecord.h"
#undef  GEN_SIZE_OFFSET
#include	<menuIvoa.h>

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
static long special();
#define get_value NULL
static long cvt_dbaddr();
#define get_array_info NULL
#define put_array_info NULL
static long get_units();
static long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
static long get_graphic_double();
static long get_control_double();
static long get_alarm_double();

struct rset scalcoutRSET={
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


/* To provide feedback to the user as to the connection status of the 
 * links (.INxV and .OUTV), the following algorithm has been implemented ...
 *
 * A new PV_LINK [either in init() or special()] is searched for using
 * dbNameToAddr. If local, it is so indicated. If not, a checkLinkCb
 * callback is scheduled to check the connectivity later using 
 * dbCaIsLinkConnected(). Anytime there are unconnected CA_LINKs, another
 * callback is scheduled. Once all connections are established, the CA_LINKs
 * are checked whenever the record processes. 
 *
 */

#define NO_CA_LINKS     0
#define CA_LINKS_ALL_OK 1
#define CA_LINKS_NOT_OK 2

struct rpvtStruct {
	CALLBACK	doOutCb;
	WDOG_ID		wd_id_0;
	CALLBACK	checkLinkCb;
	WDOG_ID		wd_id_1;
	short		wd_id_1_LOCK;
	short		caLinkStat; /* NO_CA_LINKS,CA_LINKS_ALL_OK,CA_LINKS_NOT_OK */
	short		outlink_field_type;
};

static void alarm();
static void monitor();
static int fetch_values();
static void execOutput();
static void doOutputCallback();
static void checkLinks();
static void checkLinksCallback();

volatile int    sCalcoutRecordDebug = 0;

#define ARG_MAX 12
#define STRING_ARG_MAX 12
/*
 * Strings defined in the .dbd file are assumed to be of length STRING_SIZE.
 * Strings implemented in the .dbd file with a char * pointer (for which space
 * is allocated in init_record) are known to be of this length.
 */
#define STRING_SIZE 40


static long init_record(pcalc,pass)
	struct scalcoutRecord	*pcalc;
	int pass;
{
	struct link *plink;
	int i;
	double *pvalue;
	unsigned short *plinkValid;
	short error_number;
	char *s, **ps;

	struct dbAddr       dbAddr;
	struct dbAddr       *pAddr = &dbAddr;
	struct rpvtStruct   *prpvt;

	if (pass==0) {
		pcalc->vers = VERSION;
		pcalc->rpvt = (void *)calloc(1, sizeof(struct rpvtStruct));
		/* allocate space for previous-value strings */
		s = (char *)calloc(STRING_ARG_MAX, STRING_SIZE);
		for (i=0, ps=(char **)&(pcalc->paa); i<STRING_ARG_MAX; i++, ps++)
			*ps = &s[i*STRING_SIZE];
		/* allocate and fill in array of pointers to strings AA... */
		pcalc->strs = (char **)calloc(STRING_ARG_MAX, sizeof(char *));
		if (sCalcoutRecordDebug) printf("sCalcoutRecord:init_record: strs=%p\n",
			pcalc->strs);
		s = (char *)&(pcalc->aa);
		ps = (char **)(pcalc->strs);
		for (i=0; i<STRING_ARG_MAX; i++, s+=STRING_SIZE, ps++)
			*ps = s;

		return(0);
	}
    
	prpvt = (struct rpvtStruct *)pcalc->rpvt;

	plink = &pcalc->inpa;
	pvalue = &pcalc->a;
	plinkValid = &pcalc->inav;
	for (i=0; i<(ARG_MAX+STRING_ARG_MAX+1); i++, plink++, pvalue++, plinkValid++) {
		if (plink->type == CONSTANT) {
			/* Don't InitConstantLink the string links or the output link. */
			if (i < ARG_MAX) { 
				recGblInitConstantLink(plink,DBF_DOUBLE,pvalue);
				db_post_events(pcalc,pvalue,DBE_VALUE);
			}
			*plinkValid = scalcoutINAV_CON;
			if (plink == &pcalc->out)
				prpvt->outlink_field_type = DBF_NOACCESS;
        } else if (!dbNameToAddr(plink->value.pv_link.pvname, pAddr)) {
			/* see if the PV resides on this ioc */
			*plinkValid = scalcoutINAV_LOC;
			if (plink == &pcalc->out)
				prpvt->outlink_field_type = pAddr->field_type;
			if (sCalcoutRecordDebug && (pAddr->field_type >= DBF_INLINK) &&
					(pAddr->field_type <= DBF_FWDLINK)) {
				s = strchr(plink->value.pv_link.pvname, (int)' ') + 1;
				if (strncmp(s,"CA",2)) printf("sCalcoutRecord(%s):init_record:dblink to link field\n", pcalc->name);
			}
		} else {
			/* pv is not on this ioc. Callback later for connection stat */
			*plinkValid = scalcoutINAV_EXT_NC;
			prpvt->caLinkStat = CA_LINKS_NOT_OK;
			if (plink == &pcalc->out)
				prpvt->outlink_field_type = DBF_NOACCESS; /* don't know field type */
		}
		db_post_events(pcalc,plinkValid,DBE_VALUE);
	}

	pcalc->clcv = sCalcPostfix(pcalc->calc,pcalc->rpcl,&error_number);
	if (pcalc->clcv) {
		recGblRecordError(S_db_badField,(void *)pcalc,
			"scalcout: init_record: Illegal CALC field");
		printf("sCalcPostfix returns: %d\n", error_number);
	}
	db_post_events(pcalc,&pcalc->clcv,DBE_VALUE);

	pcalc->oclv = sCalcPostfix(pcalc->ocal,(char *)pcalc->orpc,&error_number);
	if (pcalc->oclv) {
		recGblRecordError(S_db_badField,(void *)pcalc,
			"scalcout: init_record: Illegal OCAL field");
		printf("sCalcPostfix returns: %d\n", error_number);
	}
	db_post_events(pcalc,&pcalc->oclv,DBE_VALUE);

	callbackSetCallback(doOutputCallback, &prpvt->doOutCb);
	callbackSetPriority(pcalc->prio, &prpvt->doOutCb);
	callbackSetUser(pcalc, &prpvt->doOutCb);
	callbackSetCallback(checkLinksCallback, &prpvt->checkLinkCb);
	callbackSetPriority(0, &prpvt->checkLinkCb);
	callbackSetUser(pcalc, &prpvt->checkLinkCb);
	prpvt->wd_id_0 = wdCreate();
	prpvt->wd_id_1 = wdCreate();
	prpvt->wd_id_1_LOCK = 0;

	if (prpvt->caLinkStat == CA_LINKS_NOT_OK) {
		wdStart(prpvt->wd_id_1, 60, (FUNCPTR)callbackRequest,
			(int)(&prpvt->checkLinkCb));
		prpvt->wd_id_1_LOCK = 1;
	}

    return(0);
}

static long process(pcalc)
    struct scalcoutRecord     *pcalc;
{
	struct rpvtStruct   *prpvt = (struct rpvtStruct *)pcalc->rpvt;
	int		wdDelay;
	short	doOutput = 0;
	long	stat;

	if (sCalcoutRecordDebug) printf("sCalcoutRecord:process: strs=%p\n", pcalc->strs);
	if (pcalc->pact) {
		execOutput(pcalc);
		pcalc->dlya = 0;
		db_post_events(pcalc,&pcalc->dlya,DBE_VALUE);

		/* process the forward scan link record */
		recGblFwdLink(pcalc);

		pcalc->pact = FALSE;
		return(0);
	}


	pcalc->pact = TRUE;
	/* if some links are CA, check connections */
	if (prpvt->caLinkStat != NO_CA_LINKS) checkLinks(pcalc);

	if (fetch_values(pcalc)==0) {
		stat = sCalcPerform(&pcalc->a, ARG_MAX, (char **)(pcalc->strs),
				STRING_ARG_MAX, &pcalc->val, pcalc->sval, STRING_SIZE,
				pcalc->rpcl);
		if (stat)
			recGblSetSevr(pcalc,CALC_ALARM,INVALID_ALARM);
		else
			pcalc->udf = FALSE;
	}
	recGblGetTimeStamp(pcalc);
	/* check for alarms */
	alarm(pcalc);

	/* check for output link execution */
	switch (pcalc->oopt) {
	case scalcoutOOPT_Every_Time:
		doOutput = 1;
		break;
	case scalcoutOOPT_On_Change:
		if (fabs(pcalc->pval - pcalc->val) > pcalc->mdel) doOutput = 1;
		break;
	case scalcoutOOPT_Transition_To_Zero:
		if ((pcalc->pval != 0) && (pcalc->val == 0)) doOutput = 1;
		break;         
	case scalcoutOOPT_Transition_To_Non_zero:
		if ((pcalc->pval == 0) && (pcalc->val != 0)) doOutput = 1;
		break;
	case scalcoutOOPT_When_Zero:
		if (!pcalc->val) doOutput = 1;
		break;
	case scalcoutOOPT_When_Non_zero:
		if (pcalc->val) doOutput = 1;
		break;
	case scalcoutOOPT_Never:
		doOutput = 0;
		break;
	}
	pcalc->pval = pcalc->val;

	if (doOutput) {
		if (pcalc->odly > 0.0) {
			pcalc->dlya = 1;
			db_post_events(pcalc,&pcalc->dlya,DBE_VALUE);
			wdDelay = pcalc->odly * sysClkRateGet();
			callbackSetPriority(pcalc->prio, &prpvt->doOutCb);
			wdStart(prpvt->wd_id_0, wdDelay, (FUNCPTR)callbackRequest,
				(int)(&prpvt->doOutCb));
		} else {
			execOutput(pcalc);
		}
	}

	/* check event list */
	monitor(pcalc);

	/* if no delay requested, finish processing */ 
	if (!pcalc->dlya) {
		/* process the forward scan link record */
		recGblFwdLink(pcalc);
		pcalc->pact = FALSE;
	}
	return(0);
}

static long special(paddr,after)
    struct dbAddr	*paddr;
    int				after;
{
	struct scalcoutRecord *pcalc = (struct scalcoutRecord *)(paddr->precord);
	struct rpvtStruct   *prpvt = (struct rpvtStruct *)pcalc->rpvt;
	struct dbAddr       dbAddr;
	struct dbAddr       *pAddr = &dbAddr;
	short error_number;
	int                 fieldIndex = dbGetFieldIndex(paddr);
	int                 lnkIndex;
	struct link         *plink;
	double              *pvalue;
	unsigned short      *plinkValid;
	char				*s;

	if (!after) return(0);
	switch (fieldIndex) {
	case scalcoutRecordCALC:
		pcalc->clcv = sCalcPostfix(pcalc->calc, pcalc->rpcl, &error_number);
		if (pcalc->clcv) {
			recGblRecordError(S_db_badField,(void *)pcalc,
				"scalcout: special(): Illegal CALC field");
			printf("sCalcPostfix returns: %d\n", error_number);
		}
		db_post_events(pcalc,&pcalc->clcv,DBE_VALUE);
		return(0);
		break;

	case scalcoutRecordOCAL:
		pcalc->oclv = sCalcPostfix(pcalc->ocal, (char *)pcalc->orpc, &error_number);
		if (pcalc->oclv) {
			recGblRecordError(S_db_badField,(void *)pcalc,
				"scalcout: special(): Illegal OCAL field");
			printf("sCalcPostfix returns: %d\n", error_number);
		}
		db_post_events(pcalc,&pcalc->oclv,DBE_VALUE);
		return(0);
		break;

	case(scalcoutRecordINPA):
	case(scalcoutRecordINPB):
	case(scalcoutRecordINPC):
	case(scalcoutRecordINPD):
	case(scalcoutRecordINPE):
	case(scalcoutRecordINPF):
	case(scalcoutRecordINPG):
	case(scalcoutRecordINPH):
	case(scalcoutRecordINPI):
	case(scalcoutRecordINPJ):
	case(scalcoutRecordINPK):
	case(scalcoutRecordINPL):
	case(scalcoutRecordINAA):
	case(scalcoutRecordINBB):
	case(scalcoutRecordINCC):
	case(scalcoutRecordINDD):
	case(scalcoutRecordINEE):
	case(scalcoutRecordINFF):
	case(scalcoutRecordINGG):
	case(scalcoutRecordINHH):
	case(scalcoutRecordINII):
	case(scalcoutRecordINJJ):
	case(scalcoutRecordINKK):
	case(scalcoutRecordINLL):
	case(scalcoutRecordOUT):
		lnkIndex = fieldIndex - scalcoutRecordINPA;
		plink   = &pcalc->inpa + lnkIndex;
		pvalue  = &pcalc->a    + lnkIndex;
		plinkValid = &pcalc->inav + lnkIndex;

		if (plink->type == CONSTANT) {
			if (fieldIndex <= scalcoutRecordINPL) {
				recGblInitConstantLink(plink,DBF_DOUBLE,pvalue);
				db_post_events(pcalc,pvalue,DBE_VALUE);
			}
			*plinkValid = scalcoutINAV_CON;
			if (fieldIndex == scalcoutRecordOUT)
				prpvt->outlink_field_type = DBF_NOACCESS;
		} else if (!dbNameToAddr(plink->value.pv_link.pvname, pAddr)) {
			/* PV resides on this ioc */
			*plinkValid = scalcoutINAV_LOC;
			if (fieldIndex == scalcoutRecordOUT) {
				prpvt->outlink_field_type = pAddr->field_type;
			if (sCalcoutRecordDebug && (pAddr->field_type >= DBF_INLINK) &&
					(pAddr->field_type <= DBF_FWDLINK)) {
				s = strchr(plink->value.pv_link.pvname, (int)' ') + 1;
				if (strncmp(s,"CA",2)) printf("sCalcoutRecord:special:dblink to link field\n");
			}
			}
		} else {
			/* pv is not on this ioc. Callback later for connection stat */
			*plinkValid = scalcoutINAV_EXT_NC;
			/* DO_CALLBACK, if not already scheduled */
			if (!prpvt->wd_id_1_LOCK) {
				wdStart(prpvt->wd_id_1, 30, (FUNCPTR)callbackRequest,
					(int)(&prpvt->checkLinkCb));
				prpvt->wd_id_1_LOCK = 1;
				prpvt->caLinkStat = CA_LINKS_NOT_OK;
			}
			if (fieldIndex == scalcoutRecordOUT)
				prpvt->outlink_field_type = DBF_NOACCESS; /* don't know */
		}
        db_post_events(pcalc,plinkValid,DBE_VALUE);
		return(0);
		break;

	default:
		recGblDbaddrError(S_db_badChoice,paddr,"calc: special");
		return(S_db_badChoice);
	}
	return(0);
}

static long 
cvt_dbaddr(struct dbAddr *paddr)
{
	scalcoutRecord	*pcalc = (scalcoutRecord *) paddr->precord;
	char			**pfield = (char **)paddr->pfield;
	char			**paa = (char **)&(pcalc->paa);
	short			i;
    int fieldIndex = dbGetFieldIndex(paddr);

	if (sCalcoutRecordDebug > 5) printf("sCalcout: cvt_dbaddr: paddr->pfield = %p\n",
		(void *)paddr->pfield);
	if ((fieldIndex>=scalcoutRecordPAA) && (fieldIndex<=scalcoutRecordPLL)) {
		i = pfield - paa;
		paddr->pfield = paa[i];
		paddr->no_elements = STRING_SIZE;
	}
	paddr->field_type = DBF_STRING;
	paddr->field_size = STRING_SIZE;
	paddr->dbr_field_type = DBR_STRING;
	return(0);
}

static long get_units(paddr,units)
    struct dbAddr *paddr;
    char	  *units;
{
	struct scalcoutRecord	*pcalc=(struct scalcoutRecord *)paddr->precord;

	strncpy(units,pcalc->egu,DB_UNITS_SIZE);
	return(0);
}

static long get_precision(paddr,precision)
	struct dbAddr *paddr;
	long	  *precision;
{
	struct scalcoutRecord	*pcalc=(struct scalcoutRecord *)paddr->precord;
	int fieldIndex = dbGetFieldIndex(paddr);

	*precision = pcalc->prec;
	if (fieldIndex == scalcoutRecordVAL) return(0);
	recGblGetPrec(paddr,precision);
	return(0);
}

static long get_graphic_double(paddr,pgd)
    struct dbAddr *paddr;
    struct dbr_grDouble	*pgd;
{
    struct scalcoutRecord	*pcalc=(struct scalcoutRecord *)paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

	switch (fieldIndex) {
	case scalcoutRecordVAL:
	case scalcoutRecordHIHI:
	case scalcoutRecordHIGH:
	case scalcoutRecordLOW:
	case scalcoutRecordLOLO:
		pgd->upper_disp_limit = pcalc->hopr;
		pgd->lower_disp_limit = pcalc->lopr;
		return(0);
	default:
		break;
	} 

	if (fieldIndex >= scalcoutRecordA && fieldIndex <= scalcoutRecordL) {
		pgd->upper_disp_limit = pcalc->hopr;
		pgd->lower_disp_limit = pcalc->lopr;
		return(0);
	}
	if (fieldIndex >= scalcoutRecordPA && fieldIndex <= scalcoutRecordPL) {
		pgd->upper_disp_limit = pcalc->hopr;
		pgd->lower_disp_limit = pcalc->lopr;
		return(0);
	}
	return(0);
}

static long get_control_double(paddr,pcd)
	struct dbAddr *paddr;
	struct dbr_ctrlDouble *pcd;
{
	struct scalcoutRecord	*pcalc=(struct scalcoutRecord *)paddr->precord;
	int fieldIndex = dbGetFieldIndex(paddr);

	switch (fieldIndex) {
	case scalcoutRecordVAL:
	case scalcoutRecordHIHI:
	case scalcoutRecordHIGH:
	case scalcoutRecordLOW:
	case scalcoutRecordLOLO:
		pcd->upper_ctrl_limit = pcalc->hopr;
		pcd->lower_ctrl_limit = pcalc->lopr;
		return(0);
	default:
		break;
    } 

	if (fieldIndex >= scalcoutRecordA && fieldIndex <= scalcoutRecordL) {
		pcd->upper_ctrl_limit = pcalc->hopr;
		pcd->lower_ctrl_limit = pcalc->lopr;
		return(0);
	}
	if (fieldIndex >= scalcoutRecordPA && fieldIndex <= scalcoutRecordPL) {
		pcd->upper_ctrl_limit = pcalc->hopr;
		pcd->lower_ctrl_limit = pcalc->lopr;
		return(0);
	}
	return(0);
}
static long get_alarm_double(paddr,pad)
	struct dbAddr *paddr;
	struct dbr_alDouble	*pad;
{
	struct scalcoutRecord	*pcalc=(struct scalcoutRecord *)paddr->precord;
	int fieldIndex = dbGetFieldIndex(paddr);

	if (fieldIndex == scalcoutRecordVAL) {
		pad->upper_alarm_limit = pcalc->hihi;
		pad->upper_warning_limit = pcalc->high;
		pad->lower_warning_limit = pcalc->low;
		pad->lower_alarm_limit = pcalc->lolo;
	} else
		 recGblGetAlarmDouble(paddr,pad);
	return(0);
}


static void alarm(pcalc)
	struct scalcoutRecord	*pcalc;
{
	double		val;
	float		hyst, lalm, hihi, high, low, lolo;
	unsigned short	hhsv, llsv, hsv, lsv;

	if (pcalc->udf == TRUE) {
		recGblSetSevr(pcalc,UDF_ALARM,INVALID_ALARM);
		return;
	}
	hihi = pcalc->hihi; 
	lolo = pcalc->lolo; 
	high = pcalc->high;  
	low = pcalc->low;
	hhsv = pcalc->hhsv; 
	llsv = pcalc->llsv; 
	hsv = pcalc->hsv; 
	lsv = pcalc->lsv;
	val = pcalc->val; 
	hyst = pcalc->hyst; 
	lalm = pcalc->lalm;

	/* alarm condition hihi */
	if (hhsv && (val >= hihi || ((lalm==hihi) && (val >= hihi-hyst)))) {
		if (recGblSetSevr(pcalc,HIHI_ALARM,pcalc->hhsv)) pcalc->lalm = hihi;
		return;
	}

	/* alarm condition lolo */
	if (llsv && (val <= lolo || ((lalm==lolo) && (val <= lolo+hyst)))) {
		if (recGblSetSevr(pcalc,LOLO_ALARM,pcalc->llsv)) pcalc->lalm = lolo;
		return;
	}

	/* alarm condition high */
	if (hsv && (val >= high || ((lalm==high) && (val >= high-hyst)))) {
		if (recGblSetSevr(pcalc,HIGH_ALARM,pcalc->hsv)) pcalc->lalm = high;
		return;
	}

	/* alarm condition low */
	if (lsv && (val <= low || ((lalm==low) && (val <= low+hyst)))) {
		if (recGblSetSevr(pcalc,LOW_ALARM,pcalc->lsv)) pcalc->lalm = low;
		return;
	}

	/* we get here only if val is out of alarm by at least hyst */
	pcalc->lalm = val;
	return;
}

static void doOutputCallback(pcallback)
	struct callback *pcallback;
{

	dbCommon    *pcalc;
	struct rset *prset;

	callbackGetUser(pcalc, pcallback);
	prset = (struct rset *)pcalc->rset;

	dbScanLock((struct dbCommon *)pcalc);
	(*prset->process)(pcalc);
	dbScanUnlock((struct dbCommon *)pcalc);
}

    

static void execOutput(pcalc)
	struct scalcoutRecord *pcalc;
{
	long	status;
	struct rpvtStruct   *prpvt = (struct rpvtStruct *)pcalc->rpvt;
	short	ftype = prpvt->outlink_field_type;
	/* Determine output data */
	switch (pcalc->dopt) {
	case scalcoutDOPT_Use_VAL:
		pcalc->oval = pcalc->val;
		strcpy(pcalc->osv, pcalc->sval);
		break;

	case scalcoutDOPT_Use_OVAL:
		if (sCalcPerform(&pcalc->a, ARG_MAX, (char **)(pcalc->strs),
				STRING_ARG_MAX, &pcalc->oval, pcalc->osv, STRING_SIZE,
				(char *)pcalc->orpc)) {
			recGblSetSevr(pcalc,CALC_ALARM,INVALID_ALARM);
		}
		break;
	}

	/* Check to see what to do if INVALID */
	if (pcalc->sevr < INVALID_ALARM) {
		/* Output the value */
		switch (ftype) {
		case DBF_STRING: case DBF_ENUM: case DBF_MENU: case DBF_DEVICE:
		case DBF_INLINK: case DBF_OUTLINK: case DBF_FWDLINK:
			status = dbPutLink(&(pcalc->out), DBR_STRING,&(pcalc->osv),1);
			break;
		default:
			status = dbPutLink(&(pcalc->out), DBR_DOUBLE,&(pcalc->oval),1);
			break;
		}
		/* post event if output event != 0 */
		if (pcalc->oevt > 0) post_event((int)pcalc->oevt);
	} else {
		switch (pcalc->ivoa) {
		case menuIvoaContinue_normally:
			/* write the new value */
			switch (prpvt->outlink_field_type) {
			case DBF_STRING: case DBF_ENUM: case DBF_MENU: case DBF_DEVICE:
			case DBF_INLINK: case DBF_OUTLINK: case DBF_FWDLINK:
				status = dbPutLink(&(pcalc->out), DBR_STRING,&(pcalc->osv),1);
				break;
			default:
				status = dbPutLink(&(pcalc->out), DBR_DOUBLE, &(pcalc->oval),1);
				break;
			}
			/* post event if output event != 0 */
			if (pcalc->oevt > 0) post_event((int)pcalc->oevt);
			break;

		case menuIvoaDon_t_drive_outputs:
			break;

		case menuIvoaSet_output_to_IVOV:
			pcalc->oval=pcalc->ivov;
			status = dbPutLink(&(pcalc->out), DBR_DOUBLE, &(pcalc->oval),1);
			/* post event if output event != 0 */
			if (pcalc->oevt > 0) post_event((int)pcalc->oevt);
			break;

		default:
			status=-1;
			recGblRecordError(S_db_badField,(void *)pcalc,
				"scalcout:process Illegal IVOA field");
		}
	} 
}

static void monitor(pcalc)
    struct scalcoutRecord	*pcalc;
{
	unsigned short	monitor_mask;
	double			delta;
	double			*pnew, *pprev;
	char			**psnew, **psprev;
	int				i;

	monitor_mask = recGblResetAlarms(pcalc);
	/* check for value change */
	delta = pcalc->mlst - pcalc->val;
	if (delta < 0.0) delta = -delta;
	if (delta > pcalc->mdel) {
		/* post events for value change */
		monitor_mask |= DBE_VALUE;
		/* update last value monitored */
		pcalc->mlst = pcalc->val;
	}
	/* check for archive change */
	delta = pcalc->alst - pcalc->val;
	if (delta < 0.0) delta = -delta;
	if (delta > pcalc->adel) {
		/* post events on value field for archive change */
		monitor_mask |= DBE_LOG;
		/* update last archive value monitored */
		pcalc->alst = pcalc->val;
	}
	/* send out monitors connected to the value field */
	if (monitor_mask) db_post_events(pcalc,&pcalc->val,monitor_mask);

	if (strcmp(pcalc->sval, pcalc->psvl)) {
		db_post_events(pcalc, pcalc->sval, monitor_mask|DBE_VALUE|DBE_LOG);
		strcpy(pcalc->psvl, pcalc->sval);
	}
	if (strcmp(pcalc->osv, pcalc->posv)) {
		db_post_events(pcalc, pcalc->osv, monitor_mask|DBE_VALUE|DBE_LOG);
		strcpy(pcalc->posv, pcalc->osv);
	}

	/* check all input fields for changes */
	for (i=0, pnew=&pcalc->a, pprev=&pcalc->pa; i<ARG_MAX;  i++, pnew++, pprev++) {
		if ((*pnew != *pprev) || (monitor_mask&DBE_ALARM)) {
			db_post_events(pcalc,pnew,monitor_mask|DBE_VALUE|DBE_LOG);
			*pprev = *pnew;
		}
	}
	for (i=0, psnew=pcalc->strs, psprev=&pcalc->paa; i<STRING_ARG_MAX;
			i++, psnew++, psprev++) {
		if (strcmp(*psnew, *psprev)) {
			db_post_events(pcalc, *psnew, monitor_mask|DBE_VALUE|DBE_LOG);
			strcpy(*psprev, *psnew);
		}
	}
	/* Check OVAL field */
	if (pcalc->povl != pcalc->oval) {
		db_post_events(pcalc,&pcalc->oval, monitor_mask|DBE_VALUE|DBE_LOG);
		pcalc->povl = pcalc->oval;
	}
	return;
}

static int fetch_values(pcalc)
    struct scalcoutRecord *pcalc;
{
	struct link	*plink;	/* structure of the link field  */
	double		*pvalue;
	char		**psvalue;
	long		status = 0;
	int			i;
	TS_STAMP	timeStamp;

	for (i=0, plink=&pcalc->inpa, pvalue=&pcalc->a; i<ARG_MAX; 
			i++, plink++, pvalue++) {
		status = dbGetLink(plink, DBR_DOUBLE, pvalue, 0, 0);
		if (!RTN_SUCCESS(status)) return(status);
	}

	for (i=0, plink=&pcalc->inaa, psvalue=pcalc->strs; i<STRING_ARG_MAX; 
			i++, plink++, psvalue++) {
		status = dbGetLink(plink, DBR_STRING, *psvalue, 0, 0);
#if 0
		if (!RTN_SUCCESS(status)) {
			/* might be a time value */
			status = dbGetLink(plink, DBR_TIME, &timeStamp, 0, 0); /* doesn't work */
			if (!RTN_SUCCESS(status)) {
				TSgetTimeStamp((int) pcalc->tse, (struct timespec *) &timeStamp); /* works */
			}
			tsStampToText(&timeStamp, TS_TEXT_MMDDYY, *psvalue);
		}
#endif
		if (!RTN_SUCCESS(status)) {strcpy(*psvalue, "Huh?");}
	}
	return(0);
}

static void checkLinksCallback(pcallback)
    struct callback *pcallback;
{

    struct scalcoutRecord *pcalc;
    struct rpvtStruct   *prpvt;

    callbackGetUser(pcalc, pcallback);
    prpvt = (struct rpvtStruct *)pcalc->rpvt;
    
	if (!interruptAccept) {
		/* Can't call dbScanLock yet.  Schedule another CALLBACK */
		prpvt->wd_id_1_LOCK = 1;  /* make sure */
		wdStart(prpvt->wd_id_1, 30, (FUNCPTR)callbackRequest,
			(int)(&prpvt->checkLinkCb));
	} else {
	    dbScanLock((struct dbCommon *)pcalc);
	    prpvt->wd_id_1_LOCK = 0;
	    checkLinks(pcalc);
	    dbScanUnlock((struct dbCommon *)pcalc);
	}
}


static void checkLinks(pcalc)
    struct scalcoutRecord *pcalc;
{
	struct link *plink;
	struct rpvtStruct   *prpvt = (struct rpvtStruct *)pcalc->rpvt;
	int i;
	int isCaLink   = 0;
	int isCaLinkNc = 0;
	unsigned short *plinkValid;
	struct dbAddr dbAddr;
	struct dbAddr *pAddr = &dbAddr;
	char *s;

	if (sCalcoutRecordDebug) printf("checkLinks() for %p\n", pcalc);

	plink   = &pcalc->inpa;
	plinkValid = &pcalc->inav;

	for (i=0; i<ARG_MAX+STRING_ARG_MAX+1; i++, plink++, plinkValid++) {
		if (plink->type == CA_LINK) {
			isCaLink = 1;
			if (dbCaIsLinkConnected(plink)) {
				if (*plinkValid == scalcoutINAV_EXT_NC) {
					*plinkValid = scalcoutINAV_EXT;
					db_post_events(pcalc,plinkValid,DBE_VALUE);
				}
				/* if outlink, get type of field we're connected to */
				if (plink == &pcalc->out) {
					prpvt->outlink_field_type = dbCaGetLinkDBFtype(plink);
					if (sCalcoutRecordDebug) {
						printf("sCalcout:checkLinks: outlink type = %d\n",
							prpvt->outlink_field_type);
						if (!dbNameToAddr(plink->value.pv_link.pvname, pAddr)) {
							if ((pAddr->field_type >= DBF_INLINK) &&
									(pAddr->field_type <= DBF_FWDLINK)) {
								s = strchr(plink->value.pv_link.pvname, (int)' ') + 1;
								if (strncmp(s,"CA",2))
									printf("sCalcoutRecord:checkLinks:dblink to link field\n");
							}
						}
					}
				}
			} else {
				if (*plinkValid == scalcoutINAV_EXT_NC) {
					isCaLinkNc = 1;
				}
				else if (*plinkValid == scalcoutINAV_EXT) {
					*plinkValid = scalcoutINAV_EXT_NC;
					db_post_events(pcalc,plinkValid,DBE_VALUE);
					isCaLinkNc = 1;
				}
				if (plink == &pcalc->out)
					prpvt->outlink_field_type = DBF_NOACCESS; /* don't know type */
			} 
		}
	}
	if (isCaLinkNc)
		prpvt->caLinkStat = CA_LINKS_NOT_OK;
	else if (isCaLink)
		prpvt->caLinkStat = CA_LINKS_ALL_OK;
	else
		prpvt->caLinkStat = NO_CA_LINKS;

	if (!prpvt->wd_id_1_LOCK && isCaLinkNc) {
		/* Schedule another CALLBACK */
		prpvt->wd_id_1_LOCK = 1;
		wdStart(prpvt->wd_id_1, 30, (FUNCPTR)callbackRequest,
			(int)(&prpvt->checkLinkCb));
	}
}

