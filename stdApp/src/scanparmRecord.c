/* scanparmRecord.c - Record Support Routines for Scanparm record */
/*
 *	  Original Author: Tim Mooney & Xingyue Li
 *	  Date:			2-21-96
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
 *	  and
 *			  The Beamline Controls & Data Acquisition Group
 *			  Experimental Facilities Division
 *			  Advanced Photon Source
 *			  Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  02-21-96  xli initial development
 * .02  06-06-96  tmm continued initial development
 * .03  08-16-96  tmm v2.0: conversion to EPICS 3.13
 * .04  06-11-99  tmm v2.1: clear load and go always; debug output
 * .05  06-12-00  tmm v3.0: add det PV support
 */

#define VERSION 3.0



#include	<string.h>
#include        <stdio.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<special.h>
#include	<recSup.h>
#include	<recGbl.h>
#define GEN_SIZE_OFFSET
#include	"scanparmRecord.h"
#undef GEN_SIZE_OFFSET
#include        <epicsExport.h>

volatile int scanparmRecordDebug=0;

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
#define special NULL
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

struct rset scanparmRSET = {
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
epicsExportAddress(rset, scanparmRSET);

static void monitor();

static long init_record(psr,pass)
scanparmRecord *psr;
int pass;
{
	if (pass==0) {
		psr->vers = VERSION;
		return(0);
	}

	/* Initialize from any constant inlinks */
	if (psr->imp.type == CONSTANT) {
		recGblInitConstantLink(&psr->imp,DBF_SHORT,&psr->mp);
	}

	if (psr->np < 2){
		psr->np = 2;
		db_post_events(psr, &psr->np, DBE_VALUE);
	}
	psr->lstp = (psr->ep - psr->sp)/(psr->np - 1); /* Previous stepSize */
	return(0);
}


static long process(psr)
scanparmRecord *psr;
{
	long status=0;

	status = dbGetLink(&(psr->iact), DBR_SHORT, &(psr->act), NULL, NULL);
	if (status) return(status);
	if (scanparmRecordDebug) printf("scanparm(%s):process:act=%d\n",
		psr->name, psr->act);

	status = dbGetLink(&(psr->imp), DBR_SHORT, &(psr->mp), NULL, NULL);
	if (scanparmRecordDebug) printf("scanparm(%s):process:mp=%d\n", psr->name, psr->mp);
	if (status) return(status);

	if (psr->np > psr->mp){
		psr->np = psr->mp;
		db_post_events(psr, &psr->np, DBE_VALUE);
	}
	else if (psr->np < 2){
		psr->np = 2;
		db_post_events(psr, &psr->np, DBE_VALUE);
	}

	psr->step = (psr->ep - psr->sp)/(psr->np - 1);

	if (!(psr->act)) {
		if (psr->load || psr->go) {
			status = dbPutLink(&(psr->opre), DBR_SHORT, &(psr->pre), 1);
			status = dbPutLink(&(psr->oppv), DBR_STRING, &(psr->ppv), 1);
			status = dbPutLink(&(psr->orpv), DBR_STRING, &(psr->rpv), 1);
			status = dbPutLink(&(psr->otpv), DBR_STRING, &(psr->tpv), 1);
			status = dbPutLink(&(psr->odpv), DBR_STRING, &(psr->dpv), 1);
			status = dbPutLink(&(psr->osm), DBR_ENUM, &(psr->sm), 1);
			status = dbPutLink(&(psr->osp), DBR_DOUBLE, &(psr->sp), 1);
			status = dbPutLink(&(psr->oep), DBR_DOUBLE, &(psr->ep), 1);
			status = dbPutLink(&(psr->onp), DBR_SHORT, &(psr->np), 1);
			status = dbPutLink(&(psr->oar), DBR_ENUM, &(psr->ar), 1);
			status = dbPutLink(&(psr->oaft), DBR_ENUM, &(psr->aft), 1);
			if (status) return(status);

			if (psr->go) {
				if (scanparmRecordDebug) {
					printf("scanparm(%s):process:starting scan\n", psr->name);
				}
				status = dbPutLink(&(psr->oaqt), DBR_DOUBLE, &(psr->aqt), 1);
				status = dbPutLink(&(psr->osc), DBR_SHORT, &(psr->sc), 1);
				if (status) return(status);
			}
		}
	}

	if (psr->go) {
		psr->go = 0;
		db_post_events(psr, &psr->go, DBE_VALUE);
	}

	if (psr->load) {
		psr->load = 0;
		db_post_events(psr, &psr->load, DBE_VALUE);
	}

	psr->pact = TRUE;
	recGblGetTimeStamp(psr);

	/* check event list */
	monitor(psr);

	/* process the forward scan link record */
	recGblFwdLink(psr);

	psr->pact=FALSE;
	return(status);
}

static void monitor(psr)
scanparmRecord *psr;
{
	unsigned short monitor_mask;
	
	if (psr->step != psr->lstp) {
		monitor_mask = recGblResetAlarms(psr);
		db_post_events(psr, &psr->step, monitor_mask|DBE_VALUE);
		psr->lstp = psr->step;
	}
}

static long get_precision(paddr,precision)
struct dbAddr *paddr;
long *precision;
{
	scanparmRecord *psr=(scanparmRecord *)paddr->precord;

	*precision = psr->prec;
	if (paddr->pfield == (void *)&psr->vers) {
		*precision = 2;
	}
	else {
		recGblGetPrec(paddr,precision);
	}
	return(0);
}
