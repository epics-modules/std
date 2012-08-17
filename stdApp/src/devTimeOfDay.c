/* devTimeOfDay.c  - Device Support Routines */
/*
 *      Original Author: Ned D. Arnold
 *      Date:            04-12-96
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
 */

/* This device support for a string_in record reads the latest time 
 * and converts it into a string. The time is truncated to the nearest
 * second. Both timestamp formats are available, based on the .PHAS field
 * as follows :
 *                0 - TS_TEXT_MONDDYYYY  Mon dd, yyyy hh:mm:ss
 *                1 - TS_TEXT_MMDDYY     mm/dd/yy hh:mm:ss
 *
 * This device support for an ai record captures secPastEpoch.  If the
 * ai record's PHAS field is nonzero, it also captures the fractional part
 * of a second.
 *
 * Modification Log:
 * -----------------
 * .01  04-12-96        nda     initial coding           
 * .02  02-07-97        nda     added devAi support for timestamp seconds
 *      12-13-06        tmm     added fractional seconds for ai records
 *      ...
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "alarm.h"
#include "callback.h"
#include "cvtTable.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "recGbl.h"
#include "recSup.h"
#include "devSup.h"
#include "link.h"
#include "dbCommon.h"
#include "stringinRecord.h"
#include "aiRecord.h"
#include "epicsEvent.h"
#include "epicsSignal.h"
#include "epicsString.h"
#include "epicsThread.h"
#include "epicsTimer.h"
#include "epicsExport.h"


static long init_record();
static long createString();
static long aiReadTs();
static long no_op();


typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_write;  
        DEVSUPFUN       special_linconv;
} TOD_DSET;


/* define different DSET's for different input link types */

TOD_DSET devSiTodString = { 5, NULL, NULL,init_record,NULL,createString,NULL };
TOD_DSET devAiTodSeconds= { 6, NULL, NULL,init_record,NULL,aiReadTs,no_op};
epicsExportAddress(dset,devSiTodString);
epicsExportAddress(dset,devAiTodSeconds);

static long init_record(psi)
    struct stringinRecord	*psi;
{
    long  status = 0;

    return(status);
}


static long no_op(pai)
    struct aiRecord       *pai;
{
    long  status = 0;

    return(status);
}

static long createString(psi)
    struct stringinRecord       *psi;
{


        char     *pdot;
       
        recGblGetTimeStamp(psi);	/* get time stamp NOW */
        if(psi->phas) {
			epicsTimeToStrftime(psi->val, 28, "%m/%d/%y %H:%M:%S.%09f", &psi->time);
        }
        else {
			epicsTimeToStrftime(psi->val, 32, "%b %d, %Y %H:%M:%S.%09f", &psi->time);
        }

        /* truncate string to seconds */
        pdot = strrchr(psi->val, '.');
        if((pdot!=NULL)) {
            *pdot = '\0';
        }

        psi->udf = 0;
	
	return(0);
}


static long aiReadTs(pai)
    struct aiRecord       *pai;
{

        recGblGetTimeStamp(pai);        /* get time stamp NOW */

        pai->val = (double) pai->time.secPastEpoch;
        if(pai->phas) pai->val += (double) pai->time.nsec/1e9;

        pai->udf = 0;

        return(2);
}
