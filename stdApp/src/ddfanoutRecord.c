/* ddfanoutRecord.c */
/*
 * Original Author: 	Matt Bickley   (Sometime in 1994)
 * Current Author:	Johnny Tang
 *
 * Modification Log:
 * -----------------
 * .01  1994        mhb     Started with longout record to make the data fanout
 * .02  May 10, 96  jt	    Bug Fix
 * .03  January, 1998 mlr   Converted dfanout (DBF_LONG) to ddfanout (DBF_DOUBLE)
 */


#include	<vxWorks.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<lstLib.h>
#include	<string.h>

#include        <alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<dbFldTypes.h>
#include	<devSup.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<special.h>
#define GEN_SIZE_OFFSET
#include	<ddfanoutRecord.h>
#undef  GEN_SIZE_OFFSET

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
static long get_units();
static long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
static long get_graphic_double();
static long get_control_double();
static long get_alarm_double();

struct rset ddfanoutRSET={
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
	get_alarm_double };


static void alarm();
static void monitor();
static long push_values();

#define OUT_ARG_MAX 8


static long init_record(pddfanout,pass)
    struct ddfanoutRecord	*pddfanout;
    int pass;
{
    if (pass==0) return(0);

    /* get the initial value dol is a constant*/
    if (pddfanout->dol.type == CONSTANT){
	if(recGblInitConstantLink(&pddfanout->dol,DBF_DOUBLE,&pddfanout->val))
	    pddfanout->udf=FALSE;
    }
    return(0);
}

static long process(pddfanout)
        struct ddfanoutRecord     *pddfanout;
{
    long		 status=0;

    if (!pddfanout->pact && pddfanout->omsl == CLOSED_LOOP){
	status = dbGetLink(&(pddfanout->dol),DBR_DOUBLE,&(pddfanout->val),0,0);
	if(pddfanout->dol.type!=CONSTANT && RTN_SUCCESS(status)) pddfanout->udf=FALSE;
    }
    pddfanout->pact = TRUE;
    recGblGetTimeStamp(pddfanout);
    /* Push out the data to all the forward links */
    status = push_values(pddfanout);
    alarm(pddfanout);
    monitor(pddfanout);
    recGblFwdLink(pddfanout);
    pddfanout->pact=FALSE;
    return(status);
}

static long get_units(paddr,units)
    struct dbAddr *paddr;
    char	  *units;
{
    struct ddfanoutRecord	*pddfanout=(struct ddfanoutRecord *)paddr->precord;

    strncpy(units,pddfanout->egu,DB_UNITS_SIZE);
    return(0);
}

static long get_precision(DBADDR *paddr, long *precision)
{
    ddfanoutRecord   *pddfanout=(ddfanoutRecord *)paddr->precord;

    *precision = pddfanout->prec;
    if(paddr->pfield == (void *)&pddfanout->val) return(0);
    recGblGetPrec(paddr,precision);
    return(0);
}


static long get_graphic_double(paddr,pgd)
    struct dbAddr *paddr;
    struct dbr_grDouble	*pgd;
{
    struct ddfanoutRecord	*pddfanout=(struct ddfanoutRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pddfanout->val
    || paddr->pfield==(void *)&pddfanout->hihi
    || paddr->pfield==(void *)&pddfanout->high
    || paddr->pfield==(void *)&pddfanout->low
    || paddr->pfield==(void *)&pddfanout->lolo){
        pgd->upper_disp_limit = pddfanout->hopr;
        pgd->lower_disp_limit = pddfanout->lopr;
    } else recGblGetGraphicDouble(paddr,pgd);
    return(0);
}

static long get_control_double(paddr,pcd)
    struct dbAddr *paddr;
    struct dbr_ctrlDouble *pcd;
{
    struct ddfanoutRecord	*pddfanout=(struct ddfanoutRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pddfanout->val
    || paddr->pfield==(void *)&pddfanout->hihi
    || paddr->pfield==(void *)&pddfanout->high
    || paddr->pfield==(void *)&pddfanout->low
    || paddr->pfield==(void *)&pddfanout->lolo){
        pcd->upper_ctrl_limit = pddfanout->hopr;
        pcd->lower_ctrl_limit = pddfanout->lopr;
    } else recGblGetControlDouble(paddr,pcd);
    return(0);
}
static long get_alarm_double(paddr,pad)
    struct dbAddr *paddr;
    struct dbr_alDouble	*pad;
{
    struct ddfanoutRecord	*pddfanout=(struct ddfanoutRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pddfanout->val){
         pad->upper_alarm_limit = pddfanout->hihi;
         pad->upper_warning_limit = pddfanout->high;
         pad->lower_warning_limit = pddfanout->low;
         pad->lower_alarm_limit = pddfanout->lolo;
    } else recGblGetAlarmDouble(paddr,pad);
    return(0);
}

static void alarm(pddfanout)
    struct ddfanoutRecord	*pddfanout;
{
	double		val;
	float		hyst, lalm, hihi, high, low, lolo;
	unsigned short	hhsv, llsv, hsv, lsv;

	if(pddfanout->udf == TRUE ){
 		recGblSetSevr(pddfanout,UDF_ALARM,INVALID_ALARM);
		return;
	}
	hihi = pddfanout->hihi; lolo = pddfanout->lolo;
	high = pddfanout->high; low = pddfanout->low;
	hhsv = pddfanout->hhsv; llsv = pddfanout->llsv;
	hsv = pddfanout->hsv; lsv = pddfanout->lsv;
	val = pddfanout->val; hyst = pddfanout->hyst; lalm = pddfanout->lalm;
	/* alarm condition hihi */
	if (hhsv && (val >= hihi || ((lalm==hihi) && (val >= hihi-hyst)))){
	    if(recGblSetSevr(pddfanout,HIHI_ALARM,pddfanout->hhsv))
		pddfanout->lalm = hihi;
	    return;
	}
	/* alarm condition lolo */
	if (llsv && (val <= lolo || ((lalm==lolo) && (val <= lolo+hyst)))){
	    if(recGblSetSevr(pddfanout,LOLO_ALARM,pddfanout->llsv))
		pddfanout->lalm = lolo;
	    return;
	}
	/* alarm condition high */
	if (hsv && (val >= high || ((lalm==high) && (val >= high-hyst)))){
	    if(recGblSetSevr(pddfanout,HIGH_ALARM,pddfanout->hsv))
		pddfanout->lalm = high;
	    return;
	}
	/* alarm condition low */
	if (lsv && (val <= low || ((lalm==low) && (val <= low+hyst)))){
	    if(recGblSetSevr(pddfanout,LOW_ALARM,pddfanout->lsv))
		pddfanout->lalm = low;
	    return;
	}
	/* we get here only if val is out of alarm by at least hyst */
	pddfanout->lalm = val;
	return;
}

static void monitor(pddfanout)
    struct ddfanoutRecord	*pddfanout;
{
	unsigned short	monitor_mask;

	double		delta;

        monitor_mask = recGblResetAlarms(pddfanout);
        /* check for value change */
        delta = pddfanout->mlst - pddfanout->val;
        if(delta<0) delta = -delta;
        if (delta > pddfanout->mdel) {
                /* post events for value change */
                monitor_mask |= DBE_VALUE;
                /* update last value monitored */
                pddfanout->mlst = pddfanout->val;
        }
        /* check for archive change */
        delta = pddfanout->alst - pddfanout->val;
        if(delta<0) delta = -delta;
        if (delta > pddfanout->adel) {
                /* post events on value field for archive change */
                monitor_mask |= DBE_LOG;
                /* update last archive value monitored */
                pddfanout->alst = pddfanout->val;
        }

        /* send out monitors connected to the value field */
        if (monitor_mask){
                db_post_events(pddfanout,&pddfanout->val,monitor_mask);
	}
	return;
}

static long push_values(pddfanout)
struct ddfanoutRecord *pddfanout;
{
        struct link     *plink; /* structure of the link field  */
        int             i;
        long            status;

        for(i=0, plink=&(pddfanout->outa); i<OUT_ARG_MAX; i++, plink++) {
                status=dbPutLink(plink,DBR_DOUBLE,&(pddfanout->val),1);
                if (!RTN_SUCCESS(status)) return(-1);
        }
        return(0);
}
