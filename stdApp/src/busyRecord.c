#include	<stdlib.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<recGbl.h>
#include	<special.h>
#define GEN_SIZE_OFFSET
#include	<busyRecord.h>
#undef  GEN_SIZE_OFFSET
#include        <epicsExport.h>

/* this has been removed from dbDefs.h*/
#define CLOSED_LOOP 1

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long     init_record();
static long     process();
#define special NULL
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
#define get_precision NULL
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL

rset     busyRSET = {
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
epicsExportAddress(rset, busyRSET);

static long 
init_record(busyRecord *pbusy, int pass)
{
	unsigned short  ival = 0;

	if (pass == 1) {
		if (pbusy->dol.type == CONSTANT) {
			if (recGblInitConstantLink(&pbusy->dol, DBF_USHORT, &ival)) {
				pbusy->val = ival ? 1 : 0;
				pbusy->udf = FALSE;
			}
		}
	}
	pbusy->lval = pbusy->val;
	return (0);
}

static long 
process(busyRecord *pbusy)
{
	long            status = 0;
	unsigned short  val = 0;

	pbusy->pact = TRUE;
	if ((pbusy->omsl == CLOSED_LOOP) && (pbusy->dol.type != CONSTANT)) {
		status = dbGetLink(&pbusy->dol, DBR_USHORT, &val, 0, 0);
		if (status == 0) {
        	(void) recGblResetAlarms(pbusy);
			pbusy->val = val;
			pbusy->udf = FALSE;
		} else {
			pbusy->udf = TRUE;
			recGblSetSevr(pbusy, LINK_ALARM, INVALID_ALARM);
		}
	}

	recGblGetTimeStamp(pbusy);
	if (pbusy->val != pbusy->lval) {
		db_post_events(pbusy, &pbusy->val, DBE_VALUE | DBE_LOG);
	}
	if (pbusy->val == 0) recGblFwdLink(pbusy);
	pbusy->lval = pbusy->val;
	pbusy->pact = FALSE;
	return (status);
}
