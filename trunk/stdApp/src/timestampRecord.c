/* recTimestamp.c */
/* recTimestamp.c - Record Support Routines for Timestamp records */
/*
 *      Original Author:  Susumu Yoshida
 *      Current Author: 
 *      Date:            10-12-98
 *
 * Modification Log:
 * -----------------
 *      2-Jun-2000 rpc - Use tsStampToLocal to get daylight savings time
 *                       (involves changes to tsSubr.c and tsDefs.h)
 *                       Remove superfluous subroutines
 *                       Add more formats (see also timestampRecord.dbd)
 */

/* Following were removed for OSI/3.14 port */
/* #include	<vxWorks.h> */
/* #include	<types.h> */
/* #include	<stdioLib.h> */
/* #include	<lstLib.h> */
#include	<stdio.h>
#include	<string.h>

/* #include        <tsDefs.h>  */
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbFldTypes.h>
#include	<devSup.h>
#include	<recSup.h>
#include        <dbEvent.h>    /* for db_post_events() */
#include	<recGbl.h> /* 3.14.4 port */

#include 	"epicsExport.h"

#define GEN_SIZE_OFFSET
#include	<timestampRecord.h>
#undef  GEN_SIZE_OFFSET

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
#define init_record NULL
static long process();
static void monitor();
#define special NULL
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

rset timestampRSET={
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
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
epicsExportAddress(rset,timestampRSET);

static long process(struct timestampRecord *ptimestamp){

 const char * format=NULL; 
  ptimestamp->udf  = FALSE;
  ptimestamp->pact = TRUE;

  /* Device time here is the OS time appart from any epics
   * time stamp system */

  if (ptimestamp->tse==epicsTimeEventDeviceTime)
      epicsTimeFromTime_t(&ptimestamp->time,time(0));
  else
      recGblGetTimeStamp(ptimestamp);
  ptimestamp->rval=ptimestamp->time.secPastEpoch;

  if (ptimestamp->time.secPastEpoch==0)
  {
    sprintf(ptimestamp->val,"-NULL-");
  }
  else switch(ptimestamp->tst)
  {
  case timestampTST_YY_MM_DD_HH_MM_SS:
      format="%y/%m/%d %H:%M:%S";
    break;
  case timestampTST_MM_DD_YY_HH_MM_SS:
      format="%m/%d/%y %H:%M:%S";
    break;
  case timestampTST_MM_DD_HH_MM_SS_YY:
      format="%b %d %H:%M:%S %y" ;
    break;
  case timestampTST_MM_DD_HH_MM_SS:
      format="%b %d %H:%M:%S";
    break;
  case timestampTST_HH_MM_SS:
      format="%H:%M:%S";
    break;
  case timestampTST_HH_MM:
      format="%H:%M";
    break;
  case timestampTST_DD_MM_YY_HH_MM_SS:
      format="%d/%m/%y %H:%M:%S";
    break;
  case timestampTST_DD_MM_HH_MM_SS_YY:
      format="%d %b %H:%M:%S %y";
    break;
  case timestampTST_VMS:
      format="%d-%b-%Y %H:%M:%S";
    break;
  case timestampTST_MM_DD_YYYY:
      format="%b %d %Y %H:%M:%S.%03f";
    break;
  case timestampTST_MM_DD_YY:
      format="%m/%d/%y %H:%M:%S.%03f";
    break;
  default :  /* YY/MM/DD HH:MM:SS */
      format="%y/%m/%d %H:%M:%S";
    break;
  }
  if (format)
      epicsTimeToStrftime(ptimestamp->val,sizeof(ptimestamp->val),format,
	      &ptimestamp->time);
  /* check event list */
  monitor(ptimestamp);

  /* process the forward scan link record */
  recGblFwdLink(ptimestamp);
  ptimestamp->pact=FALSE;
  return(0);
}


static void monitor(ptimestamp)
     struct timestampRecord             *ptimestamp;
{
  unsigned short  monitor_mask;
  
  monitor_mask = recGblResetAlarms(ptimestamp);
  monitor_mask |=DBE_VALUE|DBE_LOG;
  if(strncmp(ptimestamp->oval,ptimestamp->val,sizeof(ptimestamp->val))) {
    db_post_events(ptimestamp,&(ptimestamp->val[0]), monitor_mask);
    db_post_events(ptimestamp,&ptimestamp->rval,monitor_mask);
    strncpy(ptimestamp->oval,ptimestamp->val,sizeof(ptimestamp->val));
  }
  return;
}
