/* vmeRecord.c - Record Support Routines for Generic VME records */
/*
 *      Author:         Mark Rivers
 *      Date:           4/9/95
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
 * Sometime in 1997 Tim Mooney converted this record from R3.12 to R3.13.0.
 *
 * 23-Jan-2001  Mark Rivers
 *              Two bugs were found in the code which dates from 1997 or 1998
 *              1) Monitors were not being posted on the VAL and SARR fields 
 *                 because there was an incorrect "&" before the addresses 
 *                 passed to db_post_events.
 *                 The VAL1 and SAR1 fields were probably added to the record
 *                 to work around this problem.
 *              2) Values were not copied from the VAL1 field to the VAL
 *                 array, so writes from CA clients to the VAL1 field did not
 *                 work.
 *              I fixed the call to db_post_events, so monitors are correctly
 *              posted on the VAL and SARR fields.  I deleted the VAL1 and
 *              SAR1 fields, since they are not necessary.  I added debugging
 *              and made minor changes to avoid compiler warnings.
 */


#include        <vxWorks.h>
#include        <vxLib.h>
#include        <types.h>
#include        <stdlib.h>
#include        <stdioLib.h>
#include        <lstLib.h>
#include        <string.h>
#include        <sysLib.h>

#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbEvent.h>
#include        <dbAccess.h>
#include        <dbFldTypes.h>
#include        <devSup.h>
#include        <errMdef.h>
#include        <recSup.h>
#include        <recGbl.h>
#include        <vme.h>
#include        <epicsExport.h>
#define GEN_SIZE_OFFSET
#include        <vmeRecord.h>
#undef GEN_SIZE_OFFSET


#ifdef NODEBUG
#define Debug(l,f,v...) ;
#else
#define Debug(l,f,v...) { if(l<=vmeRecordDebug) printf(f,## v); }
#endif
volatile int vmeRecordDebug = 0;


/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
#define special NULL
#define get_value NULL
static long cvt_dbaddr();
static long get_array_info();
static long put_array_info();
#define get_units NULL
#define get_precision NULL
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL

struct rset vmeRSET={
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
epicsExportAddress(rset, vmeRSET);

static void monitor();
static long doVmeIo();


static long init_record(pvme, pass)
   vmeRecord  *pvme;
   int pass;
{
   if (pass==0) {
      if (pvme->nmax <= 0) pvme->nmax=1;
      pvme->bptr = (char *)calloc(pvme->nmax, sizeof(long));
      pvme->sptr = (char *)calloc(pvme->nmax, sizeof(char));
      return(0);
   }
   return(0);
}

static long process(pvme)
   vmeRecord *pvme;
{
   long status;

   status=doVmeIo(pvme); /* Do VME I/O operation */
   recGblGetTimeStamp(pvme);

   /* check event list */
   monitor(pvme);
   /* process the forward scan link record */
   recGblFwdLink(pvme);

   pvme->pact=FALSE;
   return(status);
}

static long cvt_dbaddr(paddr)
struct dbAddr *paddr;
{
   vmeRecord *pvme=(vmeRecord *)paddr->precord;

   if (paddr->pfield == &(pvme->val)) {
      paddr->pfield = (void *)(pvme->bptr);
      paddr->no_elements = pvme->nmax;
      paddr->field_type = DBF_LONG;
      paddr->field_size = sizeof(long);
      paddr->dbr_field_type = DBF_LONG;
   } else if (paddr->pfield == &(pvme->sarr)) {
      paddr->pfield = (unsigned char *)(pvme->sptr);
      paddr->no_elements = pvme->nmax;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBF_CHAR;
   }
   return(0);
}

static long get_array_info(paddr,no_elements,offset)
struct dbAddr *paddr;
long *no_elements;
long *offset;
{
   vmeRecord *pvme=(vmeRecord *)paddr->precord;

   *no_elements =  pvme->nuse;
   *offset = 0;
   return(0);
}

static long put_array_info(paddr,nNew)
struct dbAddr *paddr;
long nNew;
{
vmeRecord *pvme=(vmeRecord *)paddr->precord;

   pvme->nuse = nNew;
   if (pvme->nuse > pvme->nmax) pvme->nuse = pvme->nmax;
   return(0);
}


static void monitor(pvme)
    vmeRecord   *pvme;
{
   unsigned short monitor_mask;

   /* get previous stat and sevr  and new stat and sevr*/
   monitor_mask = recGblResetAlarms(pvme);
   monitor_mask |= (DBE_LOG | DBE_VALUE);
   
   /* send out monitors connected to the value field */
   if (monitor_mask) {
      db_post_events(pvme,pvme->bptr,monitor_mask);
      db_post_events(pvme,pvme->sptr,monitor_mask);
   }
   return;
}

static long doVmeIo(pvme)
   vmeRecord   *pvme;
{
   int amod, length, mode, status=ERROR, value, i;
   int *data_array = (int *) pvme->bptr;
   unsigned char *status_array = pvme->sptr;
   char *vme_ptr;
   short word_value;
   char char_value;
   int all_amods[] = {
      VME_AM_SUP_SHORT_IO,
      VME_AM_STD_SUP_DATA,
      VME_AM_EXT_SUP_DATA};
   int all_lengths[] = {1, 2, 4};
   int all_modes[] = {VX_READ, VX_WRITE};

   amod = all_amods[pvme->amod];
   length = all_lengths[pvme->dsiz];
   mode = all_modes[pvme->rdwt];
   sysBusToLocalAdrs(amod, (char *)pvme->addr, &vme_ptr);
   if (mode == VX_READ) {
      for (i=0; i<pvme->nuse; i++) {
         switch(length) {
         case 1: status = vxMemProbe( vme_ptr, VX_READ, 1, (char *) &char_value);
                 value = char_value;
                 break;
         case 2: status = vxMemProbe( vme_ptr, VX_READ, 2, (char *) &word_value);
                 value = word_value;
                 break;
         case 4: status = vxMemProbe( vme_ptr, VX_READ, 4, (char *) &value);
                 break;
         }
         data_array[i] = value;
         status_array[i] = status;
         Debug(1, "vmeRecord: VX_READ, vme_ptr=%p, value=0x%x, status=0x%x\n", 
                   vme_ptr, value, status);
         vme_ptr += pvme->ainc;
      }
   } else {               /* VX_WRITE */
      for (i=0; i<pvme->nuse; i++) {
         switch(length) {
         case 1: char_value = data_array[i];
                 status = vxMemProbe( vme_ptr, VX_WRITE, 1, (char *) &char_value);
                 break;
         case 2: word_value = data_array[i];
                 status = vxMemProbe( vme_ptr, VX_WRITE, 2, (char *) &word_value);
                 break;
         case 4: value = data_array[i];
                 status = vxMemProbe( vme_ptr, VX_WRITE, 4, (char *) &value);
                 break;
         }
         Debug(1, "vmeRecord: VX_WRITE, vme_ptr=%p, value=0x%x, status=0x%x\n", 
                   vme_ptr, data_array[i], status);
         vme_ptr += pvme->ainc;
         status_array[i] = status;
      }
   }
   return OK;
}
