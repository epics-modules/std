/* gpibRecord.c - Record Support Routines for Generic GPIB record */
/*
 *      Author: 	Mark Rivers
 *      Date:   	3/22/96
 *
 * Modification Log:
 * -----------------
 * .01  06-03-97 tmm  Conversion to EPICS 3.13.
 */


#include        <vxWorks.h>
#include        <types.h>
#include        <rngLib.h>
#include        <stdioLib.h>
#include        <lstLib.h>
#include        <string.h>

#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <dbFldTypes.h>
#include        <devSup.h>
#include        <drvSup.h>
#include        <errMdef.h>
#include        <recSup.h>
#define GEN_SIZE_OFFSET
#include        "gpibRecord.h"
#undef GEN_SIZE_OFFSET

#include	<drvGpibInterface.h>

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

#define REMEMBER_STATE(FIELD) pCmd->old.FIELD = pgpib->FIELD
#define POST_IF_NEW(FIELD) \
    if (pgpib->FIELD != pCmd->old.FIELD) { \
        db_post_events(pgpib, &pgpib->FIELD, monitor_mask); \
        pCmd->old.FIELD = pgpib->FIELD; }

struct rset gpibRSET={
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

extern struct drvGpibSet drvGpib;       /* entry points to driver functions */

/* The following creates a dummy DSET.  This is required because we have defined
 * this record to be devGPIB so we could get the fields for GPIB_IO
 */
struct dset devGPIB = {4,NULL,NULL,NULL,NULL};

struct old_field_values {              /* Used in monitor() */
    int   nowt;               
    int   nrrd;               
    int   nord;
    int   spr;               
    int   ucmd;
    int   acmd;
};

struct gpibIntCmd {
  struct dpvtGpibHead head;
  struct gpibRecord *prec;      /* Pointer to record */
  int	linkId;			/* link number */
  int	linkType;		/* GPIB_IO or BBGPIB_IO */
  int	bug;			/* bug# if BBGPIB_IO linkType */
  struct old_field_values old;
};


static int  gpibWork();
static void monitor();
static long readValue();


static long init_record(pgpib,pass)
    struct gpibRecord	*pgpib;
    int pass;
{
    long status;
    struct gpibIntCmd *pCmd;
    struct link *plink=&pgpib->inp;
    int link_addr=0;

    if (pass != 0) return(0);

    /* Allocate and initialize private structure used by this record */
    pCmd = (struct gpibIntCmd*) malloc(sizeof(struct gpibIntCmd)); 
    pgpib->dpvt = pCmd;

    pCmd->head.workStart = gpibWork;
    pCmd->head.link = 0;
    pCmd->head.device = 0;
    pCmd->prec = pgpib;
    pCmd->linkType = plink->type;
    switch (plink->type) {
      case GPIB_IO:         /* Is a straight Network Instruments or HiDEOS link */
	pCmd->linkId = plink->value.gpibio.link;   /* NI link number */
	link_addr = plink->value.gpibio.addr;    /* gpib dev address */
        pCmd->bug = 0;
        break;
      case BBGPIB_IO:       /* Is a bitbus -> gpib link */
	pCmd->linkId = plink->value.bbgpibio.link;
	link_addr = plink->value.bbgpibio.gpibaddr; /* dev address */
	pCmd->head.bitBusDpvt = 
            (struct dpvtBitBusHead *) malloc(sizeof(struct dpvtBitBusHead));
	pCmd->head.bitBusDpvt->txMsg.data = 
            (unsigned char *) malloc(BB_MAX_DAT_LEN);
	pCmd->head.bitBusDpvt->rxMsg.data = 
            (unsigned char *) malloc(BB_MAX_DAT_LEN);
	pCmd->head.bitBusDpvt->txMsg.node = 
            plink->value.bbgpibio.bbaddr; /* bug node address */
        pCmd->bug = pCmd->head.bitBusDpvt->txMsg.node;
	pCmd->head.bitBusDpvt->link = 
            plink->value.bbgpibio.link;  /* bug link number */
	pCmd->head.bitBusDpvt->rxMaxLen = sizeof(struct bitBusMsg);
	break;
    }

    /* If the GPIB address specified in the record ADDR field is 0 and that
     * specified in the link is not 0, then use the link address
     */
    if ((pgpib->addr == 0) && (link_addr !=0)) pgpib->addr=link_addr;

    (*(drvGpib.ioctl))(pCmd->linkType, pCmd->linkId, pCmd->bug, 
            IBGENLINK, 0, NULL);
    (*(drvGpib.ioctl))(pCmd->linkType, pCmd->linkId, pCmd->bug, 
            IBGETLINK, 0, &(pCmd->head.pibLink));

    /* Allocate the space for the binary output and binary input arrays */
    if (pgpib->omax <= 0) pgpib->omax=1;
    if (pgpib->imax <= 0) pgpib->imax=1;
    pgpib->optr = (char *)calloc(pgpib->omax, sizeof(char));
    pgpib->iptr = (char *)calloc(pgpib->imax, sizeof(char));

    return(0);
}

static long cvt_dbaddr(paddr)
struct dbAddr *paddr;
{
   struct gpibRecord *pgpib=(struct gpibRecord *)paddr->precord;
   int fieldIndex = dbGetFieldIndex(paddr);

   if (fieldIndex == gpibRecordBOUT) {
      paddr->pfield = (void *)(pgpib->optr);
      paddr->no_elements = pgpib->omax;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBF_CHAR;
   } else if (fieldIndex == gpibRecordBINP) {
      paddr->pfield = (unsigned char *)(pgpib->iptr);
      paddr->no_elements = pgpib->imax;
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
   struct gpibRecord *pgpib=(struct gpibRecord *)paddr->precord;

   *no_elements =  pgpib->nord;  /* Is this correct? */
   *offset = 0;
   return(0);
}

static long put_array_info(paddr,nNew)
struct dbAddr *paddr;
long nNew;
{
struct gpibRecord *pgpib=(struct gpibRecord *)paddr->precord;

   pgpib->nowt = nNew;
   if (pgpib->nowt > pgpib->omax) pgpib->nowt = pgpib->omax;
   return(0);
}


static long process(pgpib)
	struct gpibRecord	*pgpib;
{
	long		 status;
        struct gpibIntCmd* pCmd = pgpib->dpvt;

        /* If pact is FALSE then queue message to driver and return */
        if (!pgpib->pact)
        {
            /* Remember current state of fields for monitor() */
            REMEMBER_STATE(nrrd);
            REMEMBER_STATE(nord);
            REMEMBER_STATE(nowt);
            REMEMBER_STATE(spr);
            REMEMBER_STATE(ucmd);
            REMEMBER_STATE(acmd);
            /* Make sure nrrd and nowt are valid */
            if (pgpib->nrrd > pgpib->imax) pgpib->nrrd = pgpib->imax;
            if (pgpib->nowt > pgpib->omax) pgpib->nowt = pgpib->omax;
            (*(drvGpib.qGpibReq))(pCmd, IB_Q_LOW); /* queue the msg */
            pgpib->pact = TRUE;
            return(0);
        }

        /* pact was TRUE, so we were called from gpibWork.
         * The I/O is complete, so finish up.
         */

	recGblGetTimeStamp(pgpib);

	/* check event list */
	monitor(pgpib);
	/* process the forward scan link record */
	recGblFwdLink(pgpib);

	pgpib->pact=FALSE;
	return(0);
}



static void monitor(pgpib)
    struct gpibRecord   *pgpib;
{
    unsigned short  monitor_mask;
    struct gpibIntCmd* pCmd = pgpib->dpvt;

    monitor_mask = recGblResetAlarms(pgpib) | DBE_VALUE | DBE_LOG;
    
    if(strncmp(pgpib->oinp,pgpib->ainp,sizeof(pgpib->ainp))) 
    {
       db_post_events(pgpib,pgpib->ainp, monitor_mask);
       strncpy(pgpib->oinp,pgpib->ainp,sizeof(pgpib->ainp));
    }
    if (pgpib->ifmt == gpibOFMT_Binary)
        db_post_events(pgpib, pgpib->iptr, monitor_mask);

    POST_IF_NEW(nrrd);
    POST_IF_NEW(nord);
    POST_IF_NEW(nowt);
    POST_IF_NEW(spr);
    POST_IF_NEW(ucmd);
    POST_IF_NEW(acmd);
}


/* gpibWork ******************
 * This routine is called by the driver when the message gets to the head of
 * the queue.  It calls the driver to do the actual GPIB I/O and then calls
 * the record process() function again to signal that the I/O is complete
 */
static int gpibWork(struct gpibIntCmd *pCmd)
{
   struct  gpibRecord *pgpib = pCmd->prec;
   int     nout;
   int     ninp;
   int     timeout;
   char    *inptr;
   char    *outptr;
   char    cmd_char=0;
   char    acmd[6];
   int     inlen;
   int     outlen;

   dbScanLock( (struct dbCommon*) pgpib);       /* Lock the record */
   pCmd->head.device = pgpib->addr;             /* GPIB address    */
   /* Note - the following is not guaranteed to be accurate when using HiDEOS
    * since HiDEOS may be running on another board with a different clock rate
    */
   timeout = pgpib->tmot * sysClkRateGet() / 1000;
   if (timeout < 1) timeout = 1;

   /* See if a Universal Command is to be done */
   /* See gpibRecord.dbd for definitions of constants gpibXXXX_Abcd */
   if (pgpib->ucmd != gpibUCMD_None)
   {
      switch (pgpib->ucmd)
      {
        case gpibUCMD_Device_Clear__DCL_:
            cmd_char = 20;
            break;
        case gpibUCMD_Local_Lockout__LL0_:
            cmd_char = 17;
            break;
        case gpibUCMD_Serial_Poll_Disable__SPD_:
            cmd_char = 25;
            break;
        case gpibUCMD_Serial_Poll_Enable__SPE_:
            cmd_char = 24;
            break;
        case gpibUCMD_Unlisten__UNL_: 
            cmd_char = 63;
            break;
        case gpibUCMD_Untalk__UNT_:
            cmd_char = 95;
            break;
      }
      nout =(*(drvGpib.writeIbCmd))(pCmd->head.pibLink, &cmd_char, 1);
      if (nout != 1)
         /* Something is wrong if we couldn't write */
         recGblSetSevr(pgpib, WRITE_ALARM, MAJOR_ALARM);
      pgpib->ucmd = gpibUCMD_None;  /* Reset to no Universal Command */
      goto finish;
   }

   /* See if an Addressed Command is to be done */
   if (pgpib->acmd != gpibACMD_None)
   {
      acmd[0] = 95; /* Untalk */
      acmd[1] = 63; /* Unlisten */
      acmd[2] = pgpib->addr + LADBASE;  /* GPIB address + Listen Base */
      acmd[4] = 95; /* Untalk */
      acmd[5] = 63; /* Unlisten */
      switch (pgpib->acmd)
      {
        case gpibACMD_Group_Execute_Trig___GET_:
            acmd[3] = 8;
            break;
        case gpibACMD_Go_To_Local__GTL_:
            acmd[3] = 1;
            break;
        case gpibACMD_Selected_Dev__Clear__SDC_:
            acmd[3] = 4;
            break;
        case gpibACMD_Take_Control__TCT_:
            /* This command requires Talker Base */
            acmd[2] = pgpib->addr + TADBASE;
            acmd[3] = 9;
            break;
        case gpibACMD_Serial_Poll:
            /* Serial poll. Requires 3 operations */
            /* Serial Poll Enable */
            cmd_char = 24;
            nout =(*(drvGpib.writeIbCmd))(pCmd->head.pibLink, &cmd_char, 1);
            if (nout != 1)
                /* Something is wrong if we couldn't write */
                recGblSetSevr(pgpib, WRITE_ALARM, MAJOR_ALARM);
            /* Read the response byte  */
            ninp = (*(drvGpib.readIbEos))(pCmd->head.pibLink, pCmd->head.device, 
                &pgpib->spr, 1, timeout, -1);
            if (ninp != 1)
                /* Something is wrong if we couldn't read */
                recGblSetSevr(pgpib, READ_ALARM, MAJOR_ALARM);
            /* Serial Poll Disable */
            cmd_char = 25;
            nout =(*(drvGpib.writeIbCmd))(pCmd->head.pibLink, &cmd_char, 1);
            if (nout != 1)
                /* Something is wrong if we couldn't write */
                recGblSetSevr(pgpib, WRITE_ALARM, MAJOR_ALARM);
            pgpib->acmd = gpibACMD_None;  /* Reset to no Addressed Command */
            goto finish;
            break;
      }
      nout =(*(drvGpib.writeIbCmd))(pCmd->head.pibLink, acmd, 6);
      if (nout != 6)
         /* Something is wrong if we couldn't write */
         recGblSetSevr(pgpib, WRITE_ALARM, MAJOR_ALARM);
      pgpib->acmd = gpibACMD_None;  /* Reset to no Addressed Command */
      goto finish;
   }
      

   if (pgpib->ofmt == gpibOFMT_ASCII)
   {    /* ASCII output mode */
        outptr = pgpib->aout;
        outlen = strlen(pgpib->aout);
   } else 
   {     /* Binary output mode */
        outptr = pgpib->optr;
        outlen = pgpib->nowt;
   }

   if (pgpib->ifmt == gpibOFMT_ASCII)
   {    /* ASCII input mode */
        inptr = pgpib->ainp;
        if (pgpib->nrrd != 0)
            inlen = pgpib->nrrd;
        else
            inlen = sizeof(pgpib->ainp);
   } else 
   {     /* Binary input mode */
        inptr = pgpib->iptr;
        if (pgpib->nrrd != 0)
            inlen = pgpib->nrrd;
        else
            inlen = pgpib->imax;
   }

   if (pgpib->tmod != gpibTMOD_Read)
   {
      /* write the message to the GPIB listen adrs */
      nout =(*(drvGpib.writeIb))(pCmd->head.pibLink, pCmd->head.device, 
                outptr, outlen, timeout);
      if (nout != outlen) 
         /* Something is wrong if we couldn't write everything */
         recGblSetSevr(pgpib, WRITE_ALARM, MAJOR_ALARM);
   }
    
   pgpib->val[0] = '\0';     /* Set NULL response string */
   if (pgpib->tmod != gpibTMOD_Write)
   {
      /* read the instrument  */
      ninp = (*(drvGpib.readIbEos))(pCmd->head.pibLink, pCmd->head.device, 
            inptr, inlen, timeout, pgpib->eos);
      if (ninp == 0) 
         /* Something is wrong if we didn't get any response */
         recGblSetSevr(pgpib, READ_ALARM, MAJOR_ALARM);
      if (((pgpib->ifmt == gpibOFMT_ASCII)  && (ninp > sizeof(pgpib->ainp)-1)) ||
          ((pgpib->ifmt == gpibOFMT_Binary) && (ninp > pgpib->imax)))
      {
         /* Input buffer overflow */
         recGblSetSevr(pgpib, READ_ALARM, MINOR_ALARM);
         if (pgpib->ifmt == gpibOFMT_ASCII)  /* terminate response with \0 */
             inptr[sizeof(pgpib->ainp)-1] = '\0';
      }
      else
      {
         /* If the string is terminated by the requested terminator */
         /* remove it. */
         if ((ninp != 0) && (pgpib->eos != -1) && (inptr[ninp-1] == pgpib->eos))
            inptr[ninp-1] = '\0';
         else
            /* Add a null terminator - there is room */
	    inptr[ninp] = '\0';
      } 
      pgpib->nord = ninp; /* Number of bytes read */
   }

   /* Call process, and unlock the record */
finish:
   process(pgpib);
   dbScanUnlock( (struct dbCommon*) pgpib);
}
