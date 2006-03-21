/* serialRecord.c - Record Support Routines for Generic Serial record */
/*
 *      Author: 	Mark Rivers
 *      Date:   	3/22/96
 *
 * Modification Log:
 * -----------------
 * .01  10/28/99  MLR  Added debugging statements
 * .02  9/18/00   MLR  Added "hybrid" mode.  ASCII output but to binary buffer
 *                     Minor changes to avoid compiler warnings.
 * .03 03/17/06   RLS  modified init_record() to allocate memory on pass=0.
 *
 */


#include        <vxWorks.h>
#include        <types.h>
#include        <stdlib.h>
#include        <stdioLib.h>
#include        <lstLib.h>
#include        <string.h>

#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbEvent.h>
#include        <dbAccess.h>
#include        <dbFldTypes.h>
#include        <devSup.h>
#include        <errMdef.h>
#include        <recSup.h>
#define GEN_SIZE_OFFSET
#include        <serialRecord.h>
#undef GEN_SIZE_OFFSET

#ifdef NODEBUG
#define Debug(l,f,v...) ;
#else
#define Debug(l,f,v...) { if(l<=serialRecordDebug) printf(f,## v); }
#endif
volatile int serialRecordDebug = 0;


/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
static long special();
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

struct rset serialRSET={
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

struct serialdset { /* serial input dset */
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info; /* Unused */
	DEVSUPFUN	read_serial; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	conv; /* Unused */
	DEVSUPFUN	port_setup; /* Sets port paramters */
};

static int baud_choices[]={300,600,1200,2400,4800,9600,19200,38400};
static char parity_choices[]={'N','E','O'};
static int data_bit_choices[]={5,6,7,8};
static int stop_bit_choices[]={1,2};
static char flow_control_choices[]={'N','H'};


static void monitor();


static long init_record(pserial,pass)
    serialRecord	*pserial;
    int pass;
{
    struct serialdset *pdset;
    long status;

    if (pass==0)
    {
        /* Allocate the space for the binary output and binary input arrays */
        if (pserial->omax <= 0) pserial->omax=1;
        if (pserial->imax <= 0) pserial->imax=1;
        pserial->optr = (char *)calloc(pserial->omax, sizeof(char));
        pserial->iptr = (char *)calloc(pserial->imax, sizeof(char));
        return(0);
    }

    if(!(pdset = (struct serialdset *)(pserial->dset))) {
	recGblRecordError(S_dev_noDSET,(void *)pserial,"serial: init_record");
	return(S_dev_noDSET);
    }
    /* must have read_serial function defined */
    if( (pdset->number < 5) || (pdset->read_serial == NULL) ) {
	recGblRecordError(S_dev_missingSup,(void *)pserial,"serial: init_record");
	return(S_dev_missingSup);
    }
    if( pdset->init_record ) {
	if((status=(*pdset->init_record)(pserial))) return(status);
    }

    /* Set up the serial port parameters */
    (*pdset->port_setup) (pserial,
		baud_choices[pserial->baud],
		data_bit_choices[pserial->dbit],
		stop_bit_choices[pserial->sbit],
		parity_choices[pserial->prty],
		flow_control_choices[pserial->fctl]);
    
    return(0);
}

static long cvt_dbaddr(paddr)
struct dbAddr *paddr;
{
   serialRecord *pserial=(serialRecord *)paddr->precord;

   if (paddr->pfield == &(pserial->bout)) {
      paddr->pfield = (void *)(pserial->optr);
      paddr->no_elements = pserial->omax;
      paddr->field_type = DBF_CHAR;
      paddr->field_size = sizeof(char);
      paddr->dbr_field_type = DBF_CHAR;
   } else if (paddr->pfield == &(pserial->binp)) {
      paddr->pfield = (unsigned char *)(pserial->iptr);
      paddr->no_elements = pserial->imax;
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
   serialRecord *pserial=(serialRecord *)paddr->precord;

   *no_elements =  pserial->nord;  /* Is this correct? */
   *offset = 0;
   return(0);
}

static long put_array_info(paddr,nNew)
struct dbAddr *paddr;
long nNew;
{
serialRecord *pserial=(serialRecord *)paddr->precord;

   pserial->nowt = nNew;
   if (pserial->nowt > pserial->omax) pserial->nowt = pserial->omax;
   return(0);
}


static long process(pserial)
	serialRecord	*pserial;
{
	struct serialdset	*pdset = (struct serialdset *)(pserial->dset);
	long		 status;
	unsigned char    pact=pserial->pact;
	
	Debug(1, "serialRecord %s :process entry\n", pserial->name);

	if( (pdset==NULL) || (pdset->read_serial==NULL) ) {
		Debug(1, "serialRecord %s:process no device support\n", 
			pserial->name);
		pserial->pact=TRUE;
		recGblRecordError(S_dev_missingSup,(void *)pserial,"read_serial");
		return(S_dev_missingSup);
	}

	Debug(1, "serialRecord %s :process calling read_serial\n", 
		pserial->name);
	status=(*pdset->read_serial)(pserial);
	/* check if device support set pact */
	if ( !pact && pserial->pact ) return(0);
	pserial->pact = TRUE;

	recGblGetTimeStamp(pserial);

	/* check event list */
	monitor(pserial);
	/* process the forward scan link record */
	recGblFwdLink(pserial);

	pserial->pact=FALSE;
	return(status);
}

static void monitor(pserial)
    serialRecord   *pserial;
{
    unsigned short  monitor_mask;

    monitor_mask = recGblResetAlarms(pserial) | DBE_VALUE | DBE_LOG;
    
    if(strncmp(pserial->oinp,pserial->ainp,sizeof(pserial->ainp))) 
    {
       db_post_events(pserial,pserial->ainp, monitor_mask);
       strncpy(pserial->oinp,pserial->ainp,sizeof(pserial->ainp));
    }
    if ((pserial->ifmt == serialOFMT_Binary) || (pserial->ifmt == serialOFMT_Hybrid))
        db_post_events(pserial, pserial->iptr, monitor_mask);

    if (pserial->nord != pserial->onrd)
        db_post_events(pserial, &pserial->nord, monitor_mask);

    if (pserial->nowt != pserial->onwt)
        db_post_events(pserial, &pserial->nowt, monitor_mask);
}

/* special() is called when any of the serial port parameters (baud rate, parity, etc.) 
 * are changed.  It writes the new port parameters
 */
static long special(paddr,after)
struct dbAddr *paddr;
int after;
{
	serialRecord *pserial=(serialRecord *)paddr->precord;
	struct serialdset	*pdset = (struct serialdset *)(pserial->dset);

	if (!after) return(0);

	Debug(1, "serialRecord %s :special entry\n", pserial->name);

	(*pdset->port_setup) (pserial,
		baud_choices[pserial->baud],
		data_bit_choices[pserial->dbit],
		stop_bit_choices[pserial->sbit],
		parity_choices[pserial->prty],
		flow_control_choices[pserial->fctl]);
	return 0;
}
