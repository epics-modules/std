/* devAiVaroc.c */
/* share/src/dev @(#)devAiVaroc.c	1.00     6/3/93 */

/* devAiVaroc.c - Device Support Routines */
/*
 *      Original Author: Karen J. Coulter
 *      Current Author:  Robert A. Popper 
 *      Date:  3/28/94
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
 * Modification Log:
 * -----------------
 * .00  06-03-93        kjc     original adapted from devAiDvx2502.c
 * .01	03-28-94	rap	now parm field is 
 * 	...
 */



#include	<vxWorks.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<string.h>
#include        <ctype.h>

#include	<alarm.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <recSup.h>
#include	<devSup.h>
#include	<link.h>
#include	<module_types.h>
#include	<aiRecord.h>

long init_record();
long read_ai();
long special_linconv();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
        DEVSUPFUN       get_ioint_info;
	DEVSUPFUN	read_ai;
	DEVSUPFUN	special_linconv;
} devAiVaroc={
	6,
	NULL,
	NULL,
	init_record,
	NULL,
	read_ai,
	special_linconv	};

struct dprivate {
  int enc_bits;
  int gray;
};



static long init_record(pai)
    struct aiRecord	*pai;
{
    unsigned short value;
    int num_bits;
    char sorg[1];
    struct vmeio *pvmeio;
    struct dprivate *my_dpvt;
    long status;
    long nvalue;
	


    /* ai.inp must be an VME_IO */
    pvmeio = (struct vmeio *)&(pai->inp.value);
    printf("\n%s ** pai->inp.type = %X\n ** VME_IO = %d\n ** pai->parm = %s\n",
	    __FILE__,pai->inp.type,VME_IO,pvmeio->parm); 
    printf ("\n%s ** card = %X   signal = %X\n",
	    __FILE__,pvmeio->card,pvmeio->signal);
/*    switch (pai->inp.type) {
    case (VME_IO) :
	break;
    default :
	recGblRecordError(S_db_badField,(void *)pai,
		"devAiVaroc (init_record) Illegal INP field");
	return(S_db_badField);
    }
*/
    /* set linear conversion slope and offset */
    pai->eslo = 1;
    pai->roff = -(pai->egul/pai->aslo);

/* find number of bits from start of parameter field     */
/* parameter field should look like this... 23G or 23S   */
/* the G is for Gray code, the S is for Serial	         */

    num_bits = 0;
    if (isdigit(pvmeio->parm[0])){
      sscanf(pvmeio->parm,"%d%c",&num_bits,&sorg[0]);
      }
    else {
      recGblRecordError(S_db_badField,(void *)pai,
	   "devAiVaroc (init_record) must have # of bits in PARM field col 1");
      return(S_db_badField);
    }

/* check to see if the last char in the parm field is a S or a G */
/* this will determine if the data is normal serial or gray coded serial */

    if (!((sorg[0] == 'S') || (sorg[0] == 'G'))) {
	recGblRecordError(S_db_badField,(void *)pai,"PARM field must follow # of bits by S or G");
	return(S_db_badField);
    }
    if ((num_bits < 7) || (num_bits > 25)) {
      recGblRecordError(S_db_badField,(void *)pai,
             "devAiVaroc (init_record) encoder num_bits in PARM out of range");
      return(S_db_badField);
    }

    my_dpvt = (struct dprivate *) (malloc(sizeof(struct dprivate)));
    if (!my_dpvt) return ERROR;
    pai->dpvt = (void *) my_dpvt;
    my_dpvt->enc_bits = num_bits;
    my_dpvt->gray = 0x0020;
    if (sorg[0] == 'S') {
	printf("Current port is SERIAL\n");
	printf("%s : changing default from GREY to SERIAL\n",__FILE__);
	my_dpvt->gray = 0x0000;
    }
    printf("%s : encoder bits = ** %d **\n",__FILE__,num_bits);
    printf("%s : encoder gray code = ** %X **\n",__FILE__,my_dpvt->gray);
	nvalue = 0;
    status = varoc_driver(pvmeio->card,pvmeio->signal,num_bits,my_dpvt->gray,&nvalue);
    return(0);
}


static long read_ai(pai)
    struct aiRecord	*pai;
{
  long value;
  long status;
  struct vmeio *pvmeio;
  struct dprivate *my_dpvt=(struct dprivate *)pai->dpvt;

	
  pvmeio = (struct vmeio *)&(pai->inp.value);
  status=varoc_driver(pvmeio->card,pvmeio->signal,my_dpvt->enc_bits,my_dpvt->gray,&value);
  pai->rval=value;

#if 0
  if(status==0 || status==-2) pai->rval = value;
  if(status==-1) {
    recGblSetSevr(pai,READ_ALARM,INVALID_ALARM);
    status=2; /*don't convert*/
  }else if(status==-2) {
    status=0;
    recGblSetSevr(pai,HW_LIMIT_ALARM,INVALID_ALARM);
  }
#endif

  return(status);
}



static long special_linconv(pai,after)
    struct aiRecord	*pai;
    int after;
{

    if (!after) return(0);
    /* set linear conversion slope and offset */
    pai->eslo = 1;
    pai->roff = -(pai->egul/pai->aslo);

    return(0);
}


