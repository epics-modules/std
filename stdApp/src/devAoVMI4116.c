/* Device Support Routines for VMI4116 analog output*/
/*
 *      Original Author:    Bob Dalesio
 *      Current Author:     Mike Bordua
 *      Date:		    11-22-94
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      (W-31-109-ENG-38) at Argonne National Laboratory,
 *	and (DE-AC03-76SF00098) at Lawrence Berkeley Laboratory.
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
 *		Computer Systems Group
 *		Electronics and Software Engineering Division
 *		Lawrence Berkeley Laboratory
 */

#include	<vxWorks.h>
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<stdlib.h>
#include	<string.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include    <recSup.h>
#include	<devSup.h>
#include	<special.h>
#include	<module_types.h>
#include	<aoRecord.h>

static long vmi4116_num_cards = 1;
static long vmi4116_max_chan = 8;
static void *vmi4116_addrs = (void *)0xff00;
static void *vmi4116_local_addrs = (void *)0x0;
volatile int devAoVMI4116Debug = 0;

/* Create the dset for devAoVmiVme4116 */
static long init_record(aoRecord *pao);
static long write_ao(aoRecord *pao);
static long special_linconv(aoRecord *pao, int after);
static long vmi4116_io_report(short level);
static long	vmi4116_init();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_ao;
	DEVSUPFUN	special_linconv;
} devAoVMI4116 = {
	6,
	vmi4116_io_report,
	vmi4116_init,
	init_record,
	NULL,
	write_ao,
	special_linconv
};

#define VMI_ENABLE_OUT		0x4100 /*Fail LED off, enable P3 output.*/
#define VMI_DISABLE_OUT		0x0100 /*Fail LED on, enable P3 output.*/

/* memory structure of the VMIC 4116 Interface */
struct aoVMI{
	unsigned short data[8];
	unsigned short csr;
};

LOCAL unsigned short **pao_vmi4116;


/*****************************************************
* VMI4116_setup()
* User (startup file) calls this function to configure us for the hardware.
*****************************************************/
void VMI4116_setup(int num_cards,	/* maximum number of cards in crate */
	   void *addrs)		/* Base Address(0x0000-0xffe0, 32-byte boundary) */
{
	vmi4116_num_cards = num_cards;
	vmi4116_addrs = addrs;
}

static long vmi4116_init()
{
	short			shval;
	int                     status;
	register struct aoVMI	*pcard;
	register short		i;

	pao_vmi4116 = (unsigned short  **) calloc(vmi4116_num_cards, sizeof(*pao_vmi4116));
	if (!pao_vmi4116) return ERROR;

	if ((status = sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, vmi4116_addrs,
			(char **) &vmi4116_local_addrs)) != OK){ 
		printf("Addressing error in vmi4116 driver\n");
		return ERROR;
	}

	pcard = (struct aoVMI *)((int)vmi4116_local_addrs);
	/* mark each card present into the card present array */
	for (i = 0; i < vmi4116_num_cards; i++, pcard += vmi4116_max_chan) {
		if (vxMemProbe((char *)pcard, READ, sizeof(short), (char *)&shval) == OK) {
			pao_vmi4116[i] = (unsigned short *)pcard;
			pcard->csr = VMI_ENABLE_OUT;
			if (devAoVMI4116Debug) printf("Found Vmi4116 Card %d at A16 address %p\n",i,pcard);
		} else {
			pao_vmi4116[i] = 0;
			if (devAoVMI4116Debug) {
				printf("Didn't Find Vmi4116 Card %d at A16 address %p (VME:%p)\n",
					i, pcard, vmi4116_addrs);
			}
		}
	}
	return OK;
}

static long vmi4116_io_report(short level)
{
	register int i;

	for (i = 0; i < vmi4116_num_cards; i++) {
		if (pao_vmi4116[i]) {   
			printf("AO: VMI4116:    card %d  ",i);
			if (level > 0) {
				printf("VMI4116 card cannot be read.\n");
			} else {
				printf("\n");
			}          
		}
	}
	return OK;
}
                   



static long init_record(aoRecord *pao)
{

    /* ao.out must be an VME_IO */
    switch (pao->out.type) {
    case (VME_IO) :
	break;
    default :
	recGblRecordError(S_db_badField,(void *)pao,
		"devAoVmiVme4116 (init_record) Illegal OUT field");
	return(S_db_badField);
    }

    /* set linear conversion slope*/
    pao->eslo = (pao->eguf -pao->egul)/65535.0;

    /* It is not possible to read current value of card */
    /* Tell recSup not to convert			*/
    return(2);
}

static long write_ao(aoRecord *pao)
{
	struct vmeio *pvmeio = (struct vmeio *)&(pao->out.value);
	struct aoVMI *paoVMI = (struct aoVMI *)pao_vmi4116[pvmeio->card];

	if (paoVMI == 0) {
		if (recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM) && errVerbose
				&& (pao->stat!=WRITE_ALARM || pao->sevr!=INVALID_ALARM))
			recGblRecordError(-1,(void *)pao,"vmi4116_driver Error");
	} else {
		paoVMI->data[pvmeio->signal] = (unsigned short) pao->rval;
		pao->rbv = 0;
	}
	return(0);
}


static long special_linconv(aoRecord *pao, int after)
{

    if(!after) return(0);
    /* set linear conversion slope*/
    pao->eslo = (pao->eguf -pao->egul)/65535.0;
    return(0);
}
