/* 	devHPLaserAxis.c	*/

/*****************************************************************
 *
 *      Device Support for the Hewlett-Packard 10895A Laser Axis board
 *
 *      Author: Tim Mooney
 *      Date: 8/7/95
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *****************************************************************
 *                         COPYRIGHT NOTIFICATION
 *****************************************************************
 *
 * THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
 * AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
 * AND IN ALL SOURCE LISTINGS OF THE CODE.
 *
 * (C)  COPYRIGHT 1995 UNIVERSITY OF CHICAGO
 * 
 * Argonne National Laboratory (ANL), with facilities in the States of 
 * Illinois and Idaho, is owned by the United States Government, and
 * operated by the University of Chicago under provision of a contract
 * with the Department of Energy.
 * Portions of this material resulted from work developed under a U.S.
 * Government contract and are subject to the following license:  For
 * a period of five years from March 30, 1993, the Government is
 * granted for itself and others acting on its behalf a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, and perform
 * publicly and display publicly.  With the approval of DOE, this
 * period may be renewed for two additional five year periods. 
 * Following the expiration of this period or periods, the Government
 * is granted for itself and others acting on its behalf, a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, distribute copies
 * to the public, perform publicly and display publicly, and to permit
 * others to do so.
 *
 *****************************************************************
 *                               DISCLAIMER
 *****************************************************************
 *
 * NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
 * THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
 * MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
 * LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
 * USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
 * DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
 * OWNED RIGHTS.  
 *
 *****************************************************************
 * LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
 * DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
 *****************************************************************
 *
 * Modification Log:
 * -----------------
 * 08-07-95  tmm  initial development
 * 09-29-95  tmm  default number of cards is zero.
 * 12-04-03  tmm  converted to EPICS 3.14, cleaned up compiler warnings
 */

/************************************************************************
 *                 ATTENTION: APPLICATION DEVELOPER
 ************************************************************************
 * To use this device, include the following function call in the VxWorks
 * startup script, after the object module has been loaded and before the
 * call to iocInit():
 *
 * devHPLaserAxisConfig(ncards,a16base)
 *
 * For example:
 * devHPLaserAxisConfig(1,0x1000)
 *
 * Binary and multibit-binary INPUT support is provided for the (16-bit)
 * Status register only.  The "signal" value selects the bit number.
 *
 * Binary and multibit-binary OUTPUT support is provided for the (16-bit)
 * Control and Command registers.  The "signal" value selects both
 * the register and the bit number: signal values less than or equal to 15
 * select the Control register; signal values from 16 to 31 inclusive
 * select the Command register.
 *
 * Long INPUT support is provided for appropriate registers as follows:
 *	signal	register
 *	----------------
 *	0		Status (16 bit)
 *	1		Position (32 bit)
 *	2		SampledPosition (32 bit) (tell hardware to sample; read hardware)
 *
 * Long OUTPUT support is provided for appropriate registers as follows:
 *	signal	register
 *	----------------
 *	0		Control (16 bit)
 *	1		Command (16 bit)
 *	2		Offset (32 bit)
 *	3		Clip Mask (16 bit)
 *	4		Window Mask (16 bit)
 ***************************************************************************/

#include	<vxWorks.h>
#include	<sysLib.h>
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<string.h>
#include	<iv.h>
#include	<stdlib.h>
#include	<vxLib.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<link.h>
#include	<epicsMutex.h>

#include	<recGbl.h>
#include	<aiRecord.h>
#include	<aoRecord.h>
#include	<biRecord.h>
#include	<boRecord.h>
#include	<mbbiRecord.h>
#include	<mbboRecord.h>
#include	<longinRecord.h>
#include	<longoutRecord.h>

#include	<dbScan.h>
#include	<epicsExport.h>
#include	<iocsh.h>

#define STATIC static

void devHPLaserAxisConfig(int ncards, int a16base);

static const iocshArg devHPLaserAxisConfig_Arg0 = { "numCards",iocshArgInt};
static const iocshArg devHPLaserAxisConfig_Arg1 = { "a16BaseAddr",iocshArgInt};
static const iocshArg * const devHPLaserAxisConfig_Args[2] =
	{&devHPLaserAxisConfig_Arg0, &devHPLaserAxisConfig_Arg1};
static const iocshFuncDef devHPLaserAxisConfig_FuncDef = {"devHPLaserAxisConfig",2,devHPLaserAxisConfig_Args};
static void devHPLaserAxisConfig_CallFunc(const iocshArgBuf *args)
{
    devHPLaserAxisConfig(args[0].ival, args[1].ival);
}
void devHPLaserAxis_Register(void)
{
    iocshRegister(&devHPLaserAxisConfig_FuncDef, devHPLaserAxisConfig_CallFunc);
}
epicsExportRegistrar(devHPLaserAxis_Register);


STATIC int devHPLaserAxisReport();
STATIC long init(int flag);
STATIC long init_bi_record(struct biRecord *p);
STATIC long init_bo_record(struct boRecord *p);
STATIC long init_mbbi_record(struct mbbiRecord *p);
STATIC long init_mbbo_record(struct mbboRecord *p);
STATIC long init_li_record(struct longinRecord *p);
STATIC long init_lo_record(struct longoutRecord *p);
STATIC long read_bi(struct biRecord *p);
STATIC long write_bo(struct boRecord *p);
STATIC long read_mbbi(struct mbbiRecord *p);
STATIC long write_mbbo(struct mbboRecord *p);
STATIC long write_lo(struct longoutRecord *p);
STATIC long read_li(struct longinRecord *p);

volatile int devHPLaserAxisDebug = 0; 
epicsExportAddress(int, devHPLaserAxisDebug);

/* Create the DSET for devBiHP10895LaserAxis */
typedef struct {
	long            number;
	DEVSUPFUN       report;
	DEVSUPFUN       init;
	DEVSUPFUN       init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN       read;
} devBiHP10895LaserAxis_dset;

devBiHP10895LaserAxis_dset devBiHP10895LaserAxis = {
	5,
	NULL,
	NULL,
	init_bi_record,
	NULL,
	read_bi
};

epicsExportAddress(devBiHP10895LaserAxis_dset, devBiHP10895LaserAxis);

/* Create the DSET for devBoHP10895LaserAxis */
typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write;
} devBoHP10895LaserAxis_dset;

devBoHP10895LaserAxis_dset devBoHP10895LaserAxis = {
	5,
	(DEVSUPFUN) devHPLaserAxisReport,
	init,
	init_bo_record,
	NULL,
	write_bo
};

epicsExportAddress(devBoHP10895LaserAxis_dset, devBoHP10895LaserAxis);

/* Create the DSET for devMbbiHP10895LaserAxis */
typedef struct {
	long            number;
	DEVSUPFUN       report;
	DEVSUPFUN       init;
	DEVSUPFUN       init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN       read;
} devMbbiHP10895LaserAxis_dset;

devMbbiHP10895LaserAxis_dset devMbbiHP10895LaserAxis = {
	5,
	NULL,
	NULL,
	init_mbbi_record,
	NULL,
	read_mbbi
};

epicsExportAddress(devMbbiHP10895LaserAxis_dset, devMbbiHP10895LaserAxis);

/* Create the DSET for devMbboHP10895LaserAxis */
typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write;
} devMbboHP10895LaserAxis_dset;

devMbboHP10895LaserAxis_dset devMbboHP10895LaserAxis = {
	5,
	(DEVSUPFUN) devHPLaserAxisReport,
	init,
	init_mbbo_record,
	NULL,
	write_mbbo
};

epicsExportAddress(devMbboHP10895LaserAxis_dset, devMbboHP10895LaserAxis);

/* Create the DSET for devLiHP10895LaserAxis */
typedef struct {
	long            number;
	DEVSUPFUN       report;
	DEVSUPFUN       init;
	DEVSUPFUN       init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN       read;
} devLiHP10895LaserAxis_dset;

devLiHP10895LaserAxis_dset devLiHP10895LaserAxis = {
	5,
	NULL,
	NULL,
	init_li_record,
	NULL,
	read_li
};

epicsExportAddress(devLiHP10895LaserAxis_dset, devLiHP10895LaserAxis);

/* Create the DSET for devLoHP10895LaserAxis */
typedef struct {
	long            number;
	DEVSUPFUN       report;
	DEVSUPFUN       init;
	DEVSUPFUN       init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN       write;
} devLoHP10895LaserAxis_dset;

devLoHP10895LaserAxis_dset devLoHP10895LaserAxis = {
	5,
	NULL,
	NULL,
	init_lo_record,
	NULL,
	write_lo
};

epicsExportAddress(devLoHP10895LaserAxis_dset, devLoHP10895LaserAxis);

struct readHP10895 {
	volatile unsigned short Status;
	volatile unsigned short A24BaseAddress;
	volatile unsigned short Nothing;
	volatile unsigned short InterruptVector;
	volatile unsigned long Position;
	volatile unsigned long SampledPosition;
};

struct writeHP10895 {
	volatile unsigned short Control;
	volatile unsigned short A24BaseAddress;
	volatile unsigned short Command;
	volatile unsigned short InterruptVector;
	volatile unsigned long  Offset;
	volatile unsigned short ClipMask;
	volatile unsigned short WindowMask;
};

struct ioCard {
	volatile struct readHP10895 *pread; /* pointer to read registers */
	volatile struct writeHP10895 *pwrite; /* pointer to write registers */
	short exists;
	epicsMutexId lock;
};

static int baseAddress = 0x1000;
static int numCards = 0;
static struct ioCard *card;

STATIC int devHPLaserAxisReport()
{
	return(0);
}

void devHPLaserAxisConfig(int ncards, int a16base)
{
	numCards = ncards;
	card = (struct ioCard *) calloc(numCards,sizeof(struct ioCard));
    baseAddress = a16base;
    init(0);
}

STATIC long init(int flag)
{
	int cardNum;
	char *vmeAddress;
	unsigned short probeVal;
	volatile struct writeHP10895 *pw;

	if (flag != 0) return(OK);

	for (cardNum=0; cardNum < numCards; cardNum++) {
		card[cardNum].lock = epicsMutexCreate();
		vmeAddress = (char *) (baseAddress + cardNum*256);
		if (sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, vmeAddress, (char **)&pw) == ERROR) {
			if (devHPLaserAxisDebug >= 5) {
				printf("devHPLaserAxis: sysBusToLocalAdrs() returns ERROR for card %d\n", cardNum);
			}
			if (cardNum == 0) return(ERROR);
		}
		probeVal = 0x1000; /* turn on user LED, disable interrupt generation */
		if (vxMemProbe((char*) &(pw->Control), VX_WRITE, 2, (char *)&probeVal) < OK) {
			if (devHPLaserAxisDebug >= 5)
				printf("devHPLaserAxis:init: vxMemProbe write to %p failed (card %d)\n",
					&(pw->Control), cardNum);
			if (cardNum == 0) return(ERROR);
		} else {
			card[cardNum].exists = 1;
			card[cardNum].pwrite = pw;
			card[cardNum].pread = (struct readHP10895 *)pw;
		}
	}
	return(OK);
}

STATIC long init_bi_record(struct biRecord *p)
{
	int status = 0;
	int signal = p->inp.value.vmeio.signal;
	int cardNum = p->inp.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_bi_record) Bad INP field.");
		return(status);
	}

	switch (p->inp.type) {
	case (VME_IO) :
		if (signal > 15) {
			p->pact = 1;
			status = S_db_badField;
			if (devHPLaserAxisDebug >= 10)
				printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
			recGblRecordError(status,(void *)p,
				"devHPLaserAxis (init_bi_record) Bad INP field");
		} else {
			p->mask = 1 << signal;
		}
		break;

	default:
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_bi_record) Bad INP field");
	}
	return(status);
}

STATIC long init_bo_record(struct boRecord *p)
{
    int status = 0;
	int signal = p->out.value.vmeio.signal;
 	int cardNum = p->out.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_bo_record) Bad OUT field.");
		return(status);
	}

	switch (p->out.type) {
	case (VME_IO) :
		if (signal > 31) {
			p->pact = 1;
			status = S_db_badField;
			if (devHPLaserAxisDebug >= 10)
				printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
			recGblRecordError(status,(void *)p,
				"devHPLaserAxis (init_bo_record) Bad OUT field");
		} else {
			p->mask = 1 << (signal & 0xf);
		}
        break;
         
    default :
		p->pact = 1;
		status = S_db_badField;
        if (devHPLaserAxisDebug >= 10)
           printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_bo_record) Bad OUT field");
	}
	return(status);
}

STATIC long init_mbbi_record(struct mbbiRecord *p)
{
	int status = 0;
	int signal = p->inp.value.vmeio.signal;
 	int cardNum = p->inp.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_mbbi_record) Bad INP field.");
		return(status);
	}

	switch (p->inp.type) {
	case (VME_IO) :
	if (p->inp.value.vmeio.signal > 15) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_mbbi_record) Bad SIGNAL field");
	} else {
		p->shft = signal;
		p->mask <<= p->shft;
	}
	break;

	default:
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_mbbi_record) Bad INP field");
		return(status);
	}
	return(0);
}

STATIC long init_mbbo_record(struct mbboRecord *p)
{
	int status = 0;
	int signal = p->out.value.vmeio.signal;
 	int cardNum = p->out.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_mbbo_record) Bad OUT field.");
		return(status);
	}

	switch (p->out.type) {
	case (VME_IO) :
		if (signal > 31) {
			p->pact = 1;
			status = S_db_badField;
			if (devHPLaserAxisDebug >= 10)
				printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
			recGblRecordError(status,(void *)p,
				"devHPLaserAxis (init_mbbo_record) Bad OUT field");
		} else {
			p->shft = (signal & 0xf);
			p->mask <<= p->shft;
			if (devHPLaserAxisDebug >= 5)
				printf("devHPLaserAxis: mask = 0x%lx\n", p->mask);
		}
		break;

	default:
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_mbbo_record) Bad OUT field");
	}
	return(status);
}

STATIC long init_li_record(struct longinRecord *p)
{
	int status = 0;
	int signal = p->inp.value.vmeio.signal;
 	int cardNum = p->inp.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_li_record) Bad INP field.");
		return(status);
	}

	switch (p->inp.type) {
	case (VME_IO) :
		if (signal > 2) {
			p->pact = 1;
			status = S_db_badField;
			if (devHPLaserAxisDebug >= 10)
				printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
			recGblRecordError(status,(void *)p,
				"devHPLaserAxis (init_li_record) Bad INP field");
		}
        break;

  default:
     p->pact = 1;
     status = S_db_badField;
     if (devHPLaserAxisDebug >= 10)
        printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
     recGblRecordError(status,(void *)p,
        "devHPLaserAxis (init_li_record) Bad INP field");
   }
   return(status);
}

STATIC long init_lo_record(struct longoutRecord *p)
{
    int status = 0;
	int signal = p->out.value.vmeio.signal;
  	int cardNum = p->out.value.vmeio.card;

	if (cardNum >= numCards || !card[cardNum].exists) {
		p->pact = 1;
		status = S_db_badField;
		if (devHPLaserAxisDebug >= 10)
			printf("devHPLaserAxis: Bad card number ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_lo_record) Bad OUT field.");
		return(status);
	}

    switch (p->out.type) {
    case (VME_IO) :
		if (signal > 4) {
			p->pact = 1;
			status = S_db_badField;
			if (devHPLaserAxisDebug >= 10)
				printf("devHPLaserAxis: Bad signal number ->%s<- \n", p->name);
			recGblRecordError(status,(void *)p,
				"devHPLaserAxis (init_record) Bad OUT field");
		}
        break;
         
    default :
		p->pact = 1;
		status = S_db_badField;
        if (devHPLaserAxisDebug >= 10)
           printf("devHPLaserAxis: Bad I/O type ->%s<- \n", p->name);
		recGblRecordError(status,(void *)p,
			"devHPLaserAxis (init_record) Bad OUT field");
	}
	return(status);
}

/*****************************************************************************/
STATIC long read_bi(struct biRecord *p)
{
	int cardNum = p->inp.value.vmeio.card;
	
	p->rval = card[cardNum].pread->Status & p->mask;
	return(0);
}

STATIC long write_bo(struct boRecord *p)
{
	int signal = p->out.value.vmeio.signal;
	int cardNum = p->out.value.vmeio.card;

	epicsMutexMustLock(card[cardNum].lock);
	if (signal > 15 ) {
		card[cardNum].pwrite->Command = p->rval;
	} else {
		card[cardNum].pwrite->Control = p->rval;
	}
	epicsMutexUnlock(card[cardNum].lock);
	p->rbv = p->rval;
	return(0);
}

STATIC long read_mbbi(struct mbbiRecord *p)
{
	int cardNum = p->inp.value.vmeio.card;

	p->rval = card[cardNum].pread->Status & p->mask;
	return(0);
}

STATIC long write_mbbo(struct mbboRecord *p)
{
	int signal = p->out.value.vmeio.signal;
	int cardNum = p->out.value.vmeio.card;

	epicsMutexMustLock(card[cardNum].lock);
	if (signal > 15 ) {
		card[cardNum].pwrite->Command = (p->rval & p->mask);
	} else {
		card[cardNum].pwrite->Control = (p->rval & p->mask);
	}
	epicsMutexUnlock(card[cardNum].lock);
	p->rbv = (p->rval & p->mask);
	return(0);
}

STATIC long read_li(struct longinRecord *p)
{
	int signal = p->inp.value.vmeio.signal;
	int cardNum = p->inp.value.vmeio.card;

	switch (signal) {
	case 0:
		p->val = (long) card[cardNum].pread->Status;
		break;
	case 1:
		p->val = (long) card[cardNum].pread->Position;
		break;
	case 2:
		p->val = (long) card[cardNum].pread->SampledPosition;
		break;
	default:
		break;
	}
	return(0);
}

STATIC long write_lo(struct longoutRecord *p)
{
	int signal = p->out.value.vmeio.signal;
	int cardNum = p->out.value.vmeio.card;

	epicsMutexMustLock(card[cardNum].lock);
	switch (signal) {
	case 0:
		card[cardNum].pwrite->Control = (unsigned short)(p->val&0xff);
		break;
	case 1:
		card[cardNum].pwrite->Command = (unsigned short)(p->val&0xff);
		break;
	case 2:
		card[cardNum].pwrite->Offset = (unsigned long) p->val;
		break;
	case 3:
		card[cardNum].pwrite->ClipMask = (unsigned short)(p->val&0xff);
		break;
	case 4:
		card[cardNum].pwrite->WindowMask = (unsigned short)(p->val&0xff);
		break;
	default:
		break;
	}
	epicsMutexUnlock(card[cardNum].lock);
	return(0);
}
