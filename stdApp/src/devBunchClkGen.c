/* devBunchClkGen.c -  */
/* SR Bunch Clock Generator  */
/*
 * Author:      Frank Lenkszus
 * Date:        11/14/95
 *
 *      Experimental Physics and Industrial Control System (EPICS)
*/
/*
*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
AND IN ALL SOURCE LISTINGS OF THE CODE.
 
(C)  COPYRIGHT 1995 UNIVERSITY OF CHICAGO
 
Argonne National Laboratory (ANL), with facilities in the States of 
Illinois and Idaho, is owned by the United States Government, and
operated by the University of Chicago under provision of a contract
with the Department of Energy.

Portions of this material resulted from work developed under a U.S.
Government contract and are subject to the following license:  For
a period of five years from March 30, 1995, the Government is
granted for itself and others acting on its behalf a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, and perform
publicly and display publicly.  With the approval of DOE, this
period may be renewed for two additional five year periods. 
Following the expiration of this period or periods, the Government
is granted for itself and others acting on its behalf, a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, distribute copies
to the public, perform publicly and display publicly, and to permit
others to do so.

*****************************************************************
                                DISCLAIMER
*****************************************************************

NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
OWNED RIGHTS.  

*****************************************************************
LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
*/
/*
* Modification Log:
 * -----------------
 * .01  11-14-95        frl     initial
 */

#include     <vxWorks.h>
#include     <stdlib.h>
#include     <stdio.h>
#include     <string.h>

#include <vme.h>
#include <iv.h>
#include <semLib.h>
#include <errno.h>
#include <taskLib.h>
#include <drvSup.h>
#include <devLib.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <link.h>
#include <fast_lock.h>

#define MAX_NUM_CARDS	4
#define NO_ERR_RPT	-1
#define FBUSERR		-2
#define MAX_SIGNAL   4
#define PWR_CYCLED_BIT 0x8

#define FREG_READ	1
#define FCSR_READ	FREG_READ +1
#define FREG_WRITE	FCSR_READ + 1
#define FREG_SETBITS	FREG_WRITE + 1
#define FREG_RESETBITS 	FREG_SETBITS + 1
#define FREG_WRITEBITS	FREG_RESETBITS + 1
#define FREG_READLONG	FREG_WRITEBITS +1
#define FREG_WRITELONG  FREG_READLONG +1

int drvBunchClkGenDebug = 0;
static   int	*drvDebug = &drvBunchClkGenDebug;

static long 	drvInitCard();
static long	drvIoReport();
static long check_card( unsigned short card, unsigned short signal);
static long drvReadCard(short card, short funct, short signal,
	 unsigned long	*pp1);
static long drvWriteCardBit( unsigned short card, unsigned short funct,
	unsigned short signal, unsigned short value, unsigned short mask);
static long drvReadRam(short card);
static long drvCopyRam(short card);
static long drvClearRam(short card);

/* Epics driver entry point table */

struct drvet drvBunchClkGen={
  2,
  drvIoReport,    
  drvInitCard
};


static char	*drvName="drvBunchClkGen";

/* device register map */
struct drv_regr {
  unsigned short csr;
  unsigned short address;
  unsigned short data;
  unsigned short fineDelay;
  unsigned short P0Delay;
};

struct drv_regw {
  unsigned short csr;
  unsigned short address;
  unsigned short data;
  unsigned short fineDelay;
  unsigned short P0Delay;
};
struct drvCard  {
  union {
	struct drv_regr	r;
	struct drv_regw	w;
  } reg;
};

/*
 * Define all per-card private variables here.
 */
struct drvPrivate {
	volatile struct	drvCard  *dptr;
	FAST_LOCK	lock;
	int		init_hw;
	unsigned short	ramW[1296];
	unsigned short  ramR[1296];
};


static struct  drvPrivate      Card[MAX_NUM_CARDS];
static struct drvPrivate *dio = Card;
static  int     NumCards = 0; /* User configurable # of cards */
static int ConfigureLock = 0;

/*************************************************************************/
int BunchClkGenConfigure(
int	Card,
unsigned long CardAddress)
{
	char *xname = "BunchClkGenConfigure";
	unsigned short junk;

   if(ConfigureLock != 0) {
	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
	"%s: Cannot change configuration after init -- Request ignored", xname);
	return(ERROR);
   }
   if (Card >= MAX_NUM_CARDS || Card < 0)  {
	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
	"%s: Card number %d invalid -- must be 0 to %d", xname, Card, 
		MAX_NUM_CARDS-1);
	return(ERROR);
   }
   if (CardAddress > 0xffff)  {
	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
	"%s: Card (%d)  address 0x%x invalid -- must be 0 to 0xffff", xname,
		 Card, CardAddress); 
	return(ERROR);
   }
   if ( sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, (char *)CardAddress, 
		(char **)&dio[Card].dptr) != OK) {
	dio[Card].dptr = NULL;
	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
	"%s: A16 Address map failed for Card %d", xname, Card); 
	return(ERROR);
   }
   if ( vxMemProbe( (char *)dio[Card].dptr, READ, 2, (char *)&junk) != OK ) {
        dio[Card].dptr = NULL;
	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
	"%s: vxMemProbe failed for Card %d", xname, Card); 
	return(ERROR);
   }
   if ( Card >= NumCards)
	NumCards = Card +1;
   return(OK);
}

/******************************************************************************
 *
 * Initialize all cards controlled by the  driver.
 *
 ******************************************************************************/
static long drvInitCard()
{
  long			status = 0;
  int i;
  unsigned short val;

   ConfigureLock = 1;      /* prevent Configure calls after init */

  for(i = 0; i < NumCards; i++) {
	   if(check_card(i, 0) == OK) {
		val = dio[i].dptr->reg.r.csr;
		if( (val & PWR_CYCLED_BIT) != 0) {
			dio[i].init_hw = 1;	/* not power cycled */
			drvReadRam(i);
			drvCopyRam(i);  /* initialize driver copy of
					 ram with hardware */
		} else {
			dio[i].init_hw = 0;	/* power cycled */
			dio[i].dptr->reg.w.csr = val | PWR_CYCLED_BIT;
			drvClearRam(i);
		}	
	   }
  }
  return (status);
 
}

/*************************************************************************/

static long drvIoReport(
int	level)
{
	int i;

	for ( i =0 ; i <  MAX_NUM_CARDS; i++) {

		if ( dio[i].dptr) {
			printf(
                    "%s:\tcard %d\tcsr = 0x%x, P0 delay = 0x%x, fine Delay = 0x%x\n",
			drvName, i, 
			dio[i].dptr->reg.r.csr, 
			dio[i].dptr->reg.r.P0Delay, 
			dio[i].dptr->reg.r.fineDelay) ;
		}
	}
	return(OK);
}

/***************************************************************************/

static long check_card(
unsigned short card,
unsigned short signal)
{
	if (card >= MAX_NUM_CARDS){
		if(*drvDebug)
                      errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                         "%s: Bad Card %d", drvName, card);
		return(ERROR);
	}
	if ( signal > MAX_SIGNAL) {
		if(*drvDebug)
                      errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                        "%s: Bad Signal %d for card %d", drvName, signal, card);
		return(ERROR);
	}
	if (!dio[card].dptr) {
		if(*drvDebug)
                      errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                        "%s: Can't access card %d", drvName,  card);
		return(ERROR);
	}
	return(OK);
}


/***************************************************************************
 * 
 *   Perform register writes
 *
 ***************************************************************************/
static long drvWriteCard(
short	card,
short	funct,	
short   signal,	
unsigned long	*pp1)		/* pointer to param 1 */
{
    	unsigned long	status;

	if ( (status=check_card(card, signal)) != OK)
		return(status);


	switch(funct) {
		case (FREG_WRITE) :
			*((unsigned short *)dio[card].dptr + signal) = *pp1;
			break;
                case (FREG_WRITELONG) :
                         *((unsigned short *)dio[card].dptr + signal) =
                                 (unsigned short)*pp1;
                         *((unsigned short *)dio[card].dptr + signal+1) =
                                 (unsigned short)(*pp1>>16);
                        break;
		default :
			if(*drvDebug) 
				errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
				"%s: Unknown write function", drvName);
			break;
	}

	return(OK);
}

/***************************************************************************
 * 
 *   Perform register reads
 *
 ***************************************************************************/
static long drvReadCard(
short	card,
short	funct,	
short   signal,
unsigned long	*pp1)		/* pointer to param 1 */
{
	unsigned  short		val, val1;
    	unsigned long	status;

	if ( (status=check_card(card, signal)) != OK)
		return(status);


	switch(funct) {
		case (FCSR_READ) :
			*pp1 = *((unsigned short *)dio[card].dptr);
			break;
		case (FREG_READ) :
			*pp1 = *((unsigned short *)dio[card].dptr + signal);
			break;
                case (FREG_READLONG) :
                        val = *((unsigned short *)dio[card].dptr + signal);
                        val1 = *((unsigned short *)dio[card].dptr + signal+1);
                        *pp1 = (((unsigned long)val1)<<16) +val;
                        break;
		default :
			if(*drvDebug) 
				errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
				"%s: Unknown read function", drvName);
			break;
	}

	return(OK);
}


/***************************************************************************
 * 
 *   write selected bits in a register
 *
 ***************************************************************************/
static long drvWriteCardBit(
unsigned short card,
unsigned short funct,
unsigned short signal,
unsigned short	value,		/* mask */
unsigned short	mask)		/* value */
{
	unsigned  short		tmp;
    	unsigned long	status;

	if ( (status=check_card(card, signal)) != OK)
		return(status);

	switch(funct) {
		case (FREG_WRITEBITS) :
			FASTLOCK(&dio[card].lock);
			tmp = *((unsigned short *)dio[card].dptr + signal); 
			 tmp = (tmp & ~(mask)) | value;
			*((unsigned short *)dio[card].dptr + signal) = tmp;
			FASTUNLOCK(&dio[card].lock);
			break;
		default :
			if(*drvDebug) 
				errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
				"%s: Unknown read function", drvName);
			return(ERROR);
			break;
	}
	if(*drvDebug) {
		errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
		"%s: Wrote 0x%x (mask = 0x%x) to card %d, signal %d",
		drvName, value, mask, card, signal);
	}
	return(OK);
}

/***************************************************************************
 * 
 *   write/readback  selected bits in reg
 *
 ***************************************************************************/
static long drvWriteReadBit(
short	card,		/* */
short   signal,		/* signal == register # */
unsigned short	mask,		/* mask */
unsigned short	value,		/* value to write */
unsigned short  shift,		/* num to left shift */
unsigned short *prbval)		/* place to put readback */
{
	unsigned  short		val;
    	unsigned long	status;


	if ( (status=check_card(card, signal)) != OK)
		return(status);


	FASTLOCK(&dio[card].lock);
	 val = *((unsigned short *)dio[card].dptr + signal);
	 val = (val & ~(mask)) | (value << shift);
	*((unsigned short *)dio[card].dptr + signal) = val;
	FASTUNLOCK(&dio[card].lock);
	*prbval =
	     ((*((unsigned short *)dio[card].dptr + signal)) & mask) >> shift;
	return(OK);
}


/******************************************************************************
 *
 * Place any user-required functions here.
 *
 ******************************************************************************/

#define BCGDisable 0x04
#define RamSize 1296/16

/***********************************************************************
 *
 * Write Ram location
 *
 * Any sanity checking must be done prior to calling this routine
 *
 ***********************************************************************/
static long drvWriteRamLoc(
short card,
int location,
unsigned short val)
{
	unsigned short csr;

	FASTLOCK(&dio[card].lock);
	csr = dio[card].dptr->reg.r.csr;
	dio[card].dptr->reg.w.csr = csr | BCGDisable;
	dio[card].dptr->reg.w.address = location;
	dio[card].dptr->reg.w.data = val;
	dio[card].dptr->reg.w.csr = csr;
	FASTUNLOCK(&dio[card].lock);
	
	if(*drvDebug > 10)
		errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
			"%s:drvWriteRamLoc: address = %d, val = 0x%x\n",
			 drvName, location, val);
	return(0);
}
/***********************************************************************
 *
 * Write Ram
 *
 ***********************************************************************/
static long drvWriteRam(
short	card)
{
	int i;
	unsigned short csr;
	unsigned long status;

        if ( (status=check_card(card, 0)) != OK)
                return(status);

	FASTLOCK(&dio[card].lock);
	csr = dio[card].dptr->reg.r.csr;
	dio[card].dptr->reg.w.csr = csr | BCGDisable;
	dio[card].dptr->reg.w.address = 0;
	for (i=0; i< RamSize; i++ ) {
		dio[card].dptr->reg.w.data = dio[card].ramW[i];
	}
	dio[card].dptr->reg.w.csr = csr;
	FASTUNLOCK(&dio[card].lock);
	return(OK);
}

/***********************************************************************
 *
 * Read Ram
 *
 ***********************************************************************/
static long drvReadRam(
short	card)
{
	int i;
	unsigned short csr;
	unsigned long status;

        if ( (status=check_card(card, 0)) != OK)
                return(status);

	FASTLOCK(&dio[card].lock);
	csr = dio[card].dptr->reg.r.csr;
	dio[card].dptr->reg.w.csr = csr | BCGDisable;
	dio[card].dptr->reg.w.address = 0;
	for (i=0; i< RamSize; i++ ) {
		dio[card].ramR[i] = dio[card].dptr->reg.r.data;
	}
	dio[card].dptr->reg.w.csr = csr;
	FASTUNLOCK(&dio[card].lock);
	return(OK);
}
/***********************************************************************
 *
 *  Copy Read Ram contents to Write Ram Contents
 *
 ***********************************************************************/
static long drvCopyRam(
short	card)
{
	int i;
	unsigned long status;

        if ( (status=check_card(card, 0)) != OK)
                return(status);

	for (i=0; i< RamSize; i++ ) {
		dio[card].ramW[i] = dio[card].ramR[i];
	}
	return(OK);
}

/***********************************************************************
 *
 * Clear Ram
 *
 ***********************************************************************/
static long drvClearRam(
short	card)
{
	int i;
	unsigned short csr;
	unsigned long status;

        if ( (status=check_card(card, 0)) != OK)
                return(status);

	FASTLOCK(&dio[card].lock);
	csr = dio[card].dptr->reg.r.csr;
	dio[card].dptr->reg.w.csr = csr | BCGDisable;
	dio[card].dptr->reg.w.address = 0;
	for (i=0; i< RamSize; i++ ) {
		dio[card].ramW[i] = 0;
		dio[card].ramR[i] = 0;
		dio[card].dptr->reg.w.data = 0;
	}
	dio[card].dptr->reg.w.csr = csr;
	FASTUNLOCK(&dio[card].lock);
	return(OK);
}

/**************************************************************************/
static long drvListBuckets(
int	card,
short	*buf,
int	max,
int	*actual)
{
	int	num, j, ramloc;
	short bucket;
	unsigned short val;
	int notDone;
	
	notDone = 1;
	ramloc = num = bucket = 0;
	while ( notDone) {
		val = dio[card].ramR[ramloc++];
		for ( j=0; j < 16; j++) {
			if (val & 0x01) {
				*buf++ = bucket;
				num++;
			}
			if( num >= max || ++bucket >=1296){
				notDone = 0;
				break;
				}
			val >>= 1;
		}
	}
	*actual = num; 
	return(0);
}

/**************************************************************************/
static long drvShowPattern(
int	card,
short	*buf,
int	max,
int	*actual)
{
	int	num, j, ramloc;
	unsigned short val;
	int notDone;
	
	notDone = 1;
	ramloc = num = 0;
	while ( notDone) {
		val = dio[card].ramR[ramloc];
		for ( j=0; j < 16; j++) {
			if (val & 0x01) 
				*buf++ = 1;
			else
				*buf++ = 0;
			if( ++num >= max || ramloc >=RamSize){
				notDone = 0;
				break;
				}
			val >>= 1;
		}
		ramloc++;
	}
	*actual = num; 
	return(0);
}

/*************************************************************************/
long	drvBunchClkGenDump( int card)
{
#define PERLINE 16
	int i, j, bucket, num, ramloc;
	int notDone;
	unsigned short val;
	unsigned short buckets[PERLINE];


	if( drvReadRam(card) != OK) {
		printf("Error on accessing card\n");
		return(-1);
	}
	printf("P0 Delay = 0x%x, Fine Delay = 0x%x\n",
		 dio[card].dptr->reg.r.P0Delay, dio[card].dptr->reg.r.fineDelay);
	
	notDone = 1;
	bucket = 0;
	ramloc = 0;
	j = 16;
	val = 0;
	while ( notDone) {
		num = 0;
		while ( num < PERLINE && notDone) {
			if ( j >= 16) {
				val = dio[card].ramR[ramloc++];
				j = 0;
			}
			for(; j < 16 && num < PERLINE; j++, bucket++) {
				if(val & 0x01) {
					buckets[num++] = bucket;
				}
				val >>= 1;
			}
			if(ramloc >= RamSize && j >= 16)
				notDone = 0;
		}
		for( i =0; i < num; i++)
			printf("%4.4d ", buckets[i]);
		printf("\n");
	}
	return(0);
}
/*************************************************************************/
long	drvBunchClkGenDumpRam( int card)
{
#define PERLINE1 10
	int i, ramloc;
	int notDone;
	unsigned short val;


	if( drvReadRam(card) != OK) {
		printf("Error on accessing card\n");
		return(-1);
	}
	printf("P0 Delay = 0x%x, Fine Delay = 0x%x\n",
		 dio[card].dptr->reg.r.P0Delay, dio[card].dptr->reg.r.fineDelay);
	
	notDone = 1;
	ramloc = 0;
	val = 0;
	while ( notDone) {
		printf("%4.4d  ", ramloc);
		for( i= 0; (i < PERLINE1) && notDone ; i++, ramloc++) {
			val = dio[card].ramR[ramloc++];
			printf("0x%4.4x ", val);
			if(ramloc >= RamSize )
				notDone = 0;
		}
		printf("\n");
	}
	return(0);
}

/***************************************************************************
 *
 *  Write a bucket in Ram
 *
 **************************************************************************/
static long drvWriteBucket(
short	card,
short	bucket,
short	value)
{
	int ramloc;
	int bit;
	unsigned short mask, val, tmp;
	unsigned long status;

        if ( (status=check_card(card, 0)) != OK)
                return(status);
	if( bucket >= 1296) {
		if(*drvDebug)
			errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
			"%s: bucket number >=1296\n", drvName);
	}
	ramloc = bucket/16;
	bit = bucket % 16;
	mask = 0x01 << bit;
	if (value)
		val = mask;
	else
		val = 0;
	tmp = (dio[card].ramW[ramloc] & ~mask) | val;
	printf("value = %d, val = %d tmp = 0x%x\n", value, val, tmp);
	dio[card].ramW[ramloc] = tmp;
	drvWriteRamLoc(card, ramloc, dio[card].ramW[ramloc]);
	return(0);
}

/*************************************************************************/
static int drvGetPwrOnStatus(short card)
{

        if ( check_card(card, 0) != OK)
                return(-1);
	else
		return(dio[card].init_hw);
	
}

/******************************************************************************
 *
 *  "Device"  Support stuff follows
 *
 *****************************************************************************/
/* create the dsets */

#include	<alarm.h>
#include	<devSup.h>
#include	<recSup.h>
#include        <biRecord.h>
#include        <boRecord.h>
#include 	<aoRecord.h>
#include 	<aiRecord.h>
#include	<waveformRecord.h>

#define         PARAM_BIT_FIELD 1
#define         PARAM_MASK 2
#define		PARAM_ASCII 3

#define		PARAM_BIT 1
#define		PARAM_SHORT 2
#define		PARAM_LONG 3
#define         PARAM_MBB 4

#define		PARAM_RW 1
#define		PARAM_RO 2
#define		PARAM_WO 3

#define		NUMCHANNELS 0
#define		CHANNELREGS 1

static char	*devName="devBunchClkGen";

int	devBunchClkGenDebug	= 0;
static  int *devDebug = &devBunchClkGenDebug;

static int      init_hw = 0;    /* flag to init  modules */

static long initBiRecord();
static long readBi();
static long initBoRecord();
static long writeBo();
static long initAoRecord();
static long writeAo();
static long specialLinconvAo();
static long initAiRecord();
static long readAi();
static long specialLinconvAi();
static long initWfRecord();
static long readWf();

typedef struct {
	long		number;
	DEVSUPFUN	report;
    	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN   	get_ioint_info;
	DEVSUPFUN	read_write;
} DSET;
DSET devBiBunchClkGen={ 5, NULL, NULL, initBiRecord, NULL, readBi };
DSET devBoBunchClkGen ={ 5, NULL, NULL, initBoRecord, NULL, writeBo };

typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} DSETA;
DSETA devAoBunchClkGen = {6, NULL, NULL, initAoRecord, NULL, writeAo, specialLinconvAo};
DSETA devAiBunchClkGen = {6, NULL, NULL, initAiRecord, NULL, readAi, specialLinconvAi};
DSETA devWfBunchClkGen = {6, NULL, NULL, initWfRecord, NULL, readWf, NULL};

static struct	paramEntrys{
	char	*param;
	short	type;
	short	access;
	short	signal;
	short	bit;
	short	shift;
	short	special;
	unsigned long	mask;
}  paramEntry[] = {
"Running", PARAM_BIT, PARAM_RO, 0, 0, 0, 0, 0x0,
"P0_Detect", PARAM_BIT, PARAM_RO, 0, 1, 0, 0, 0x0,
"Disable", PARAM_BIT, PARAM_RW, 0, 2, 0, 0, 0xfff4,
"PwrCycle", PARAM_BIT, PARAM_RW, 0, 3, 0, 0, 0xfff8,
"InitRam", PARAM_BIT, PARAM_WO, 1, 0, 0, 0, 0x1,
"WriteRam", PARAM_BIT, PARAM_WO, 2, 0, 0, 0, 0x1
};
static struct paramTbls{
	int	num;
	struct	paramEntrys *pentry;
} paramTbl = {
	sizeof(paramEntry)/sizeof(struct paramEntrys),
	paramEntry
};

struct PvtBi {
	short  signal;
	short  special;
};


struct PvtBo {
	unsigned short mask;
	short  signal;
};

static long init1(after)
int     after;
{
                init_hw = 0;
        return(0);
}

/********************************************************************/
static long lookUpParam(char *parm, unsigned long *pval)
{
	int i;
	char	*xname="lookUpParam";

	if( parm == NULL) {
		errPrintf(NO_ERR_RPT, __FILE__, __LINE__, 
		"%s: NULL pointer encounterd for param", xname);
		return(ERROR);
	}
	for(i = 0; i < paramTbl.num; i++) {
		if(strcmp(parm, paramTbl.pentry[i].param) == 0) {
			*pval = i;
			break;
		}
	}			
	if( i >= paramTbl.num )
		return(ERROR);
	return(OK);

}

/********************************************************************/

static long initParam(
char	*parm,
unsigned long *pval,
unsigned short mode)
{
	unsigned short val;
	unsigned long	val1;
	char 	*xname="initParam";

  if( parm == NULL) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
		"%s: NULL pointer for Parameter String", xname);
	}
      return(ERROR);
  }
  val = 1;
  switch( mode) {
    case (PARAM_BIT_FIELD) : 
	if ( *parm >= '0' && *parm <= '9') {
		*pval = val << (*parm - '0');
		return(OK);
	} else if ( *parm >= 'a' && *parm <= 'f') {
		*pval = val << ( *parm - 'a' + 10 );
		return(OK);
	} else if ( *parm >= 'A' && *parm <= 'F') {
		*pval = val << ( *parm - 'A' + 10 );
		return(OK);
	} else
		return(ERROR);
	break;
    case ( PARAM_MASK) :
	if ( sscanf(parm, "%lx", &val1) != 1 ) {
      		if (*devDebug) {
     			errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
				"%s: Bad Parameter = %s ", xname, parm);
		}
      		return(ERROR);
        }
	*pval = val1;
	return(OK);
	break;
    case ( PARAM_ASCII) :
		return(lookUpParam(parm, pval));
	break;
    default :
	return(ERROR);
	break;
}

}

/*********************************************************************/

static long initBiRecord(
    struct biRecord	*pbi)
{
  struct vmeio *pvmeio;
  struct PvtBi *ptr;
  long status;
  unsigned short *pReg;
  unsigned short val;
  unsigned long lval;
  unsigned long lval1;

  /* bi.inp must be an VME_IO */

  switch (pbi->inp.type) {
  case (VME_IO) :
    pvmeio = (struct vmeio *)&(pbi->inp.value);
    break;
  default :
    recGblRecordError(S_dev_badInpType, (void *)pbi,
        "devBiBunchClkGen (initBiRecord) : not a VME device!!!");
	return(S_dev_badInpType);
  }

  /* call driver so that it configures card */
  if ((status = check_card(pvmeio->card, pvmeio->signal)) != OK) {
     recGblRecordError(S_dev_badCard,(void *)pbi,
         "devBiBunchClkGen (initBiRecord) : init failed!!!");
     return(S_dev_badCard);
  } 
/* check param */
   if ( initParam(pvmeio->parm, &lval, PARAM_ASCII) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devBiBunchClkGen: card = 0x%x param = %s (BAD PARAM)\n",
        	   pvmeio->card, pvmeio->parm);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pbi,
         	"devBiBunchClkGen (initBiRecord) : init failed!!!");
     	return(S_dev_badSignal);
   }
   lval1=1;
   pbi->mask = (lval1 << paramTbl.pentry[lval].bit);
  if(( pbi->dpvt = malloc(sizeof( struct PvtBi))) == NULL ) {
      if (*devDebug) {
        errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                "devBiBunchClkGen: card = %d  sig = %d (Malloc failed)\n",
                   pvmeio->card, pvmeio->signal);
        }
        recGblRecordError(S_dev_noMemory,(void *)pbi,
                "devBiBunchClkGen (initBiRecord) : init failed!!!");
        return(S_dev_noMemory);
   }
   ptr = ( struct PvtBi *)pbi->dpvt;
   if( pvmeio->signal < NUMCHANNELS  ) {
   	ptr->signal = (paramTbl.pentry[lval].signal + 
			pvmeio->signal*CHANNELREGS);
   } else {
   	ptr->signal = paramTbl.pentry[lval].signal;
   }
  ptr->special = paramTbl.pentry[lval].special;

/*  Check Register Exists */ 
   pReg = (unsigned short *) dio[pvmeio->card].dptr + ptr->signal;
   if (vxMemProbe( (char *)pReg, READ, sizeof(val), (char *)&val) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
     	  "devBiBunchClkGen: vxMemProbe failed!!! card = 0x%x, sig = 0x%x\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pbi,
         	"devBiBunchClkGen (initBiRecord) : init failed!!!");
   	pbi->dpvt = NULL;
     	return(S_dev_badSignal);
   }
  return(0);
}

/*****************************************************************/
static long readBi(
    struct biRecord	*pbi)
{
  unsigned  long value;
  struct vmeio *pvmeio = (struct vmeio *)&(pbi->inp.value);
  struct PvtBi *ptr;

  if(!pbi->dpvt)
	return(S_dev_NoInit);
   ptr = ( struct PvtBi *)pbi->dpvt;
   
	 if(drvReadCard( pvmeio->card, FREG_READ, ptr->signal, &value)
		!= OK) {
      		if (*devDebug) {
     			errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devBiBunchClkGen (readBi) : read Error, card = %d, sig = %d",
			 pvmeio->card, pvmeio->signal);
      		}
    		recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM);
    		return (2); /* don't convert */
    	}
  
  pbi->rval = ((unsigned long)value) & pbi->mask;
      if (*devDebug >= 10) {
       printf("devBunchClkGen: card = %d, signal = %d, raw read value = 0x%x\n",
			pvmeio->card, ptr->signal,  pbi->rval);
      }
  return(0);
}

/************************************************************************/
static long initBoRecord(
    struct boRecord	*pbo)
{
  unsigned short value;
  unsigned long lval;
  unsigned long lval1;
  unsigned short *pReg;
  struct vmeio *pvmeio;
  struct PvtBo *ptr;
  long status;

  /* bo.out must be an VME_IO */

  switch (pbo->out.type) {
  case (VME_IO) :
    pvmeio = (struct vmeio *)&(pbo->out.value);
    break;
  default :
    recGblRecordError(S_dev_badOutType, (void *)pbo,
        "devBoBunchClkGen (initBorecord)");
	return(S_dev_badOutType);
  }

  /* call driver so that it configures card */
  if ((status = check_card(pvmeio->card, pvmeio->signal)) != OK) {
     recGblRecordError(S_dev_badCard,(void *)pbo,
         "devBoBunchClkGen (initBorecord) : init failed!!!");
     return(S_dev_badCard);
  } 

/* check param */
   if ( initParam(pvmeio->parm, &lval, PARAM_ASCII) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devBoBunchClkGen: card = %d Sig = %d  PARM = %s (BAD PARAM)\n",
        	   pvmeio->card, pvmeio->signal,  pvmeio->parm);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pbo,
         	"devBoBunchClkGen (initBoRecord) : init failed!!!");
     	return(S_dev_badSignal);
   }
   lval1 = 1;
   pbo->mask = (lval1 << paramTbl.pentry[lval].bit);
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devBoBunchClkGen: card = %d sig = %d  PARM = %s mask = %d",
        	   pvmeio->card, pvmeio->signal,  pvmeio->parm, pbo->mask);
      }
  if(( pbo->dpvt = malloc(sizeof( struct PvtBo))) == NULL ) {
      if (*devDebug) {
        errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                "devBoBunchClkGen: card = %d  sig = %d (Malloc failed)\n",
                   pvmeio->card, pvmeio->signal);
        }
        recGblRecordError(S_dev_noMemory,(void *)pbo,
                "devBoBunchClkGen (initBoRecord) : init failed!!!");
        return(S_dev_noMemory);
   }
   ptr = ( struct PvtBo *)pbo->dpvt;
   ptr->mask = (unsigned short)(paramTbl.pentry[lval].mask);
   if( pvmeio->signal < NUMCHANNELS  ) {
        ptr->signal = (paramTbl.pentry[lval].signal +
                        pvmeio->signal*CHANNELREGS);
   } else {
        ptr->signal = paramTbl.pentry[lval].signal;
   }

/*  Check Register Exists */ 
   pReg = (unsigned short *) dio[pvmeio->card].dptr + ptr->signal;
   if (vxMemProbe( (char *)pReg, READ, sizeof(value), (char *)&value) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    	     "devBoBunchClkGen: vxMemProbe failed!!! card = %d, sig = %d\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pbo,
         	"devBoBunchClkGen (initBorecord) : init failed!!!");
	pbo->dpvt=NULL;
     	return(S_dev_badSignal);
   }
   if(ptr->signal > 0)  /* only signal 0 corresponds to hardware reg
				signals 1 and 2 are virtual */
   	pbo->rval = pbo->rbv = 0;
   else
   	pbo->rval = pbo->rbv = (unsigned long)value & pbo->mask;
  if( drvGetPwrOnStatus(pvmeio->card) == 1)
  	return(0);	/* initialize to readback value */	
  else
	return(2);	/* don't convert */
}

/**************************************************************************/
static long writeBo(
    struct boRecord *pbo)
{
  unsigned short value;
  unsigned short mask;
  unsigned short *pReg;
  struct vmeio *pvmeio = (struct vmeio *) & (pbo->out.value);
  struct PvtBo *ptr;

  if(!pbo->dpvt)
	return(S_dev_NoInit);
   ptr = (struct PvtBo *)pbo->dpvt;
   value = (unsigned short) pbo->rval;
   mask = ptr->mask;
   if ( ptr->signal == 1) {
	drvClearRam(pvmeio->card);
	return(0);
   } else if ( ptr->signal == 2) {
	drvWriteRam(pvmeio->card); 
	return(0);
   }
   if(drvWriteCardBit( pvmeio->card, FREG_WRITEBITS, ptr->signal,
		 value, mask) != OK) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devBoBunchClkGen (writeBo) : Write Error, card = %d", pvmeio->card);
      }
      recGblSetSevr(pbo,WRITE_ALARM,INVALID_ALARM);
      return (2); /* don't convert */
   }
   pReg = (unsigned short *) dio[pvmeio->card].dptr + ptr->signal;
   value = *pReg;
   pbo->rbv = (unsigned long)value & pbo->mask;
      if (*devDebug >= 10) {
	printf("devBoBunchClkGen: card = %d, signal = %d,  write value = 0x%x\n",
			pvmeio->card, ptr->signal,  pbo->rval);
      }
  return(0);
}
/***************************************************************************/
static long initAoRecord(
    struct aoRecord	*pao)
{
  unsigned short value;
  unsigned short *pReg;
  struct vmeio *pvmeio;
  long status;

  /* ao.out must be an VME_IO */

  switch (pao->out.type) {
  case (VME_IO) :
    pvmeio = (struct vmeio *)&(pao->out.value);
    break;
  default :
    recGblRecordError(S_dev_badOutType, (void *)pao,
        "devAoBunchClkGen (initAoRecord)");
	return(S_dev_badOutType);
  }

  /* call driver so that it configures card */
  if ((status = check_card(pvmeio->card, pvmeio->signal)) != OK) {
     recGblRecordError(S_dev_badCard,(void *)pao,
         "devAoBunchClkGen (initAoRecord) : init failed!!!");
     return(S_dev_badCard);
  } 
  if( (pvmeio->signal > 4) || (pvmeio->signal < 0)) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    	     "devAoBunchClkGen : Signal out of range!!! card = %d, sig = %d\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pao,
         	"devAoBunchClkGen (initAoRecord) : init failed!!!");
     	return(S_dev_badSignal);
  }
/*  Check Card Exists */ 
   pReg = (unsigned short *) dio[pvmeio->card].dptr;
   if (vxMemProbe( (char *)pReg, READ, sizeof(value), (char *)&value) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    	     "devAoBunchClkGen : vxMemProbe failed!!! card = %d, sig = %d\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pao,
         	"devAoBunchClkGen (initAoRecord) : init failed!!!");
     	return(S_dev_badSignal);
   }

   pReg = (unsigned short *) dio[pvmeio->card].dptr + pvmeio->signal;
   value = *pReg;

    /* set linear conversion slope*/
    switch (pvmeio->signal) {
	case	3 :		/* fine delay */
		pao->eslo = (pao->eguf - pao->egul)/255;
		pao->rval = pao->rbv =(long)( 0xff & value);
		break;
	case	4 :		/* P0 Delay */
		pao->eslo = 1;
		pao->rval = pao->rbv =(long)(65535 - value);
		break; 
	default :
   		pao->rval = pao->rbv =0;
		pao->eslo =  1;
		
    }

   if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devAoBunchClkGen : card = %d sig = %d",
        	   pvmeio->card, pvmeio->signal);
   }
  pao->dpvt = pao;
  if( drvGetPwrOnStatus(pvmeio->card) == 1)
  	return(0);	/* initialize to readback value */	
  else
	return(2);	/* don't convert */
}

/*****************************************************************/
static long writeAo(
    struct aoRecord *pao)
{
  unsigned short value;
  unsigned long lval;
  short val;
  unsigned short *pReg;
  struct vmeio *pvmeio = (struct vmeio *) & (pao->out.value);

  if(!pao->dpvt)
	return(S_dev_NoInit);

  if ( pvmeio->signal == 0 || pvmeio->signal == 1 ) {
	if (pvmeio->signal == 0)
		val = 1;
	else 
		val = 0;
   		if(drvWriteBucket(pvmeio->card, pao->val, val) != OK) {
      			if (*devDebug) {
     				errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
 	"devAoBunchClkGen (writeAo) : Write Error, card = %d, signal = %d",
				 pvmeio->card, pvmeio->signal);
      			}
      			recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
      			return (2); /* don't convert */
   			}
   		pao->rbv = pao->rval;
  } else if (pvmeio->signal == 4) {
   	lval = (unsigned long)( 65535 - pao->rval );
   	if(drvWriteCard( pvmeio->card, FREG_WRITE, pvmeio->signal,
                 &lval) != OK) {
      		if (*devDebug) {
        		errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                "devAoBunchClkGen (writeAo) : Write Error, card = %d",
				 pvmeio->card);
      		}
      		recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
      		return (2); /* don't convert */
   	}
   	pReg = (unsigned short *) dio[pvmeio->card].dptr + pvmeio->signal;
   	value = *pReg;
   	pao->rbv = (long)(65535 - value);
  } else {
   	lval = (unsigned long) pao->rval;
   	if(drvWriteCard( pvmeio->card, FREG_WRITE, pvmeio->signal,
                 &lval) != OK) {
      		if (*devDebug) {
        		errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
                "devAoBunchClkGen (writeAo) : Write Error, card = %d",
				 pvmeio->card);
      		}
      		recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
      		return (2); /* don't convert */
   	}
   	pReg = (unsigned short *) dio[pvmeio->card].dptr + pvmeio->signal;
   	value = *pReg;
	if (pvmeio->signal == 3)
   		pao->rbv = (long)(0xff & value);
	else
   		pao->rbv = (long)value;
  }

      if (*devDebug >= 10) {
       printf("devAoBunchClkGen: card = %d, signal = %d,  write value = 0x%x\n",
		pvmeio->card, pvmeio->signal,  pao->rval);
      }
  return(0);
}
/*********************************************************************/
static long specialLinconvAo(
    struct aoRecord     *pao,
    int after)
{
  struct vmeio *pvmeio = (struct vmeio *) & (pao->out.value);

	if(!after)
		return(0);
    /* set linear conversion slope*/
	switch (pvmeio->signal) {
		case 0:
		case 1:
		case 3:
    			pao->eslo = 1;
			break;
		case 4:
			pao->eslo = (pao->eguf - pao->egul)/255;
			break;
		default :
			pao->eslo = 1;
	}			
    return(0);
}

/**************************************************************************/
static long initWfRecord(
struct waveformRecord *pRec)

{
	struct vmeio *pvmeio;
  	unsigned short value;
  	unsigned short *pReg;

  /* ao.out must be an VME_IO */

  switch (pRec->inp.type) {
  case (VME_IO) :
    pvmeio = (struct vmeio *)&(pRec->inp.value);
    break;
  default :
    recGblRecordError(S_dev_badInpType, (void *)pRec,
        "devWfBunchClkGen (initWfRecord)");
	return(S_dev_badInpType);
  }
/* check signal  in range */
  if( pvmeio->signal < 0 || pvmeio->signal > 1) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    	     "devWfBunchClkGen : Signal out of range!!! card = %d, sig = %d\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pRec,
         	"devWfBunchClkGen (initWfRecord) : init failed!!!");
     	return(S_dev_badSignal);
  }
/*  Check Card Exists */ 
   pReg = (unsigned short *) dio[pvmeio->card].dptr;
   if (vxMemProbe( (char *)pReg, READ, sizeof(value), (char *)&value) != OK ) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    	     "devWfBunchClkGen : vxMemProbe failed!!! card = %d, sig = %d\n",
			pvmeio->card, pvmeio->signal);
        }
     	recGblRecordError(S_dev_badSignal,(void *)pRec,
         	"devWfBunchClkGen (initWfRecord) : init failed!!!");
     	return(S_dev_badSignal);
   }
/* check for proper data type */
   if (pRec->ftvl != DBF_SHORT) {
        recGblRecordError(S_db_badField,(void *)pRec,
                "devWfBunchClkGen: init failed: ftvl not a short!!!");
        return(S_db_badField);

   }
   pRec->dpvt = pRec;
	return(0);
}

/*************************************************************************/
static long readWf(
struct waveformRecord *pRec)
{
  struct vmeio *pvmeio = (struct vmeio *) & (pRec->inp.value);
	int num;

	if (!pRec->dpvt)
		return(0);

	if(drvReadRam( pvmeio->card))
		return(0);
	if(pvmeio->signal == 0) {
		drvListBuckets(pvmeio->card, pRec->bptr, pRec->nelm, &num);
	} else if ( pvmeio->signal == 1) {
		drvShowPattern(pvmeio->card, pRec->bptr, pRec->nelm, &num);
	}
	pRec->nord = num;

	return(0);
}

/**************************************************************************/
static long initAiRecord(
    struct aiRecord	*pai)
{
  struct vmeio *pvmeio;
  long status;

  /* ao.out must be an VME_IO */

  switch (pai->inp.type) {
  case (VME_IO) :
    pvmeio = (struct vmeio *)&(pai->inp.value);
    break;
  default :
    recGblRecordError(S_dev_badInpType, (void *)pai,
        "devAiBunchClkGen (initAiRecord)");
	return(S_dev_badInpType);
  }


  if(pvmeio->signal > 4 || pvmeio->signal < 3) {
     recGblRecordError(S_dev_badSignal,(void *)pai,
         "devAiBunchClkGen (initAiRecord) : init failed!!!");
     return(S_dev_badSignal);
  }

  /* call driver so that it configures card */
  if ((status = check_card(pvmeio->card, pvmeio->signal)) != OK) {
     recGblRecordError(S_dev_badCard,(void *)pai,
         "devAiBunchClkGen (initAiRecord) : init failed!!!");
     return(S_dev_badCard);
  } 


    /* set linear conversion slope*/
    switch (pvmeio->signal) {
		case 3 :		/* fine delay */
    			pai->eslo = (pai->eguf -pai->egul)/255.0;
			break;
		case 4 :
    			pai->eslo = 1;
			break;
		default :
			break;
	}

   if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devAiBunchClkGen : card = %d sig = %d",
        	   pvmeio->card, pvmeio->signal);
   }
  pai->dpvt = pai;
  return(0);
}

/*****************************************************************/
static long readAi(
    struct aiRecord *pai)
{
  unsigned long lval;
  struct vmeio *pvmeio = (struct vmeio *) & (pai->inp.value);

  if(!pai->dpvt)
	return(S_dev_NoInit);
   if(drvReadCard( pvmeio->card, FREG_READ, pvmeio->signal,
		 &lval) != OK) {
      if (*devDebug) {
     	errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
    		"devAiBunchClkGen (readAo) : Write Error, card = %d", pvmeio->card);
      }
      recGblSetSevr(pai,WRITE_ALARM,INVALID_ALARM);
      return (2); /* don't convert */
   }
	if( pvmeio->signal == 4) {
   		pai->rval = (long)(65535 - lval);
	} else if(pvmeio->signal == 3){
   		pai->rval = (long)(0xff & lval);
	} else {
   		pai->rval = (long)lval;
	}
      if (*devDebug >= 10) {
	printf("devAiBunchClkGen: card = %d, signal = %d,  write value = 0x%x\n",
			pvmeio->card, pvmeio->signal,  pai->rval);
      }
  return(0);
}
/*********************************************************************/
static long specialLinconvAi(
    struct aiRecord     *pai,
    int after)
{
  struct vmeio *pvmeio = (struct vmeio *) & (pai->inp.value);
	if(!after)
		return(0);
    /* set linear conversion slope*/
    switch (pvmeio->signal) {
		case 4 :		/* fine delay */
    			pai->eslo = (pai->eguf -pai->egul)/255.0;
			break;
		case 3 :
    			pai->eslo = 1;
			break;
		default :
			break;
	}
    return(0);
}
