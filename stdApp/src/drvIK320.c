/* $Id: drvIK320.c,v 1.1.1.1.2.1 2003-08-11 19:30:50 sluiter Exp $ */

/* DISCLAIMER: This software is provided `as is' and without _any_ kind of
 *             warranty. Use it at your own risk - I won't be responsible
 *             if your dog drowns as a consequence of using my software
 *             blah & blah.
 */

/*
 * EPICS Driver code for the Heidenhain IK320
 *
 * Author: Till Straumann (PTB, 1999)
 *
 * $Log: not supported by cvs2svn $
 * Revision 1.1.1.1  2001/07/03 20:05:28  sluiter
 * Creating
 *
 * Revision 1.9  1999/05/05 16:25:16  strauman
 *  - added doc: README
 *
 *  Driver:
 *
 *  - busy groups are now locked until response from all axes comes in.
 *  - busy flag is now mutexed by intLock (interrupt handler accesses it, too;
 *    need to do this, so we may decrement the lock count from the irq handler).
 *  - cardQuery() tries to read other axes if IRQ_MASK_X1 is set.
 *  - added support for generic parameter set/get (FUNC_SET_PARMS, FUNC_GET_PARMS).
 *    IK320ValueRecs are automatically converted from/to doubles by these requests.
 *    This conversions honours P03, i.e. one increment of the (double) value
 *    corresponds to one `interpolated encoder increment' using P03 bits of inter-
 *    polation.
 *  - added drvIK320SetInterpBits() to demonstrate the use of FUNC_SET_PARMS
 *
 *  DevSup:
 *
 *  - added support for stringoutRecord to get/set card parameters by
 *    FUNC_GET_PARMS / FUNC_SET_PARMS.
 *  - added alarm status to group aiRecord.
 *  - mode / direction parameter records are initialized to the card values
 *    if !pini.
 *
 * Revision 1.8  1999/04/27 10:00:20  strauman
 *  - card parameters are not set to default values after soft reboot.
 *  - added support for X3 mode (disabled, X1+X2, X1-X2, (X1+X2)/2 )
 *  - added support for simultaneous reference search of both axes
 *    (using function nr 0x18)
 *  - function call requests to the card now have a timeout
 *  - setting card parameters _requires_ a 0xa function call.
 *  - need delay between `load parameters' and `search reference'
 *    function calls.
 * --------------------------------------------------------------------
 * 2.0  tmm  changed iround to NINT macro (PowerPC version of vxWorks
 *           doesn't have iround().  Local header files included with
 *           "", rather than <>. 
 *
 * 2.1  rls  bug fix for bus error when harware is missing; i.e., probe for
 *           memory. bug fix drvIK320report() trashing all the other reports.
 */

#include <vxWorks.h>
#include <tyLib.h>
#include <stdioLib.h>
#include <setjmp.h>
#include <taskLib.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <sysLib.h>
#include <intLib.h>

#include <ellLib.h>
#include <drvSup.h>
#include <devLib.h>
#include <epicsPrint.h>
#include <semLib.h>
#include <dbAccess.h>

#include "drvIK320.h"

#ifdef __cplusplus
extern "C" long locationProbe(epicsAddressType, char *);
#else
extern long locationProbe(epicsAddressType, char *);
#endif

#ifndef NODEBUG
int drvIK320Debug=0;
#define DM(LEVEL,FMT,ARGS...) {if (LEVEL<=drvIK320Debug) \
            errPrintf(-1,__FILE__,__LINE__,FMT,## ARGS); }
#else
#define DM(LEVEL,FMT,ARGS...) ;
#endif

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define A24BASE(switch) ((void*)(0xc00000 + ((switch)<<14)))
#define A16PORT(switch) ((void*)((((switch) & 0xe0)<<8) + (((switch) & 0x1f) <<1)))

#define INTERRUPT(drv)	{ (*((drv)->port)=1); }
#define GROUPIRQ(drv)	(*((drv)->port - ((drv)->sw1 & 0x1f)))

#if 0
#define S_drvIK320_readX1IK1		( M_drvIK320 | 0x0101 ) /* read X1; triggered by channel 1 */
#define S_drvIK320_readX1IK2		( M_drvIK320 | 0x0102 ) /* read X1; triggered by channel 2 */
#define S_drvIK320_readX1Sync		( M_drvIK320 | 0x0103 ) /* read X1; triggered by sync event */
#define S_drvIK320_readX1F1			( M_drvIK320 | 0x0104 ) /* read X1; triggered by F1 */
#define S_drvIK320_readX1F2			( M_drvIK320 | 0x0105 ) /* read X1; triggered by F2 */
#define S_drvIK320_readX1IRQMstr	( M_drvIK320 | 0x0106 ) /* read X1; triggered by master IRQ */
#define S_drvIK320_readX2IK1		( M_drvIK320 | 0x0201 ) /* read X2; triggered by channel 1 */
#define S_drvIK320_readX2IK2		( M_drvIK320 | 0x0202 ) /* read X2; triggered by channel 2 */
#define S_drvIK320_readX2Sync		( M_drvIK320 | 0x0203 ) /* read X2; triggered by sync event */
#define S_drvIK320_readX2F1			( M_drvIK320 | 0x0204 ) /* read X2; triggered by F1 */
#define S_drvIK320_readX2F2			( M_drvIK320 | 0x0205 ) /* read X2; triggered by F2 */
#define S_drvIK320_readX2IRQMstr	( M_drvIK320 | 0x0206 ) /* read X2; triggered by master IRQ */
#define S_drvIK320_readX3IK1		( M_drvIK320 | 0x0301 ) /* read X3; triggered by channel 1 */
#define S_drvIK320_readX3IK2		( M_drvIK320 | 0x0302 ) /* read X3; triggered by channel 2 */
#define S_drvIK320_readX3Sync		( M_drvIK320 | 0x0303 ) /* read X3; triggered by sync event */
#define S_drvIK320_readX3F1			( M_drvIK320 | 0x0304 ) /* read X3; triggered by F1 */
#define S_drvIK320_readX3F2			( M_drvIK320 | 0x0305 ) /* read X3; triggered by F2 */
#define S_drvIK320_readX3IRQMstr	( M_drvIK320 | 0x0306 ) /* read X3; triggered by master IRQ */
#define S_drvIK320_latchErrorX1		( M_drvIK320 | 0x0401 ) /* no latch signal on X1 */
#define S_drvIK320_latchErrorX2		( M_drvIK320 | 0x0402 ) /* no latch signal on X1 */
#define S_drvIK320_multiTrigX1		( M_drvIK320 | 0x0501 ) /* multiple trigger on X1 */
#define S_drvIK320_multiTrigX2		( M_drvIK320 | 0x0502 ) /* multiple trigger on X2 */
#define S_drvIK320_xferSetX1		( M_drvIK320 | 0x0601 ) /* xfer marker set on X1 */
#define S_drvIK320_xferSetX2		( M_drvIK320 | 0x0602 ) /* xfer marker set on X1 */
#define S_drvIK320_xferSetX3		( M_drvIK320 | 0x0603 ) /* xfer marker set on X1 */
#define S_drvIK320_postOK			( M_drvIK320 | 0x0700 ) /* xfer marker set on X1 */
#endif

/*
 * forward declarations
 */
static void
IK320IrqHandler(void*);

static void
IK320IrqCatcher(void*);

static long
cardInit(IK320Card crd);

static long
cardQuery(IK320Driver drv);

static long drvIK320report();
static long drvIK320init();

struct {
	long 		number;
	DRVSUPFUN	report;
	DRVSUPFUN	init;
} drvIK320 = {
	2,
	drvIK320report,
	drvIK320init
};

/* we have to lockout interrupts when testing/modifying the busy
 * flag because the interrupt handler also accesses the busy flag.
 */
#define USE_INTLOCK

typedef struct IK320DriverRec_ {
#ifndef USE_INTLOCK
	SEM_ID			mutex;			/* mutex for the busy flag */
#endif
	char			busy;			/* no access to the card while busy */
	SEM_ID			sync;			/* sync semaphore for synchronous processing */
	IK320Card		card;			/* pointer to hardware registers */
	unsigned short	*port;			/* pointer to VMEA16 interrupt address */
	dbCommon		*record;		/* EPICS record that issued a request */
	CALLBACK		procCbk;		/* callback to process the record */
	int				sw1,sw2,irqLevel;
	IOSCANPVT		*scanPvt[3];	/* records to scan after irq occurs */ 
	int				waiting;		/* waiting for sync flag */
	unsigned char	savedMask;		/* storage to temporarily save IRQ mask */
	struct IK320DriverRec_ *next;	/* next card in driver linked list */
} IK320DriverRec;

IK320Card drvIK320CARD(IK320Driver drv) { return drv->card; }

/*
 * private global vars
 */

static struct {
	SEM_ID		mutex;
	IK320Driver first;
} allCards = { 0, 0 };

static unsigned short *groupBase=0;


#ifndef NODEBUG
static SEM_ID
drvIK320LoggerSem;

static char drvIK320LoggerString[256];

static int drvIK320IrqLogger();
#endif

long
drvIK320Connect(int sw1, int sw2, int irqLevel, IK320Driver *pDrv)
{
void 		*localA24=0;
void 		*localA16=0;
IK320Driver	rval=0;
void		(*irqHandler)()=0;
int			intEnabled=0;
long		status;
int			i,needsPOST=1;

	if (!pDrv) return S_drvIK320_invalidParm;

	/* lazy init; we hope that this is not called concurrently
	 * by several people.
	 */
	if (!allCards.mutex) drvIK320init();

	/* is there already a driver struct for this card ? */

	semTake(allCards.mutex,WAIT_FOREVER);
	for (rval = allCards.first; rval; rval=rval->next) {
		if (rval->sw2 == sw2) {
			if (rval->sw1 != sw1 || rval->irqLevel!=irqLevel) {
				epicsPrintf("drvIK320: other card with same sw2 settings?\n");
				rval = 0;
			}
			semGive(allCards.mutex);
			*pDrv = rval;
			return OK;
		}
	}

	if ( ! (rval = (IK320Driver) malloc(sizeof(IK320DriverRec))) ) {
		status = S_dev_noMemory;
		goto cleanup;
	}
#ifndef USE_INTLOCK
	if ( ! (rval->mutex=semMCreate(SEM_Q_FIFO))  ) {
		status = S_dev_noMemory;
		goto cleanup;
	}
#endif
	/* use counting semaphore, so we are able to keep track of
	 * pending interrupts.
	 */
	if ( ! (rval->sync=semCCreate(SEM_Q_FIFO,SEM_EMPTY))  ) {
		status = S_dev_noMemory;
		goto cleanup;
	}
	if ((status=devRegisterAddress("drvIK320",atVMEA24,A24BASE(sw2),0x4000,&localA24)))
		goto cleanup;
	if ((status=devRegisterAddress("drvIK320",atVMEA16,A16PORT(sw1),0x2,&localA16)))
		goto cleanup;
	/* we don't bother registering the 'group trigger address' (switch1 & 0xe <<8).
     * It is used by several cards of one group and thus overlap occurs (although
	 * harmless and intentionally). We just assume everything to be alright and
	 * ignore the group trigger address here.
	 */

	rval->card=(IK320Card)localA24;
	rval->port=(unsigned short*)localA16;

	/* set the base address for all groups (must be the same for all cards) */
	if ( ! groupBase ) {
		groupBase = (unsigned short*)( (unsigned)rval->port & (unsigned)~0xffff );
	} else {
		assert( (unsigned)groupBase == ((unsigned)rval->port & (unsigned)~0xffff ));
	}

	/* could check for jumper 3 */
	status = locationProbe(atVMEA24, (char *) localA24);
	if (status != 0)
	{
            epicsPrintf("drvIK320Connect(): bus error at address = 0x%08.8x\n", (uint_t) localA24);
	    goto cleanup;
	}

	if ((status=devConnectInterrupt(intVME, sw1, irqHandler=IK320IrqCatcher, (void*)rval)))
		goto cleanup;

	rval->record=0;
	rval->sw1 = sw1;
	rval->sw2 = sw2;
	rval->irqLevel=irqLevel;
	rval->busy = 0;
	rval->next = allCards.first;
	rval->waiting = 0;
	rval->savedMask = IRQ_MASK_INVALID;
	for(i=0; i<3; i++) rval->scanPvt[i]=0;

#define crd (rval->card)
	needsPOST = ( 	crd->CRC != crd->actualCRC
		||	crd->CRC1 != crd->actualCRC1
		||	crd->CRC2 != crd->actualCRC2 );
	/* maybe the card was just powered up ? */
	
    /* could set the compVelocities here... if needsPOST */
	  /* unknown
	   *	crd->compDirX1			=
	   *	crd->compDirX2			=
	   */
#undef crd

	if ((status=devEnableInterruptLevel(intVME,irqLevel)))
		goto cleanup;
	intEnabled = 1;

	allCards.first = rval;

	if ( ! needsPOST) {
		/* hmmm... the CRC values seem to be OK. However, if the
		 * card was reset without removing the power (e.g. by hitting
		 * the crate's reset button) _nothing_ will work without
		 * a POST, i.e. the driver will hang!
		 * We better make sure we get a reply from the card.
		 */
		needsPOST = cardQuery(rval);
	}

	status=devDisableInterruptLevel(intVME,irqLevel);
	intEnabled = 0;
	if (status) goto cleanup;

	/* now install the real irq handler */
	if ((status=devDisconnectInterrupt(intVME,sw1,irqHandler))) {
		irqHandler=0;
		goto cleanup;
	}
	if ((status=devConnectInterrupt(intVME,
									sw1,
									(irqHandler=IK320IrqHandler),
									(void*)rval)))
		goto cleanup;

	/* reenable irqs */
	if ((status=devEnableInterruptLevel(intVME,irqLevel)))
		goto cleanup;
	intEnabled = 1;

	if (needsPOST) {
		/* we need to do a POST, so initialize the card to defaults */
		cardInit(rval->card);
		epicsPrintf("drvIK320: doing POST on 0x%x/0x%x\n",sw1,sw2);
		status = drvIK320Request(rval,
								 0 /* do sync request */,
								 FUNC_POST,
								 0 /* no parm */);
		/* release */
		drvIK320Finish(rval);
		if (status) goto cleanup;
	}

	epicsPrintf("drvIK320: card at 0x%x/0x%x (version HW: %i SW: %*s) initialized successfully\n",
				sw1,sw2,
				rval->card->hwVersion,
				sizeof( ((IK320Card)0)->swVersion ) / sizeof( ((IK320Card)0)->swVersion[0] ),
				rval->card->swVersion); 

	semGive(allCards.mutex);
	*pDrv = rval;

	return OK;

cleanup:
	if (intEnabled)   devDisableInterruptLevel(intVME,irqLevel);
	if (irqHandler) devDisconnectInterrupt(intVME,sw1,irqHandler);
	if (localA24) 	  devUnregisterAddress(atVMEA24,A24BASE(sw2),"drvIK320");
	if (localA16) 	  devUnregisterAddress(atVMEA16,A16PORT(sw1),"drvIK320");
	if (rval)	  	  {
#ifndef USE_INTLOCK
		if (rval->mutex) semDelete(rval->mutex);
#endif
		if (rval->sync) semDelete(rval->sync);
		free(rval);
	}
	semGive(allCards.mutex);
	*pDrv=0;
	return status;
}

long
drvIK320RegisterIOScan(IK320Driver drv, IOSCANPVT *pScanPvt, int axis)
{
int key;
int wasBusy;
  /* protect against concurrent access of scanPvt
   * (no need to be fine grained because this is rarely called)
   */
#ifndef USE_INTLOCK
  semTake(drv->mutex,WAIT_FOREVER);
#else
  {
	int key=intLock();
#endif
	if ( ! (wasBusy=drv->busy) ) {
		drv->busy=1;
	}
#ifndef USE_INTLOCK
  semGive(drv->mutex);
#else
	intUnlock(key);
  }
#endif

  if (wasBusy)
	return S_drvIK320_cardBusy;

  semTake(allCards.mutex,WAIT_FOREVER);
	key = intLock();
	drv->scanPvt[axis-1] = pScanPvt;
	intUnlock(key);
  semGive(allCards.mutex);

  return OK;
}

long
drvIK320Disconnect(IK320Driver drv)
{
int busy;
	
	if (!drv) return S_drvIK320_invalidParm;

#ifndef USE_INTLOCK
	semTake(drv->mutex,WAIT_FOREVER);
#else
	{
		int key=intLock();
#endif
		if ( ! (busy=drv->busy) ) {
			drv->busy=1;
			devDisableInterruptLevel(intVME,drv->irqLevel);
		}
#ifndef USE_INTLOCK
	semGive(drv->mutex);
#else
		intUnlock(key);
	}
#endif

	if (busy) return S_drvIK320_cardBusy;

#ifndef USE_INTLOCK
	semTake(drv->mutex,WAIT_FOREVER);
	semDelete(drv->mutex);
#endif
	semDelete(drv->sync);
	devDisconnectInterrupt(intVME,drv->sw1,IK320IrqHandler);
	devUnregisterAddress(atVMEA24,A24BASE(drv->sw2),"drvIK320");
	devUnregisterAddress(atVMEA16,A16PORT(drv->sw1),"drvIK320");
	free(drv);
	return 0;
}

static void
IK320IrqHandler(void *parm)
{
	IK320Driver drv = (IK320Driver)parm;
	IK320Card	card = drv->card;
	unsigned	char sHi = (card->irqStatus>>8) & 0xff;
	unsigned	char sLo = card->irqStatus & 0xff;
	int			error=0,doClear=0;

	/* check if this is an irq which might request I/O scan handling */
	if ( 0<sHi && 4>sHi && 6!=sLo ) { /* ok, this is a candidate */
		if (drv->scanPvt[sHi-1])
			scanIoRequest( *drv->scanPvt[sHi-1] ); /* scanned record MUST call drvIK320Finish() */
		else {
			/* it's up to us to clear the marker and the status register */
			doClear = 1;
		}
	} else {
	  if (drv->busy <= 1) {
		if (drv->record)
			callbackRequestProcessCallback(&drv->procCbk,priorityHigh,drv->record);
		else if (drv->waiting) /* somebody's waiting */
			semGive(drv->sync); /* semaphore state should still remain empty */
		else { /* UH OH, an IRQ which nobody wants; */
			error = 1;
		}
	  } else {
		/* some calls generate more than one interrupt; notification of the caller
		 * happens only for the last one;
		 */
		doClear=1;
	  }
	}

#ifndef NODEBUG
	if (error) {
		sprintf(drvIK320LoggerString,"UNSOLICITED IRQ Status: %x",(int)drv->card->irqStatus);
		semGive(drvIK320LoggerSem);
	} else if (drvIK320Debug>=1) {
		sprintf(drvIK320LoggerString,"irqStatus: %x",(int)drv->card->irqStatus);
		semGive(drvIK320LoggerSem);
	}
#endif
	if (error || doClear) {
		if ( 0<sHi && 4>sHi ) card->X[sHi-1].xfer = 0;
		card->irqStatus = 0;
		{ int key = intLock();
		  if ( drv->busy > 0 ) drv->busy--;
		  intUnlock(key);
		}
	}
}

static void
IK320IrqCatcher(void *parm)
{
	IK320Driver drv = (IK320Driver)parm;
	int key = intLock();
		drv->card->X[0].xfer = 0;
		drv->card->X[1].xfer = 0;
		drv->card->X[2].xfer = 0;
		drv->card->irqStatus = 0;
	intUnlock(key);
	semGive(drv->sync);
}

/*
 * this routine tries to trigger the card. It's not
 * important whether the value is valid or not (i.e. if
 * the references have been set etc.) but if we get an
 * answer at all. I.e. after resetting the card, we
 * MUST do a POST.
 * Here's how we find out if we need a POST (although the
 * checksums seem alright):
 * - trigger the card.
 * - if we get no irq for a while, this is a candidate.
 */
static long
cardQuery(IK320Driver drv)
{
long rval;
	/* we assume this is only used during initialization, i.e.
	 * we needn't take care of mutual access exclusion since we're
	 * the only people talking to the card at this moment.
	 */
	drv->card->irqStatus=0;
	drv->record=0; /* sync request */
	if ( drv->card->enableIRQ  & IRQ_MASK_X1 ) {
		if (drv->card->enableIRQ & IRQ_MASK_X2 ) {
			if (drv->card->modeX3 != DISABLED) {
				drv->card->function = FUNC_READ_ALL;
				drv->card->X[2].xfer=0;
			} else {
				return ERROR;
			}
		} else {
			drv->card->function = FUNC_READ_X2;
			drv->card->X[1].xfer=0;
		}
	} else {
		drv->card->function = FUNC_READ_X1;
		drv->card->X[0].xfer=0;
	}
	INTERRUPT(drv);
	rval = semTake(drv->sync, 1 /* this card is fast */);
	drvIK320Finish(drv);
	DM(1,"drvIK320: cardQuery status %i\n",rval);
	return rval;
}

#ifndef NODEBUG
/* print irq handler messages to console */

static int
drvIK320IrqLogger(	int a0, int a1, int a2, int a3, int a4,
				 	int a5, int a6, int a7, int a8, int a9)
{
	printf("drvIK320 starting irq logger...\n");
	while (1) {
		semTake(drvIK320LoggerSem,WAIT_FOREVER);
		printf("drvIK320 (IRQ logger): %s\n",drvIK320LoggerString);
	}
}

/*
 * Some handy routines that operate on a IK320Driver pointer.
 * These may be called from the vxWorks shell for testing.
 */

unsigned long gc(IK320Driver drv, int offset)
{
	unsigned char *ptr = ((unsigned char*)drv->card)+offset;
	return (unsigned long)*ptr;
}
unsigned short gs(IK320Driver drv, int offset)
{
	unsigned short *ptr = (unsigned short*) (((unsigned char*)drv->card)+offset);
	return (unsigned long)*ptr;
}
unsigned long gw(IK320Driver drv, int offset)
{
	unsigned long *ptr = (unsigned long*) (((unsigned char*)drv->card)+offset);
	return (unsigned long)*ptr;
}
void pc(IK320Driver drv, int offset, unsigned long data)
{
	unsigned char *ptr = ((unsigned char*)drv->card)+offset;
	*ptr=(unsigned char)(data & 0xff);
}
void ps(IK320Driver drv, int offset, unsigned long data)
{
	unsigned short *ptr = (unsigned short*) (((unsigned char*)drv->card)+offset);
	*ptr=(unsigned short)(data & 0xffff);
}
void pw(IK320Driver drv, int offset, unsigned long data)
{
	unsigned long *ptr = (unsigned long*) (((unsigned char*)drv->card)+offset);
	*ptr = data;
}

void setp(IK320Driver drv)
{
	*(drv->port) =1;
}

unsigned short getp(IK320Driver drv)
{
	return *(drv->port);
}

unsigned short grpTrig(IK320Driver drv)
{
	return *(drv->port - (drv->sw1 & 0x1f));
}

#endif

/*
 * initialize the parameter region of the card
 */
static long
cardInit(IK320Card crd)
{
int  i;
long status = OK;
	for (i=0; i<3; i++) {
		crd->direction[i] 	= DIR_NORMAL;
		crd->linAng[i]		= MODE_LIN;
		/*
		 * the following are unknown:
		 *
		 * crd->nPeriods[i]	=
		 */
	}
	crd->interpBits			= 12;		/* KG - was 16 */
	crd->deltaRefX1			= DISABLED;
	crd->deltaRefX2			= DISABLED;
	crd->useCompX1			= DISABLED;
	crd->useCompX2			= DISABLED;
	/* unknown
	 *	crd->compStartX1=
	 *	crd->compStartX2=
	 *	crd->nSamplesX1=
	 *	crd->nSamplesX2=
	 *	crd->sampleWidthX1=
	 *	crd->sampleWidthX2=
	 */
	/* enable irqs from both axes */
	crd->enableIRQ			= 0;
	crd->refOffsetX1		= 0;
	crd->refOffsetX2		= 0;
	crd->modeX3				= DISABLED;
	/* don't set compVelocity; can only be set after power on */
	for (i=0; i<3; i++)	{
		crd->extPreset[i].count	= 0;
		crd->extPreset[i].interpol= 0;
		crd->vmePreset[i].count	= 0;
		crd->vmePreset[i].interpol= 0;
		crd->offset[i].count	= 0;
		crd->offset[i].interpol= 0;
	}
	crd->extFunctionX1	= DISABLED;
	crd->extFunctionX2	= DISABLED;
	crd->function		= DISABLED;

	/* reset irq status to enable irqs */
	crd->irqStatus		= 0;
	return status;
}


long
drvIK320Request(IK320Driver drv, dbCommon *prec, int func, void *parm)
{
char			wasBusy;
long			status=ERROR;
char 			sync = (0 == prec);
unsigned char	comp1, comp2, irqEnbl;
unsigned char	dir = 0;
int				which = 0,axis,last;
IK320Card		card=drv->card;
int				timeout = WAIT_FOREVER;
double			ftmp;

	/* if this is a group trigger, switch to special routine */
	if ( FUNC_GRP_TRIG == func )
		return drvIK320GroupTrigger(GROUP_NR(drv->sw1));

	/* check if the card is busy */
#ifndef USE_INTLOCK
	semTake(drv->mutex,WAIT_FOREVER);
#else
	{
		int key=intLock();
#endif
		if ( ! (wasBusy=drv->busy) ) 
			drv->busy = 1;
#ifndef USE_INTLOCK
	semGive(drv->mutex);
#else
		intUnlock(key);
	}
#endif

	if (wasBusy) {
		/* contrary to the docs, the card does't abort a reference search
		 * command. If this is ever re-enabled, you should think
		 * about handling pact (race condition if setting it after INTERRUPT)
		 */
		switch (func) {
#ifdef THIS_DOESNT_WORK
			case FUNC_ABORT:
					drv->card->function = FUNC_ABORT;
					INTERRUPT(drv);
					/* rest should be done when IRQ comes in */
			return OK;
#endif
			default:
					return S_drvIK320_cardBusy;
		}
	}

	switch ( func ) {
		default:	status = S_drvIK320_invalidFunc;
		break;

		case FUNC_ABORT:	/* this probably does nothing */
					status = OK;
					timeout = 1;
		break;

		case FUNC_READ_ALL:	which++; /* fall through */
		case FUNC_READ_X2:  which++;
		case FUNC_READ_X1:	/* is irq of this channel disabled? */
							last = (2==which ? 0 : which);
							status = OK;
							for (axis = which; axis>=last; axis--) {
								if (    (which != 2 && (card->enableIRQ & (axis ? IRQ_MASK_X2
																		   : IRQ_MASK_X1)))
									 || (2==which && 0==card->modeX3) ) {
									status = S_drvIK320_maskedChannel;
									break;
								}
								if (card->X[axis].xfer) {
									epicsPrintf("drvIK320Request(): error -  xfer marker is set for %i\n",
												which+1);
									status = S_drvIK320_xferSetX1 + which;
								}
							}
							/* FIXME: We're not shure what do if FUNC_READ_ALL. Should we
							 *		  disable interrupts for X1 and X2 ?
							 *		  We leave that to the user however because it would imply
							 *        some overhead, namely one call to FUNC_SET_PARMS in order
							 *		  to mask the IRQs and another one to reset the mask when
							 *		  the IRQ for the combined axis comes in.
							 *		  Therefore, it is the user's responsibility to disable
							 *		  X1 / X2 irq if he/she doesn't want to see them.
							 *		  Note that we also provide special calls to set the
							 *		  combined mode which disable individual irqs.
							 */

							/*
							 *        IMPORTANT NOTE: in case of an asynchronous FUNC_READ_ALL
							 *		  X1/X2 irq MUST be disabled because X3's record could be
							 *		  processed as a result of X1/X2 irq.
							 */
							if ( which == 2 &&
								 (card->enableIRQ & (IRQ_MASK_X1 | IRQ_MASK_X2)) != (IRQ_MASK_X1 | IRQ_MASK_X2) ) {
								status = S_drvIK320_invalidParm;
							}
							timeout = 1; /* this should be fast */
		break;

		case FUNC_POST:	
					status = OK;
					timeout = sysClkRateGet() * 10; /* wait for 10 secs */
		break;

							/* set the direction flag;
							 */
		case FUNC_DIR_X1_NEG:	which--;   /* fall through */
		case FUNC_DIR_X2_NEG: 	dir = 1;   /* fall through */
		case FUNC_DIR_X2_POS:	which++;   /* fall through */
		case FUNC_DIR_X1_POS:
			card->direction[which] = dir;
			func = FUNC_SET_PARMS;
			timeout = 1; /* this should be fast */
			status = OK;
		break;

		case FUNC_MODE_X3_DISABLE:
			/* re_enable the individual interrupts */
			card->enableIRQ &= ~(IRQ_MASK_X1 | IRQ_MASK_X2);
			card->modeX3 = DISABLED;
			func = FUNC_SET_PARMS;
			timeout = 1; /* this should be fast */
			status = OK;
		break;

		case FUNC_MODE_X3_SUM:
		case FUNC_MODE_X3_DIFF:
		case FUNC_MODE_X3_MEAN:
			/* disable individual irqs */
			card->enableIRQ |= IRQ_MASK_X1 | IRQ_MASK_X2;
			/* strip off `special' flag */
			card->modeX3 = func & 0xffff;
			func = FUNC_SET_PARMS;
			timeout = 1; /* this should be fast */
			status = OK;
		break;

		case FUNC_PRE_X3:	which++; /* fall through */
		case FUNC_PRE_X2:	which++;
		case FUNC_PRE_X1:
			if (parm) {
				card->vmePreset[which] = *(IK320Value)parm;
			} else {
				card->vmePreset[which].count = 0;
				card->vmePreset[which].interpol = 0;
			}
			timeout = 1; /* this should be fast */
			status = OK;
		break;

		case FUNC_SET_PARMS: /* generic set parameter call */
#define ikparm ((IK320Parm)parm)
#define dval   (*(double*)ikparm->from)
			/* copy to the card memory */
			if ( ! parm ) {
				status = S_drvIK320_invalidParm;
			} else {
				if (ikparm->type == six) {
					/* convert (double) to an IK320ValueRec */
					IK320ValueRec tmp;
					tmp.count 	= (long) floor( dval / (double)(1<<card->interpBits));
					ftmp = fmod(dval,(double)(1<<card->interpBits));
					tmp.interpol = (unsigned short) (NINT(ftmp) << (16 - card->interpBits));
					memmove((char*)(drv->card) + ikparm->offset,
							(char*)&tmp,
							(size_t)ikparm->type);
				} else {
					memmove((char*)(drv->card) + ikparm->offset,
							ikparm->from,
							(size_t)ikparm->type);
				}
				timeout = 1;
				status = OK;
			}
#undef dval
#undef ikparm
		break;

		case FUNC_GET_PARMS: /* generic read parameter call */
#define ikparm ((IK320Parm)parm)
#define sixval ((IK320Value)((char*)drv->card + ikparm->offset))
			/* copy to the card memory */
			if ( ! parm ) {
				/* suppose they just want to lock the driver */
			} else {
				if (ikparm->type == six) {
					/* convert 48-bit register value to double */
					double val=sixval->count;
					val *= (1<<card->interpBits);
					val += (double) (sixval->interpol >> (16 - card->interpBits));
					*(double*)ikparm->from = val;
				} else {
					memmove(ikparm->from,
							(char*)(drv->card) + ikparm->offset,
							(size_t)ikparm->type);
				}
			}
#undef sixval
#undef ikparm
		return OK;

		/* before allowing a reference run, we test if
		 * an encoder is there, because the reference run
		 * can not be aborted.
		 *
		 * We do this by trying to read the device and check
		 * for the 'signal too low' status. Note that for
		 * sake of simplicity, we do a synchronous read here.
		 * ( a read takes ~180 us)
		 */
		case FUNC_REF_BOTH: which++; /* fall through */
		case FUNC_REF_X2:   which++; /* fall through */
		case FUNC_REF_X1:
			timeout = WAIT_FOREVER; /* we cannot know how long this gonna take */
			
			/* save compensation flag and irq mask */
			comp1 = card->useCompX1;
			comp2 = card->useCompX2;
			irqEnbl = card->enableIRQ;
			card->useCompX1 = 0;
			card->useCompX2 = 0;
			card->enableIRQ &= ~(IRQ_MASK_X1 | IRQ_MASK_X2);

			drv->record = 0;
			drv->waiting++; /* NOTE: we assume ++ to be ATOMIC; increase the counter
							 *		 _before_ interrupting to prevent race condition.
							 */
			/* IK320 doesn't use the new IRQ mask if not notified */
			card->function = FUNC_SET_PARMS;
			DM(2,"drvIK320: reference; set irq mask...");
			card->irqStatus = 0;
			INTERRUPT(drv);
			if (semTake(drv->sync,1)) {
				status = S_drvIK320_timeout;
				break;
			}
			/* NOTE: IK320 seems to have another firmware bug.
			 *		 Although the reply to the `set parameters'
			 *       request came in, the card seems to need some
			 *       time to get ready again :-( :-( :-(.
			 *       The card will ignore the next interrupt
			 *       if it is issued too shortly after getting the
			 *       reply to FUNC_SET_PARMS.
			 *       
			 *       At this point, we just do a taskDelay().
			 */
			taskDelay(1);
			DM(2,"ok\ndrvIK320: reference; trying to read...");

			last = (2==which ? 1 : which);
			for (axis = (2 == which ? 0 : which), status = OK;
				 axis<=last && OK==status;
				 axis++) {

				/* now try to read something */
				card->function = (axis ? FUNC_READ_X2 : FUNC_READ_X1);
				card->irqStatus = 0;
				INTERRUPT(drv);
				if (semTake(drv->sync,1)) {
					status = S_drvIK320_timeout;
				} else {
					status = ((card->X[axis].status & ASTAT_NO_SIGNAL) ?
								S_drvIK320_noSignal : OK);
				}
				DM(2,"ok, status %i...",status);

				card->X[axis].xfer=0;
			}
			DM(2,"\ndrvIK320: reference; resetting mask...");
			/* restore compensation flag and irq mask */
			card->useCompX1 = comp1;
			card->useCompX2 = comp2;
			card->enableIRQ = irqEnbl;

			/* notify the card of changed parameters */
			card->function = FUNC_SET_PARMS;
			card->irqStatus = 0;
			INTERRUPT(drv);
			if (semTake(drv->sync,1)) {
				status = S_drvIK320_timeout;
			}
			/* See the note above on why we need to do that */
			taskDelay(1);
			DM(2,"done\n");
			drv->waiting--; /* NOTE: we assume -- to be ATOMIC */
			card->irqStatus = 0;
		break;	
	}

	if (OK==status) {
		drv->card->function=(unsigned short)func;	
		drv->record = (sync ? 0 : prec);

		/* set pact before interrupting the card. This MUST be done in order to
		 * avoid a race condition:
		 *
		 * e.g. 'read_ai()' calls drvIK320Request() and then sets pact:
		 *
		 * while still in drvIK320Request(), the IRQ may come in and be handled
		 * dispatching scanning the record for the second time BEFORE pact gets a
		 * chance to be set!
		 */
		if ( drv->record ) drv->record->pact = TRUE;

		/* a reference run for two axes generates two IRQs */
		if ( FUNC_REF_BOTH == func ) {
			int key=intLock();
			drv->busy++;
			intUnlock(key);
		}
		/* interrupt the IK320 */
		drv->waiting++; /* NOTE: we assume ++ to be ATOMIC; increase the counter
						 *		 _before_ interrupting to prevent race condition.
						 */
		INTERRUPT(drv);

		/* if this is a synchronous request, wait for it
		 * to complete.
		 */
		if (sync) {
			status = semTake(drv->sync,timeout) ? S_drvIK320_timeout : OK;
			/* NOTE: the caller MUST call drvIK320Finish() after
			 * 		 processing the result in order to release the card
			 */
		} else {
			status = S_drvIK320_asyncStarted;
		}
		drv->waiting--; /* NOTE: we assume -- to be ATOMIC */
	} 

	if ( status != OK && status != S_drvIK320_asyncStarted ) {
		/* the request could not be sent; there will be no
		 * interrupt, hence we must reset the busy flag.
		 */
		drv->record=0;
#ifndef USE_INTLOCK
		semTake(drv->mutex,WAIT_FOREVER);
			drv->busy=0;
		semGive(drv->mutex);
#else
		{
			int key=intLock();
			drv->busy=0;
			intUnlock(key);
		}
#endif
	}
	return status;
}

void
drvIK320Finish(IK320Driver drv)
{
int axis = (drv->card->irqStatus>>8) & 0xff;
int wasBusy;
	drv->record = 0;
	/* clear the associated transfer marker (just in case) */
	if ( 0 < axis && 4 > axis) {
		drv->card->X[axis-1].xfer=0;
	}	
	drv->card->irqStatus=0; /* ready for new interrupt */

#ifndef USE_INTLOCK
	semTake(drv->mutex,WAIT_FOREVER);
#else
	{ int key=intLock();
#endif
		wasBusy = drv->busy;
		if ( drv->busy > 0 ) {
			drv->busy--;
		}
#ifndef USE_INTLOCK
	semGive(drv->mutex);
#else
		intUnlock(key);
	}
#endif
	DM(1,"drvIK320Finish(): busy %i\n", wasBusy);
}

static long
drvIK320report()
{
  printf("drvIK320: sorry, report is not implemented yet\n"); return OK;
}

static long
drvIK320init()
{
  assert(allCards.mutex=semMCreate(SEM_Q_FIFO));
  allCards.first=0;
#ifndef NODEBUG
  assert(drvIK320LoggerSem = semBCreate(SEM_Q_FIFO,SEM_EMPTY));
  drvIK320LoggerString[0] = 0;
  taskSpawn("IK320Logger",130,VX_STDIO,2048,drvIK320IrqLogger,0,0,0,0,0,0,0,0,0,0);
#endif

  return OK;
}

/*
 * count all axes with their scanPvt field set that belong
 * to a common group.
 *
 * NOTE: drvIK320RegisterIOScan() must NOT be called by other
 *		 tasks while this is in progress.
 */

int
drvIK320getNGroupListeners(int groupNr)
{
int rval = 0;
IK320Driver drv;
  semTake(allCards.mutex,WAIT_FOREVER);
	for (drv=allCards.first; drv; drv=drv->next) { 
		int i;
		if (GROUP_NR(drv->sw1) == groupNr) {
			for (i=0; i<3; i++)
				if (drv->scanPvt[i]) {
					rval++;
				}
		}
	}
  semGive(allCards.mutex);
  return rval;
}

int
drvIK320GroupNr(IK320Driver drv)
{
	return GROUP_NR(drv->sw1);
}

/*
 * mark a group busy. This is done recursively.
 *
 * RETURNS: OK, S_drvIK320_cardBusy if any member is busy.
 */
static long
markGroupBusy(IK320Driver drv, int groupNr)
{
int wasBusy;

	/* search next member */
	 while (drv && GROUP_NR(drv->sw1) != groupNr) drv=drv->next;

	 if ( drv ) {
		/* mark this member of our group busy */
#ifndef USE_INTLOCK
		semTake(drv->mutex,WAIT_FOREVER);
#else
		{ int key=intLock();
#endif
			if ( !(wasBusy = drv->busy) ) {
				drv->busy = 2;
			}
#ifndef USE_INTLOCK
		semGive(drv->mutex);
#else
			intUnlock(key);
		}
#endif

		if (wasBusy) return S_drvIK320_cardBusy;

		drv->record = 0;

		/* if we are not busy, try to mark the next member */
		if (markGroupBusy(drv->next,groupNr)) {
			/* failed; reset the busy flag and return */
#ifndef USE_INTLOCK
			semTake(drv->mutex,WAIT_FOREVER);
#else
			{ int key=intLock();
#endif
				drv->busy=0;
#ifndef USE_INTLOCK
			semGive(drv->mutex);
#else
				intUnlock(key);
			}
#endif
			return S_drvIK320_cardBusy;
		}

		/* adjust the busy flag so it reflects the number of
		 * interrupts that are expected.
		 */
		wasBusy=2;
		if (drv->card->enableIRQ & IRQ_MASK_X1) wasBusy--;
		if (drv->card->enableIRQ & IRQ_MASK_X2) wasBusy--;
		if (wasBusy != drv->busy) {
#ifndef USE_INTLOCK
			semTake(drv->mutex,WAIT_FOREVER);
#else
			{ int key=intLock();
#endif
				drv->busy=wasBusy;
#ifndef USE_INTLOCK
			semGive(drv->mutex);
#else
				intUnlock(key);
			}
#endif
		}
	  }
	return OK;
}


#if 0 /* this currently is not used */
/*
 * unmark a group busy; the group must have successfully marked
 * busy before.
 */

static void
unmarkGroupBusy(int groupNr)
{
int 		wasBusy;
IK320Driver drv;
	semTake(allCards.mutex,WAIT_FOREVER);
	for (drv=allCards.first; drv; drv=drv->next) {
		/* member ? */
		if (GROUP_NR(drv->sw1)==groupNr) {
#ifndef USE_INTLOCK
			semTake(drv->mutex,WAIT_FOREVER);
#else
			{
				int key=intLock();
#endif
				wasBusy=drv->busy; drv->busy=0;
#ifndef USE_INTLOCK
			semGive(drv->mutex);
#else
				intUnlock(key);
			}
#endif
			/*
			 * the busy flag might be reset by somebody calling drvIK320Finish
			 * before we get here.
			 *
			 * assert(wasBusy);
			 */
		}
	}
	semGive(allCards.mutex);
}
#endif

/*
 * issue a group trigger command. First be must make sure that
 * no member of the group is currently busy.
 */

long
drvIK320GroupTrigger(int groupNr)
{
long rval;

	if (0>groupNr || MAX_IK320_GROUPS <= groupNr)
		return S_drvIK320_invalidParm;

	semTake(allCards.mutex,WAIT_FOREVER);
		rval = markGroupBusy(allCards.first,groupNr);
	semGive(allCards.mutex);

	if ( OK == rval ) { /* nobody is busy */
		/* force the compiler to generate code */
		rval = (long)((*(groupBase + (groupNr<<12))) & (unsigned short)(groupNr & 0xf0));
#if 0
		unmarkGroupBusy(groupNr);
		/* nobody probably should talk to the card until 
		 * the group completes. There's no protection however.
		 */
#endif
	}
	return rval;
}

long
drvIK320SetInterpBits(IK320Driver drv, int *pnBits)
{
  IK320ParmRec parm;
  unsigned short from_bits = (unsigned short) *pnBits;
  long rval;

  /* fill the IK320ParmRec for the `interpBits' card parameter */
  IK320_FILL_PARM( &parm, &from_bits , interpBits);

  /* do a synchronous request and supply the IK320Parm structure pointer */
  if (OK == (rval = drvIK320Request(drv,0,FUNC_SET_PARMS,&parm)) ) {
	/*
	 * if the parameter was out of range, IK320 set it to a default value
	 * which we write back.
	 */
	if ( from_bits != drv->card->interpBits ) {
		*pnBits = (int)from_bits;
		rval = S_drvIK320_invalidParm;
	}
	drvIK320Finish(drv);
  }
  return rval;
}
