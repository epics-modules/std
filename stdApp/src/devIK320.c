/* $Id: devIK320.c,v 1.1.1.1.2.2 2004-01-16 18:07:59 sluiter Exp $ */

/* DISCLAIMER: This software is provided `as is' and without _any_ kind of
 *             warranty. Use it at your own risk - I won't be responsible
 *             if your dog drowns as a consequence of using my software
 *             blah & blah.
 */

/*
 * EPICS device support for the Heidenhain IK320
 *
 * Author: Till Straumann (PTB, 1999)
 *
 * $Log: not supported by cvs2svn $
 * Revision 1.1.1.1.2.1  2003/08/11 19:24:32  sluiter
 * Bug fix for bus error in get_ioint_info() if hardware is missing.
 *
 * Revision 1.1.1.1  2001/07/03 20:05:27  sluiter
 * Creating
 *
 * Revision 1.8  1999/05/05 16:25:16  strauman
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
 * Revision 1.7  1999/04/27 10:00:20  strauman
 *  - card parameters are not set to default values after soft reboot.
 *  - added support for X3 mode (disabled, X1+X2, X1-X2, (X1+X2)/2 )
 *  - added support for simultaneous reference search of both axes
 *    (using function nr 0x18)
 *  - function call requests to the card now have a timeout
 *  - setting card parameters _requires_ a 0xa function call.
 *  - need delay between `load parameters' and `search reference'
 *    function calls.
 *
 * Modification Log:
 * -----------------
 * .01 10.20.99 kag removed 'ZERO' function from menus
 * .02 08.11.03 rls bug fix for bus error in get_ioint_info() if hardware
 *			is missing.
 * .03 01.15.03 rls more bug fixes for when hardware is missing.
 *
 */

#include <vxWorks.h>
#include <tyLib.h>
#include <stdioLib.h>
#include <setjmp.h>
#include <taskLib.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <sysLib.h>

#include <string.h>

#include <ellLib.h>
#include <recSup.h>
#include <devSup.h>
#include <devLib.h>
#include <epicsPrint.h>
#include <semLib.h>
#include <dbAccess.h>
#include <dbEvent.h>

#include <drvIK320.h>

#include <mbboRecord.h>
#include <aiRecord.h>
#include <stringoutRecord.h>

#include <aiCvtDouble.h>

#ifndef NODEBUG
int devIK320Debug=0;
#define STATIC
#define DM(LEVEL,FMT,ARGS...) {if (LEVEL<=devIK320Debug) \
            errPrintf(-1,__FILE__,__LINE__,FMT,## ARGS); }
#else
#define DM(LEVEL,FMT,ARGS...) ;
#define STATIC static
#endif

#define NUM_ELS(arr) (sizeof(arr)/sizeof((arr)[0]))

typedef struct IK320FunctionDescRec_ {
	int				code;
	char			*name;
} IK320FunctionDescRec, *IK320FunctionDesc;

typedef struct DevIK320MbboRec_ {
	IK320Driver			 drv;
	IK320FunctionDescRec *menu;
	int					 nMenu;
	short				 axis; /* 1..3 */
} DevIK320MbboRec, *DevIK320Mbbo;

IK320FunctionDescRec devDirMenu[][2]= {
 { 	{ FUNC_DIR_X1_POS,			"Pos"},
	{ FUNC_DIR_X1_NEG,			"Neg" },
 },
 { 	{ FUNC_DIR_X2_POS,			"Pos"},
	{ FUNC_DIR_X2_NEG,			"Neg" },
 },
 { 	{ FUNC_NONE,				"Not Supported"},
	{ FUNC_NONE,				"Not Supported"},
 },
};

IK320FunctionDescRec devX3ModeMenu[]={
	{ FUNC_MODE_X3_DISABLE,	"Disabled"},
	{ FUNC_MODE_X3_SUM, 	"X1+X2"},
	{ FUNC_MODE_X3_DIFF, 	"X1-X2"},
	{ FUNC_MODE_X3_MEAN, 	"(X1+X2)/2"},
};

IK320FunctionDescRec devFunctMenu[][5]= {
 {	{ FUNC_NONE,	"Idle"},
	{ FUNC_POST, 	"POST"},
	{ FUNC_GRP_TRIG,"Group Trigger"},
	{ FUNC_REF_X1, 	"Reference X1"},
	{ FUNC_REF_BOTH,"Reference X1/X2"},	
 },
 {	{ FUNC_NONE,	"Idle"},
	{ FUNC_POST, 	"POST"},
	{ FUNC_GRP_TRIG,"Group Trigger"},
	{ FUNC_REF_X2, 	"Reference X2"},
	{ FUNC_REF_BOTH,"Reference X1/X2"},	
 },
 {	{ FUNC_NONE,	"Idle"},
	{ FUNC_POST, 	"POST"},
	{ FUNC_GRP_TRIG,"Group Trigger"},
	{ FUNC_REF_BOTH,"Reference X1/X2"},	
	{ 0,			0},
 },
};

STATIC long ik320InitFunct();
STATIC long ik320WriteFunct();
STATIC long ik320InitDir();
STATIC long ik320InitModeX3();
STATIC long ik320WriteMbboSync();
STATIC long ik320InitMbbo();
STATIC long get_ioint_info();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_mbbo;
} devIK320Funct = {
	5,
	NULL,
	NULL,
	ik320InitFunct,
	NULL,
	ik320WriteFunct
};

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_mbbo;
} devIK320Dir = {
	5,
	NULL,
	NULL,
	ik320InitDir,
	NULL,
	ik320WriteMbboSync,
};

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_mbbo;
} devIK320ModeX3 = {
	5,
	NULL,
	NULL,
	ik320InitModeX3,
	NULL,
	ik320WriteMbboSync,
};

typedef struct DevIK320AiRec_ {
	IK320Driver	drv;
	short		axis; /* 1..3 */
	double		value;
	IOSCANPVT	scanPvt;
} DevIK320AiRec, *DevIK320Ai;

STATIC long ik320InitAiRec();
STATIC long ik320ReadAi();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_ai;
	DEVSUPFUN	special_linconv;
} devIK320Ai = {
	6,
	NULL,
	NULL,
	ik320InitAiRec,
	get_ioint_info,
	ik320ReadAi,
	NULL
};

typedef struct DevIK320GroupAiRec_ {
	CALLBACK cbk;
	short  nAxes;
	short  nIrqs;
	short  nr;
	short  valid;
	long   status;
	double value,lvalue;
	SEM_ID mutex;
} DevIK320GroupAiRec, *DevIK320GroupAi;

STATIC long ik320InitGroupAi();
STATIC long ik320InitGroupAiRec();
STATIC long ik320ReadGroupAi();

STATIC void ik320GroupPost(DevIK320GroupAi grp);

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_ai;
	DEVSUPFUN	special_linconv;
} devIK320GroupAi = {
	6,
	NULL,
	ik320InitGroupAi,
	ik320InitGroupAiRec,
	NULL,
	ik320ReadGroupAi,
	NULL
};

/*
 * type definitions for the stringout that supports
 * generic parameter setting.
 */

#define PE(number,field) { (char*)number,\
						(char*)&(((IK320Card)0)->field) - (char*)0,\
						sizeof( ((IK320Card)0)->field ) }

STATIC int
ik320ParmCompare(const void *key, const void *element)
{
	return (int)key - (int)((IK320Parm)element)->from;
}

/* the entries of the parmTable must be sorted in ascending order
 * (abuse the `from' field to store the key/function number)
 */
STATIC IK320ParmRec parmTable[]={
	PE(11, 	direction[0]),
	PE(12, 	direction[1]),
	PE(21, 	linAng[0]),
	PE(22, 	linAng[1]),
	PE(23, 	linAng[2]),
	PE(30,	interpBits),
	PE(41, 	deltaRefX1),
	PE(42, 	deltaRefX2),
	PE(51, 	nPeriods[0]),
	PE(52, 	nPeriods[1]),
	PE(53, 	nPeriods[2]),
	PE(61, 	useCompX1),
	PE(62, 	useCompX2),
	PE(71, 	compStartX1),
	PE(72, 	compStartX2),
	PE(81, 	nSamplesX1),
	PE(82, 	nSamplesX2),
	PE(91, 	sampleWidthX1),
	PE(92, 	sampleWidthX2),
	PE(100,	enableIRQ),
#if 0 /* don't use these */
	PE(191, refOffsetX1),
	PE(192, refOffsetX2),
#endif
	PE(210,	modeX3),
#if 0 /* frequency must be set BEFORE 1st POST; do this in the startup script */
	PE(301, compDirX1),
	PE(302, compDirX2),
#endif
	PE(701, extPreset[0]),
	PE(702, extPreset[1]),
	PE(703, extPreset[2]),
	PE(711, vmePreset[0]),
	PE(712, vmePreset[1]),
	PE(713, vmePreset[2]),
	PE(721, offset[0]),
	PE(722, offset[1]),
	PE(723, offset[2]),
	PE(801, extFunctionX1),
	PE(802, extFunctionX2),
#if 0 /* not a normal parameter */
	PE(810,	function),
#endif
};
#undef PE

STATIC long ik320InitParm();
STATIC long ik320WriteParm();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_stringout;
} devIK320Parm = {
	5,
	NULL,
	NULL,
	ik320InitParm,
	NULL,
	ik320WriteParm,
};

/* table of all groups; since there are few,
 * we keep a static table.
 */
STATIC aiRecord* ik320GroupStatic[MAX_IK320_GROUPS];

STATIC void
ik320GroupChanged(int unregister, int groupNr)
{
aiRecord		*prec = ik320GroupStatic[groupNr];
DevIK320GroupAi grp;

	if (!prec) return;
	grp = (DevIK320GroupAi)(prec->dpvt);

	semTake(grp->mutex,WAIT_FOREVER);
		grp->value *= grp->nAxes;
		if (unregister) grp->nAxes--; else grp->nAxes++;
		/* adjust mean value */
		if (grp->nAxes>0) {
			grp->value/=grp->nAxes;	
			if (grp->nIrqs>=grp->nAxes) {
				ik320GroupPost(grp);
			}
		} else {
			grp->value=0;
			grp->valid=0;
		}
	semGive(grp->mutex);
	DM(2,"ik320GroupChanged() 1 axis %s group %i, now %i members\n",
		unregister ? "left" : "joined", groupNr, grp->nAxes);
}

STATIC void
ik320GroupAddValue(int groupNr, double value, long status)
{
aiRecord		*prec = ik320GroupStatic[groupNr];
DevIK320GroupAi grp;
	if (!prec) return;
	grp=(DevIK320GroupAi)(prec->dpvt);
	semTake(grp->mutex,WAIT_FOREVER);
		grp->value += value/grp->nAxes;
		if (status)
			grp->status = READ_ALARM;
		if (++grp->nIrqs>=grp->nAxes)
			ik320GroupPost(grp);
	semGive(grp->mutex);
}

STATIC long
ik320InitGroupAi(int after)
{
int i;
	if (after) {
		/* records are set up. Now we have to find out
		 * how many axes belong to each group we manage.
		 */
		for (i=0; i<MAX_IK320_GROUPS; i++) {
			aiRecord *prec = ik320GroupStatic[i];
			if (prec) {
				((DevIK320GroupAi)(prec->dpvt))->nAxes=drvIK320getNGroupListeners(i);
				DM(4,"ik320InitGroupAi(): group %i has %i listeners\n",
					i,
					((DevIK320GroupAi)(prec->dpvt))->nAxes);
			}
		}
	} else {
		/* clear the group table */
		for (i=0; i<MAX_IK320_GROUPS; i++)
			ik320GroupStatic[i]=0;
	}
	return OK;
}


STATIC long
ik320InitGroupAiRec(aiRecord *prec)
{
DevIK320GroupAi devState;
long			status=S_dev_noMemory;

	int grpNr;
	/* check the link type; must be VME_IO */
	if (VME_IO != prec->inp.type)
		return S_dev_badBus;

	/* get the group number */
	grpNr=prec->inp.value.vmeio.card;
	if (0 > grpNr || MAX_IK320_GROUPS <= grpNr)
		return S_drvIK320_invalidParm;

	if (! (prec->dpvt=devState=(DevIK320GroupAi)malloc(sizeof(DevIK320GroupAiRec))) )
		goto cleanup;

	if ( ! (devState->mutex=semMCreate(SEM_Q_FIFO)))
		goto cleanup;

	devState->nAxes=0;
	devState->nIrqs=0;
	devState->lvalue=devState->value=0.;
	devState->valid=0;
	devState->nr=grpNr;
	devState->status = NO_ALARM;
	ik320GroupStatic[grpNr]=prec;

	DM(4,"ik320InitGroupAiRec(): initialized group nr %i\n",grpNr);

	return OK;

cleanup:
	if (devState) {
		if (devState->mutex) semDelete(devState->mutex);
		free(devState);
	}
	prec->pact=TRUE;
	return status;
}

STATIC long
ik320ReadGroupAi(aiRecord *prec)
{
DevIK320GroupAi grp = (DevIK320GroupAi)(prec->dpvt);
	/*
	 * valid && pact : completion phase of async processing;
	 *				   group was triggered by this record;
	 * valid && !pact: completion phase; group trigger was issued
	 *				   by somebody else (used for IO Intr scanning)
	 * !valid && !pact:record is idle 
	 * !valid && pact: async processing is in progress.
	 */
	DM(2,"ik320ReadGroupAi(): entering...");
	if ( ! grp->valid && ! prec->pact ) {
		/* issue group trigger
		 * if there are no axes in this group, there will never be
		 * an answer!
		 */
		DM(2,"issueing group %i trigger..",grp->nr);
		switch (drvIK320GroupTrigger(grp->nr)) {
			case S_drvIK320_cardBusy:
				DM(2,"failed! (cardBusy)");
				recGblSetSevr(prec,WRITE_ALARM,INVALID_ALARM);
			break;

			case OK:
				DM(2,"OK!");
			break;

			default:
				assert(0); /* never get here */
			break;
		}
	} else {
		/* completion phase; this is due to a GroupPost()
		 */
		semTake(grp->mutex,WAIT_FOREVER);
		prec->val = grp->lvalue;
		grp->valid = 0;
		if (grp->status) {
			recGblSetSevr(prec,grp->status,INVALID_ALARM);
			grp->status = NO_ALARM;
		}
		semGive(grp->mutex);
		DM(2,"completion phase; new (raw) value %g\n",prec->val);
		aiCvtDouble(prec);
	}
	DM(2,"\n");
	return 2; /* did conversion on our own */
}

STATIC void
ik320GroupPost(DevIK320GroupAi grp)
{
	/* save value in case more interrupts come in before the record is
	 * processed.
	 */
	semTake(grp->mutex,WAIT_FOREVER);
		grp->lvalue = grp->value;
		grp->valid	= TRUE;
		grp->nIrqs	= 0;
		grp->value	= 0.; /* reset the sum */
	semGive(grp->mutex);
	DM(2,"ik320GroupPost(): posting new value %g to group %i\n",
		grp->lvalue, grp->nr);
	callbackRequestProcessCallback(&grp->cbk,priorityHigh,ik320GroupStatic[grp->nr]);
}

/*
 * utility routine to connect to a driver structure given an output link
 */

STATIC long
ik320Connect(struct link *link, short *axis, IK320Driver *drv)
{
	int  			sw1,sw2,irqLevel;
	int				aval;

	/* check the link type; must be VME_IO */
	if (VME_IO != link->type)
		return S_dev_badBus;

	sw1 = link->value.vmeio.card;
	sw2 = sw1 & 0xff;
	sw1 = (sw1>>8) & 0xff;
	irqLevel = link->value.vmeio.signal;
	if (	irqLevel<1 || irqLevel > 7
		||  1!=sscanf(link->value.vmeio.parm,"%i",&aval)
		||  1>aval || 3<aval ) {
		return S_drvIK320_invalidParm;
	}
	if (axis) *axis = (short) aval;
	return drvIK320Connect(sw1,sw2,irqLevel,drv);
}

STATIC long
ik320InitFunct(mbboRecord *prec)
{
	return ik320InitMbbo(prec, devFunctMenu, NUM_ELS(devFunctMenu[0]), 1);
}

STATIC long
ik320InitDir(mbboRecord *prec)
{
    long rval = ik320InitMbbo(prec, devDirMenu, NUM_ELS(devDirMenu[0]), 1);
    if (OK == rval)
    {
	if (!prec->pini)
	{
	    DevIK320Mbbo tmp = (DevIK320Mbbo)(prec->dpvt);
	    if (tmp->drv == NULL)
		return(ERROR);
	    /* get value from the card */
	    prec->rval = drvIK320CARD(tmp->drv)->direction[tmp->axis - 1];
	}
    }
    return(rval);
}

STATIC long ik320InitModeX3(mbboRecord *prec)
{
    long rval = ik320InitMbbo(prec, devX3ModeMenu, NUM_ELS(devX3ModeMenu), 0);
    if (OK == rval)
    {
	if (!prec->pini)
	{
	    if (prec->dpvt == NULL)
		rval = ERROR;
	    else /* get value from the card */
		prec->rval = drvIK320CARD(((DevIK320Mbbo)(prec->dpvt))->drv)->modeX3;
	}
    }
    return(rval);
}

STATIC long
ik320InitMbbo(mbboRecord *prec, IK320FunctionDescRec *menu, int nels, int multiDim)
{
	long 			status;
	int				i;
	DevIK320Mbbo	devState=0;
	unsigned long	*valptr;
	char			*nameptr;
	int				idx;

	DM(1,"devIK320InitFunct() entering\n");
	if (! (devState=(DevIK320Mbbo)malloc(sizeof(DevIK320MbboRec))) ) {
		status = S_dev_noMemory;
		goto cleanup;
	}

	if ((status=ik320Connect(&prec->out,&devState->axis,&devState->drv)))
		goto cleanup;

	idx= multiDim ? (devState->axis-1)*nels : 0;

	valptr=&(prec->zrvl);
	nameptr=prec->zrst;	
	
	for (i=0; i< nels && menu[idx+i].name; i++) {
		if (*nameptr == '\0' ) { /* dont override settings */
			strcpy(nameptr,menu[idx+i].name);
			*valptr=i;
		}
		valptr++;
		nameptr+=sizeof(prec->zrst);
	}

	devState->menu = menu+idx;
	devState->nMenu = nels;
	prec->dpvt=devState;

	return OK;

cleanup:
	prec->pact=TRUE;
	if (devState) free(devState);
	
	return status;
}

STATIC long
ik320WriteFunct(mbboRecord *prec)
{
int				cmd;
DevIK320Mbbo	devState=(DevIK320Mbbo)prec->dpvt;
IK320Card		card = drvIK320CARD(devState->drv);
long			status=ERROR;
int				idx = devState->axis-1;

	DM(1,"ik320WriteFunct() entering...\n");

	/* check if value is in valid range */
	if (prec->val >= NUM_ELS(devFunctMenu[0]) || 0==devFunctMenu[idx][prec->val].name) {
		status = S_drvIK320_invalidParm;
		goto cleanup;
	}

	if (!prec->pact) { /* first time we are called */
		switch (cmd=devFunctMenu[idx][prec->val].code) {
			
			default:	/* should never get here */
						status = S_drvIK320_invalidParm;
			goto cleanup;

			case FUNC_NONE:	status = OK;
			goto cleanup;

			case FUNC_GRP_TRIG:
				status = drvIK320Request(devState->drv,0 /* synchrouous request */, cmd, 0);
			goto cleanup;

			case FUNC_POST:		/* fall through */
			case FUNC_REF_X1:
			case FUNC_REF_X2:
			case FUNC_REF_BOTH:
			case FUNC_PRE_X1:
			case FUNC_PRE_X2:
			case FUNC_PRE_X3:
				status = drvIK320Request(devState->drv,(dbCommon*)prec,cmd,0);

				if (OK==status) { /* request was handled synchronously */
					DM(1,"ik320WriteFunct() sync request (only one phase)...\n");
					if (card->irqStatus&0xff) {
						status=ERROR;
						epicsPrintf("devIK320Funct(): error irqStatus %x\n",card->irqStatus);
					}
					drvIK320Finish(devState->drv);
					goto cleanup;
				}

				if (S_drvIK320_asyncStarted == status) { /* ok, wait for completion */
					DM(1,"ik320WriteFunct() sent request (1st phase)...\n");
					prec->rbv = prec->val;
					/* send out monitors to val, because record support will do this only
					 * at the end of processing.
					 */
					db_post_events(prec,&prec->val,DBE_VALUE);
					prec->mlst=prec->val;
					return OK;
				}
				DM(1,"ik320WriteFunct() sending request (1st phase) failed...\n");

				/* something failed; return status */
				goto cleanup;
		}
	} else { 			/* completion phase */
		DM(1,"ik320WriteFunct() completion phase (irq status 0x%x)...\n",card->irqStatus);

		switch ((card->irqStatus>>8)&0xff) {
			case FUNC_POST:
			case FUNC_REF_X1:
			case FUNC_REF_X2:
					if ( 0 == (card->irqStatus & 0xff)) {
						status = OK;
					}
			break;

			case 0x10: /* vme preset */
					if (devState->axis == (int)(card->irqStatus & 0xff)) status = OK;
			break;

			default:
			break;
		}
		if (status) {
			epicsPrintf("devIK320Funct(): error irqStatus %x\n",card->irqStatus);
		}

		drvIK320Finish(devState->drv);
	}
cleanup:
	prec->rbv = 0;
	prec->val = 0;
	if (status) {
		prec->val = prec->mlst;
		recGblRecordError(status,prec,"ik320WriteFunct()");
		recGblSetSevr(prec,
				WRITE_ALARM,
				(S_drvIK320_cardBusy==status) ? MINOR_ALARM : INVALID_ALARM);
	}
	return OK;
}

/*

STATIC long
ik320IRQStatus(unsigned short irqStatus, int cmd)
{
	unsigned char sHi= (irqStatus>>8)&0xff;
	unsigned char sLo= (irqStatus & 0xff);
	switch (
}
 */

STATIC long
ik320WriteMbboSync(mbboRecord *prec)
{
int				cmd;
DevIK320Mbbo	devState=(DevIK320Mbbo)prec->dpvt;
long			status=ERROR;

	DM(1,"ik320WriteMbboSync() entering...\n");

	/* check if value is in valid range */
	if ( prec->val >= devState->nMenu || 0 == devState->menu[prec->val].name ) {
		status = S_drvIK320_invalidParm;
		goto cleanup;
	}

	cmd = devState->menu[prec->val].code;

	if (prec->pact || FUNC_NONE == cmd) return OK;

	status = drvIK320Request(devState->drv,0 /* sync request */,cmd,0 /* no parameter */);

	if (OK==status) { /* request was handled synchronously */
		IK320Card		card = drvIK320CARD(devState->drv);
		DM(1,"ik320WriteMbboSync() sync request (irq status 0x%x)...\n",card->irqStatus);
		if (card->irqStatus & 0xff) {
			status = ERROR;
			epicsPrintf("devIK320Dir(): error irqStatus %x\n",card->irqStatus);
		}
		drvIK320Finish(devState->drv);
	} else {
		DM(1,"ik320WriteMbboSync() sending request (synchronous) failed...\n");
	}

cleanup:
	if (status) {
		prec->val = prec->mlst;
		recGblRecordError(status,prec,"ik320WriteMbboSync()");
		epicsPrintf("status = %x",status);
		recGblSetSevr(prec,
					  WRITE_ALARM,
					  (S_drvIK320_cardBusy==status) ? MINOR_ALARM : INVALID_ALARM);
	}
	return OK;
}

STATIC long
ik320InitAiRec(aiRecord *prec)
{
    long            status;
    DevIK320Ai      devState=0;

    DM(1,"devIK320InitAiRec() entering\n");
    if (!(devState = (DevIK320Ai) malloc(sizeof(DevIK320AiRec))))
    {
	status = S_dev_noMemory;
	goto cleanup;
    }

    if ((status=ik320Connect(&prec->inp,&devState->axis,&devState->drv)))
	goto cleanup;

    prec->dpvt=devState;

    scanIoInit( &devState->scanPvt );

    return (OK);

    cleanup:
    prec->pact=TRUE;
    if (devState) free(devState);

    return (status);
}

STATIC long
get_ioint_info(int cmd, dbCommon *prec, IOSCANPVT *ppvt)
{
    DevIK320Ai devState = (DevIK320Ai)prec->dpvt;

    if (devState == NULL)
	return(ERROR);

    /* tell the driver we switched on/off io event scanning
     * NOTE: no interrupt must occur until the caller of get_iont_info
     *		 is in a safe state. Otherwise the result is undefined and
     *		 deadlock might occur (nobody calling drvIK320Finish()).	
     *		 We'd need a `special' processing routine here, so the
     *		 driver could be released in special(,after).
     */
    *ppvt = devState->scanPvt;
    
    if (devState->drv == NULL)
	return(ERROR);
    
    if (drvIK320RegisterIOScan( devState->drv, cmd ? 0 : & devState->scanPvt,
				devState->axis))
    {
	/* card is busy */
	return(ERROR);
    }
    ik320GroupChanged(cmd,drvIK320GroupNr(devState->drv));

    /* should be able to call this after processing finished :-( */
    drvIK320Finish(devState->drv);
    return(OK);
}

STATIC long
ik320ReadAiFinish(aiRecord *prec)
{
DevIK320Ai		devState=(DevIK320Ai)prec->dpvt;
IK320Card		card = drvIK320CARD(devState->drv);
double			rval;
long			status = ERROR;
int				idx = devState->axis-1;
int				astat;

		DM(1,"ik320ReadAiFinish() (irq status 0x%x)...\n",card->irqStatus);

		if (((card->irqStatus>>8)&0xff)==devState->axis) {
			/* common channel has no status */
			astat = (idx>1 ? ASTAT_RUNNING : card->X[idx].status & ASTAT_MASK);
			if ( (astat & ~ASTAT_COMPENS) == ASTAT_RUNNING ) { /* everything ok */
				status = OK;
				rval   = (double) card->X[idx].count;
				rval   *= (1<<card->interpBits);
				rval   += (double) (card->X[idx].interpol >> (16 - card->interpBits));
				DM(4,"rval = %g",rval);
				devState->value = prec->val = rval;
				/* do conversion on the (double) value; the original aiRecord
				 * does conversion only on the (long) rval field :-( :-( :-(
				 */
				aiCvtDouble(prec);
			} else if (astat & ASTAT_NO_SIGNAL) status = S_drvIK320_noSignal;
			else if ( ! (astat & ASTAT_RUNNING)) status = S_drvIK320_needsRef;
			else {
				status = S_drvIK320_HWreadError;
				epicsPrintf("ik320ReadAiFinish(): axis status %x\n",card->X[idx].status);
			}
			card->X[idx].xfer = 0; /* clear marker */
		} else {
			epicsPrintf("ik320ReadAiFinish(): error irqStatus %x\n",card->irqStatus);
		}

		drvIK320Finish(devState->drv);

		return status;
}

STATIC long
ik320ReadAi(aiRecord *prec)
{
DevIK320Ai		devState=(DevIK320Ai)prec->dpvt;
IK320Card		card = drvIK320CARD(devState->drv);
long			status=ERROR;
int				cmd;

	DM(1,"ik320ReadAi() entering...\n");

	if (0 == card->X[devState->axis-1].xfer) { /* no value present, trigger the card */

		switch (devState->axis) {
			case 1:		cmd=FUNC_READ_X1; break;
			case 2:		cmd=FUNC_READ_X2; break;
			case 3:		cmd=FUNC_READ_ALL; break; /* should set parms to get only IRQ for X3 */
			
			default:	status = S_drvIK320_invalidParm;
			goto cleanup;
		}

		status = drvIK320Request(devState->drv,(dbCommon*)prec,cmd,0);
		if (OK==status) { /* ok, request was handled synchronously */
			DM(1,"ik320ReadAi() sync request (1 phase only)...\n");
			status = ik320ReadAiFinish(prec);	
			goto cleanup;
		}

		if (S_drvIK320_asyncStarted == status) { /* ok, wait for completion */
			DM(1,"ik320ReadAi() sent request (1st phase)...\n");
			status = OK;
			goto cleanup;
		}
		DM(1,"ik320ReadAi() sending request (1st phase) failed...\n");

		/* something failed; return status */
		goto cleanup;
	} else { 			/* completion phase */
		status=ik320ReadAiFinish(prec);
		if ( ! prec->pact ) {
			/* if a value could be read, but this is not
			 * the completion phase of async processing, this means that
			 * somebody else triggered the card, i.e. the group must be
			 * notified.
			 */
			/* send the unconverted value to the group */
			ik320GroupAddValue(drvIK320GroupNr(devState->drv), devState->value, status);
		}
	}
cleanup:
	DM(1,"ik320ReadAi() exit (status %x).\n",status);
	
	if (status) {
		recGblRecordError(status,prec,"ik320ReadAi()");
		recGblSetSevr(prec,
				READ_ALARM,
				(S_drvIK320_cardBusy==status) ? MINOR_ALARM : INVALID_ALARM);
	}
	return 2; /* no conversion; we did it on our own */
}

STATIC long
ik320InitParm(stringoutRecord *prec)
{
 long rval = ik320Connect(&prec->out,
						  (short*)0 /* no axis info needed */,
						  (IK320Driver*)&prec->dpvt);
 if (rval) prec->pact = TRUE;
 return rval;
}

/*
 * scan the string argument for the syntax
 *
 * ^P[0-9]{2}([.][0-9]){0,1}
 *
 * and returns the corresponding function number on success
 * (e.g. P82.1 returns 821). -1 is returned on error.
 * 
 * pstr is adjusted to point to the trailing space or 'NULL' char.
 */
STATIC long
ik320scan(char **chpt)
{
int  i;
long rval=0;
long ch;
  if ( *(*chpt)++ != 'P' ) return -1;
  for (i=0; i<2; i++) {
	ch=(long)(*(*chpt)++) - '0';
	if ( ch < 0 || ch > 9 ) return -1;
	rval = 10 * rval + ch;
  }
  switch (*(*chpt)++) {
	case 0:
	case ':':
	case ' ': 	return 10*rval;	/* done, no '.x' */

	case '.':	break;			/* continue scanning */

	default:	return -1;		/* oops */
  }

  ch = (long)(*(*chpt)++) - '0';
  if ( ch < 0 || ch > 9 ) return -1;
  rval = 10 * rval + ch;

  switch (**chpt) {
	case 0:
	case ' ': return rval;

	default: return -1;
  }
  return -1;
}

STATIC long
ik320WriteParm(stringoutRecord *prec)
{
char *chpt = prec->val;
IK320Driver drv=(IK320Driver)prec->dpvt;
int			written;

/* a memory region to store arbitrarily typed parameter values */
union ValUnion_ {
	char			c;
	short			s;
	long			l;
	double			d;
	IK320ValueRec	v;
} rVal, wVal;

IK320ParmRec parm;

char *fmt;

IK320Parm entry = (IK320Parm) bsearch(	(void*)ik320scan(&chpt),
										parmTable,
										sizeof(parmTable)/sizeof(parmTable[0]),
										sizeof(IK320ParmRec),
										ik320ParmCompare);

	if ( NULL == entry ) { /* parameter table entry not found */
		/* invalid `val' */
		strncpy(prec->oval,prec->val,sizeof(prec->val));
		sprintf(prec->val,"??");
		return 0;
	}

	/* get the correct format string for this parameter */
	switch( entry->type ) {
		case byte:	fmt="%c"; break;
		case shrt:	fmt="%hi"; break;
		case wrd:	fmt="%i"; break;
		case six: 	fmt="%lf"; break;
		default:
			epicsPrintf("ik320WriteParm: unknown parameter size!\n");
			strncpy(prec->oval,prec->val,sizeof(prec->val));
			sprintf(prec->val,"error");
			return 0;
	}

	/* copy the parameter description to the 'parm' struct */
	parm.from = (char*) &wVal;
	parm.offset = entry->offset;
	/* `type' is an enum; however the elements are assigned a value equal to the
	 * size of the type they represent.
	 */
	parm.type = entry->type;


	/* check if they want to read or set the parameter */
	if ( 0 < (written=sscanf(chpt, fmt, &wVal)) ) {
		/* (synchronously) write the parameter */
		if (drvIK320Request(drv, 0, FUNC_SET_PARMS, &parm)) {
			recGblSetSevr(prec,WRITE_ALARM,MAJOR_ALARM);
			written = 0;
		} else {
			/* write succeeded; release the driver */
			drvIK320Finish(drv);	
		} 
	} 
	/* now try to read back */
	parm.from = (char*)&rVal;
	if (drvIK320Request(drv, 0, FUNC_GET_PARMS, &parm)) {
		strncpy(prec->oval,prec->val,sizeof(prec->val));
		sprintf(chpt,": read error");
		recGblSetSevr(prec,READ_ALARM,INVALID_ALARM);
	} else {
		unsigned long rv=0,wv=0;
		/* release the driver */
		drvIK320Finish(drv);
		switch(parm.type) {
			case byte: rv =  rVal.c; wv =  wVal.c; break;
			case shrt: rv =  rVal.s; wv =  wVal.s; break;
			case wrd:  rv =  rVal.l; wv =  wVal.l; break;
			case six: break;
		}

		/* compare the read back value */
		if ( written ) {
			if (parm.type == six) {
				if (rVal.d != wVal.d)	written=0;
			} else {
				if (rv != wv) 			written=0;
			}
		}

		if ( ! written ) { /* print read back value */
			strncpy(prec->oval,prec->val,sizeof(prec->val));
			if (parm.type == six)
				sprintf(chpt,": %f",rVal.d);
			else 
				sprintf(chpt,": %lu",rv);
		}
	}

	return 0;
}
