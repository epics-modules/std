/* $Id: drvIK320.h,v 1.1.1.1 2001-07-03 20:05:28 sluiter Exp $ */
#ifndef DRV_IK320_H
#define DRV_IK320_H

/* DISCLAIMER: This software is provided `as is' and without _any_ kind of
 *             warranty. Use it at your own risk - I won't be responsible
 *             if your dog drowns as a consequence of using my software
 *             blah & blah.
 */

/*
 * driver for the Heidenhain IK320
 *
 * Author: Till Straumann (PTB, 1999) 
 *
 * $Log: not supported by cvs2svn $
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
 * Revision 1.8  1999/04/27 10:00:21  strauman
 *  - card parameters are not set to default values after soft reboot.
 *  - added support for X3 mode (disabled, X1+X2, X1-X2, (X1+X2)/2 )
 *  - added support for simultaneous reference search of both axes
 *    (using function nr 0x18)
 *  - function call requests to the card now have a timeout
 *  - setting card parameters _requires_ a 0xa function call.
 *  - need delay between `load parameters' and `search reference'
 *    function calls.
 *
 *
 */

#include <vxWorks.h>
#include <semLib.h>

/* Note: defining USE_WITHOUT_EPICS is broke and not supported anymore.
 */
#ifndef USE_WITHOUT_EPICS
#include <dbAccess.h>
#include <dbScan.h>
#else
typedef struct dbCommonRec *dbCommon;
#endif

/* error codes */
#define M_drvIK320					( 1033<<16 )	
#define S_drvIK320_cardBusy 		( M_drvIK320 | 1 ) /* IK320 card busy */						/* 1 */
#define S_drvIK320_invalidFunc		( M_drvIK320 | 2 ) /* IK320 invalid VME function call */		/* 2 */
#define S_drvIK320_asyncStarted		( M_drvIK320 | 3 ) /* IK320 asynchronous processing started */	/* 0 */
#define S_drvIK320_invalidParm		( M_drvIK320 | 4 ) /* IK320 invalid parameter */				/* 2 */
#define S_drvIK320_HWreadError		( M_drvIK320 | 5 ) /* IK320 hardware read error */				/* 3 */
#define S_drvIK320_needsRef			( M_drvIK320 | 6 ) /* IK320 reference not set */				/* 3 */
#define S_drvIK320_noSignal			( M_drvIK320 | 7 ) /* IK320 signal level too low */				/* 3 */
#define S_drvIK320_maskedChannel	( M_drvIK320 | 8 ) /* IK320 channel is deactivated */			/* 3 */
#define S_drvIK320_needsPOST		( M_drvIK320 | 9 ) /* IK320 needs POST */						/* 3 */
#define S_drvIK320_timeout			( M_drvIK320 | 10 )/* IK320 command timeout */					/* 3 */


/* hw error codes */
#define S_drvIK320_xferSetX1		( M_drvIK320 | 0x0601 ) /* xfer marker set on X1 */				/* 3 */
#define S_drvIK320_xferSetX2		( M_drvIK320 | 0x0602 ) /* xfer marker set on X1 */				/* 3 */
#define S_drvIK320_xferSetX3		( M_drvIK320 | 0x0603 ) /* xfer marker set on X1 */				/* 3 */

#define DISABLED	0
#define DIR_NORMAL	0
#define DIR_INVERSE	1
#define MODE_LIN	1
#define MODE_ANG	2
#define IRQ_MASK_X1		((unsigned char)(1<<0))
#define IRQ_MASK_X2		((unsigned char)(1<<1))
#define IRQ_MASK_EXTERN ((unsigned char)(1<<4))
#define IRQ_MASK_INVALID ((unsigned char)(0xff))
#define MODE_X3_SUM  1
#define MODE_X3_DIFF 2
#define MODE_X3_MEAN 3


/* max. possible number of synchronous groups */
#define MAX_IK320_GROUPS 8

#define GROUP_NR(switch) (((switch)&0xe0)>>5)

/*
 * The driver function calls
 */

#define FUNC_NONE		0
#define FUNC_ABORT		0

#define FUNC_READ_X1	1	
#define FUNC_READ_X2	2	
#define FUNC_READ_ALL	3	
#define FUNC_PRE_X1		4
#define FUNC_PRE_X2		5
#define FUNC_PRE_X3		6
#define FUNC_POST		7
#define FUNC_REF_X1		8
#define FUNC_REF_X2		9
#define FUNC_SET_PARMS	0xa	
#define FUNC_COMP_X1	0xb	
#define FUNC_COMP_X2	0xc	

#define FUNC_REF_BOTH	0x18

#define FUNC_GRP_TRIG	0x0b00

/* special functions */
/* set the combination mode; 
 * NOTE: if X3 is not DISABLED, interrupts for X1/X2 are switched off,
 *		 i.e. the individual axes cannot be read.
 */
#define FUNC_MODE_X3_DISABLE	0x10000 
#define FUNC_MODE_X3_SUM		(0x10000 | MODE_X3_SUM )
#define FUNC_MODE_X3_DIFF 		(0x10000 | MODE_X3_DIFF )
#define FUNC_MODE_X3_MEAN		(0x10000 | MODE_X3_MEAN )
/* set the direction flag */
#define FUNC_DIR_X1_POS			0x20000
#define FUNC_DIR_X1_NEG			0x20001
#define FUNC_DIR_X2_POS			0x20100
#define FUNC_DIR_X2_NEG			0x20101
/* get a parameter (lock driver) */
#define FUNC_GET_PARMS			0x3000a


/*
 * axis status bits
 */
#define ASTAT_COMPENS	(1<<0)	/* compensation active */
#define ASTAT_RUNNING	(1<<2)	/* OK, counter running */
#define ASTAT_NO_SIGNAL	(1<<3)	/* signal too low */
#define ASTAT_FREQ		(1<<4)	/* frequency too high */
#define ASTAT_REFING	(1<<5)	/* reference run active */
#define ASTAT_COMPING	(1<<7)	/* compensation calculation active */

#define ASTAT_MASK		0xbd	/* mask of defined bits */

/* IK320 has 48-bit registers */

typedef struct IK320ValueRec_ {
	long			count		__attribute__ ((packed));
	unsigned short	interpol	__attribute__ ((packed));
} IK320ValueRec, *IK320Value;

/* Type of individual IK320 parameters */

typedef enum {
	byte=sizeof(char),
	shrt=sizeof(short),
	wrd=sizeof(long),
	six=sizeof(IK320ValueRec)
} IK320Type;

/* If a parameter is to be set by calling FUNC_SET_PARMS, an IK320ParmRec
 * has to be passed.
 *
 * Since there is no 48-bit integer, we assume the user passes us
 * a pointer to a (double).
 * The (double) `from' value is automatically converted to an 48-bit
 * `six'-typed register value. Note that the content of the
 * `interpBits' register is honoured when the conversion is done,
 * i.e. the (double) is converted to a 6-byte integer and shifted
 * left (16 - interpBits) bits.
 *
 * likewise, on FUNC_GET_PARMS, if the parameter is of type `six'
 * the user must provide a (double) for storage. The 48-bit register
 * value is converted to a double and divided by 2^(16-interpBits). 
 */
typedef struct IK320ParmRec_ {
	char			*from;
	short			offset;
	IK320Type		type;
} IK320ParmRec, *IK320Parm;

typedef struct IK320DataRec_ {
	long			count		__attribute__ ((packed));
	unsigned short	interpol	__attribute__ ((packed));
	unsigned char	status		__attribute__ ((packed));
	unsigned char	xfer		__attribute__ ((packed));
} IK320DataRec, *IK320Data;

/* dual port ram memory map layout */

typedef struct IK320CardRec_ {
	IK320DataRec 	X[3]			__attribute__ ((packed));
	unsigned short	irqStatus		__attribute__ ((packed));
	unsigned short	CRC				__attribute__ ((packed));
	unsigned short	actualCRC		__attribute__ ((packed));
	unsigned long	CRC1			__attribute__ ((packed));
	unsigned long	actualCRC1		__attribute__ ((packed));
	unsigned long	CRC2			__attribute__ ((packed));
	unsigned long	actualCRC2		__attribute__ ((packed));
	unsigned short	hwVersion		__attribute__ ((packed));
	char			swVersion[12]	__attribute__ ((packed));
	char			sysVar[2]		__attribute__ ((packed)); /* for internal use only */
	IK320ValueRec	actPreset[3]	__attribute__ ((packed));
	unsigned char	free[0xA8]		__attribute__ ((packed));
	short  			X1I				__attribute__ ((packed));
	short  			X1Q				__attribute__ ((packed));
	short  			X2I				__attribute__ ((packed));
	short  			X2Q				__attribute__ ((packed));
	/* Parameter region */
	unsigned short  function		__attribute__ ((packed));
	unsigned char   direction[4]	__attribute__ ((packed)); /* 1 padding byte */
	unsigned char   linAng[4]		__attribute__ ((packed)); /* 1 padding byte */
	unsigned short	interpBits		__attribute__ ((packed));
	unsigned short	deltaRefX1		__attribute__ ((packed));
	unsigned short	deltaRefX2		__attribute__ ((packed));
	unsigned long   nPeriods[3]		__attribute__ ((packed));
	unsigned char   useCompX1		__attribute__ ((packed));
	unsigned char   useCompX2		__attribute__ ((packed));
	unsigned long   compStartX1		__attribute__ ((packed));
	unsigned long   compStartX2		__attribute__ ((packed));
	unsigned short	nSamplesX1		__attribute__ ((packed));
	unsigned short	nSamplesX2 		__attribute__ ((packed));
	unsigned short	sampleWidthX1	__attribute__ ((packed));
	unsigned short	sampleWidthX2	__attribute__ ((packed));
	unsigned char   enableIRQ		__attribute__ ((packed));
	unsigned char   pad				__attribute__ ((packed)); /* 1 padding byte */
	long			refOffsetX1		__attribute__ ((packed));
	long			refOffsetX2		__attribute__ ((packed));
	unsigned char	modeX3			__attribute__ ((packed));
	unsigned char   pad1			__attribute__ ((packed)); /* 1 padding byte */
	unsigned char   compDirX1		__attribute__ ((packed));
	unsigned char   compDirX2		__attribute__ ((packed));
	IK320ValueRec	extPreset[3]	__attribute__ ((packed));
	IK320ValueRec	vmePreset[3]	__attribute__ ((packed));
	IK320ValueRec	offset[3]		__attribute__ ((packed));
	unsigned char   extFunctionX1	__attribute__ ((packed));
	unsigned char   extFunctionX2	__attribute__ ((packed));
} IK320CardRec, *IK320Card;

/* fill a IK320ParmRec for a specific field in the IK320CardRec
 *
 * NOTE: the base type of the `from' pointer must be the same type
 *		 as `field'.
 */

#define IK320_FILL_PARM(parm,pfrom,field) { (parm)->from = (char*)(pfrom);\
									 (parm)->offset = (char*)(&((IK320Card)0)->field) - (char*)0;\
									 (parm)->type = sizeof( ((IK320Card)0)->field );\
									 assert(   sizeof( *(pfrom) ) == (parm)->type \
											|| ((parm)->type == sizeof(IK320ValueRec)\
												&& sizeof( *(pfrom) ) == sizeof(double))); }

typedef struct IK320DriverRec_ *IK320Driver;


/*
 * obtain a driver handle (in *pDrvRtn) for an IK320 card with switch settings
 * sw1/sw3 and the specified irqLevel. Return value is the error status.
 */

long
drvIK320Connect(int sw1, int sw2, int irqLevel, IK320Driver *pDrvRtn);

/*
 * FOR DEBUGGING PURPOSE ONLY
 *
 * Release the driver memory, interrupt and VME address space.
 * This is handy, because you may unload/reload a new version of
 * the driver module (testing new versions without rebooting).
 + This is possible at the driver level (EPICS knows little about
 * drivers).
 */
long
drvIK320Disconnect(IK320Driver drv);

/*
 * call a driver function.
 * If no record is passed in 'prec', the call is synchronous, i.e.
 * the caller will block for the interrupt of the IK card.
 * The caller may access the card registers for reading (no other
 * requests will be processed while this one is in progress.)
 * It is the responsibility of the caller
 * to call 'drvIK320Finish()' in order to release the driver.
 *
 * Alternatively, a record pointer may be passed. If 'drvIK320Request()'
 * considers 'func' to be a slow operation it returns S_drvIK320_asyncStarted
 * and 'prec' will be processed by the card interrupt handler (of course this
 * is done indirectly, i.e. by the high priority callback task).
 *
 * This behavior implements the asynchronous record processing semantics
 * i.e.
 *	- record processing calls drvIK320Request
 *  - if the return value will be S_drvIK320_asyncStarted, pact is set TRUE 
 *	  (by drvIK320Request()).
 *	- interrupt handler will cause the record to be processed again
 *  - record processing recognizes that this is now phase two, reads
 *	  any values from the card.
 *  - record processing calls drvIK320Finish() to release the driver
 *    and returns.
 * 
 * Note: This driver is quite simple. It does no request queuing and
 *       simply rejects requests while another being in progress (for
 *       a single card - multiple cards _can_ be accessed simultaneously).
 *		 Once a request has been granted and is in progress, the requestor
 *		 `owns' the driver and may e.g. access the hardware registers (in
 *       order to read them). It is the responsibility of the requestor to
 *		 `release' the driver/card by calling drvIK320Finish(), so other
 *		 requests may be processed.
 */
long
drvIK320Request(IK320Driver drv, dbCommon *prec, int func, void *parm);

/* this MUST be called after accessing the hardware registers, which
 * may only be done after a successful request completion.
 *
 */
void
drvIK320Finish(IK320Driver drv);

/*
 * register a record for IO event scanning. A pointer to IOSCANPVT
 * is passed to the driver along with the channel number. An irq
 * by the specified channel results in a io scan request.
 * Passing pIOscanPvt==0 disables further scanning.
 *
 * IMPORTANT NOTE: in order for io/scanning to work, JUMPER 3 has
 * to be set on one card of each group, so DTACK signal is generated.
 *
 * returns: OK or S_drvIK320_cardBusy
 */

#ifndef USE_WITHOUT_EPICS
long
drvIK320RegisterIOScan(IK320Driver drv, IOSCANPVT *pIOscanPvt, int axis);
#endif

/*
 * obtain a pointer to the hardware structure.
 * Note that this pointer may only be used after successfully
 * having called drvIK320Request() (synchronous or asynchronous
 * call completion)  and before relinquishing the
 * card by drvIK320Finish().
 * 
 */
IK320Card
drvIK320CARD(IK320Driver drv);

/*
 * determine the number of axes that have their scanPvt field
 * set and belong to a common group.
 *
 */
int
drvIK320getNGroupListeners(int groupNr);

/*
 * return the group number of this driver
 */
int
drvIK320GroupNr(IK320Driver drv);

/*
 * issue a group trigger command;
 *
 * RETURNS: S_drvIK320_cardBusy if any member of the
 *			group is currently busy.
 */
long
drvIK320GroupTrigger(int groupNr);

/*
 * utility routine to set the number of interpolation bits.
 *
 * This is a demo for the use of IK320_FILL_PARM / FUNC_SET_PARMS
 *
 * pnBits is a pointer to an integer containing the new value.
 * If this value is out of range, the card uses a default which
 * is written back to *pnBits.
 *
 * RETURNS: error status.
 */
long
drvIK320SetInterpBits(IK320Driver drv, int *pnBits);

#if 0 /* not implemented */
/*
 * conversion routines IK320ValueRec <--> (double)
 * Some registers of the IK320 are 48-bit. Externally
 * we use a (double) to hold such values.
 *
 */

double
drvIK320Val2Double(IK320Value val);

void
drvIK320Double2Val(double d, IK320Value val);
#endif

#endif
