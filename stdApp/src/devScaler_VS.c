/*******************************************************************************
devScalerVS.c
Device-support routines for Joerger VS (64,32,16)-channel, 32-bit
read-on-the-fly scaler.

Original Author: Tim Mooney
Date: 7/1/03

Experimental Physics and Industrial Control System (EPICS)

Copyright 2003, the University of Chicago Board of Governors.

This software was produced under U.S. Government contract
W-31-109-ENG-38 at Argonne National Laboratory.

Initial development by:
	The Beamline Controls & Data Acquisition Group
	APS Operations Division
	Advanced Photon Source
	Argonne National Laboratory

Modification Log:
-----------------
.01  07/01/03	tmm     Development from devScaler.c
*******************************************************************************/
#include	<vxWorks.h>
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<stdlib.h>
#include	<string.h>
#include	<math.h>
#include	<iv.h>
#include	<wdLib.h>
#include	<rebootLib.h>

#include	<devLib.h> 
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<dbScan.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<drvSup.h>
#include	<dbScan.h>
#include	<special.h>
#include	<module_types.h>
#include	<dbEvent.h>

#include	"scalerRecord.h"
#include	"devScaler.h"

extern long locationProbe();
extern int logMsg(char *fmt, ...);

#define MAX(a,b) (a)>(b)?(a):(b)

/*** Debug support ***/
#define DEBUG 1
#if DEBUG
#define STATIC
#define Debug0(l,FMT) {  if(l <= devScaler_VSDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devScaler_VSDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#define Debug2(l,FMT,V,W) {  if(l <= devScaler_VSDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V,W); } }
#else
#define STATIC static
#define Debug0(l,f) ;
#define Debug(l,f,v) ;
#define Debug2(l,f,v,w) ;
#endif
volatile int devScaler_VSDebug=0;

#define CARD_ADDRESS_SPACE				0x800
#define READ_XFER_REG_OFFSET			0x000
#define READ_CLEAR_XFER_REG_OFFSET		0x100
#define READ_FLY_XFER_REG__OFFSET		0x200
#define STATUS_OFFSET					0x400
#define CTRL_OFFSET						0x402
#define A32BASE_HI_OFFSET				0x404
#define A32BASE_LO_OFFSET				0x406
/* Note the next three are D08(0) -- byte transfer at odd address */
#define IRQ_1_OVERFLOW_VECTOR_OFFSET		0x408
#define IRQ_2_FP_XFER_VECTOR_OFFSET			0x40A
#define IRQ_3_GATE_VECTOR_OFFSET			0x40C
#define IRQ_SETUP_OFFSET				0x40E
#define CLOCK_TRIG_MODE_OFFSET			0x410
#define INTERNAL_GATE_SIZE_OFFSET		0x412
#define REF_CLOCK_MODE_OFFSET			0x414
#define ID_OFFSET						0x41E
#define MASTER_RESET_OFFSET				0x420
#define CLOCK_XFER_REG_OFFSET			0x422
#define COUNT_ENABLE_OFFSET				0x424
#define COUNT_DISABLE_OFFSET			0x426
#define RESET_ALL_COUNTERS_OFFSET		0x428
#define ARM_OFFSET						0x42A
#define DISARM_OFFSET					0x42C
#define TRIG_GATE_OFFSET				0x42E
#define TEST_PULSE_OFFSET				0x430
#define CLEAR_INTERRUPT_2_3_OFFSET		0x432

/* Status register bits */
#define STATUS_ARM_OUT					0x1000
#define STATUS_ARM_IN					0x0800
#define STATUS_FP_GATE					0x0400
#define STATUS_PROGRAMMED_GATE			0x0200
#define STATUS_FP_RESET					0x0100
#define STATUS_IRQ_3_GATE				0x0080
#define STATUS_IRQ_2_FP_XFER			0x0040
#define STATUS_IRQ_1_OVERFLOW			0x0020
#define STATUS_IRQ_3_SOURCE_GATE		0x0010
#define STATUS_IRQ_2_SOURCE_FP_XFER		0x0008
#define STATUS_IRQ_1_SOURCE_OVERFLOW	0x0004
#define STATUS_GLOBAL_COUNT_ENABLE_FF	0x0002
#define STATUS_GLOBAL_COUNT_ENABLE		0x0001

/* Control register bits */
#define CTRL_RESET_ON_FP_XFER			0x0002
#define CTRL_RESET_ON_VME_XFER			0x0001

/* Module types */
struct VS_module_type {
	char type[16];
	char num_channels;
};

/* after subtracting 16 from (module_id>>10), we get an index into this table */
struct VS_module_type VS_module_types[14] = {
	{"VS64 TTL", 64},
	{"VS32 TTL", 32},
	{"VS16 TTL", 16},
	{"VS32 ECL", 32},
	{"VS16 ECL", 16},
	{"VS32 NIM", 32},
	{"VS16 NIM", 16},
	{"VS64D TTL", 64},
	{"VS32D TTL", 32},
	{"VS16D TTL", 16},
	{"VS32D ECL", 32},
	{"VS16D ECL", 16},
	{"VS32D NIM", 32},
	{"VS16D NIM", 16}
};

static long vs_num_cards = 2;
STATIC char *vs_addrs = (char *)0xd000;
STATIC unsigned char vs_InterruptVector = 0;
STATIC int vs_InterruptLevel = 5;

STATIC long scalerVS_report(int level);
STATIC long scalerVS_init(int after);
STATIC long scalerVS_init_record(scalerRecord *pscal);
#define scalerVS_get_ioint_info NULL
STATIC long scalerVS_reset(int card);
STATIC long scalerVS_read(int card, long *val);
STATIC long scalerVS_write_preset(int card, int signal, long val);
STATIC long scalerVS_arm(int card, int val);
STATIC long scalerVS_done(int card);

SCALERDSET devScaler_VS = {
	7, 
	scalerVS_report,
	scalerVS_init,
	scalerVS_init_record,
	scalerVS_get_ioint_info,
	scalerVS_reset,
	scalerVS_read,
	scalerVS_write_preset,
	scalerVS_arm,
	scalerVS_done
};

STATIC int scalerVS_total_cards;
STATIC struct scalerVS_state {
	int card_exists;
	int num_channels;
	int card_in_use;
	int count_in_progress; /* count in progress? */
	unsigned short ident; /* identification info for this card */
	char *localaddr; /* address of this card */
	int done;	/* 1: hardware has finished counting and record hasn't yet reacted */
	int preset[MAX_SCALER_CHANNELS];
	int gate_periods;
	int gate_freq_ix;
	struct callback *pcallback;
};
STATIC struct scalerVS_state **scalerVS_state = 0;

/**************************************************
* scalerVS_report()
***************************************************/
STATIC long scalerVS_report(int level)
{
	int card;

	if (vs_num_cards <=0) {
		printf("    No Joerger VSnn scaler cards found.\n");
	} else {
		for (card = 0; card < vs_num_cards; card++) {
			if (scalerVS_state[card]) {
				printf("    Joerger VS%-2d card %d @ %p, id: 0x%x %s, vector=%d\n",
					scalerVS_state[card]->num_channels,
					card, 
					scalerVS_state[card]->localaddr, 
					(unsigned int) scalerVS_state[card]->ident,
					scalerVS_state[card]->card_in_use ? "(in use)": "(NOT in use)",
					vs_InterruptVector + card);
			}
		}
	}
	return (0);
}

/**************************************************
* scalerVS_shutdown()
***************************************************/
STATIC int scalerVS_shutdown()
{
	int i;
	for (i=0; i<scalerVS_total_cards; i++) {
		scalerVS_reset(i);
	}
	return(0);
}


/**************************************************
* scalerEndOfGateISR()
***************************************************/
STATIC void scalerEndOfGateISR(int card)
{
	if (devScaler_VSDebug >= 5) logMsg("scalerEndOfGateISR: entry\n");
	if ((card+1) <= scalerVS_total_cards) {
		/* tell record support the hardware is done counting */
		scalerVS_state[card]->done = 1;

		/* get the record processed */
		callbackRequest((CALLBACK *)scalerVS_state[card]->pcallback);
	}
}


/**************************************************
* scalerEndOfGateISRSetup()
***************************************************/
STATIC int scalerEndOfGateISRSetup(int card)
{
	long status;
	volatile char *addr;
	volatile unsigned char *p8;
	volatile unsigned short *p16;
	unsigned short us;
	unsigned char uc;

	Debug(5, "scalerEndOfGateISRSetup: Entry, card #%d\n", card);
	if ((card+1) > scalerVS_total_cards) return(ERROR);
	addr = scalerVS_state[card]->localaddr;

	status = devConnectInterrupt(intVME, vs_InterruptVector + card,
		&scalerEndOfGateISR, (void *) card);
	if (!RTN_SUCCESS(status)) {
		errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n",
			  vs_InterruptVector + card);
		return (ERROR);
	}

	/* write interrupt level to hardware, and tell EPICS to enable that level */
	p16 = (unsigned short *)(addr+IRQ_SETUP_OFFSET);
	/* specify level for end-of-gate interrupt */
	*p16 = (*p16 & 0x0ff) | (vs_InterruptLevel << 8);
	status = devEnableInterruptLevel(intVME, vs_InterruptLevel);
	if (!RTN_SUCCESS(status)) {
		errPrintf(status, __FILE__, __LINE__,
			  "Can't enable enterrupt level %d\n", vs_InterruptLevel);
		return (ERROR);
	}
	/* Write interrupt vector to hardware */
	p16 = (unsigned short *)(addr + IRQ_3_GATE_VECTOR_OFFSET);
	*p16 = vs_InterruptVector + card;

	Debug(5, "scalerEndOfGateISRSetup: Exit, card #%d\n", card);
	return (OK);
}


/***************************************************
* initialize all software and hardware
* scalerVS_init()
****************************************************/
STATIC long scalerVS_init(int after)
{
	char *localaddr;
	unsigned long status;
	volatile unsigned short *p16;
	char *baseAddr, *probeAddr;
	int card, card_type;

	Debug(2,"scalerVS_init(): entry, after = %d\n", after);
	if (after) return(0);

	/* allocate scalerVS_state structures, array of pointers */
	if (scalerVS_state == NULL) {
    	scalerVS_state = (struct scalerVS_state **)
				calloc(1, vs_num_cards * sizeof(struct scalerVS_state *));

		scalerVS_total_cards=0;
		for (card=0; card<vs_num_cards; card++) {
		    scalerVS_state[card] = (struct scalerVS_state *)
					calloc(1, sizeof(struct scalerVS_state));
		}
	}

	/* Check out the hardware. */
	for (card=0; card<vs_num_cards; card++) {
		baseAddr = (char *)(vs_addrs + card*CARD_ADDRESS_SPACE);
		probeAddr = baseAddr+READ_XFER_REG_OFFSET;

		/* Can we read from scaler data bank? */
		status = locationProbe(atVMEA16, probeAddr);
		if (status != S_dev_addressOverlap) {
			if (card==0) {
				errPrintf(status, __FILE__, __LINE__,
					"locationProbe failed: address %p (VME A16)\n", probeAddr);
			}
			return(ERROR);
		}

		/* Can we reserve the required block of VME address space? */
		status = devRegisterAddress(__FILE__, atVMEA16, baseAddr,
			CARD_ADDRESS_SPACE, (void **)&localaddr);
		if (!RTN_SUCCESS(status)) {
			errPrintf(status, __FILE__, __LINE__,
				"Can't register 2048-byte block in VME A16 at address %p\n", baseAddr);
			return (ERROR);
		}

		/* Declare victory. */
		Debug(2,"scalerVS_init: we own 2048 bytes in VME A16 starting at %p\n", localaddr);
		scalerVS_total_cards++;
		scalerVS_state[card]->localaddr = localaddr;

		/* reset this card */
		p16 = (unsigned short *)(localaddr + MASTER_RESET_OFFSET);
		*p16 = 0; /* any write to this address causes reset */

		/* get this card's type, serial number, and number fo channels */
		p16 = (unsigned short *)(localaddr + ID_OFFSET);
		scalerVS_state[card]->ident = *p16;
		Debug(3,"scalerVS_init: Serial # = %d\n", scalerVS_state[card]->ident & 0x3FF);
		scalerVS_state[card]->card_exists = 1;

		/* get this card's type */
		card_type = *p16 >> 10;
		if ((card_type > 22) || (card_type < 16)) {
			errPrintf(status, __FILE__, __LINE__, "unrecognized module\n");
			scalerVS_state[card]->num_channels = 0;
			scalerVS_state[card]->card_exists = 0;
		}
		scalerVS_state[card]->num_channels = VS_module_types[card_type-16].num_channels;
		Debug(3,"scalerVS_init: nchan = %d\n", scalerVS_state[card]->num_channels);
	}

	Debug(3,"scalerVS_init: Total cards = %d\n\n",scalerVS_total_cards);

    if (rebootHookAdd(scalerVS_shutdown) < 0)
		epicsPrintf ("scalerVS_init: rebootHookAdd() failed\n");
	Debug(3,"scalerVS_init: scalers initialized%c\n", '.');
	return(0);
}

/***************************************************
* scalerVS_init_record()
****************************************************/
STATIC long scalerVS_init_record(struct scalerRecord *pscal)
{
	int card = pscal->out.value.vmeio.card;
	int status;
	struct callback *pcallbacks;

	/* out must be an VME_IO */
	switch (pscal->out.type)
	{
	case (VME_IO) : break;
	default:
		recGblRecordError(S_dev_badBus,(void *)pscal,
			"devScaler_VS (init_record) Illegal OUT Bus Type");
		return(S_dev_badBus);
	}

	Debug(5,"scalerVS_init_record: card %d\n", card);
	if (!scalerVS_state[card]->card_exists)
	{
		recGblRecordError(S_dev_badCard,(void *)pscal,
		    "devScaler_VS (init_record) card does not exist!");
		return(S_dev_badCard);
    }

	if (scalerVS_state[card]->card_in_use)
	{
		recGblRecordError(S_dev_badSignal,(void *)pscal,
		    "devScaler_VS (init_record) card already in use!");
		return(S_dev_badSignal);
    }
	scalerVS_state[card]->card_in_use = 1;
	pscal->nch = scalerVS_state[card]->num_channels;
	/* set hardware-done flag */
	scalerVS_state[card]->done = 1;

	/* setup interrupt handler */
	pcallbacks = (struct callback *)pscal->dpvt;
	scalerVS_state[card]->pcallback = (struct callback *)&(pcallbacks[3]);
	status = scalerEndOfGateISRSetup(card);

	return(0);
}


/***************************************************
* scalerVS_reset()
****************************************************/
STATIC long scalerVS_reset(int card)
{
	volatile char *addr;
	volatile unsigned short *pdata;

	Debug(5,"scalerVS_reset: card %d\n", card);
	if ((card+1) > scalerVS_total_cards) return(ERROR);
	addr = scalerVS_state[card]->localaddr;

	/* reset board */
	pdata = (unsigned short *)(addr + MASTER_RESET_OFFSET);
	*pdata = 0;

	/* clear hardware-done flag */
	scalerVS_state[card]->done = 0;

if (devScaler_VSDebug >= 1) {
	int i, j;
	for(i=0; i<10000; i++) j = i;
}
	return(0);
}

/***************************************************
* scalerVS_read()
* return pointer to array of scaler values (on the card)
****************************************************/
STATIC long scalerVS_read(int card, long *val)
{
	volatile char *addr;
	volatile long *pdata;
	volatile unsigned short *p16;
	unsigned short status;
	int i;

	Debug(8,"scalerVS_read: card %d\n", card);
	if ((card+1) > scalerVS_total_cards) return(ERROR);
	addr = scalerVS_state[card]->localaddr;

	p16 = (unsigned short *) (addr + CLOCK_XFER_REG_OFFSET);
	*p16 = 1; /* any write clocks all transfer registers */

	pdata = (long *) (addr + READ_XFER_REG_OFFSET);
	for (i=0; i < scalerVS_state[card]->num_channels; i++) {
		val[i] = pdata[i];
		if (i==0) {
			Debug2(10,"scalerVS_read: ...(chan %d = %ld)\n", i, val[i]);
		} else {
			Debug2(20,"scalerVS_read: ...(chan %d = %ld)\n", i, val[i]);
		}
	}
	p16 = (unsigned short *) (addr + STATUS_OFFSET);
	status = *p16;

	/* Write interrupt vector to hardware */
	p16 = (unsigned short *)(addr + IRQ_3_GATE_VECTOR_OFFSET);

	Debug2(10,"scalerVS_read: status=0x%x; irq vector=0x%x\n", status, *p16&0xff);
	return(0);
}

#define GATE_FREQ_TABLE_LENGTH 12

double gate_freq_table[GATE_FREQ_TABLE_LENGTH] = {
1e7, 5e6, 2.5e6, 1e6, 5e5, 2.5e5, 1e5, 5e4, 2.5e4, 11e4, 1e3, 100
};

int gate_freq_bits[GATE_FREQ_TABLE_LENGTH] = {
0,   3,   4,     5,   6,   7,     8,   9,   10,    11,   12,  13
};

/***************************************************
* scalerVS_write_preset()
* This hardware has no preset capability, but we can set a time gate.
* What we do here is put the hardware in "trigger" mode, set the gate-clock
* frequency and the number of gate-clock periods to count for.  We're going to
* get called once for each channel, but we do all the real work on the first call.
* From then on, we just make sure the presets are zero, and fix them if they aren't.
****************************************************/
STATIC long scalerVS_write_preset(int card, int signal, long val)
{
    scalerRecord *pscal;
	long *ppreset;
	unsigned short *pgate;
	volatile char *addr;
	volatile unsigned short *p16, gate_freq_ix;
	double gate_time, gate_freq, gate_periods;

	if (devScaler_VSDebug >= 5)
		printf("scalerVS_write_preset: card %d, signal %d, val %ld\n", card, signal, val);

	if ((card+1) > scalerVS_total_cards) return(ERROR);
	if ((signal+1) > MAX_SCALER_CHANNELS) return(ERROR);

	addr = scalerVS_state[card]->localaddr;
	pscal = (scalerRecord *)scalerVS_state[card]->pcallback->precord;

	if (signal > 0) {
		if (val != 0) {
			ppreset = &(pscal->pr1);
			ppreset[signal] = 0;
			db_post_events(pscal,&(ppreset[signal]),DBE_VALUE);
			pgate = &(pscal->g1);
			pgate[signal] = 0;
			db_post_events(pscal,&(pgate[signal]),DBE_VALUE);
		}
		return(0);
	}

	if (pscal->g1 != 1) {
		pscal->g1 = 1;
		db_post_events(pscal,&(pscal->g1),DBE_VALUE);
	}

	/*** set count time ***/
	gate_time = val / pscal->freq;
	/*
	 * find largest gate-clock frequency that will allow us to specify the
	 * requested count time with the 16-bit gate-preset register
	 */
	gate_freq_ix = 0;
	do {
		gate_freq = gate_freq_table[gate_freq_ix];
		gate_periods =  gate_time * gate_freq;
	} while ((gate_periods > 65535) && (++gate_freq_ix < GATE_FREQ_TABLE_LENGTH));
	p16 = (volatile unsigned short *) (addr + CLOCK_TRIG_MODE_OFFSET);
	*p16 = gate_freq_bits[gate_freq_ix] | 0x10;	/* software triggers gate start */

	/* set the gate-size register to the number of clock periods to count */
	p16 = (volatile unsigned short *) (addr + INTERNAL_GATE_SIZE_OFFSET);
	*p16 = (unsigned short)(MAX(4,gate_periods)); /* docs recommend min of 4 periods */

	/* save preset and frequency mask in scalerVS_state */
	scalerVS_state[card]->gate_periods = gate_periods;
	scalerVS_state[card]->gate_freq_ix = gate_freq_ix;

	/* tell record what preset and clock rate we're using  */
	pscal->pr1 = gate_periods;
	pscal->freq = gate_freq_table[gate_freq_ix];

	Debug2(5,"scalerVS_write_preset: gate_periods=%f, gate_freq=%f\n",
		gate_periods, gate_freq);

	return(0);
}

/***************************************************
* scalerVS_arm()
* Make scaler ready to count.  If ARM output is connected
* to ARM input, and GATE permits, the scaler will
* actually start counting.
****************************************************/
STATIC long scalerVS_arm(int card, int val)
{
    scalerRecord *pscal;
	volatile char *addr;
	volatile unsigned short *p16;
	volatile unsigned char *p8;
	unsigned short irq_setup;

	if (devScaler_VSDebug >= 5)
		printf("scalerVS_arm: card %d, val %d\n", card, val);

	if ((card+1) > scalerVS_total_cards) return(ERROR);
	addr = scalerVS_state[card]->localaddr;
	pscal = (scalerRecord *)scalerVS_state[card]->pcallback->precord;

	/* disable end-of-gate interrupt */
	p16 = (unsigned short *) (addr + IRQ_SETUP_OFFSET);
	*p16 &= (unsigned short) 0x07ff;

	if (val) {
		/* Set up and enable interrupts */
		/* Write interrupt vector to hardware */
		p16 = (unsigned short *)(addr + IRQ_3_GATE_VECTOR_OFFSET);
		*p16 = vs_InterruptVector + card;
		/* set end-of-gate interrupt level, and enable the interrupt */
		p16 = (unsigned short *)(addr+IRQ_SETUP_OFFSET);
		*p16 = (*p16 & 0x0ff) | (vs_InterruptLevel << 8) | 0x800;

		/*** start counting ***/

		/*
		 * How do I make sure the internal-gate counter is zero, and that it
		 * won't start counting as soon as ARM goes TRUE?
		 */

		/* clear hardware-done flag */
		scalerVS_state[card]->done = 0;

		/* enable all channels */
		p16 = (unsigned short *) (addr + COUNT_ENABLE_OFFSET);
		*p16 = 1; /* any write enables */

		/* arm scaler */
		p16 = (unsigned short *) (addr + ARM_OFFSET);
		*p16 = 1; /* any write sets ARM */

		/* Set trigger mode for internal gate */
		p16 = (volatile unsigned short *) (addr + CLOCK_TRIG_MODE_OFFSET);
		*p16 |= 0x0010;

		/* trigger gate */
		p16 = (unsigned short *) (addr + TRIG_GATE_OFFSET);
		*p16 = 1; /* any write triggers gate */

		p16 = (unsigned short *)(addr + IRQ_3_GATE_VECTOR_OFFSET);
		Debug(10,"scalerVS_arm: irq vector=0x%x\n", *p16&0xff);

	} else {
		/*** stop counting ***/
		/* disarm scaler */
		p16 = (unsigned short *) (addr + DISARM_OFFSET);
		*p16 = 1; /* any write resets ARM */

		/*
		 * Stop counter (change trigger mode from internal gate to external gate
		 * (which should be 1?)
		 */
		p16 = (volatile unsigned short *) (addr + CLOCK_TRIG_MODE_OFFSET);
		*p16 &= 0x000f;

		/* set hardware-done flag */
		scalerVS_state[card]->done = 1;

	}

	return(0);
}


/***************************************************
* scalerVS_done()
* On the first call to this function after the scaler stops counting, we return 1.
* else, return 0.
****************************************************/
STATIC long scalerVS_done(int card)
{
	if ((card+1) > scalerVS_total_cards) return(ERROR);

	if (scalerVS_state[card]->done) {
		/* clear hardware-done flag */
		scalerVS_state[card]->done = 0;
		return(1);
	} else {
		return(0);
	}
}


/*****************************************************
* scalerVS_VS_Setup()
* User (startup file) calls this function to configure
* us for the hardware.
*****************************************************/
void scalerVS_VS_Setup(int num_cards,	/* maximum number of cards in crate */
	int addrs,		/* Base Address(0x800-0xf800, 2048-byte boundary) */
	int vector,	/* valid vectors(64-255) */
	int intlevel
)	
{
	vs_num_cards = num_cards;
	vs_addrs = (char *)addrs;
	vs_InterruptVector = (unsigned char) vector;
	vs_InterruptLevel = intlevel & 0x07;
}

/* debugging function */
void scalerVS_show(int card, int level)
{
	char *addr = scalerVS_state[card]->localaddr;
	volatile unsigned char *p8;
	volatile unsigned short *p16;
	volatile unsigned long *p32;
	int i, num_channels;
	char module_type[16];

	printf("scalerVS_show: card %d\n", card);

	p16 = (unsigned short *) (addr+CTRL_OFFSET);
	printf("scalerVS_show: control reg = 0x%x\n", *p16&0x3);

	p16 = (unsigned short *) (addr+STATUS_OFFSET);
	printf("scalerVS_show: status reg = 0x%x\n", *p16&0x1ff);

	p16 = (unsigned short *) (addr+IRQ_SETUP_OFFSET);
	printf("scalerVS_show: irq level/enable = 0x%x\n", *p16&0xfff);

	p16 = (unsigned short *)(addr + IRQ_3_GATE_VECTOR_OFFSET);
	printf("scalerVS_show: irq 3 (end-of-gate) interrupt vector = %d\n", *p16&0xff);

	p16 = (unsigned short *) (addr+ID_OFFSET);
	i = (*p16&0xfc00) >> 10;
	if ((i >= 16) && (i <= 29)) {
		strncpy(module_type, VS_module_types[i-16].type, 15);
		num_channels = VS_module_types[i-16].num_channels;
	} else {
		sprintf(module_type, "unknown(%d)", i);
		num_channels = 0;
	}
	printf("scalerVS_show: module type = %d ('%s'), %d channels, serial # %d\n",
			i, module_type, num_channels, *p16&0x3ff);

	p32 = (unsigned long *) (addr + READ_XFER_REG_OFFSET);
	num_channels = level ? scalerVS_state[card]->num_channels : 1;
	for (i=0; i<num_channels; i++) {
		printf("    scalerVS_show: channel %d xfer-reg counts = %ld\n", i, p32[i]);
	}
	printf("scalerVS_show: scalerVS_state[card]->done = %d\n", scalerVS_state[card]->done);
	return;
}
