/*******************************************************************************
devScaler.c
Device-support routines for Lecroy 1151 16-channel, 32-bit scaler

Original Author: Tim Mooney
Date: 1/16/95

Experimental Physics and Industrial Control System (EPICS)

Copyright 1995, the University of Chicago Board of Governors.

This software was produced under U.S. Government contract
W-31-109-ENG-38 at Argonne National Laboratory.

Initial development by:
	The X-ray Optics Group
	Experimental Facilities Division
	Advanced Photon Source
	Argonne National Laboratory

OSI  by S. Kate Feng 3/03

Modification Log:
-----------------
.01  6/26/93	tmm     Lecroy scaler
.02  1/16/95    tmm     v1.0 Joerger scaler
.03  6/11/96    tmm     v1.1 fixed test distinguishing VSC16 and VSC8
.04  6/28/96    tmm     v1.2 use vsc_num_cards instead of MAX_SCALER_CARDS
.05  8/20/96    tmm     v1.3 scalerstate, localaddr no longer marked 'volatile'
.06 10/03/96    tmm     v1.4 fix off-by-one problem
.07  3/03/96    tmm     v1.5 fix test distinguishing VSC16 and VSC8 for ECL/NIM
.08  3/03/97    tmm     v1.6 add reboot hook (disable interrupts & reset)
.09  4/24/98    tmm     v1.7 use callbackRequest instead of scanIoReq
.10  3/20/03    skf     OSI
*******************************************************************************/
#include        <epicsVersion.h>

#if EPICS_VERSION < 3 || (EPICS_VERSION==3 && EPICS_REVISION < 14)
#define NOT_YET_OSI
#endif

#ifdef HAS_IOOPS_H
#include        <basicIoOps.h>
#endif

typedef unsigned int uint32;
typedef unsigned short uint16;

#if defined(NOT_YET_OSI) || defined(VXWORKSTARGET)
/* version 1.5 */
#include	<vxWorks.h>
#ifndef __vxworks
#define __vxworks 1
#endif
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<iv.h>
#include	<wdLib.h>
#include	<module_types.h>
#include        <rebootLib.h>
#else

#include	<stdlib.h>
#include	<stdio.h>
#include        <epicsTimer.h>
#include        <epicsThread.h>
#include        <epicsExport.h>

epicsTimerQueueId	scalerWdTimerQ=0;


#ifndef         ERROR
#define ERROR   -1
#endif

#ifndef         OK
#define OK      0
#endif

#endif

#include        <devLib.h>
 
#include	<string.h>
#include	<math.h>

#include	<alarm.h>
/* #include	<dbRecType.h>  tmm: EPICS 3.13 */
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<dbScan.h>
#include        <recGbl.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<drvSup.h>
#include	<dbScan.h>
#include	<special.h>
#include        <callback.h>

#include	"scalerRecord.h"
#include	"devScaler.h"

/*** Debug support ***/
#define PRIVATE_FUNCTIONS 0	/* normal:1, debug:0 */
#if PRIVATE_FUNCTIONS
#define STATIC static
#else
#define STATIC
#endif
#ifdef NODEBUG
#define Debug0(l,f) ;
#define Debug(l,f,v) ;
#else
#define Debug0(l,FMT) {  if(l <= devScalerdebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= devScalerdebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif
volatile int devScalerdebug=0;

#define CARD_0_ADDRESS 0x40000000
#define CARD_ADDRESS_SPACE 0x100
#define RESET_OFFSET 0x00
#define CTRL_OFFSET 0x04
#define DIRECTION_OFFSET 0x08
#define STATUS_ID_OFFSET 0x10
#define IRQ_VECTOR_OFFSET 0x10
#define IRQ_LEVEL_ENABLE_OFFSET 0x14
#define IRQ_MASK_OFFSET 0x18
#define IRQ_RESET_OFFSET 0x1C
#define REV_SERIAL_NO_OFFSET 0x20
#define MODULE_TYPE_OFFSET 0x24
#define MANUFACTURER_ID_OFFSET 0x28
#define DATA_0_OFFSET 0x80
#define PRESET_0_OFFSET 0xC0
static long vsc_num_cards = 2;
STATIC void *vsc_addrs = (void *)0x00d00000;
STATIC long	vsc_InterruptVector = 0;

STATIC long scaler_report(int level);
STATIC long scaler_init(int after);
STATIC long scaler_init_record(struct scalerRecord *psr);
STATIC long scaler_get_ioint_info(int cmd, struct dbCommon *p1, IOSCANPVT *p2);
STATIC long scaler_reset(int card);
STATIC long scaler_read(int card, long *val);
STATIC long scaler_write_preset(int card, int signal, long val);
STATIC long scaler_arm(int card, int val);
STATIC long scaler_done(int card);

SCALERDSET devScaler = {
	7, 
	scaler_report,
	scaler_init,
	scaler_init_record,
	scaler_get_ioint_info,
	scaler_reset,
	scaler_read,
	scaler_write_preset,
	scaler_arm,
	scaler_done
};
epicsExportAddress(SCALERDSET, devScaler);

STATIC int scaler_total_cards;
STATIC struct scaler_state {
	int card_exists;
	int num_channels;
	int card_in_use;
	int count_in_progress; /* count in progress? */
	unsigned short ident; /* identification info for this card */
	char *localaddr; /* address of this card */
	IOSCANPVT ioscanpvt;
	int done;
	int preset[MAX_SCALER_CHANNELS];
	CALLBACK *pcallback;
};
STATIC struct scaler_state **scaler_state = 0;

/**************************************************
* scaler_report()
***************************************************/
STATIC long scaler_report(int level)
{
	int card;

	if (vsc_num_cards <=0) {
		printf("    No Joerger VSCxx scaler cards found.\n");
	} else {
		for (card = 0; card < vsc_num_cards; card++) {
			if (scaler_state[card]) {
				printf("    Joerger VSC%-2d card %d @ %p, id: %d %s\n",
					scaler_state[card]->num_channels,
					card, 
					scaler_state[card]->localaddr, 
					(unsigned int) scaler_state[card]->ident,
					scaler_state[card]->card_in_use ? "(in use)": "(NOT in use)");
			}
		}
	}
	return (0);
}

/**************************************************
* scaler_shutdown()
***************************************************/
STATIC int scaler_shutdown()
{
	int i;
	for (i=0; i<scaler_total_cards; i++) {
		if (scaler_reset(i) <0) return(ERROR);
	}
        return(0);
}

static void writeReg16(volatile char *a16, int offset,uint16 value)
{
#ifdef HAS_IOOPS_H
   out_be16((volatile void*)(a16+offset), value);
#else
   volatile uint16 *reg;

    reg = (volatile uint16 *)(a16+offset);
    *reg = value;
#endif
}

static uint16 readReg16(volatile char *a16, int offset)
{
#ifdef HAS_IOOPS_H
	return in_be16((volatile void*)(a16+offset));
#else
    volatile uint16 *reg;
    uint16 value;

    reg = (volatile uint16 *)(a16+offset);
    value = *reg;
    return(value);
#endif
}

static void writeReg32(volatile char *a32, int offset,uint32 value)
{
#ifdef HAS_IOOPS_H
   out_be32((volatile void*)(a32+offset), value);
#else
   volatile uint32 *reg;

    reg = (volatile uint32 *)(a32+offset);
    *reg = value;
#endif
}

static uint32 readReg32(volatile char *a32, int offset)
{
#ifdef HAS_IOOPS_H
	return in_be32((volatile void*)(a32+offset));
#else
    volatile uint32 *reg;
    uint32 value;

    reg = (volatile uint32 *)(a32+offset);
    value = *reg;
    return(value);
#endif
}

/**************************************************
* scalerISR()
***************************************************/
STATIC int scalerISR(int card)
{
	volatile char *addr;
        uint16 value;	

	if ((card+1) > scaler_total_cards) return(ERROR);

	addr = scaler_state[card]->localaddr;
	/* disable interrupts during processing */
	value = readReg16(addr, IRQ_LEVEL_ENABLE_OFFSET) & 0x7f;
        writeReg16(addr, IRQ_LEVEL_ENABLE_OFFSET, value);

	/* clear interrupt */
        writeReg16(addr,IRQ_RESET_OFFSET, 0);

	/* tell record support the hardware is done counting */
	scaler_state[card]->done = 1;

	/* get the record processed */
	/* scanIoRequest(scaler_state[card]->ioscanpvt); */
	callbackRequest(scaler_state[card]->pcallback);

	/* enable interrupts */
	value = readReg16(addr, IRQ_LEVEL_ENABLE_OFFSET) | 0x80;
        writeReg16(addr, IRQ_LEVEL_ENABLE_OFFSET, value);

	return(0);
}


/**************************************************
* scalerISRSetup()
***************************************************/
STATIC int scalerISRSetup(int card)
{
	long status;
	volatile char *addr;
	volatile unsigned short *p16;
	int intLevel;
	
	Debug(5, "scalerISRSetup: Entry, card #%d\n", card);
	if ((card+1) > scaler_total_cards) return(ERROR);
	addr = scaler_state[card]->localaddr;

	/* scanIoInit(&(scaler_state[card]->ioscanpvt)); */

	status = devConnectInterrupt(intVME, vsc_InterruptVector + card,
		(void *) &scalerISR, (void *) card);
	if (!RTN_SUCCESS(status)) {
		errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n",
			  vsc_InterruptVector + card);
		return (ERROR);
	}

	/* get interrupt level from hardware, and enable that level in EPICS */
	intLevel = readReg16(addr,IRQ_LEVEL_ENABLE_OFFSET) & 3;
	Debug(5, "scalerISRSetup: Interrupt level %d\n", intLevel);
	status = devEnableInterruptLevel(intVME, intLevel);
	if (!RTN_SUCCESS(status)) {
		errPrintf(status, __FILE__, __LINE__,
			  "Can't enable enterrupt level %d\n", intLevel);
		return (ERROR);
	}
	/* Write interrupt vector to hardware */
	writeReg16(addr,IRQ_VECTOR_OFFSET,(unsigned short) (vsc_InterruptVector + card));
	Debug(5, "scalerISRSetup: Exit, card #%d\n", card);
	return (OK);
}


/***************************************************
* initialize all software and hardware
* scaler_init()
****************************************************/
STATIC long scaler_init(int after)
{
	void *localaddr;
	unsigned long status;
	void *baseAddr;
	volatile char *addr;
	int card, i;
        uint32 probeValue = 0;
 
	Debug(2,"scaler_init(): entry, after = %d\n", after);
	if (after) return(0);

	/* allocate scaler_state structures, array of pointers */
	if (scaler_state == NULL) {
    	scaler_state = (struct scaler_state **)
				calloc(1, vsc_num_cards * sizeof(struct scaler_state *));

		scaler_total_cards=0;
		for (card=0; card<vsc_num_cards; card++) {
		    scaler_state[card] = (struct scaler_state *)
					calloc(1, sizeof(struct scaler_state));
		}
	}

	/* Check out the hardware. */
	for (card=0; card<vsc_num_cards; card++) {
		baseAddr = (void *)(vsc_addrs + card*CARD_ADDRESS_SPACE);

		/* Can we reserve the required block of VME address space? */
		status = devRegisterAddress(__FILE__, atVMEA32, (size_t)baseAddr,
			CARD_ADDRESS_SPACE, &localaddr);
		if (!RTN_SUCCESS(status)) {
			errPrintf(status, __FILE__, __LINE__,
				"Can't register 0x%x-byte block at address 0x%x\n", CARD_ADDRESS_SPACE, baseAddr);
			return (ERROR);
		}

		/* Declare victory. */
		Debug(2,"scaler_init: we own 256 bytes starting at %p\n",localaddr);
		scaler_total_cards++;
		scaler_state[card]->localaddr = localaddr;
		addr = localaddr;

#ifdef NOT_YET_OSI
                if(vxMemProbe(addr+DATA_0_OFFSET,VX_READ,4,(char *)&probeValue)!=OK)
#else
	        if (devReadProbe(4,addr+DATA_0_OFFSET,(void*)&probeValue))
#endif
	        {
                   printf("no VSC card at %p\n",addr);
                   return(0);
                }

		/* reset this card */
		writeReg16(addr,RESET_OFFSET,0);

		/* get this card's identification */
		scaler_state[card]->ident = readReg16(addr,REV_SERIAL_NO_OFFSET);
		Debug(3,"scaler_init: Serial # = %d\n", scaler_state[card]->ident);
		scaler_state[card]->card_exists = 1;

		/* get this card's type (8 or 16 channels?) */
		Debug(2,"scaler_init:Base Address=0x%8.8x\n",(int)baseAddr);
		Debug(2,"scaler_init:Local Address=0x%8.8x\n",(int)localaddr);
		scaler_state[card]->num_channels =  readReg16(addr,MODULE_TYPE_OFFSET) & 0x18;
		Debug(3,"scaler_init: nchan = %d\n", scaler_state[card]->num_channels);

		for (i=0; i<MAX_SCALER_CHANNELS; i++) {
			scaler_state[card]->preset[i] = 0;
		}
	}

	Debug(3,"scaler_init: Total cards = %d\n\n",scaler_total_cards);

#ifdef __vxworks
    if (rebootHookAdd(scaler_shutdown) < 0)
		epicsPrintf ("scaler_init: rebootHookAdd() failed\n"); 
#else
    if (scaler_shutdown() < 0)
		epicsPrintf ("scaler_init: scaler_shutdown() failed\n"); 
#endif
#ifndef  NOT_YET_OSI
        scalerWdTimerQ=epicsTimerQueueAllocate(
				1, /* ok to share queue */
				epicsThreadPriorityLow);  
 	Debug0(3,"scalerWdTimerQ created\n");
#endif
	Debug0(3,"scaler_init: scalers initialized\n");
	return(0);
}

/***************************************************
* scaler_init_record()
****************************************************/
STATIC long scaler_init_record(struct scalerRecord *psr)
{
	int card = psr->out.value.vmeio.card;
	int status;
	struct callback *pcallbacks;

	Debug(5,"scaler_init_record: card %d\n", card);

	/* out must be an VME_IO */
	switch (psr->out.type)
	{
	case (VME_IO) : break;
	default:
		recGblRecordError(S_dev_badBus,(void *)psr,
			"devScaler (init_record) Illegal OUT Bus Type");
		return(S_dev_badBus);
	}

	Debug(5,"VME scaler: card %d\n", card);
	if (!scaler_state[card]->card_exists)
	{
		recGblRecordError(S_dev_badCard,(void *)psr,
		    "devScaler (init_record) card does not exist!");
		return(S_dev_badCard);
    }

	if (scaler_state[card]->card_in_use)
	{
		recGblRecordError(S_dev_badSignal,(void *)psr,
		    "devScaler (init_record) card already in use!");
		return(S_dev_badSignal);
    }
	scaler_state[card]->card_in_use = 1;
	psr->nch = scaler_state[card]->num_channels;

	/* setup interrupt handler */
	pcallbacks = (struct callback *)psr->dpvt;
	scaler_state[card]->pcallback = (CALLBACK *)&(pcallbacks[3]);
	status = scalerISRSetup(card);

	return(0);
}


/***************************************************
* scaler_get_ioint_info()
****************************************************/
STATIC long scaler_get_ioint_info(
	int cmd,
	struct dbCommon *prec,
	IOSCANPVT *ppvt)
{
	struct scalerRecord *psr = (struct scalerRecord *)prec;
	int card = psr->out.value.vmeio.card;

	Debug(5,"scaler_get_ioint_info: cmd = %d\n", cmd);
	*ppvt = scaler_state[card]->ioscanpvt;
	return(0);
}

/***************************************************
* scaler_reset()
****************************************************/
STATIC long scaler_reset(int card)
{
	volatile char *addr;
	volatile unsigned short *pdata;
	int i;
	uint16 value;

	Debug(5,"scaler_reset: card %d\n", card);
	if ((card+1) > scaler_total_cards) return(ERROR);
	addr = scaler_state[card]->localaddr;

	/* disable interrupt */
	value = readReg16(addr, IRQ_LEVEL_ENABLE_OFFSET) & 0x7f;
        writeReg16(addr, IRQ_LEVEL_ENABLE_OFFSET, value);

	/* reset board */
        writeReg16(addr,RESET_OFFSET, 0);

	/* zero local copy of scaler presets */
	for (i=0; i<MAX_SCALER_CHANNELS; i++) {
		scaler_state[card]->preset[i] = 0;
	}

	/* clear hardware-done flag */
	scaler_state[card]->done = 0;

	return(0);
}

/***************************************************
* scaler_read()
* return pointer to array of scaler values (on the card)
****************************************************/
STATIC long scaler_read(int card, long *val)
{
	long preset;
	volatile char *addr= scaler_state[card]->localaddr;
	int i, offset= DATA_0_OFFSET;
	unsigned short mask;

	Debug(8,"scaler_read: card %d\n", card);
	if ((card+1) > scaler_total_cards) return(ERROR);
	mask = readReg16(scaler_state[card]->localaddr,DIRECTION_OFFSET);
	for (i=0; i < scaler_state[card]->num_channels; i++, offset+=4) {
	    preset = scaler_state[card]->preset[i];
	    val[i] = (mask & (1<<i)) ? preset-readReg32(addr,offset):readReg32(addr,offset);
	    Debug(20,"scaler_read: ...(dir i = %s)\n", (mask & (1<<i)) ? "DN":"UP");
	    Debug(20,"scaler_read: ...(preset i = 0x%x)\n", (unsigned int)preset);
	    Debug(20,"scaler_read: ...(data i = 0x%x)\n",readReg32(addr,offset));
	    Debug(10,"scaler_read: ...(chan i = 0x%x)\n\n", (unsigned int)val[i]);
	}
	return(0);
}

/***************************************************
* scaler_write_preset()
****************************************************/
STATIC long scaler_write_preset(int card, int signal, long val)
{
	volatile char *addr= scaler_state[card]->localaddr;
	short mask;
	int offset= PRESET_0_OFFSET+signal*4;

	Debug(5,"scaler_write_preset: card %d\n", card);
	Debug(5,"scaler_write_preset: signal %d\n", signal);
	Debug(5,"scaler_write_preset: val = %d\n", (int)val);

	if ((card+1) > scaler_total_cards) return(ERROR);
	if ((signal+1) > MAX_SCALER_CHANNELS) return(ERROR);

	/* write preset; save a copy in scaler_state */
        writeReg32(addr,offset,val-1);
	scaler_state[card]->preset[signal] = val-1;

	/* make the preset scaler count down */
	mask = readReg16(addr,DIRECTION_OFFSET);
	mask |= 1<<signal;
	Debug(5,"scaler_write_preset: up/down mask = 0x%x\n", mask);
	writeReg16(addr,DIRECTION_OFFSET,mask);

	/* enable IRQ from preset channel */
	mask = readReg16(addr,IRQ_MASK_OFFSET);
	mask |= 1<<signal;
	Debug(5,"scaler_write_preset: IRQ mask = 0x%x\n", mask);
	writeReg16(addr,IRQ_MASK_OFFSET,mask);
	return(0);
}

/***************************************************
* scaler_arm()
* Make scaler ready to count.  If ARM output is connected
* to ARM input, and GATE input permits, the scaler will
* actually start counting.
****************************************************/
STATIC long scaler_arm(int card, int val)
{
	volatile char *addr=scaler_state[card]->localaddr;
	short ctrl_data;
	uint16  value;

	Debug(5,"scaler_arm: card %d\n", card);
	Debug(5,"scaler_arm: val = %d\n", val);
	if ((card+1) > scaler_total_cards) return(ERROR);

	/* disable interrupt */
	value = readReg16(addr,IRQ_LEVEL_ENABLE_OFFSET) & 0x7f;
	writeReg16(addr,IRQ_LEVEL_ENABLE_OFFSET,value);

	if (val) {
	   /* write the interrupt vector to the board */
	   writeReg16(addr,IRQ_VECTOR_OFFSET,(unsigned short)(vsc_InterruptVector+card));

	   /* enable interrupt-when-done */
	   value = readReg16(addr,IRQ_LEVEL_ENABLE_OFFSET) | 0x80;
	   writeReg16(addr,IRQ_LEVEL_ENABLE_OFFSET,value);
	}

	/* clear hardware-done flag */
	scaler_state[card]->done = 0;

	/* arm scaler */
	ctrl_data = readReg16(addr,CTRL_OFFSET);
	if (val) ctrl_data |= 1; else ctrl_data &= 0x0E;
	writeReg16(addr,CTRL_OFFSET,ctrl_data);
	Debug(5,"scaler_arm: ctrl reg => 0x%x\n",readReg16(addr,CTRL_OFFSET) & 0xf);
	return(0);
}


/***************************************************
* scaler_done()
****************************************************/
STATIC long scaler_done(int card)
{
	if ((card+1) > scaler_total_cards) return(ERROR);

	if (scaler_state[card]->done) {
		/* clear hardware-done flag */
		scaler_state[card]->done = 0;
		return(1);
	} else {
		return(0);
	}
}


/*****************************************************
* VSCSetup()
* User (startup file) calls this function to configure
* us for the hardware.
*****************************************************/
void VSCSetup(int num_cards,	/* maximum number of cards in crate */
	   void *addrs,		/* Base Address(0x100-0xffffff00, 256-byte boundary) */
	   unsigned vector)	/* noninterrupting(0), valid vectors(64-255) */
{
	vsc_num_cards = num_cards;
	vsc_addrs = addrs;
	vsc_InterruptVector = vector;
}

/* debugging function */
void scaler_show(int card)
{
	char *addr = scaler_state[card]->localaddr;
	int i, offset;

	printf("scaler_show: card %d\n", card);
	printf("scaler_show: ctrl reg = 0x%x\n", readReg16(addr,CTRL_OFFSET) &0xf);
	printf("scaler_show: dir reg = 0x%x\n",readReg16(addr,DIRECTION_OFFSET) );
	printf("scaler_show: irq vector = 0x%x\n",readReg16(addr,STATUS_ID_OFFSET) &0xff);
	printf("scaler_show: irq level/enable = 0x%x\n",readReg16(addr,IRQ_LEVEL_ENABLE_OFFSET) &0xff);
	printf("scaler_show: irq mask reg = 0x%x\n", readReg16(addr,IRQ_MASK_OFFSET));
	printf("scaler_show: module type = 0x%x\n",readReg16(addr,MODULE_TYPE_OFFSET) &0xff);
	offset = DATA_0_OFFSET;
	for (i=0; i<scaler_state[card]->num_channels; i++, offset+=4 ) {
		printf("    scaler_show: channel %d counts = %d\n", i,readReg32(addr,offset) );
	}
	printf("scaler_show: scaler_state[card]->done = %d\n", scaler_state[card]->done);
}
