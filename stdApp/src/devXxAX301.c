/* devXxHeidenhainGpib.c */
/* share/src/devOpt @(#)devXxSkeletonGpib.c	1.2 7/1/93 */
/*
 *      Author: John Winans
 *      Date:   02-18-92
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1988, 1989, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
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
 * All rights reserved. No part of this publication may be reproduced, 
 * stored in a retrieval system, transmitted, in any form or by any
 * means,  electronic, mechanical, photocopying, recording, or otherwise
 * without prior written permission of Los Alamos National Laboratory
 * and Argonne National Laboratory.
 *
 * Modification Log:
 * -----------------
 * .01  02-18-92        jrw     Initial Release
 * .02  07-01-93        ddr     Modified Template to run Heidenhain Encoder
 * .03  04-27-94	ddr	Modified to run Queensgate Piezo controller
 * .04	10-12-94	ddr	Added atomic reads and writes
 */

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * NOTE: The dsets are referenced by the entries in the command table.
 *
 ******************************************************************************/
#define	DSET_AI		devAiAX301
#define	DSET_AO		devAoAX301
#define DSET_BO		devBoAX301
#define DSET_LO		devLoAX301

#include	<stdlib.h>
#include	<string.h>
#include	<stdio.h>

#include	<alarm.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<devSup.h>
#include	<recSup.h>
#include	<drvSup.h>
#include	<link.h>
#include	<dbCommon.h>
#include	<aiRecord.h>
#include	<aoRecord.h>
#include	<biRecord.h>
#include	<boRecord.h>
#include	<mbbiRecord.h>
#include	<mbboRecord.h>
#include	<stringinRecord.h>
#include	<stringoutRecord.h>
#include	<longinRecord.h>
#include	<longoutRecord.h>

#include	<drvGpibInterface.h>
#include	<devCommonGpib.h>
#include	<devGpib.h>	/* needed to exportAddress the DSETS defined above */

#ifndef VALID_ALARM
#define VALID_ALARM INVALID_ALARM
#endif


/******************************************************************************
 *
 * Debugging flags that can be accessed from the shell.
 *
 ******************************************************************************/
int AX301Debug = 0;
extern int ibSrqDebug;		/* declared in the GPIB driver */

/******************************************************************************
 *
 * Use the TIME_WINDOW defn to indicate how long commands should be ignored
 * for a given device after it times out.  The ignored commands will be
 * returned as errors to device support.
 *
 * Use the DMA_TIME to define how long you wish to wait for an I/O operation
 * to complete once started.
 *
 * These are to be declared in 60ths of a second.
 *
 ******************************************************************************/

#define TIME_WINDOW	600		/* 10 seconds */
#define	DMA_TIME	60		/* 1 second */



/******************************************************************************
 *
 * String arrays for EFAST operations.  Note that the last entry must be 
 * NULL.
 *
 * On input operations, only as many bytes as are found in the string array
 * elements are compared.  If there are more bytes than that in the input
 * message, they are ignored.  The first matching string found (starting
 * from the 0'th element) will be used as a match.
 *
 * NOTE: For the input operations, the strings are compared literally!  This
 * can cause problems if the instrument is returning things like \r and \n
 * characters.  You must take care when defining input strings so you include
 * them as well.
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Array of structures that define all GPIB messages
 * supported for this type of instrument.
 *
 *     This is the parameter table for the gpib messages.  These parameters are
 *     what is entered into the param space in DCT  ---ddr
 *
 ******************************************************************************/

/* forward declarations of some custom convert routines */
static int readconvert();
static int writeconverta();
static int writeconvertb();

static struct gpibCmd gpibCmds[] = 
{
    /* Initialization sequence: Sets ports to be inputs */
    /* Param 0 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "*QT", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Select module 1 */ 
    /* Param 1 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "M!14", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Select module 2 */
    /* Param 2 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "M!24", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Put into Listen mode */
    /* Param 3 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "N!3", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Start Conversion */
    /* Param 4 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "N!4", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Talk Mode */
    /* Param 5 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "N!0", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Stop Conversion and read */
    /* Param 6 */
  {&DSET_AI, GPIBRAWREAD, IB_Q_HIGH, NULL, "%lf\n", 0, 32,
  readconvert, 0, 0, NULL, NULL, -1},  

    /* Write Data */
    /* Param 7 */
  {&DSET_LO, GPIBWRITE, IB_Q_HIGH, NULL, "I!%4.4X", 0, 32,
  writeconverta, 0, 0, NULL, NULL, -1},

    /* Read mod. 1 data in one message --- test */
    /* Param 8 */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "M!14\nN!4\nN!0\nN!4\n", "%lf\n", 0, 32,
  readconvert, 0, 0, NULL, NULL, -1},

    /* Write mod. 1 data in one message --- test */
    /* Param 9 */
  {&DSET_LO, GPIBWRITE, IB_Q_HIGH, "M!14\nN!3\nI!%4.4X", NULL,  0,
32,
  writeconverta, 0, 0, NULL, NULL, -1},

    /* Read mod. 2 data in one message --- test */
    /* Param 10 */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "M!24\nN!4\nN!0\nN!4\n", "%lf\n", 0, 32,
 readconvert, 0, 0, NULL, NULL, -1},

    /* Write mod. 2 data in one message --- test */
    /* Param 11 */
  {&DSET_LO, GPIBWRITE, IB_Q_HIGH, "M!24\nN!3\nI!%4.4X", NULL, 0,
32,
  writeconvertb, 0, 0, NULL, NULL, -1},

};

/* The following is the number of elements in the command array above.  */
#define NUMPARAMS	sizeof(gpibCmds)/sizeof(struct gpibCmd)

/****************************************************************************
 *
 * 
 * Custom read convert routine for Queensgate AX301 encoder 
 *                      4-29-94  ddr 
 *
 *
 ***************************************************************************/
static int readconvert(struct gpibDpvt *pdpvt, int	p1, int	p2, char **p3)
{
	unsigned int holdingInt = 0;
	int value = 0;

	aiRecord *pairec = (struct aiRecord *) (pdpvt->precord);
	char *msg=(char *)(pdpvt->msg);

	sscanf (msg, "%x", &holdingInt);

	if (holdingInt & 0x8000) {
		value = (~holdingInt);
		value = (0x0000FFF0 & value);
	} else {
		value = -holdingInt;
	}
	pairec->val = (value);

	return(0);
}
/****************************************************************************
 *
 *
 * Custom write convert routine for Queensgate AX301 encoder module 1
 *                      5-4-94  ddr
 *
 *
 ***************************************************************************/
static int writeconverta(struct gpibDpvt *pdpvt, int p1, int p2, char **p3)
{
	long value = 0;

	longoutRecord *plorec = (struct longoutRecord *) (pdpvt->precord);
	char *msg = (char *)(pdpvt->msg);

	value = plorec->val ;

	if (value < 0) {
		sprintf(msg, "I!%4.4lX", ((~(-value))&0x3FFF));
	} else {
		sprintf(msg, "I!%4.4lX", value);
	}
	return(0);
}

/****************************************************************************
 *
 *
 * Custom write convert routine for Queensgate AX301 encoder module 2
 *                      5-4-94  ddr
 *                  mod: 10-17-94 ddr
 *
***************************************************************************/
static int writeconvertb(struct gpibDpvt *pdpvt, int p1, int p2, char **p3)
{
	long value = 0;

	longoutRecord *plorec = (struct longoutRecord *)(pdpvt->precord);
	char*msg=(char *)(pdpvt->msg);

	value = plorec->val ;

	if (value < 0) {
		sprintf(msg, "M!24\nN!3\nI!%4.4lX", ((~(-value))&0x3FFF));
	} else {
		sprintf(msg, "M!24\nN!3\nI!%4.4lX",value);
	}

	return(0);
}

/******************************************************************************
 *
 * InitialIZATion for device support
 * This is called one time before any records are initialized with a parm
 * value of 0.  And then again AFTER all record-level init is complete
 * with a param value of 1.
 *
 ******************************************************************************/

static struct  devGpibParmBlock devSupParms;
static long init_ai(int parm)
{
	if (parm==0)  {
		devSupParms.debugFlag = &AX301Debug;
		devSupParms.respond2Writes = -1;
		devSupParms.timeWindow = TIME_WINDOW;
		devSupParms.hwpvtHead = 0;
		devSupParms.gpibCmds = gpibCmds;
		devSupParms.numparams = NUMPARAMS;
		devSupParms.magicSrq = 0;
		devSupParms.name = "devXxAX301";
		devSupParms.timeout = DMA_TIME;
		devSupParms.srqHandler = devGpibLib_srqHandler;
		devSupParms.wrConversion = 0;
	}
	return(devGpibLib_initDevSup(parm, &DSET_AI));
}
