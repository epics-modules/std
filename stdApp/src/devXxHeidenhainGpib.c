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
 */

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * NOTE: The dsets are referenced by the entries in the command table.
 *
 ******************************************************************************/
#define	DSET_AI		devAiHeidAWE1024
#define	DSET_AO		devAoHeidAWE1024
#define	DSET_LI		devLiHeidAWE1024
#define	DSET_LO		devLoHeidAWE1024
#define	DSET_BI		devBiHeidAWE1024
#define	DSET_BO		devBoHeidAWE1024
#define	DSET_MBBO	devMbboHeidAWE1024
#define	DSET_MBBI	devMbbiHeidAWE1024
#define	DSET_SI		devSiHeidAWE1024
#define	DSET_SO		devSoHeidAWE1024

#include	<stdlib.h>
#include	<string.h>

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
int devXxHeidenhainDebug = 0;
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
 * EXAMPLE:
 * static char	*(userOffOn[]) = {"USER OFF;", "USER ON;", NULL};
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
static int convert();

static struct gpibCmd gpibCmds[] = 
{
    /* Initialization sequence */
    /* Param 0 */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "R1F2T2X", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Read the encoder status */
    /* Param 1 */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "A0X","%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Read the encoder status */
    /* Param 2 */
  {&DSET_AI, GPIBRAWREAD, IB_Q_HIGH, NULL,"%lf", 0, 32,
  convert, 0, 0, NULL, NULL, -1},
};

/* The following is the number of elements in the command array above.  */
#define NUMPARAMS	sizeof(gpibCmds)/sizeof(struct gpibCmd)

/****************************************************************************
 *
 * 
 * Custom convert routines for Heidenhain AWE1024 encoder 
 *                      7-20-94  ddr 
 *
 *
 ***************************************************************************/
static int convert(struct gpibDpvt *pdpvt, int	p1, int	p2, char **p3)
{
	struct aiRecord *pairec = (struct aiRecord *) (pdpvt->precord);
	unsigned char*msg=(unsigned char *)(pdpvt->msg);
	pairec->val = (((msg[3]*256+msg[2])*256+msg[1])*256+msg[0])/102400.0;
	return(0);
}
/******************************************************************************
 *
 * Initialization for device support
 * This is called one time before any records are initialized with a parm
 * value of 0.  And then again AFTER all record-level init is complete
 * with a param value of 1.
 *
 ******************************************************************************/
static struct  devGpibParmBlock devSupParms;
static long init_ai(int parm)
{
	if (parm==0)  {
		devSupParms.debugFlag = &devXxHeidenhainDebug;
		devSupParms.respond2Writes = -1;
		devSupParms.timeWindow = TIME_WINDOW;
		devSupParms.hwpvtHead = 0;
		devSupParms.gpibCmds = gpibCmds;
		devSupParms.numparams = NUMPARAMS;
		devSupParms.magicSrq = 0;
		devSupParms.name = "devXxHeidenhainGpib";
		devSupParms.timeout = DMA_TIME;
		devSupParms.srqHandler = devGpibLib_srqHandler;
		devSupParms.wrConversion = 0;
	}
 	return(devGpibLib_initDevSup(parm, &DSET_AI));
}
