/* devXxKeithleyDMM199Gpib.c */
/* share/src/devOpt $Id: devXxKeithleyDMM199Gpib.c,v 1.2 2003-12-10 21:41:18 mooney Exp $ */
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
 * .01  02-18-92  jrw  Initial Release
 *      12-04-03  tmm  removed unused srq-handler function
 *
 */

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * NOTE: The dsets are referenced by the entries in the command table.
 *
 ******************************************************************************/
#define	DSET_AI		devAiKeithleyDMM199
#define	DSET_AO		devAoKeithleyDMM199
#define	DSET_LI		devLiKeithleyDMM199
#define	DSET_LO		devLoKeithleyDMM199
#define	DSET_BI		devBiKeithleyDMM199
#define	DSET_BO		devBoKeithleyDMM199
#define	DSET_MBBO	devMbboKeithleyDMM199
#define	DSET_MBBI	devMbbiKeithleyDMM199
#define	DSET_SI		devSiKeithleyDMM199
#define	DSET_SO		devSoKeithleyDMM199

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

#define STATIC static

static struct devGpibParmBlock devXxKeithleyDMM199Gpib_Parms;

/******************************************************************************
 *
 * Debugging flags that can be accessed from the shell.
 *
 ******************************************************************************/
int KeithleyDMM199Debug = 0;
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
#define TIME_WINDOW	60		/* 1 seconds */
#define	DMA_TIME	120		/* 2 second */

/******************************************************************************
 *
 * Strings used by the init routines to fill in the znam, onam, ...
 * fields in BI and BO record types.
 *
 ******************************************************************************/


/******************************************************************************
 *
 * Structures used by the init routines to fill in the onst, twst,... and the
 * onvl, twvl,... fields in MBBI and MBBO record types.
 *
 * Note that the intExtSsBm and intExtSsBmStop structures use the same
 * intExtSsBmStopList and intExtSsBmStopVal lists but have a different number
 * of elements in them that they use... The intExtSsBm structure only represents
 * 4 elements, while the intExtSsBmStop structure represents 5.
 *
 ******************************************************************************/


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
 ******************************************************************************/


/******************************************************************************
 *
 * Array of structures that define all GPIB messages
 * supported for this type of instrument.
 *
 ******************************************************************************/

static struct gpibCmd gpibCmds[] = 
{
    /* Param 0 -- initialize: trigger on TALK; -1.234567E+0 format */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "F2R2Z0P1S1T1B0G1M0K0Y3A1O1W100X", 0, 50,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 1 -- get error string  */
  {&DSET_SI, GPIBREAD, IB_Q_HIGH, "U1X", "%s", 0, 50,
  NULL, 0, 0, NULL, NULL, -1},

   /* Param 2 -- read conversion (assumed data format "-1.234567E+0" */
  {&DSET_AI, GPIBREAD, IB_Q_LOW, NULL, "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

   /* Param 3 -- send command in stringout's VAL field */
  {&DSET_SO, GPIBWRITE, IB_Q_LOW, NULL, "%s", 0, 50,
  NULL, 0, 0, NULL, NULL, -1}
};

/* The following is the number of elements in the command array above.  */
#define NUMPARAMS	sizeof(gpibCmds)/sizeof(struct gpibCmd)

/******************************************************************************
 *
 * Structure containing the user's functions and operating parameters needed
 * by the gpib library functions.
 *
 * The magic SRQ parm is the parm number that, if specified on a passive
 * record, will cause the record to be processed automatically when an
 * unsolicited SRQ interrupt is detected from the device.
 *
 * If the parm is specified on a non-passive record, it will NOT be processed
 * when an unsolicited SRQ is detected.
 *
 * In the future, the magic SRQ parm records will be processed as "I/O event
 * scanned"... not passive.
 *
 ******************************************************************************/
static struct  devGpibParmBlock devXxKeithleyDMM199Gpib_Parms = {
  &KeithleyDMM199Debug,         /* debugging flag pointer */
  -1,                   /* device does not respond to writes */
  TIME_WINDOW,          /* # of clock ticks to skip after a device times out */
  NULL,                 /* hwpvt list head */
  gpibCmds,             /* GPIB command array */
  NUMPARAMS,            /* number of supported parameters */
  -1,			/* magic SRQ param number (-1 if none) */
  "devXxKeithleyDMM199Gpib",	/* device support module type name */
  DMA_TIME,		/* # of clock ticks to wait for DMA completions */
  NULL,			/* SRQ handler function (NULL if none) */
  NULL			/* secondary conversion routine (NULL if none) */
};

/******************************************************************************
 *
 * Initialization for device support
 * This is called one time before any records are initialized with a parm
 * value of 0.  And then again AFTER all record-level init is complete
 * with a param value of 1.
 *
 ******************************************************************************/
STATIC long 
init_ai(int parm)
{
  return(devGpibLib_initDevSup(parm,&DSET_AI));
}
