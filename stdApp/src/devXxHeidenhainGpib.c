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

#include	<vxWorks.h>
#include	<taskLib.h>
#include	<rngLib.h>
#include	<types.h>
#include	<stdioLib.h>

#include	<alarm.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<devSup.h>
#include	<recSup.h>
#include	<drvSup.h>
#include	<link.h>
#include	<module_types.h>
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


#ifndef VALID_ALARM
#define VALID_ALARM INVALID_ALARM
#endif
static long	init_dev_sup();
int	srqHandler();
int	aiGpibSrq(), liGpibSrq(), biGpibSrq(), mbbiGpibSrq(), stringinGpibSrq();
extern	struct  devGpibParmBlock devXxHeidenhainGpib_Parms;


/******************************************************************************
 *
 * Define all the dset's.
 *
 * Note that the dset names are provided via the #define lines at the top of
 * this file.
 *
 * Other than for the debugging flag(s), these DSETs are the only items that
 * will appear in the global name space within the IOC.
 *
 * The last 3 items in the DSET structure are used to point to the parm 
 * structure, the  work functions used for each record type, and the srq 
 * handler for each record type.
 *
 ******************************************************************************/

gDset DSET_AI   = {6, {NULL, init_dev_sup, devGpibLib_initAi, NULL,
        devGpibLib_readAi, NULL, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_aiGpibWork, (DRVSUPFUN)devGpibLib_aiGpibSrq}};

gDset DSET_AO   = {6, {NULL, NULL, devGpibLib_initAo, NULL,
        devGpibLib_writeAo, NULL, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_aoGpibWork, NULL}};

gDset DSET_BI   = {5, {NULL, NULL, devGpibLib_initBi, NULL,
        devGpibLib_readBi, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_biGpibWork,(DRVSUPFUN)devGpibLib_biGpibSrq}};

gDset DSET_BO   = {5, {NULL, NULL, devGpibLib_initBo, NULL,
        devGpibLib_writeBo, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_boGpibWork, NULL}};

gDset DSET_MBBI = {5, {NULL, NULL, devGpibLib_initMbbi, NULL,
        devGpibLib_readMbbi, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_mbbiGpibWork,(DRVSUPFUN)devGpibLib_mbbiGpibSrq}};

gDset DSET_MBBO = {5, {NULL, NULL, devGpibLib_initMbbo, NULL,
        devGpibLib_writeMbbo, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_mbboGpibWork, NULL}};

gDset DSET_SI   = {5, {NULL, NULL, devGpibLib_initSi, NULL,
        devGpibLib_readSi, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)&devGpibLib_stringinGpibWork,(DRVSUPFUN)devGpibLib_stringinGpibSrq}};

gDset DSET_SO   = {5, {NULL, NULL, devGpibLib_initSo, NULL,
        devGpibLib_writeSo, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_stringoutGpibWork, NULL}};

gDset DSET_LI   = {5, {NULL, NULL, devGpibLib_initLi, NULL,
        devGpibLib_readLi, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_liGpibWork, (DRVSUPFUN)devGpibLib_liGpibSrq}};

gDset DSET_LO   = {5, {NULL, NULL, devGpibLib_initLo, NULL,
        devGpibLib_writeLo, (DRVSUPFUN)&devXxHeidenhainGpib_Parms,
        (DRVSUPFUN)devGpibLib_loGpibWork, NULL}};


/******************************************************************************
 *
 * Debugging flags that can be accessed from the shell.
 *
 ******************************************************************************/
int SkeletonDebug = 0;
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

static char	*(userOffOn[]) = {"USER OFF;", "USER ON;", NULL};

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
static struct  devGpibParmBlock devXxHeidenhainGpib_Parms = {
  &SkeletonDebug,         /* debugging flag pointer */
  -1,                   /* device does not respond to writes */
  TIME_WINDOW,          /* # of clock ticks to skip after a device times out */
  NULL,                 /* hwpvt list head */
  gpibCmds,             /* GPIB command array */
  NUMPARAMS,            /* number of supported parameters */
  -1,			/* magic SRQ param number (-1 if none) */
  "devXxHeidenhainGpib",/* device support module type name */
  DMA_TIME,		/* # of clock ticks to wait for DMA completions */
  NULL,			/* SRQ handler function (NULL if none) */
  NULL			/* secondary conversion routine (NULL if none) */
};
/****************************************************************************
 *
 * 
 * Custom convert routines for Heidenhain AWE1024 encoder 
 *                      7-20-94  ddr 
 *
 *
 ***************************************************************************/
static int convert(pdpvt, p1, p2, p3)
struct gpibDpvt *pdpvt;
int	p1;
int	p2;
char	**p3;
{
struct aiRecord *pairec = (struct aiRecord *) (pdpvt->precord);
unsigned char*msg=(unsigned char *)(pdpvt->msg);
pairec->val = (((msg[3]*256+msg[2])*256+msg[1])*256+msg[0])/102400.0;
return(OK);
}
/******************************************************************************
 *
 * Initialization for device support
 * This is called one time before any records are initialized with a parm
 * value of 0.  And then again AFTER all record-level init is complete
 * with a param value of 1.
 *
 ******************************************************************************/
static long 
init_dev_sup(parm)
int	parm;
{
  return(devGpibLib_initDevSup(parm,&DSET_AI));
}


