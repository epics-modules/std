/*
/*
 *      Author: 
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
 * .02  01-19-94	sjs	Added support for GP307 vac gauge
 */

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * NOTE: The dsets are referenced by the entries in the command table.
 *
 ******************************************************************************/
#define DSET_AI         devAiGP307Gpib
#define DSET_AO         devAoGP307Gpib
#define DSET_LI         devLiGP307Gpib
#define DSET_LO         devLoGP307Gpib
#define DSET_BI         devBiGP307Gpib
#define DSET_BO         devBoGP307Gpib
#define DSET_MBBO       devMbboGP307Gpib
#define DSET_MBBI       devMbbiGP307Gpib
#define DSET_SI         devSiGP307Gpib
#define DSET_SO         devSoGP307Gpib

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

static long	init_dev_sup(), report();
static int	srqHandler();
extern struct devGpibParmBlock devGP307gpib_Parms;

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
gDset DSET_AI   = {6, {report, init_dev_sup, devGpibLib_initAi, NULL, 
	devGpibLib_readAi, NULL, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_aiGpibWork, (DRVSUPFUN)devGpibLib_aiGpibSrq}};

gDset DSET_AO   = {6, {NULL, NULL, devGpibLib_initAo, NULL, 
	devGpibLib_writeAo, NULL, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_aoGpibWork, NULL}};

gDset DSET_BI   = {5, {NULL, NULL, devGpibLib_initBi, NULL, 
	devGpibLib_readBi, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_biGpibWork, (DRVSUPFUN)devGpibLib_biGpibSrq}};

gDset DSET_BO   = {5, {NULL, NULL, devGpibLib_initBo, NULL, 
	devGpibLib_writeBo, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_boGpibWork, NULL}};

gDset DSET_MBBI = {5, {NULL, NULL, devGpibLib_initMbbi, NULL, 
	devGpibLib_readMbbi, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_mbbiGpibWork, (DRVSUPFUN)devGpibLib_mbbiGpibSrq}};

gDset DSET_MBBO = {5, {NULL, NULL, devGpibLib_initMbbo, NULL, 
	devGpibLib_writeMbbo, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)devGpibLib_mbboGpibWork, NULL}};

gDset DSET_SI   = {5, {NULL, NULL, devGpibLib_initSi, NULL, 
	devGpibLib_readSi, (DRVSUPFUN)&devGP307gpib_Parms,
	(DRVSUPFUN)&devGpibLib_stringinGpibWork, (DRVSUPFUN)devGpibLib_stringinGpibSrq}};

gDset DSET_SO   = {5, {NULL, NULL, devGpibLib_initSo, NULL, 
	devGpibLib_writeSo, (DRVSUPFUN)&devGP307gpib_Parms, 
	(DRVSUPFUN)devGpibLib_stringoutGpibWork, NULL}};

gDset DSET_LI   = {5, {NULL, NULL, devGpibLib_initLi, NULL, 
	devGpibLib_readLi, (DRVSUPFUN)&devGP307gpib_Parms, 
	(DRVSUPFUN)devGpibLib_liGpibWork, (DRVSUPFUN)devGpibLib_liGpibSrq}};

gDset DSET_LO   = {5, {NULL, NULL, devGpibLib_initLo, NULL, 
	devGpibLib_writeLo, (DRVSUPFUN)&devGP307gpib_Parms, 
	(DRVSUPFUN)devGpibLib_loGpibWork, NULL}};

/******************************************************************************
 *
 * Debugging flags that can be accessed from the shell.
 *
 ******************************************************************************/
int GP307Debug = 0;
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
 * Strings used by the init routines to fill in the znam, onam, ...
 * fields in BI and BO record types.
 *
 ******************************************************************************/

static  char            *offOnList[] = { "Off", "On" };
static  struct  devGpibNames   offOn = { 2, offOnList, NULL, 1 };

static  char            *initNamesList[] = { "Init", "Init" };
static  struct  devGpibNames   initNames = { 2, initNamesList, NULL, 1 };

static  char    *disableEnableList[] = { "Disable", "Enable" };
static  struct  devGpibNames   disableEnable = { 2, disableEnableList, NULL, 1 };

static  char    *resetList[] = { "Reset", "Reset" };
static  struct  devGpibNames   reset = { 2, resetList, NULL, 1 };

static  char    *lozHizList[] = { "50 OHM", "IB_Q_HIGH Z" };
static  struct  devGpibNames   lozHiz = {2, lozHizList, NULL, 1};

static  char    *invertNormList[] = { "INVERT", "NORM" };
static  struct  devGpibNames   invertNorm = { 2, invertNormList, NULL, 1 };

static  char    *fallingRisingList[] = { "FALLING", "RISING" };
static  struct  devGpibNames   fallingRising = { 2, fallingRisingList, NULL, 1 };

static  char    *clearList[] = { "CLEAR", "CLEAR" };
static  struct  devGpibNames   clear = { 2, clearList, NULL, 1 };

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

static  char            *intExtSsBmStopList[] = { "INTERNAL", "EXTERNAL",
				"SINGLE SHOT", "BURST MODE", "STOP" };

static  unsigned long   intExtSsBmStopVal[] = { 0, 1, 2, 3, 2 };

static  struct  devGpibNames    intExtSsBm = { 4, intExtSsBmStopList, 
				intExtSsBmStopVal, 2 };

static  struct  devGpibNames    intExtSsBmStop = { 5, intExtSsBmStopList,
                                        intExtSsBmStopVal, 3 };

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
 * device common stuff               
 ******************************************************************************/
#define UDF "Undefined"
static int getRespStr();
static int putRespStr();

/* Binary OUT strings */

static char	*dgOnStrs[] = {NULL,"DG ON"};

static char	*dgOffStrs[]= {NULL,"DG OFF"};

static char 	*ig1OnStrs[]= {NULL,"IG1 ON"};

static char	*ig1OffStrs[]={NULL,"IG1 OFF"};

static char 	*ig2OnStrs[]= {NULL,"IG2 ON"};

static char	*ig2OffStrs[]={NULL,"IG2 OFF"};

/* Binary IN strings */

static char	*zeroOneStrs[]={"0","1",NULL};


/******************************************************************************
 *
 * Array of structures that define all GPIB messages
 * supported for this type of instrument.
 *
 ******************************************************************************/

static struct gpibCmd gpibCmds[] = 
{
    /* Param 0 : Degas ON*/
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL,20, 0, NULL,
   0, 0, dgOnStrs, NULL, -1},

    /* Param 1 : Degas OFF*/
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL, 20, 0, NULL,
   0, 0, dgOffStrs, NULL, -1},

    /* Param 2 : Degas status, 1=on 0=off*/
  {&DSET_BI, GPIBEFASTI, IB_Q_LOW, "DGS", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 3 : Ion Guage 1 pressure */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "DS IG1", "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 4 : Ion Guage 2 pressure */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "DS IG2", "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 5 : Active Ion Guage pressure */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "DS IG", "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 6 : Convectron Guage 1 pressure */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "DS CG1", "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 7 : Convectron Guage 2 pressure */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "DS CG2", "%lf", 0, 32,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 8 : Ion Guage 1 ON */
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL, 20, 0, NULL,
   0, 0, ig1OnStrs, NULL, -1},

    /* Param 9 : Ion Guage 1 OFF */
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL, 20, 0, NULL,
   0, 0, ig1OffStrs, NULL, -1},

    /* Param 10 : Ion Guage 2 ON */
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL, 20, 0, NULL,
   0, 0, ig2OnStrs, NULL, -1},

    /* Param 11 : Ion Guage 2 OFF */
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH,NULL,NULL, 20, 0, NULL,
   0, 0, ig2OffStrs, NULL, -1},

    /* Param 12 : PCS Relay 1 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 1", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 13 : PCS Relay 2 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 2", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 14 : PCS Relay 3 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 3", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 15 : PCS Relay 4 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 4", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 16 : PCS Relay 5 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 5", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},

    /* Param 17 : PCS Relay 6 status */
  {&DSET_BI, GPIBEFASTI, IB_Q_HIGH, "PCS 6", NULL, 0, 20,
  NULL, 0, 0, zeroOneStrs, NULL, -1},
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
static struct  devGpibParmBlock devGP307gpib_Parms = {
  &GP307Debug,         /* debugging flag pointer */
  1,                   /* device does not respond to writes */
  TIME_WINDOW,          /* # of clock ticks to skip after a device times out */
  NULL,                 /* hwpvt list head */
  gpibCmds,             /* GPIB command array */
  NUMPARAMS,            /* number of supported parameters */
  -1,			/* magic SRQ param number (-1 if none) */
  "devXxGP307Gpib",	/* device support module type name */
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
static long 
init_dev_sup(parm)
int	parm;
{
  return(devGpibLib_initDevSup(parm,&DSET_AI));
}

/******************************************************************************
 *
 * Print a report of operating statistics for all devices supported by this
 * module.
 *
 * This function will no longer be required after epics 3.3 is released
 *
 ******************************************************************************/
static long
report()
{
  return(devGpibLib_report(&DSET_AI));
}
/**********************************************
 * Secondary convert routine
 **********************************************/
 static int putRespStr(status,pdpvt)
	struct gpibDpvt *pdpvt;
	int status;

	{
		if(status != ERROR)
		{
			pdpvt->phwpvt->pupvt = pdpvt->rsp;
		}
		else
		{
			pdpvt->phwpvt->pupvt = UDF;
	    	}
		return(status);
	}

/**********************************************
 * Primary convert routine
 **********************************************/
 static int getRespStr(pdpvt,P1,P2,P3)
	struct gpibDpvt *pdpvt;
	int P1;
	int P2;
	char **P3;

	{
		struct stringinRecord *pstringin= (struct stringinRecord *)
		(pdpvt->precord);
		struct gpibCmd *pCmd = &gpibCmds[pdpvt->parm];

		pstringin->udf = FALSE;
		strncpy(pstringin->val,pdpvt->phwpvt->pupvt,39);
		pstringin->val[40]='\0';
		return(2);
	}
