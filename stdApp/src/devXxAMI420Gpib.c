/* devXxAMI420Gpib.c */
/*
 *      Author: Tim Mooney
 *      Date:   09-18-03
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 2003, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *
 * Modification Log:
 * -----------------
 * .01  09-18-03 tmm Initial version, based on devXxKepco4886Gpib.c
 */

/******************************************************************************
 *
 * The following define statements are used to declare the names to be used
 * for the dset tables.   
 *
 * NOTE: The dsets are referenced by the entries in the command table.
 *
 ******************************************************************************/
#define	DSET_AI		devAiAMI420Gpib
#define	DSET_AO		devAoAMI420Gpib
#define	DSET_LI		devLiAMI420Gpib
#define	DSET_LO		devLoAMI420Gpib
#define	DSET_BI		devBiAMI420Gpib
#define	DSET_BO		devBoAMI420Gpib
#define	DSET_MBBO	devMbboAMI420Gpib
#define	DSET_MBBI	devMbbiAMI420Gpib
#define	DSET_SI		devSiAMI420Gpib
#define	DSET_SO		devSoAMI420Gpib

#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<taskLib.h>
#include	<rngLib.h>

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

#define STATIC static

STATIC long	init_dev_sup(), report();
static	struct  devGpibParmBlock devSupParms;

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
	devGpibLib_readAi, NULL, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_aiGpibWork, (DRVSUPFUN)devGpibLib_aiGpibSrq}};

gDset DSET_AO   = {6, {NULL, NULL, devGpibLib_initAo, NULL, 
	devGpibLib_writeAo, NULL, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_aoGpibWork, NULL}};

gDset DSET_BI   = {5, {NULL, NULL, devGpibLib_initBi, NULL, 
	devGpibLib_readBi, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_biGpibWork, (DRVSUPFUN)devGpibLib_biGpibSrq}};

gDset DSET_BO   = {5, {NULL, NULL, devGpibLib_initBo, NULL, 
	devGpibLib_writeBo, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_boGpibWork, NULL}};

gDset DSET_MBBI = {5, {NULL, NULL, devGpibLib_initMbbi, NULL, 
	devGpibLib_readMbbi, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_mbbiGpibWork, (DRVSUPFUN)devGpibLib_mbbiGpibSrq}};

gDset DSET_MBBO = {5, {NULL, NULL, devGpibLib_initMbbo, NULL, 
	devGpibLib_writeMbbo, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)devGpibLib_mbboGpibWork, NULL}};

gDset DSET_SI   = {5, {NULL, NULL, devGpibLib_initSi, NULL, 
	devGpibLib_readSi, (DRVSUPFUN)&devSupParms,
	(DRVSUPFUN)&devGpibLib_stringinGpibWork, (DRVSUPFUN)devGpibLib_stringinGpibSrq}};

gDset DSET_SO   = {5, {NULL, NULL, devGpibLib_initSo, NULL, 
	devGpibLib_writeSo, (DRVSUPFUN)&devSupParms, 
	(DRVSUPFUN)devGpibLib_stringoutGpibWork, NULL}};

gDset DSET_LI   = {5, {NULL, NULL, devGpibLib_initLi, NULL, 
	devGpibLib_readLi, (DRVSUPFUN)&devSupParms, 
	(DRVSUPFUN)devGpibLib_liGpibWork, (DRVSUPFUN)devGpibLib_liGpibSrq}};

gDset DSET_LO   = {5, {NULL, NULL, devGpibLib_initLo, NULL, 
	devGpibLib_writeLo, (DRVSUPFUN)&devSupParms, 
	(DRVSUPFUN)devGpibLib_loGpibWork, NULL}};

/******************************************************************************
 *
 * Debugging flags that can be accessed from the shell.
 *
 ******************************************************************************/
int devXxAMI420GpibDebug = 0;
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
#define	DMA_TIME	60		/* 1 second */

/******************************************************************************
 *
 * Strings used by the init routines to fill in the znam, onam, ...
 * fields in BI and BO record types.
 *
 *	struct devGpibNames {
 *		int           count;
 *		char          **item;
 *		unsigned long *value;
 *		short         nobt;
 *	};
 *
 * Example of use:
 * static char *offOnList[] = { "Off", "On" };
 * static struct devGpibNames offOn = { 2, offOnList, NULL, 1 };
 ******************************************************************************/


/******************************************************************************
 *
 * Structures used by the init routines to fill in the onst, twst,... and the
 * onvl, twvl,... fields in MBBI and MBBO record types.
 *
 * Note that the intExtSsBm and intExtSsBmStop structures use the same
 * intExtSsBmStopList and zeroToN lists but have a different number
 * of elements in them that they use... The intExtSsBm structure only represents
 * 4 elements, while the intExtSsBmStop structure represents 5.
 *
 * Example of use:
 * static char *intExtSsBmStopList[] = { "INTERNAL", "EXTERNAL",
 *	"SINGLE SHOT", "BURST MODE", "STOP" };
 *
 * static unsigned long zeroToN[] = { 0, 1, 2, 3, 4 };
 *
 * static struct devGpibNames intExtSsBm = { 4, intExtSsBmStopList, zeroToN, 2 };
 *
 * static struct devGpibNames intExtSsBmStop = { 5, intExtSsBmStopList, zeroToN, 3 };
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
 * Example of use:
 * static char	*(localRemote_write[]) = {"SYST:LOC\n", "SYST:REM\n", NULL};
 ******************************************************************************/
static char	*(localRemote_write[]) = {"SYST:LOC\n", "SYST:REM\n", NULL};


/******************************************************************************
 *
 * Array of structures that define all GPIB messages
 * supported for this type of instrument.
 *
 *	struct gpibCmd {
 *		gDset *rec_typ;		-- used to indicate record type supported
 *		int   type;			-- enum - GPIBREAD, GPIBWRITE, GPIBCMD...
 *		short pri;			-- request priority--IB_Q_HIGH or IB_Q_LOW
 *		char  *cmd;			-- CONSTANT STRING to send to instrument
 *		char  *format;		-- string used to generate or interpret msg
 *		long  rspLen;		-- room for response error message
 *		long  msgLen;		-- room for return data message length
 *	
 *		int   (*convert)();	-- custom routine for conversions
 *		int   P1;			-- user defined parameter used in convert()
 *		int   P2;			-- user defined parameter used in convert()
 *		char  **P3;			-- user defined parameter used in convert()
 *	
 *		struct devGpibNames *namelist;  -- pointer to name strings
 *		int	companion;		-- companion command (used at init time)
 *	};
 *	
 ******************************************************************************/

static struct gpibCmd gpibCmds[] = 
{
    /* Param 0:  read magnet voltage */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "VOLT:MAG?", "%lf", 80, 80,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 1: read magnet current */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "CURR:MAG?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 2: read magnet field */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "FIELD:MAG?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 3: write local/remote */
  {&DSET_BO, GPIBEFASTO, IB_Q_HIGH, NULL, NULL, 40, 40,
  NULL, 0, 0, localRemote_write, NULL, -1},

    /* Param 4:  read coil constant */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "COIL?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 5: write coil constant */
  {&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:COIL %.3lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 6: read field units */
  {&DSET_BI, GPIBREAD, IB_Q_HIGH, "FIELD:UNITS?", "%1d", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 7: write field units */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:FIELD:UNITS %1d", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 8: read ramp-rate units */
  {&DSET_BI, GPIBREAD, IB_Q_HIGH, "RAMP:RATE:UNITS?", "%1d", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 9: write ramp-rate units */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:RAMP:RATE:UNITS %1d", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 10: read program field */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "FIELD:PROG?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 11: write program field */
  {&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:FIELD:PROG %.6lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 12: read ramp rate */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "RAMP:RATE:FIELD?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 13: write ramp rate */
  {&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:RAMP:RATE:FIELD %.3lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 14: read voltage limit */
  {&DSET_AI, GPIBREAD, IB_Q_HIGH, "VOLT:LIM?", "%lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 15: write voltage limit */
  {&DSET_AO, GPIBWRITE, IB_Q_HIGH, NULL, "CONF:VOLT:LIM %.3lf", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 16: set to ramp mode */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "RAMP", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 17: set to zero mode */
  {&DSET_BO, GPIBWRITE, IB_Q_HIGH, NULL, "ZERO", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

    /* Param 18: send user command ("; *OPC?" guarantees a reply)*/
  {&DSET_SO, GPIBWRITE, IB_Q_HIGH, NULL, "%s; *OPC?", 80, 80,
  NULL, 0, 0, NULL, NULL, -1},

   /* Param 19: get user-command reply */
  {&DSET_SI, GPIBREAD, IB_Q_LOW, NULL, "%s", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

   /* Param 20: read state number */
  {&DSET_MBBI, GPIBREAD, IB_Q_LOW, "STATE?", "%1d", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},

   /* Param 21: read error buffer */
  {&DSET_SI, GPIBREAD, IB_Q_LOW, "SYST:ERR", "%s", 40, 40,
  NULL, 0, 0, NULL, NULL, -1},
};

/* The following is the number of elements in the command array above.  */
#define NUMPARAMS	sizeof(gpibCmds)/sizeof(struct gpibCmd)

/******************************************************************************
 *
 * Initialization for device support
 * This is called one time before any records are initialized with a parm
 * value of 0.  And then again AFTER all record-level init is complete
 * with a param value of 1.
 *
 *****************************************************************************/
/******************************************************************************
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
 *****************************************************************************/
STATIC long 
init_dev_sup(int parm)
{
  if(parm==0)  {
    devSupParms.debugFlag = &devXxAMI420GpibDebug;
    devSupParms.respond2Writes = -1;
    devSupParms.timeWindow = TIME_WINDOW;
    devSupParms.hwpvtHead = 0;
    devSupParms.gpibCmds = gpibCmds;
    devSupParms.numparams = NUMPARAMS;
    devSupParms.magicSrq = -1;
    devSupParms.name = "devXxAMI420Gpib";
    devSupParms.dmaTimeout = DMA_TIME;
    devSupParms.srqHandler = 0;
    devSupParms.wrConversion = 0;
  }
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
STATIC long
report(void)
{
  return(devGpibLib_report(&DSET_AI));
}
