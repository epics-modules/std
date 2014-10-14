/* File:    drvScalerSoft.c
 * Author:  Mark Rivers
 * Date:    22-May-2007
 *
 * Purpose: 
 * This module provides the driver support for scaler using database links as the inputs.
 */

/*******************/
/* System includes */
/*******************/

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/******************/
/* EPICS includes */
/******************/

/* EPICS includes */
#include <dbAccess.h>
#include <asynDriver.h>
#include <asynInt32.h>
#include <asynInt32Array.h>
#include <asynDrvUser.h>
#include <cantProceed.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <errlog.h>
#include <iocsh.h>

/*******************/
/* Custom includes */
/*******************/

#include <devScalerAsyn.h>
#include <drvScalerSoft.h>

/***************/
/* Definitions */
/***************/
#define MAX_PVNAME_SIZE 40

/**************/
/* Structures */
/**************/
typedef enum {
    scalerResetCommand,
    scalerChannelsCommand,
    scalerReadCommand,
    scalerPresetCommand,
    scalerArmCommand,
    scalerDoneCommand,
} scalerCommand;

typedef enum{
    stateUnconnected,
    stateConnected,
    stateFailed,
} connectState;

typedef struct {
    scalerCommand command;
    char *commandString;
} scalerCommandStruct;

static scalerCommandStruct scalerCommands[MAX_SCALER_COMMANDS] = {
    {scalerResetCommand,           SCALER_RESET_COMMAND_STRING},    /* int32, write */
    {scalerChannelsCommand,        SCALER_CHANNELS_COMMAND_STRING}, /* int32, read */
    {scalerReadCommand,            SCALER_READ_COMMAND_STRING},     /* int32Array, read */
    {scalerPresetCommand,          SCALER_PRESET_COMMAND_STRING},   /* int32, write */
    {scalerArmCommand,             SCALER_ARM_COMMAND_STRING},      /* int32, write */
    {scalerDoneCommand,            SCALER_DONE_COMMAND_STRING},     /* int32, write/read */
};

typedef struct scalerPvt {
    char *portName;
    char *pvTemplate;
    char *driver;
    int maxChans;
    int acquiring;
    int prevAcquiring;
    long *counts;         /* maxChans */
    long *presetCounts;   /* maxChans */
    epicsMutexId lock;
    asynUser *pasynUser;
    asynInterface common;
    asynInterface int32;
    asynInterface drvUser;
    void *int32InterruptPvt;
    asynInterface int32Array;
    void *int32ArrayInterruptPvt;
    epicsEventId doneEventId;
    epicsThreadId callbackThreadId;
    connectState connectState;
    DBADDR *inputAddr;   /* maxChans */
} scalerPvt;

/*******************************/
/* Global variable declaration */
/*******************************/

/**************/
/* Prototypes */
/**************/

/* These functions are in public interfaces */
static asynStatus int32Write        (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 value);
static asynStatus int32Read         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *value);
static asynStatus getBounds         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *low, epicsInt32 *high);
static asynStatus int32ArrayRead    (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *data, size_t maxChans,
                                     size_t *nactual);
static asynStatus int32ArrayWrite   (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *data, size_t maxChans);
static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);

static void scalerReport            (void *drvPvt, FILE *fp, int details);
static asynStatus scalerConnect     (void *drvPvt, asynUser *pasynUser);
static asynStatus scalerDisconnect  (void *drvPvt, asynUser *pasynUser);

/* Functions not in interfaces */
static int connectPVs(scalerPvt *pPvt);
static void checkAcquireDone(scalerPvt *pPvt);
static void acquireDoneCallbacks(scalerPvt *pPvt);

/*
 * asynCommon methods
 */
static struct asynCommon scalerCommon = {
    scalerReport,
    scalerConnect,
    scalerDisconnect
};

/* asynInt32 methods */
static asynInt32 scalerInt32 = {
    int32Write,
    int32Read,
    getBounds,
    NULL,
    NULL
};

/* asynInt32Array methods */
static asynInt32Array scalerInt32Array = {
    int32ArrayWrite,
    int32ArrayRead,
    NULL,
    NULL
};

static asynDrvUser scalerDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};


/********/
/* Code */
/********/

/* iocsh commands */

/* This command needs to be called once for each Pilatus */
int drvScalerSoftConfigure(char  *portName,
                           int   maxChans,
                           char  *pvTemplate)
{
    scalerPvt *pPvt;
    int status;

    /* Allocate and initialize scalerPvt*/

    pPvt = callocMustSucceed(1, sizeof(scalerPvt), "drvScalerSoftConfigure");
    pPvt->portName = epicsStrDup(portName);
    pPvt->driver = "drvScalerSoft";
    pPvt->maxChans = maxChans;
    pPvt->pvTemplate = epicsStrDup(pvTemplate);

    /* Allocate memory buffers */
    pPvt->counts       = (long *)callocMustSucceed(maxChans, sizeof(long), "drvScalerSoftConfigure");
    pPvt->presetCounts = (long *)callocMustSucceed(maxChans, sizeof(long), "drvScalerSoftConfigure");
    pPvt->inputAddr  = (DBADDR *)callocMustSucceed(maxChans, sizeof(DBADDR), "drvScalerSoftConfigure");

    /* Create asynUser for debugging */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);

    pPvt->acquiring = 0;
    pPvt->prevAcquiring = 0;
    pPvt->connectState = stateUnconnected;

   /* Create the EPICS event used to wake up the callback task */
    pPvt->doneEventId = epicsEventCreate(epicsEventEmpty);

    /* Create the thread that will do the actual I/O */ 
    pPvt->callbackThreadId = epicsThreadCreate("scalerSimCallbacks",
           epicsThreadPriorityMedium,
           epicsThreadGetStackSize(epicsThreadStackMedium),
           (EPICSTHREADFUNC)acquireDoneCallbacks, 
           pPvt);

    /* Link with higher level routines */
    pPvt->common.interfaceType = asynCommonType;
    pPvt->common.pinterface  = (void *)&scalerCommon;
    pPvt->common.drvPvt = pPvt;
    pPvt->int32.interfaceType = asynInt32Type;
    pPvt->int32.pinterface  = (void *)&scalerInt32;
    pPvt->int32.drvPvt = pPvt;
    pPvt->int32Array.interfaceType = asynInt32ArrayType;
    pPvt->int32Array.pinterface  = (void *)&scalerInt32Array;
    pPvt->int32Array.drvPvt = pPvt;
    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface = (void *)&scalerDrvUser;
    pPvt->drvUser.drvPvt = pPvt;

    status = pasynManager->registerPort(pPvt->portName,
        ASYN_MULTIDEVICE, 
        1, /* autoconnect */
        0, /* medium priority */
        0); /* default stacksize */
    if (status != asynSuccess)
    {
      errlogPrintf("drvScalerSoftConfigure ERROR: Can't register myself.\n");
      return -1;
    }

    status = pasynManager->registerInterface(pPvt->portName, &pPvt->common);
    if (status != asynSuccess) 
    {
      errlogPrintf("drvScalerSoftConfigure: Can't register common.\n");
      return -1;
    }

    status = pasynInt32Base->initialize(pPvt->portName, &pPvt->int32);
    if (status != asynSuccess) 
    {
      errlogPrintf("drvScalerSoftConfigure: Can't register int32.\n");
      return -1;
    }

    status = pasynInt32ArrayBase->initialize(pPvt->portName, &pPvt->int32Array);
    if (status != asynSuccess) 
    {
      errlogPrintf("drvScalerSoftConfigure: Can't register int32Array.\n");
      return -1;
    }

    pasynManager->registerInterruptSource(portName, &pPvt->int32,
        &pPvt->int32InterruptPvt);

    status = pasynManager->registerInterface(pPvt->portName,&pPvt->drvUser);
    if (status != asynSuccess) 
    {
      errlogPrintf("drvScalerSoftConfigure ERROR: Can't register drvUser\n");
      return -1;
    }

    /* Connect pasynUser to device for debugging */
    status = pasynManager->connectDevice(pPvt->pasynUser, portName, 0);
    if (status != asynSuccess) 
    {
      errlogPrintf("drvScalerSoftConfigure, connectDevice failed for scaler\n");
      return -1;
    }

    return (0);
}


/* Routines to write to the scaler */

static asynStatus int32Write(void *drvPvt, asynUser *pasynUser,
    epicsInt32 value)
{
    scalerPvt *pPvt = (scalerPvt *)drvPvt;
    int command=pasynUser->reason;
    int signal;
    int i;
 
    pasynManager->getAddr(pasynUser, &signal);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s::int32Write entry, command=%d, signal=%d, "
        "value=%d\n", pPvt->driver, command, signal, value);
    switch (command) 
    {
        case scalerResetCommand:
          /* Reset scaler */
           pPvt->acquiring = 0;
           pPvt->prevAcquiring = 0;
          /* Clear all of the presets */
          for (i=0; i<pPvt->maxChans; i++) {
            pPvt->presetCounts[i] = 0;
          }
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Write scalerResetCommand\n", pPvt->driver);
          break;

        case scalerArmCommand:

          /* clear scaler data, to avoid immediate done */
          if (pPvt->connectState != stateConnected) return(asynError);
          for (i=0; i<pPvt->maxChans; i++) {
              pPvt->counts[i] = 0;
              (void)dbPutField(&pPvt->inputAddr[i], DBR_LONG, &pPvt->counts[i], 1);
          }

          /* Arm or disarm scaler */
          pPvt->acquiring = value;
          if (value) pPvt->prevAcquiring = value;
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Write scalerArmCommand=%d\n", pPvt->driver, value);
          break;

        case scalerPresetCommand:
          /* Set scaler preset */
          pPvt->presetCounts[signal] = value;
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Write scalerPresetCommand channel %d=%d\n", pPvt->driver, signal, value);
          break;

        default:
          asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s::int32Write port %s got illegal command %d\n",
              pPvt->driver, pPvt->portName, command);
          break;
    }
    return(asynSuccess);
}


static asynStatus int32Read(void *drvPvt, asynUser *pasynUser,
    epicsInt32 *value)
{
    scalerPvt *pPvt = (scalerPvt *)drvPvt;
    int command = pasynUser->reason;
    asynStatus status=asynSuccess;
    int signal;

    pasynManager->getAddr(pasynUser, &signal);
    checkAcquireDone(pPvt);
 
    switch (command) {
        case scalerChannelsCommand:
          /* Return the number of scaler channels */
          *value = pPvt->maxChans;
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Read scalerChanneksCommand %d\n", pPvt->driver, *value);
          break;

        case scalerReadCommand:
          /* Read a single scaler channel */
          if (pPvt->connectState != stateConnected) return(asynError);
          status = dbGetField(&pPvt->inputAddr[signal], DBR_LONG, &pPvt->counts[signal], NULL, NULL, NULL);
          *value = pPvt->counts[signal];
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Read int32ReadCommand channel%d=%d\n", pPvt->driver, signal, *value);
          break;

        case scalerDoneCommand:
          /* Return scaler done, which is opposite of pPvt->acquiring */
          *value = 1 - pPvt->acquiring;
          asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "%s::int32Read scalerDoneCommand, returning %d\n", pPvt->driver, *value);
          break;

        default:
          asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s::int32Read got illegal command %d\n",
              pPvt->driver, command);
          status = asynError;
          break;
    }
    return(status);
}


static asynStatus getBounds(void *drvPvt, asynUser *pasynUser,
    epicsInt32 *low, epicsInt32 *high)
{
    *low = 0;
    *high = 0;
    return(asynSuccess);
}


static asynStatus int32ArrayRead(void *drvPvt, asynUser *pasynUser, epicsInt32 *data, 
                                 size_t maxChans, size_t *nactual)
{
    scalerPvt *pPvt = (scalerPvt *)drvPvt;
    int command = pasynUser->reason;
    asynStatus status = asynSuccess;
    int i;

    checkAcquireDone(pPvt);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s::int32ArrayRead entry, maxChans=%d\n", 
        pPvt->driver, maxChans);

    switch (command) {
        case scalerReadCommand:
            if (maxChans > pPvt->maxChans) maxChans = pPvt->maxChans;
            if (pPvt->connectState != stateConnected) return(asynError);
            for (i=0; i<pPvt->maxChans; i++) {
                status = dbGetField(&pPvt->inputAddr[i], DBR_LONG, &pPvt->counts[i], NULL, NULL, NULL);
                data[i] = pPvt->counts[i];
            }
            if (status) {
                errlogPrintf("%s::int32ArrayRead port %s"
                             " can't get value of input %d.\n",
                             pPvt->driver, pPvt->portName, i);
                return(asynError);
            }
            *nactual = maxChans;
            asynPrint(pasynUser, ASYN_TRACE_FLOW, 
                "%s::int32ArrayRead scalerReadCommand: read %d chans, channel[0]=%d\n", 
                pPvt->driver, maxChans, data[0]);
            break;

        default:
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s::int32ArrayRead got illegal command %d\n",
                pPvt->driver, command);
            status = asynError;
            break;
    }
    return(status);
}


static asynStatus int32ArrayWrite(void *drvPvt, asynUser *pasynUser,
    epicsInt32 *data, size_t maxChans)
{
  scalerPvt *pPvt = (scalerPvt *)drvPvt;

  /* This function is not implemented in this driver. */
  asynPrint(pasynUser, ASYN_TRACE_ERROR,
      "%s::int32ArrayWrite, write to scaler not implemented\n", pPvt->driver);
  return(asynError);
}


/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
    const char *drvInfo,
    const char **pptypeName, size_t *psize)
{
  int i;
  char *pstring;
  scalerPvt *pPvt = (scalerPvt *)drvPvt;

  for (i=0; i<MAX_SCALER_COMMANDS; i++) {
    pstring = scalerCommands[i].commandString;
    if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
      pasynUser->reason = scalerCommands[i].command;
      if (pptypeName) *pptypeName = epicsStrDup(pstring);
      if (psize) *psize = sizeof(scalerCommands[i].command);
      asynPrint(pasynUser, ASYN_TRACE_FLOW,
          "%s::drvUserCreate, command=%s\n", pPvt->driver, pstring);
      return(asynSuccess);
    }
  }
  asynPrint(pasynUser, ASYN_TRACE_ERROR,
      "%s::drvUserCreate, unknown command=%s\n", pPvt->driver, drvInfo);
  return(asynError);
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
    const char **pptypeName, size_t *psize)
{
  int command = pasynUser->reason;

  *pptypeName = NULL;
  *psize = 0;
  if (pptypeName)
    *pptypeName = epicsStrDup(scalerCommands[command].commandString);
  if (psize) *psize = sizeof(command);
  return(asynSuccess);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
  return(asynSuccess);
}



/***********************/
/* asynCommon routines */
/***********************/

/* Report  parameters */
static void scalerReport(void *drvPvt, FILE *fp, int details)
{
    scalerPvt *pPvt = (scalerPvt *)drvPvt;
    int i;

    assert(pPvt);
    fprintf(fp, "%s: port %s, ", pPvt->driver, pPvt->portName);
    if (pPvt->connectState == stateUnconnected) fprintf(fp, "PVs not yet connected, ");
    if (pPvt->connectState == stateConnected)   fprintf(fp, "PVs connected, ");
    if (pPvt->connectState == stateFailed)      fprintf(fp, "PV connection failed, ");
    fprintf(fp, "maxChans=%d\n", pPvt->maxChans);
    if (details) {
        for (i=0; i<pPvt->maxChans; i++) {
            fprintf(fp, "   channel %d, preset=%ld, counts=%ld\n",
                i+1, pPvt->presetCounts[i], pPvt->counts[i]);
        }
    }
}

/* Connect */
static asynStatus scalerConnect(void *drvPvt, asynUser *pasynUser)
{
  scalerPvt *pPvt = (scalerPvt *)drvPvt;
  int signal;
  
  pasynManager->getAddr(pasynUser, &signal);
  if (signal < pPvt->maxChans) {
    pasynManager->exceptionConnect(pasynUser);
    return(asynSuccess);
  } else {
    return(asynError);
  }
}

/* Disconnect */
static asynStatus scalerDisconnect(void *drvPvt, asynUser *pasynUser)
{
  /* Does nothing for now.  
   * May be used if connection management is implemented */
  pasynManager->exceptionDisconnect(pasynUser);
  return(asynSuccess);
}


/**********************/
/* Internal functions */
/**********************/

static int connectPVs(scalerPvt *pPvt)
{
    /* This task does the actual communication with camserver, so that the driver
     * does not block.  This is needed because the scaler record requires synchronous device
     * support */
    int status;
    char pvName[MAX_PVNAME_SIZE];
    int i;

    /* Look up the database addresses of the PVs we need */
    for (i=0; i<pPvt->maxChans; i++) {
        epicsSnprintf(pvName, sizeof(pvName), pPvt->pvTemplate, i+1);
        status = dbNameToAddr(pvName, &pPvt->inputAddr[i]);
        if (status) {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR,
                "%s::connectPVs port %s can't find PV %s.\n",
                pPvt->driver, pPvt->portName, pvName);
            pPvt->connectState = stateFailed;
            return(-1);
        } else {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW,
                "%s::connectPVs port %s successful connections to PV %s.\n",
                pPvt->driver, pPvt->portName, pvName);
        }
    }
    pPvt->connectState = stateConnected;
    return(0);
}


static void checkAcquireDone(scalerPvt *pPvt)
{
    int i;
    
    for (i=0; i<pPvt->maxChans; i++) {
        if (pPvt->presetCounts[i] > 0) {
            if (pPvt->counts[i] >= pPvt->presetCounts[i]) {
                pPvt->acquiring = 0;
                epicsEventSignal(pPvt->doneEventId);
            }
        }
    }
}

static void acquireDoneCallbacks(scalerPvt *pPvt)
{
    int reason;
    ELLLIST *pclientList;
    interruptNode *pNode;
    asynInt32Interrupt *pint32Interrupt;
    asynStatus status;

    /* First connect to PVs as soon as interruptAccept is 1 */
    while (!interruptAccept) epicsThreadSleep(0.1);
    status = connectPVs(pPvt);
    if (status) return;
    
    while (1) {
        epicsEventWait(pPvt->doneEventId);
        /* Make callbacks to any clients that have requested notification when acquisition completes */
        /* Only do this when pPvt->acquiring == 0 and pPvt->prevAcquiring == 1 */
        if ((pPvt->acquiring == 1) || (pPvt->prevAcquiring == 0)) continue;
        pPvt->prevAcquiring = 0;
        pasynManager->interruptStart(pPvt->int32InterruptPvt, &pclientList);
        pNode = (interruptNode *)ellFirst(pclientList);
        while (pNode) {
            pint32Interrupt = pNode->drvPvt;
            reason = pint32Interrupt->pasynUser->reason;
            if (reason == scalerDoneCommand) {
                asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW, 
                    "%s::acquireDoneCallbacks, making pint32Interrupt->Callback\n",
                    pPvt->driver);
                pint32Interrupt->callback(pint32Interrupt->userPvt,
                    pint32Interrupt->pasynUser,
                    1);
            }
            pNode = (interruptNode *)ellNext(&pNode->node);
        }
        pasynManager->interruptEnd(pPvt->int32InterruptPvt);
    }
}



int drvScalerSoftConfigure(char  *portName,
                          int   maxChans,
                          char  *pvTemplate);
static const iocshArg ConfigureArg0 = {"Port name",            iocshArgString};
static const iocshArg ConfigureArg1 = {"Max. channels",        iocshArgInt};
static const iocshArg ConfigureArg2 = {"PV template",          iocshArgString};

static const iocshArg * const drvScalerSoftConfigureArgs[3] = {
	&ConfigureArg0,
	&ConfigureArg1,
	&ConfigureArg2
};

static const iocshFuncDef drvScalerSoftConfigureFuncDef=
                                                    {"drvScalerSoftConfigure", 3,
                                                     drvScalerSoftConfigureArgs};
static void drvScalerSoftConfigureCallFunc(const iocshArgBuf *args)
{
  drvScalerSoftConfigure(args[0].sval, args[1].ival, args[2].sval);
}

static void drvScalerSoftRegister(void)
{
  iocshRegister(&drvScalerSoftConfigureFuncDef,drvScalerSoftConfigureCallFunc);
}

epicsExportRegistrar(drvScalerSoftRegister);
