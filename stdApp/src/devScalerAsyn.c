/* devScalerAsyn.c

    Author: Mark Rivers
    25-Oct-2006

    This is device support for the scaler record with asyn drivers.
  
*/


#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <recGbl.h>
#include <dbAccess.h>
#include <dbDefs.h>
#include <link.h>
#include <errlog.h>
#include <dbCommon.h>
#include <dbScan.h>
#include <callback.h>
#include <cantProceed.h>
#include <recSup.h>
#include <devSup.h>
#include <alarm.h>

#include <asynDriver.h>
#include <asynInt32.h>
#include <asynInt32Array.h>
#include <asynDrvUser.h>
#include <asynEpicsUtils.h>
#include <epicsExport.h>

#include "scalerRecord.h"
#include "devScaler.h"
#include "devScalerAsyn.h"


typedef struct {
    int command;
    long val;
    epicsInt32 *pval;
} scalerAsynMessage;

typedef struct {
    scalerRecord *psr;
    asynUser *pasynUser[MAX_SCALER_CHANNELS];
    asynInt32 *pasynInt32;
    void *asynInt32Pvt;
    asynInt32Array *pasynInt32Array;
    void *asynInt32ArrayPvt;
    asynDrvUser *pasynDrvUser;
    void  *asynDrvUserPvt;
    int channelsCommand;
    int resetCommand;
    int readCommand;
    int presetCommand;
    int armCommand;
    int doneCommand;
    int done;
    void *registrarPvt;
    CALLBACK *pcallback;
} scalerAsynPvt;

/* These functions are in the DSET */
static long scaler_init_record(struct scalerRecord *psr, CALLBACK *pcallback);
static long scaler_reset(scalerRecord *psr);
static long scaler_read(scalerRecord *psr, unsigned long *val);
static long scaler_write_preset(scalerRecord *psr, int signal, unsigned long val);
static long scaler_arm(scalerRecord *psr, int val);
static long scaler_done(scalerRecord *psr);

/* These are other functions */
static long scaler_command(scalerRecord *psr, int command, long channel, long val, long *pval);
static void asynCallback(asynUser *pasynUser);
static void interruptCallback(void *drvPvt,  asynUser *pasynUser, epicsInt32 value);

SCALERDSET devScalerAsyn = {
    7,
    NULL,
    NULL,
    scaler_init_record,
    NULL,
    scaler_reset,
    scaler_read,
    scaler_write_preset,
    scaler_arm,
    scaler_done
};
epicsExportAddress(dset, devScalerAsyn);


static long scaler_init_record(scalerRecord *psr, CALLBACK *pcallback)
{
    asynUser *pasynUser;
    char *port, *userParam;
    int signal;
    asynStatus status;
    asynInterface *pasynInterface;
    scalerAsynPvt *pPvt;
    int i;
    int nchans;

    /* Allocate asynMcaPvt private structure */
    pPvt = callocMustSucceed(1, sizeof(scalerAsynPvt), "devScalerAsyn init_record()");
    pasynUser = pasynManager->createAsynUser(asynCallback, 0);
    pPvt->psr = psr;
    psr->dpvt = pPvt;
    pPvt->pcallback = pcallback;
    status = pasynEpicsUtils->parseLink(pasynUser, &psr->out,
                                    &port, &signal, &userParam);
    if (status != asynSuccess) {
        errlogPrintf("devScalerAsyn::init_record %s bad link %s\n",
                     psr->name, pasynUser->errorMessage);
        goto bad;
    }

    /* Create a pasynUser for initialization */
    pasynUser = pasynManager->createAsynUser(asynCallback, 0);
    pasynUser->userPvt = pPvt;
    /* Connect to device */
    status = pasynManager->connectDevice(pasynUser, port, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devScalerAsyn::init_record, %s initial connectDevice failed to %s\n",
                  psr->name, port);
        goto bad;
    }

    /* Get the asynInt32 interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynInt32Type, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devScalerAsyn::init_record, %s find int32 interface failed\n",
                  psr->name);
        goto bad;
    }
    pPvt->pasynInt32 = (asynInt32 *)pasynInterface->pinterface;
    pPvt->asynInt32Pvt = pasynInterface->drvPvt;

    /* Get the asynInt32Array interface */
    pasynInterface = pasynManager->findInterface(pasynUser, 
                                                 asynInt32ArrayType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devScalerAsyn::init_record, %s find int32Array interface failed\n",
                  psr->name);
        goto bad;
    }
    pPvt->pasynInt32Array = (asynInt32Array *)pasynInterface->pinterface;
    pPvt->asynInt32ArrayPvt = pasynInterface->drvPvt;

    /* Get the asynDrvUser interface for translating commands */
    pasynInterface = pasynManager->findInterface(pasynUser, asynDrvUserType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devScalerAsyn::init_record, %s find drvUser interface failed\n",
                  psr->name);
        goto bad;
    }
    pPvt->pasynDrvUser = (asynDrvUser *)pasynInterface->pinterface;
    pPvt->asynDrvUserPvt = pasynInterface->drvPvt;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_CHANNELS_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_CHANNELS %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->channelsCommand = pasynUser->reason;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_READ_COMMAND_STRING, 0, 0);
    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_RESET_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_RESET %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->resetCommand = pasynUser->reason;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_READ_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_READ %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->readCommand = pasynUser->reason;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_PRESET_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_PRESET %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->presetCommand = pasynUser->reason;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser,
                                        SCALER_ARM_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_ARM %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->armCommand = pasynUser->reason;

    status = pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, 
                                        SCALER_DONE_COMMAND_STRING, 0, 0);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record drvUserCreate SCALER_DONE %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    pPvt->doneCommand = pasynUser->reason;

    /* Ask how many channels the scaler supports */
    pasynUser->reason = pPvt->channelsCommand;
    status = pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser, &nchans);
    if(status!=asynSuccess) {
        printf("%s scaler_init_record error reading nchans %s\n",
                psr->name, pasynUser->errorMessage);
        goto bad;
    }
    if (nchans > MAX_SCALER_CHANNELS) nchans = MAX_SCALER_CHANNELS;
    psr->nch = nchans;

    /* Create asynUsers for each channel */
    for (i=0; i<psr->nch; i++) {
        pasynUser = pasynManager->createAsynUser(asynCallback, 0);
        pasynUser->userPvt = pPvt;
        pPvt->pasynUser[i] = pasynUser;
  
        /* Connect to device */
        status = pasynManager->connectDevice(pasynUser, port, i);
        if (status != asynSuccess) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "devScalerAsyn::init_record, %s connectDevice failed to %s for channel %d\n",
                      psr->name, port);
            goto bad;
        }
    }

    /* Register for callbacks when acquisition completes - use channel 0 */
    pasynUser = pPvt->pasynUser[0];
    pasynUser->reason = pPvt->doneCommand;
    status = pPvt->pasynInt32->registerInterruptUser(pPvt->asynInt32Pvt,
                                                     pasynUser,
                                                     interruptCallback,pPvt,
                                                     &pPvt->registrarPvt);
    if (status!=asynSuccess) {
        printf("%s scaler_init_record registerInterruptUser %s\n",
               psr->name, pasynUser->errorMessage);
        goto bad;
    }

    return(0);

bad:
    psr->pact=1;
    return(0);
}


static long scaler_reset(scalerRecord *psr)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    return(scaler_command(psr, pPvt->resetCommand, 0, 0, 0));
}

static long scaler_read(scalerRecord *psr, unsigned long *val)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    return(scaler_command(psr, pPvt->readCommand, 0, 0, (long*)val));
}

static long scaler_write_preset(scalerRecord *psr, int signal, unsigned long val)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    return(scaler_command(psr, pPvt->presetCommand, signal, val, NULL));
}

static long scaler_arm(scalerRecord *psr, int val)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    return(scaler_command(psr, pPvt->armCommand, 0, val, NULL));
}

static long scaler_done(scalerRecord *psr)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    if (pPvt->done) {
        pPvt->done = 0;
        return(1);
    } else {
        return(0);
    }
}


static long scaler_command(scalerRecord *psr, int command, long channel, long val, long *pval)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)psr->dpvt;
    asynUser *pasynUser = pPvt->pasynUser[channel];
    scalerAsynMessage *pmsg;
    int status;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "devScalerAsyn::scaler_command: %s entry\n", 
              psr->name);

    /* If we are already in COMM_ALARM then this server is not reachable, return */
    if ((psr->nsta == COMM_ALARM) || (psr->stat == COMM_ALARM)) return(-1);
   
   /* Make a copy of asynUser.  This is needed because we can have multiple
    * requests queued.  It will be freed in the callback */
    pasynUser = pasynManager->duplicateAsynUser(pasynUser, asynCallback, 0);
    pmsg = pasynManager->memMalloc(sizeof *pmsg);

    pmsg->command = command;
    pmsg->val = val;
    pmsg->pval = (epicsInt32 *)pval;
    pasynUser->userData = pmsg;

    /* Queue asyn request, so we get a callback when driver is ready */
    status = pasynManager->queueRequest(pasynUser, 0, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                  "devScalerAsyn::scaler_reset: %s error calling queueRequest, %s\n", 
                  psr->name, pasynUser->errorMessage);
        return(-1);
    }
    /* This device support only works with synchronous drivers, which means that the I/O is complete
     * at this point */
    return(0);
}


static void asynCallback(asynUser *pasynUser)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)pasynUser->userPvt;
    scalerRecord *psr = pPvt->psr;
    scalerAsynMessage *pmsg = pasynUser->userData;
    int status;
    size_t nread;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
              "devScalerAsyn::asynCallback: %s command=%d, val=%d, pval=%p\n",
              psr->name, pmsg->command, pmsg->val, pmsg->pval);
    pasynUser->reason = pmsg->command;

    if (pmsg->command == pPvt->resetCommand) {
        /* Send reset command */
       pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser, 0);
    }
    else if (pmsg->command == pPvt->readCommand) {
        /* Read the values of the scalers */
       pPvt->pasynInt32Array->read(pPvt->asynInt32ArrayPvt, pasynUser, 
                                   pmsg->pval, psr->nch, &nread);
       asynPrint(pasynUser, ASYN_TRACE_FLOW,
         "devScalerAsyn::asynCallback readCommand nread=%d, counts[0]=%d\n",
         nread, pmsg->pval[0]);        
    }
    else if (pmsg->command == pPvt->presetCommand) {
        /* Send preset command */
       pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser, pmsg->val);
    }
    else if (pmsg->command == pPvt->armCommand) {
        /* Send arm command */
       pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser, pmsg->val);
    }
    else if (pmsg->command == pPvt->doneCommand) {
        /* Send done command */
       pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser, &pPvt->done);
    }

    pasynManager->memFree(pmsg, sizeof(*pmsg));
    status = pasynManager->freeAsynUser(pasynUser);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                  "devScalerAsyn::asynCallback: %s error in freeAsynUser, %s\n",
                  psr->name, pasynUser->errorMessage);
    }
}

/* Callback routine gets called when acquisition completes */
static void interruptCallback(void *drvPvt,  asynUser *pasynUser, epicsInt32 value)
{
    scalerAsynPvt *pPvt = (scalerAsynPvt *)drvPvt;
    scalerRecord *psr = pPvt->psr;

    /* Ignore callbacks when done value is 0 */
    if (value == 0) return;
    pPvt->done = 1;
    asynPrint(pPvt->pasynUser[0], ASYN_TRACEIO_DEVICE,
        "%s devScalerAsyn::interruptCallback new value=%d\n",
        psr->name, value);
    callbackRequest(pPvt->pcallback);
}

