#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/******************/
/* EPICS includes */
/******************/

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <errlog.h>
#include <iocsh.h>

#include "devScalerAsyn.h"

#define MAX_CHANNELS 4

#define timeOut 0.1
static const char *driverName= "Scaler974";

class Scaler974:public asynPortDriver
{
public:
    Scaler974(const char *portName, const char *serialPort, int serialAddr, int poll);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32Array(asynUser *pasynUser,epicsInt32 *value, size_t nElements, size_t *nIn);
    virtual void report(FILE *fp, int details);
    void eventThread();

private:
    int scalerReset;
    int scalerChannels;
    int scalerRead;
    int scalerReadSingle;
    int scalerPreset;
    int scalerArm;
    int scalerDone;
    double polltime;
    epicsEventId eventId;
    asynUser *pasynUserScaler;
    asynStatus sendCommand(const char *command, char *statusString, size_t maxStatusLen, size_t *statusLen);
    asynStatus sendCommand(const char *command, char *statusString, size_t maxStatusLen, size_t *statusLen,
                           char *response, size_t maxResponseLen, size_t *responseLen);
};

static void eventThreadC(void *pPvt)
{
    Scaler974 *pScaler974 = (Scaler974 *)pPvt;
    pScaler974->eventThread();
}

Scaler974::Scaler974(const char *portName, const char *serialPort, int serialAddr, int poll)
    :asynPortDriver(portName, MAX_CHANNELS, 7, 
                    asynInt32Mask | asynInt32ArrayMask | asynDrvUserMask,
                    asynInt32Mask,
                    /* Should also be ASYN_CANBLOCK, but device support does not work with asynchronous devices */                 
                    ASYN_MULTIDEVICE,1,0,0) 
{
    int i;
    asynStatus status;
    static const char *functionName="Scaler974";
    
    if (poll==0) poll=100;
    this->polltime=poll / 1000.;
    this->eventId = epicsEventCreate(epicsEventEmpty);
    status = pasynOctetSyncIO->connect(serialPort, serialAddr, &this->pasynUserScaler, NULL);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error connecting to port %s address %d\n",
            driverName, functionName, serialPort, serialAddr);
        return;
    }
    
    createParam(SCALER_RESET_COMMAND_STRING,        asynParamInt32,     &this->scalerReset);
    createParam(SCALER_CHANNELS_COMMAND_STRING,     asynParamInt32,     &this->scalerChannels);
    createParam(SCALER_READ_COMMAND_STRING,         asynParamInt32Array,&this->scalerRead);
    createParam(SCALER_READ_SINGLE_COMMAND_STRING,  asynParamInt32,     &this->scalerReadSingle);
    createParam(SCALER_PRESET_COMMAND_STRING,       asynParamInt32,     &this->scalerPreset);
    createParam(SCALER_ARM_COMMAND_STRING,          asynParamInt32,     &this->scalerArm);
    createParam(SCALER_DONE_COMMAND_STRING,         asynParamInt32,     &this->scalerDone);

    setIntegerParam(scalerChannels, MAX_CHANNELS);
    setIntegerParam(scalerDone, 1);
    for (i=0; i<MAX_CHANNELS; i++) setIntegerParam(i, scalerReadSingle, 0);

    epicsThreadCreate("Scaler974",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)eventThreadC,this);
}

asynStatus Scaler974::sendCommand(const char *command, char *statusString, size_t maxStatusLen, size_t *statusLen)
{
    return sendCommand(command, statusString, maxStatusLen, statusLen, NULL, 0, NULL);
}

asynStatus Scaler974::sendCommand(const char *command, char *statusString, size_t maxStatusLen, size_t *statusLen,
                                  char *response, size_t maxResponseLen, size_t *responseLen)
{
    size_t nWrite;
    asynStatus status;
    double timeout = 1.0;
    int eomReason;
    static const char *functionName = "sendCommand";

    if (response != NULL) {
        status = pasynOctetSyncIO->writeRead(this->pasynUserScaler, command, strlen(command), 
                                             response, maxResponseLen, timeout, &nWrite, responseLen, &eomReason);
        if (status != asynSuccess) goto done;
        status = pasynOctetSyncIO->read(this->pasynUserScaler,
                                        statusString, maxStatusLen, timeout, statusLen, &eomReason);
    } else {
        status = pasynOctetSyncIO->writeRead(this->pasynUserScaler, command, strlen(command), 
                                             statusString, maxStatusLen, timeout, &nWrite, statusLen, &eomReason);
    }
    done:
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: writing command %s, error=%s\n",
                  driverName, functionName, command, this->pasynUserScaler->errorMessage);
    }
    return(status);
}

asynStatus Scaler974::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    char response[256];
    size_t responseLen;
    asynStatus status = asynSuccess;
    
    static const char *functionName="writeInt32";
    setIntegerParam(function, value);
    
    if(function==this->scalerReset)
    {
        this->sendCommand("STOP", response, sizeof(response), &responseLen);
        this->sendCommand("CLEAR_ALL", response, sizeof(response), &responseLen);
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s scalerReset\n",driverName, functionName);     
    }
    else if(function==this->scalerArm)
    {
        if(value !=0)
        {
		/* Start counting */
            this->sendCommand("START", response, sizeof(response), &responseLen);
            setIntegerParam(scalerDone, 0);
            epicsEventSignal(this->eventId);
        }
        else
        { 
		/* Stop counting */
            status = this->sendCommand("STOP", response, sizeof(response), &responseLen);
            setIntegerParam(scalerDone, 1);
        }
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s scalerArm=%d\n", driverName, functionName,value);
    }
    else if(function==this->scalerPreset)
    {
        int m,n;
        char newstr[20];

        n=(int)log10(double(value));
        m=(int)(value/pow(10.0,n));
        
        sprintf(newstr, "SET_COUNT_PRESET %d,%d", m, n);
        this->sendCommand(newstr, response, sizeof(response), &responseLen);

        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s;%s scalerPreset channel", driverName, functionName);
    }
    else
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"%s:%s got illegal function %d\n", driverName, functionName, function);
    }
    callParamCallbacks();
    return(status);
}

asynStatus Scaler974::readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t maxChannel, size_t *nIn)
{
    static const char *functionName="readInt32Array";
    int function = pasynUser->reason;
    int temp;
    int i;

    if (maxChannel > MAX_CHANNELS) maxChannel = MAX_CHANNELS;
    *nIn = maxChannel;
   
    if (function==scalerRead)
    {
        for (i=0; i<(int)maxChannel; i++) 
        {
            getIntegerParam(i, scalerReadSingle, &temp);
            value[i] = temp;
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: value=%d %d %d %d\n",
            driverName, functionName, value[0], value[1], value[2], value[3]);
    } else {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"%s:%s got illegal function %d\n", driverName, functionName,  function);
        return(asynError);
    }
    return(asynSuccess);
}

void Scaler974::report(FILE *fp, int details)
{
    asynPortDriver::report(fp, details);
}

void Scaler974::eventThread()
{
    int done, presetCount;
    char response[100], statusString[20];
    size_t responseLen, statusLen;
    int counts[MAX_CHANNELS];
    int i;
    asynStatus status;
    static const char *functionName="eventThread";
    
    while(1)
    {
        epicsEventWait(this->eventId);
        while(1)
        {
            status = this->sendCommand("SHOW_COUNTS", statusString, sizeof(statusString), &statusLen,
                                       response, sizeof(response), &responseLen);
            sscanf(response, "%d;%d;%d;%d;", 
                   &counts[0], &counts[1], &counts[2], &counts[3]);
            asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
                "%s:%s status=%d, counts=%d %d %d %d\n",
                driverName, functionName, status, counts[0], counts[1], counts[2], counts[3]);
            this->lock();
            /* Get value of done in case scaler was stopped by scalerArm(0) */
            getIntegerParam(scalerDone, &done);
            getIntegerParam(scalerPreset, &presetCount);
            if (!done && (counts[0] >= presetCount)) done = 1;
            setIntegerParam(scalerDone, done);
            for (i=0; i<MAX_CHANNELS; i++) {
                setIntegerParam(i, scalerReadSingle, counts[i]);
                callParamCallbacks(i, i);
            }
            this->unlock();
            if (done) break; 
            epicsThreadSleep(this->polltime);
        }
    }
}

extern "C" int initScaler974(const char *portName, const char *serialPort, int serialAddr, int poll)
{
    new Scaler974(portName, serialPort, serialAddr, poll);
    return(asynSuccess);
}

/* iocsh function */
static const iocshArg initArg0 = {"Port Name", iocshArgString};
static const iocshArg initArg1 = {"Scaler Port", iocshArgString};
static const iocshArg initArg2 = {"GPIB address", iocshArgString};
static const iocshArg initArg3 = {"Poll", iocshArgInt};
static const iocshArg *const initArgs[] = {&initArg0,
                                           &initArg1,
                                           &initArg2,
                                           &initArg3
                                          };
static const iocshFuncDef initFuncDef = {"initScaler974",4, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    initScaler974(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void Scaler974Register(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
epicsExportRegistrar(Scaler974Register);
}
