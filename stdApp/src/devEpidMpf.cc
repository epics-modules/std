// devEpidMpf.cc

/********************COPYRIGHT NOTIFICATION**********************************
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
****************************************************************************/
/*
    Author: Mark Rivers
    Date: 10/11/99

*/


#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
 
#include "dbAccess.h"
#include "dbDefs.h"
#include "link.h"
#include "epicsPrint.h"
#include "dbCommon.h"
#include "epidRecord.h"
#include "recSup.h"
#include "recGbl.h"
#include "alarm.h"

#include "Message.h"
#include "Float64Message.h"
#include "Float64ArrayMessage.h"
#include "DevMpf.h"
#include "fastPID.h"
#include <epicsExport.h>

/*----------------debugging-----------------*/

#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int devEpidMpfDebug = 0;
	#define DEBUG(l, f, args...) {if (l <= devEpidMpfDebug) printf(f, ## args);}
    #else
	#define DEBUG(l, f, args...)
    #endif
#else
    #define DEBUG()
#endif

class DevEpidMpf : public DevMpf
{
public:
    DevEpidMpf(dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    virtual void connectIO(dbCommon *pr, Message *message);
    virtual void receiveReply(dbCommon* pr, Message* m);
    static long dev_init(void*);
private:
    long sendFloat64Message(int cmd, float value);
    void sendParams(epidRecord *pepid, bool forceSend);
    int feedBackOn;
    float highLimit;
    float lowLimit;
    float KP; 
    float KI; 
    float KD;
    float I;
    float secondsPerScan;
    float setPoint;
};

MAKE_DSET(devEpidMpf, (DEVSUPFUN) DevEpidMpf::dev_init)

DevEpidMpf::DevEpidMpf(dbCommon* pr,DBLINK* l) : DevMpf(pr,l,false)
{
    // Nothing to be done in constructor for now
}

long DevEpidMpf::dev_init(void* v)
{
    epidRecord* pr = (epidRecord*)v;
    DevEpidMpf* pDevEpidMpf
            = new DevEpidMpf((dbCommon*)pr,&(pr->inp));
    pDevEpidMpf->bind();
    return pDevEpidMpf->getStatus();
}

void DevEpidMpf::connectIO(dbCommon *pr, Message* message)
{
    // Set the PID parameters when the server connects
    epidRecord* pepid = (epidRecord*)pr;
    ConnectMessage *pConnectMessage = (ConnectMessage *)message;
    DEBUG(5,"DevEpidMpf::connectIO, enter, record=%s, status=%d\n", pepid->name, pConnectMessage->status);
    if (pConnectMessage->status != connectYes) goto finish;

    sendParams(pepid, TRUE);

  finish:
    DevMpf::connectIO(pr, message);  // Call the base class method
}


long DevEpidMpf::startIO(dbCommon* pr)
{
    epidRecord* pepid = (epidRecord*)pr;
    Float64Message *pfm = new Float64Message;
    
    DEBUG(5,"DevEpidMpf::startIO, enter, record=%s\n", pepid->name);

    // If any of the parameters have changed then send a message
    sendParams(pepid, FALSE);
    
    // Now send a query for the current parameters, this will complete in
    // completeIO
    pfm->cmd = cmdGetParams;
    return(sendReply(pfm));
}

long DevEpidMpf::completeIO(dbCommon* pr,Message* m)
{
    epidRecord* pepid = (epidRecord*)pr;
    if((m->getType()) != messageTypeFloat64Array) {
        epicsPrintf("%s ::completeIO illegal message type %d\n",
            pepid->name,m->getType());
        recGblSetSevr(pepid,READ_ALARM,INVALID_ALARM);
        delete m;
        return(MPF_NoConvert);
    }
    Float64ArrayMessage *pam = (Float64ArrayMessage *)m;
    if(pam->status) {
        DEBUG(1,"DevEpidMpf::completeIO, record=%s, status=%d\n", pepid->name, pam->status);
        recGblSetSevr(pepid,READ_ALARM,INVALID_ALARM);
        delete m;
        return(MPF_NoConvert);
    }
    // Copy values from array message to record
    pepid->cval = pam->value[offsetActual];
    pepid->err  = pam->value[offsetError];
    pepid->oval = pam->value[offsetOutput];
    pepid->p    = pam->value[offsetP];
    pepid->i    = pam->value[offsetI];
    pepid->d    = pam->value[offsetD];
    pepid->dt   = pam->value[offsetSecondsPerScan];
    DEBUG(5,"DevEpidMpf::completeIO, record=%s, cval=%f, err=%f, oval=%f,\n", pepid->name, pepid->cval, pepid->err, pepid->oval);
    DEBUG(5,"         p=%f, i=%f, d=%f, dt=%f\n", pepid->p, pepid->i, pepid->d, pepid->dt);
    pepid->udf=0;
    delete m;
    return(MPF_OK);
}

void DevEpidMpf::sendParams(epidRecord *pepid, bool forceSend)
{
    // If any of the parameters have changed, or if forceSend is true, 
    // then send a message
    if ((pepid->fbon != feedBackOn) || forceSend) {
       feedBackOn = pepid->fbon;
       if (pepid->fbon) sendFloat64Message(cmdStartFeedback, 0);
       else             sendFloat64Message(cmdStopFeedback, 0);
    }

    if ((pepid->drvh != highLimit) || forceSend) {
       highLimit = pepid->drvh;
       sendFloat64Message(cmdSetHighLimit, highLimit);
    }

    if ((pepid->drvl != lowLimit) || forceSend) {
       lowLimit = pepid->drvl;
       sendFloat64Message(cmdSetLowLimit, lowLimit);
    }

    if ((pepid->kp != KP) || forceSend) {
       KP = pepid->kp;
       sendFloat64Message(cmdSetKP, KP);
    }

    if ((pepid->ki != KI) || forceSend) {
       KI = pepid->ki;
       sendFloat64Message(cmdSetKI, KI);
    }

    if ((pepid->kd != KD) || forceSend) {
       KD = pepid->kd;
       sendFloat64Message(cmdSetKD, KD);
    }

    if ((pepid->dt != secondsPerScan) || forceSend) {
       secondsPerScan = pepid->dt;
       sendFloat64Message(cmdSetSecondsPerScan, secondsPerScan);
     }

    if ((pepid->val != setPoint) || forceSend) {
       setPoint = pepid->val;
       sendFloat64Message(cmdSetSetPoint, setPoint);
    }
}

long DevEpidMpf::sendFloat64Message(int cmd, float value)
{
    Float64Message *pfm = new Float64Message;

    // If we are not connected don't send the message.  This could be called
    // before we connect, and that screws up DevMpf.
    if (!isConnected()) return(-1);
    pfm->cmd = cmd;
    pfm->value = value;
    return(send(pfm, replyTypeReceiveReply));
}

void DevEpidMpf::receiveReply(dbCommon* pr, Message* m)
{
    epidRecord* pepid = (epidRecord*)pr;
    DEBUG(5, "%s DevEpidMpf:receiveReply enter\n", pepid->name);
    delete m;
}
