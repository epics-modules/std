//fastPID.cc

/*
    Author: Mark Rivers
    10/27/99

    04/20/00  MLR  Changed timing logic.  No longer reprogram IP-330 clock,
                   just change whether the callback is executed when called.
    05/02/00  MLR  Fixed bug in time calculation for integral term
    05/03/00  MLR  Added more sanity checks to integral term:
                   If KI is 0. then set I to DRVL if KP is greater than 0,
                   set I to DRVH is KP is less than 0.
                   If feedback is off don't change integral term.
    05/17/00  MLR  Added another sanity checks to integral term:
                   When feedback goes from OFF to ON then set integral term
                   equal to the current value of the DAC output.  This ensures
                   a "bumpless" turn-on of feedback
    01/16/01 MLR   Added check for valid pFast in fastPID::init
    04/08/03 MLR   Converted to abstract base class, fastPID, from Ip300PID.
                   fastPID is device-independent.  This class must be derived
                   by a device-dependent class that implements writeOutput(),
                   readOutput(), and periodically calls doPID(), typically when
                   a new input value is available.
                   Converted Ip330PIDServer to base class fastPIDServer, 
                   moved to this file.
*/

#include <vxWorks.h>
#include <iv.h>
#include <string.h>
#include <stdio.h>

#include "Message.h"
#include "Float64Message.h"
#include "Float64ArrayMessage.h"
#include "mpfType.h"

#include "fastPID.h"

extern "C"
{
#ifdef NODEBUG
#define DEBUG(l,f,v) ;
#else
#define DEBUG(l,f,v...) { if(l<fastPIDDebug) printf(f,## v); }
#endif
volatile int fastPIDDebug = 0;
}

fastPID::fastPID(): KP(1), KI(0), KD(0), 
                    P(0), I(0), D(0), 
                    lowLimit(1), highLimit(-1), 
                    feedbackOn(0), prevError(0)
{
    // Nothing else to do in constructor
}

void fastPID:: doPID(double readBack)
{
    double dt;
    double derror;
    double dI;

    dt = getMicroSecondsPerScan() / 1.e6;
    actual = readBack;
    prevError = error;
    error = setPoint - actual;
    derror = error - prevError;
    P = KP*error;
    /* Sanity checks on integral term:
        * 1) Don't increase I if output >= highLimit
        * 2) Don't decrease I if output <= lowLimit
        * 3) Don't change I if feedback is off
        * 4) Limit the integral term to be in the range betweem DRLV and DRVH
        * 5) If KI is zero then set the sum to DRVL (if KI is positive), or
        *    DRVH (if KP is negative) to allow easily turning off the 
        *    integral term for PID tuning.
    */
    dI = KP*KI*error*dt;
    if (feedbackOn) {
        if (!prevFeedbackOn) {
            I = readOutput();
        }
        else {
            if (((output > lowLimit) && (output < highLimit)) ||
                ((output >= highLimit) && ( dI < 0.)) ||
                ((output <= lowLimit)  && ( dI > 0.))) {
                I = I + dI;
                if (I < lowLimit) I = lowLimit;
                if (I > highLimit) I = highLimit;
            }
        }
    }
    if (KI == 0.) {
        if (KP > 0.) I = lowLimit; else I = highLimit;
    }
    if (dt>0.0) D = KP*KD*(derror/dt); else D = 0.0;
    output = (P + I + D);
    // Limit output to range from low to high limit
    if (output > highLimit) output = highLimit;
    if (output < lowLimit) output = lowLimit;

    // If feedback is on write output
    if (feedbackOn) {
        writeOutput(output);
    }
    // Save state of feedback
    prevFeedbackOn = feedbackOn;
}


fastPIDServer::fastPIDServer(const char *serverName, fastPID *pFastPID,
                             int queueSize) :
                             pFastPID(pFastPID)
{
    pMessageServer = new MessageServer(serverName,queueSize);
}

void fastPIDServer::fastServer(fastPIDServer *pFastPIDServer)
{
    while(true) {
        MessageServer *pMessageServer = pFastPIDServer->pMessageServer;
        fastPID *pFastPID = pFastPIDServer->pFastPID;
        pMessageServer->waitForMessage();
        Message *inmsg;
        int cmd;
        double value;
        
        while((inmsg = pMessageServer->receive())) {
            Float64Message *pim = (Float64Message *)inmsg;
            Float64ArrayMessage *pam = NULL;
            if (inmsg->getType() != messageTypeFloat64) {
                printf("%s ip330Server got illegal message type %d\n",
                    pMessageServer->getName(), inmsg->getType());
                delete inmsg;
                break;
            }
            pim->status = 0;
            cmd = pim->cmd;
            value = pim->value;
            if ((fastPIDDebug > 0) && (cmd != cmdGetParams))
                printf("fastPIDServer, cmd=%d, value=%f\n", cmd, value);
            if ((fastPIDDebug > 5) && (cmd == cmdGetParams))
                printf("fastPIDServer, cmd=%d, value=%f\n", cmd, value);
            switch (cmd) {
            case cmdStartFeedback:
               pFastPID->feedbackOn = 1;
               break;
            case cmdStopFeedback:
               pFastPID->feedbackOn = 0;
               break;
            case cmdSetLowLimit:
               pFastPID->lowLimit = value;
               break;
            case cmdSetHighLimit:
               pFastPID->highLimit = value;
               break;
            case cmdSetKP:
               pFastPID->KP = value;
               break;
            case cmdSetKI:
               pFastPID->KI = value;
               break;
            case cmdSetKD:
               pFastPID->KD = value;
               break;
            case cmdSetI:
               pFastPID->I = value;
               break;
            case cmdSetSecondsPerScan:
               pFastPID->setMicroSecondsPerScan(value*1.e6);
               break;
            case cmdSetSetPoint:
               pFastPID->setPoint = value;
               break;
            case cmdGetParams:
               pam = (Float64ArrayMessage *)pMessageServer->allocReplyMessage(
                                          pim, messageTypeFloat64Array);
               delete pim;
               pam->allocValue(numFastPIDOffsets);
               pam->setLength(numFastPIDOffsets);
               pam->value[offsetActual] = pFastPID->actual;
               pam->value[offsetError] = pFastPID->error;
               pam->value[offsetP] = pFastPID->P;
               pam->value[offsetI] = pFastPID->I;
               pam->value[offsetD] = pFastPID->D;
               pam->value[offsetOutput] = pFastPID->output;
               pam->value[offsetSecondsPerScan] =
                                 pFastPID->getMicroSecondsPerScan()*1.e-6;
               break;
            default:
               printf("%s fastPIDServer got illegal command %d\n",
               pMessageServer->getName(), pim->cmd);
               break;
            }
            if (cmd == cmdGetParams)
               pMessageServer->reply(pam);
            else
               pMessageServer->reply(pim);
        }
    }
}
