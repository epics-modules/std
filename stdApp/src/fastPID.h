//fastPID.h

/*
    This header file defines an abstract base class, fastPID for fast feedback.
    It is device-independent.  In order to use it a derived class must be
    created that implements the writeOutput(), readOutput(), 
    setMicroSecondsPerScan and getMicroSecondsPerScan functions.

    Author: Mark Rivers
    Date: 10/27/99

    04/20/00  MLR  Added new private variables for timing
    04/08/03  MLR  Changed Ip330PID.h to fastPID.h, creating base class

*/

#ifndef fastPIDH
#define fastPIDH

enum fastPIDCommands {cmdStartFeedback, cmdStopFeedback, cmdSetHighLimit,
                      cmdSetLowLimit, cmdSetKP, cmdSetKI, cmdSetKD,
                      cmdSetI, cmdSetSecondsPerScan, cmdSetSetPoint, 
                      cmdGetParams};
#define numFastPIDOffsets 7
enum fastPIDOffsets {offsetActual, offsetError, offsetOutput, 
                     offsetP, offsetI, offsetD, offsetSecondsPerScan};

class fastPID
{
public:
    fastPID();
    virtual void doPID(double actual);
    virtual double setMicroSecondsPerScan(double microSeconds)=0;
    virtual double getMicroSecondsPerScan()=0;
    virtual double readOutput()=0;
    virtual void writeOutput(double output)=0;
    double setPoint;
    double actual;
    double error;
    double KP;
    double KI;
    double KD;
    double P;
    double I;
    double D;
    double lowLimit;
    double highLimit;
    double output;
    int feedbackOn;
    int prevFeedbackOn;
    double prevError;
};

class fastPIDServer {
public:
    fastPIDServer(const char *serverName, fastPID *pFastPID, int queueSize);
    fastPID *pFastPID;
    MessageServer *pMessageServer;
    static void fastServer(fastPIDServer *);
};
#endif //fastPIDH
