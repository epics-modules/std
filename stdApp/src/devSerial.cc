// devSerial.cc
// Device support for the Generic Serial Record
// Author: Mark Rivers
// Date: 3/22/96

#define MAKE_DEBUG devSerialHideosDebug

#include "devHideos.h"
extern "C" {
#include "dbAccess.h"
#include "dbAccess.h"
#include "recSup.h"
};
#include "dbDefs.h"
#include "link.h"
#include "dbCommon.h"
#include "dbScan.h"
#include "serialRecord.h"
#include "menuScan.h"

#include "hideos/utils.h"
#include "gen/all_msg_ids.h"
#include "msg/sysmsg.h"
#include "msg/serial_config_msg.h"
#include "msg/string_msg.h"


// This device support module cannot use the standard HiDEOS macros for creating
// DSETS because we want to define additional functions which those macros do 
// not allow.

typedef struct {
	long        number;
	DEVSUPFUN   report;
	DEVSUPFUN   init;
	DEVSUPFUN   init_record;
	long (*get_ioint_info)(int,dbCommon*,IOSCANPVT*);
	DEVSUPFUN   read_write;
	long (*conv)(void*,int);
        long (*port_setup)(serialRecord *pr, 
                            int baud, int data_bits, int stop_bits, 
                            char parity, char flow_control);
} DEVSERIAL_DSET;


class Serial : public HideosDevice
{
public:
	Serial(dbCommon*,DBLINK*);

	long StartIO(dbCommon* pr);
	long CompleteIO(dbCommon* pr,Message* m);
	long CancelIO();
        long PortSetup(int baud, int data_bits, int stop_bits,
                                    char parity, char flow_control); 

	static long dev_init(void*);
        static long port_setup(serialRecord *pr,
                                    int baud, int data_bits, int stop_bits,
                                    char parity, char flow_control); 
        static long get_ioint_info(int cmd, struct dbCommon *pr, 
                                        IOSCANPVT *ppvt);
private:
	char outbuff[100];
	char inbuff[100];
        IOSCANPVT ioscanpvt;
};

extern "C" {DEVSERIAL_DSET devSerialHideos = 
  { 5,NULL,NULL,Serial::dev_init,Serial::get_ioint_info,
        HideosDevice::read_write,NULL,Serial::port_setup };};


long Serial::dev_init(void* v)
{
	serialRecord* pr = (serialRecord*)v;
	Serial* soi = new Serial((dbCommon*)pr,&(pr->inp));
	return soi->DeviceStatus();
}

Serial::Serial(dbCommon* pr,link* l) : HideosDevice(pr,l)
{
	// can get user parm area from here with GetUserParm() and do stuff
	scanIoInit(&ioscanpvt);
}

long Serial::get_ioint_info(int cmd, struct dbCommon *pr, IOSCANPVT *ppvt)
{
	UserDebug1(1,"Entering get_ioint_info,cmd=%d\n", cmd);
	serialRecord* soi = (serialRecord*)pr;
	Serial* ser = (Serial*)pr->dpvt;
	// If this record was just added to a scan list then send a message
	// However, only do this if the TMOD field is serialTMOD_Read. No other
	// transaction mode makes sense to be I/O Event scanned
	if ((cmd == 0) && (soi->tmod == serialTMOD_Read))  ser->StartIO(pr);
	else if (cmd == 1) ser->CancelIO();
	*ppvt = ser->ioscanpvt;
	return((long)0);
}
        

long Serial::StartIO(dbCommon* pr)
{
	serialRecord* soi = (serialRecord*)pr;
	char *inptr=NULL, *outptr=NULL;
	int inlen=0, outlen=0;
	long timeout;

	IndirectStringMsg* lm =
		(IndirectStringMsg*)GetMessageBuffer(IndirectStringMsgType);

	// If the record is in IO_EVENT scan mode then set the timeout
	// to be infinite, else set it to the TMOT field.
	if (soi->scan == menuScanI_O_Intr)
		timeout = SerialWaitForever;
	else
		timeout = soi->tmot;

		UserDebug1(1, "Entering StartIO, timeout=%ld\n", timeout);

		// Set operation to be performed depends upon the TMOD field in the record
		switch(soi->tmod) {
		case serialTMOD_Write_Read:
			// Do a purge to clear input buffer of any previous characters
			lm->SetPurgeWriteRead(timeout);
			break;
		case serialTMOD_Write:
			lm->SetWrite(timeout);
			break;
		case serialTMOD_Read:
			lm->SetRead(timeout);
			break;
        }

        //  Set things up depending upon ASCII or binary modes
		if (soi->tmod != serialTMOD_Read)
        {
            if (soi->ofmt == serialOFMT_ASCII)
            {
                strncpy(outbuff, soi->aout, sizeof(outbuff));
                //  strncpy appends nulls to the end of outbuff
                //  Add the output delimiter
                if (soi->odel != -1) outbuff[strlen(outbuff)]=soi->odel;
                outptr = outbuff;
                outlen = strlen(outbuff);
            }
            else // Binary output mode
            {
                outptr = (char *)soi->optr;
                outlen = soi->nowt;
                // Append delimiter if there is room
                if ((soi->odel != -1) && (outlen < soi->omax))
                {
                    outptr[outlen]=soi->odel;
                    outlen = outlen + 1;    
                }
            }
        }

	if (soi->tmod != serialTMOD_Write)
        {
            if (soi->idel == -1)
                lm->SetComplexDelimiter((char *)NULL,(char *)NULL,0);  // No delimiter
            else
                lm->SetSimpleDelimiter(soi->idel);  // The delimiter

	    if (soi->ifmt == serialOFMT_ASCII)
            {
                inptr = inbuff;
                if (soi->nrrd != 0)
                    inlen = soi->nrrd;
                else
                    inlen = sizeof(inbuff);
            }
            else // Binary input mode
            {
                inptr = (char *)soi->iptr;
                if (soi->nrrd != 0)
                    inlen = soi->nrrd;
                else
                    inlen = soi->imax;
            }
        }

	lm->SetBuffers((unsigned char *)outptr,outlen,(unsigned char *)inptr,inlen);

	return Send(lm);
}

long Serial::CompleteIO(dbCommon* pr,Message* m)
{
	serialRecord* soi = (serialRecord*)pr;
	IndirectStringMsg* sm = (IndirectStringMsg*)m;
        char *inptr;

        UserDebug0(1,"Entering CompleteIO\n");

        soi->udf=0;

        switch (soi->tmod)
        {
        case serialTMOD_Write:
            break;

        default:  // serialTMOD_Write_Read or serialTMOD_Write
	    if (sm->return_code!=0)  
	        recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
    	    soi->nord = sm->GetDestSize();
            // Action depends upon input format (ASCII or binary)
            UserDebug2(1,
                "   return code=%ld, nord=%ld\n", sm->return_code, soi->nord);
            switch (soi->ifmt)
            {
            case serialOFMT_ASCII:
                // If the string is terminated by the requested terminator
                // replace with NULL.
                if ((soi->nord != 0) && 
                    (soi->idel != -1) && 
                    (inbuff[soi->nord-1] == soi->idel))
                            inbuff[soi->nord-1] = '\0';
                else
                    // Make sure input string is null terminated.
		    inbuff[soi->nord]='\0';
		strncpy(soi->ainp,inbuff,sizeof(soi->ainp));
	        break;
            case serialOFMT_Binary:
                inptr = (char *)soi->iptr;
                // If the string is terminated by the requested terminator
                // remove it.
                if ((soi->nord != 0) && 
                    (soi->idel != -1) &&
                    (inptr[soi->nord-1] == soi->idel))
                            inptr[soi->nord-1] = '\0';
                else
                    // Add a null terminator if there is room
		    if (soi->nord < soi->imax) inptr[soi->nord]='\0';
                break;
            }
        }

	FreeMessageBuffer(m);

        // If the record is in IO_EVENT scan mode then call StartIO again
        // However, only do this if the TMOD field is serialTMOD_Read. No other
        // transaction mode makes sense to be I/O Event scanned.
        if ((soi->scan == menuScanI_O_Intr) && (soi->tmod == serialTMOD_Read))
                StartIO(pr);
	return 2;
}


long Serial::CancelIO()
{
	InterruptMsg* lm =
		(InterruptMsg*)GetMessageBuffer(InterruptMsgType);
        lm->status = SerialCancel;
        return Send(lm);
}




long Serial::port_setup(serialRecord *pr,
                            int baud, int data_bits, int stop_bits, 
                            char parity, char flow_control)
{
        Serial* ser = (Serial*)pr->dpvt;
        return ser->PortSetup(baud, data_bits, stop_bits, parity,
                                    flow_control);
}

long Serial::PortSetup(int baud, int data_bits, int stop_bits, 
                            char parity, char flow_control)
{
	// get a DirectStringMsg buffer from the message pool
        SerialConfigMsg* sc= 
                (SerialConfigMsg*)GetMessageBuffer(SerialConfigMsgType);

	sc->SetBaud(baud);
	sc->SetStopBits(stop_bits);
	sc->SetBitsPerCharacter(data_bits);

	switch(parity) // parity
	{
	case 'E': sc->SetEvenParity(); break;
	case 'O': sc->SetOddParity(); break;
	case 'N':
	default: sc->SetParityNone(); break;
	}

	switch(flow_control) // flow control
	{
	case 'H': sc->SetFlowHardware(); break;
	case 'N':
	default: sc->SetFlowNone(); break;
	}

	// send the message to HiDEOS task
	Send(sc);
        return 0;
}
