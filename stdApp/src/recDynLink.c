/*recDynLink.c*/
/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO
 
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/


/*
 * 02/11/00  tmm Added checks to recDynLinkPutCallback, so user won't hang
 *               waiting for a callback that will never come.
 * 03/08/01  tmm Guarantee that client gets search callback before monitor
 *               callback
 * 11/12/03  tmm Fixed mem leak.  (Wasn't calling epicsMutexDestroy.)
 *               Changed ring buffer to message queue.  DEBUG macro.
 *
 */

#include <epicsMessageQueue.h>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsEvent.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <taskwd.h>
#include <dbDefs.h>
#include <epicsPrint.h>
#include <db_access.h>
#include <db_access_routines.h>
#include <cadef.h>
#include <caerr.h>
#include <caeventmask.h>
#include <tsDefs.h>
#include "recDynLink.h"

volatile int recDynINPCallPend = 1;
volatile int recDynOUTCallPend = 1;

#ifdef NODEBUG
#define DEBUG(l,f,v...) ;
#else
#ifdef __GNUC__
#define DEBUG(l,f,v...) { if(l<=recDynLinkDebug) printf(f ,## v); }
#else
#ifdef __SUNPRO_CC
#define DEBUG(l,...) { if(l<=recDynLinkDebug) printf(__VA_ARGS__); }
#else
#define DEBUG(l,f,v) { if(l<=recDynLinkDebug) printf(f, v); }
#endif
#endif
#endif
volatile int recDynLinkDebug = 0;

/*Definitions to map between old and new database access*/
/*because we are using CA must include db_access.h*/
/* new field types */
#define newDBF_STRING	0
#define	newDBF_CHAR	1
#define	newDBF_UCHAR	2
#define	newDBF_SHORT	3
#define	newDBF_USHORT	4
#define	newDBF_LONG	5
#define	newDBF_ULONG	6
#define	newDBF_FLOAT	7
#define	newDBF_DOUBLE	8
#define	newDBF_ENUM	9

/* new data request buffer types */
#define newDBR_STRING      newDBF_STRING
#define newDBR_CHAR        newDBF_CHAR
#define newDBR_UCHAR       newDBF_UCHAR
#define newDBR_SHORT       newDBF_SHORT
#define newDBR_USHORT      newDBF_USHORT
#define newDBR_LONG        newDBF_LONG
#define newDBR_ULONG       newDBF_ULONG
#define newDBR_FLOAT       newDBF_FLOAT
#define newDBR_DOUBLE      newDBF_DOUBLE
#ifndef newDBR_ENUM
#define newDBR_ENUM        newDBF_ENUM
#endif
#define VALID_newDB_REQ(x) ((x >= 0) && (x <= newDBR_ENUM))
static short mapNewToOld[newDBR_ENUM+1] = {
	DBF_STRING,DBF_CHAR,DBF_CHAR,DBF_SHORT,DBF_SHORT,
	DBF_LONG,DBF_LONG,DBF_FLOAT,DBF_DOUBLE,DBF_ENUM};

int   recDynLinkQsize = 256;
   
LOCAL epicsThreadId inpTaskId=NULL;
LOCAL epicsThreadId	outTaskId=NULL;
LOCAL epicsEventId	wakeUpEvt;
epicsMessageQueueId	recDynLinkInpMsgQ = NULL;
epicsMessageQueueId	recDynLinkOutMsgQ = NULL;

typedef enum{cmdSearch,cmdClear,cmdPut,cmdPutCallback} cmdType;
typedef enum{ioInput,ioOutput} ioType;
typedef enum{stateStarting,stateSearching,stateGetting,stateConnected} stateType;

typedef struct dynLinkPvt{
    epicsMutexId	lock;
    char		*pvname;
    chid		chid;
    evid		evid;
    recDynCallback	searchCallback;
    recDynCallback	monitorCallback;
    recDynCallback	notifyCallback;
	short notifyInProgress;
    TS_STAMP		timestamp;
    short		status;
    short		severity;
    void		*pbuffer;
    size_t		nRequest;
    short		dbrType;
    double		graphicLow,graphHigh;
    double		controlLow,controlHigh;
    char		units[MAX_UNITS_SIZE];
    short		precision;
    ioType		io;
    stateType		state;
    short		scalar;
} dynLinkPvt;

/* For cmdClear data is pdynLinkPvt. For all other commands precDynLink */
typedef struct {
	union {
		recDynLink	*precDynLink;
		dynLinkPvt	*pdynLinkPvt;
	} data;
	cmdType	cmd;
} msgQCmd;

LOCAL void recDynLinkStartInput(void);
LOCAL void recDynLinkStartOutput(void);
LOCAL void connectCallback(struct connection_handler_args cha);
LOCAL void getCallback(struct event_handler_args eha);
LOCAL void monitorCallback(struct event_handler_args eha);
LOCAL void notifyCallback(struct event_handler_args eha);
LOCAL void recDynLinkInp(void);
LOCAL void recDynLinkOut(void);

long recDynLinkAddInput(recDynLink *precDynLink,char *pvname,
	short dbrType,int options,
	recDynCallback searchCallback,recDynCallback monitorCallback)
{
	dynLinkPvt		*pdynLinkPvt;
	struct dbAddr	dbaddr;
	msgQCmd		cmd;
    
	if (options&rdlDBONLY  && db_name_to_addr(pvname,&dbaddr)) return(-1);
	if (!inpTaskId) recDynLinkStartInput();
	if (precDynLink->pdynLinkPvt) recDynLinkClear(precDynLink);
	pdynLinkPvt = (dynLinkPvt *)calloc(1,sizeof(dynLinkPvt));
	if (!pdynLinkPvt) {
		printf("recDynLinkAddInput can't allocate storage");
		epicsThreadSuspendSelf();
	}
	pdynLinkPvt->lock = epicsMutexMustCreate();
	precDynLink->pdynLinkPvt = pdynLinkPvt;
	pdynLinkPvt->pvname = pvname;
	pdynLinkPvt->dbrType = dbrType;
	pdynLinkPvt->searchCallback = searchCallback;
	pdynLinkPvt->monitorCallback = monitorCallback;
	pdynLinkPvt->io = ioInput;
	pdynLinkPvt->scalar = (options&rdlSCALAR) ? TRUE : FALSE;
	pdynLinkPvt->state = stateStarting;
	cmd.data.precDynLink = precDynLink;
	cmd.cmd = cmdSearch;
	if (epicsMessageQueueTrySend(recDynLinkInpMsgQ, (void *)&cmd, sizeof(cmd))) {
		errMessage(0,"recDynLinkAddInput: epicsMessageQueueTrySend error");
	}
	return(0);
}

long recDynLinkAddOutput(recDynLink *precDynLink,char *pvname,
	short dbrType, int options, recDynCallback searchCallback)
{
	dynLinkPvt		*pdynLinkPvt;
	struct dbAddr	dbaddr;
	msgQCmd		cmd;
    
	if (options&rdlDBONLY  && db_name_to_addr(pvname,&dbaddr)) return(-1);
	if (!outTaskId) recDynLinkStartOutput();
	if (precDynLink->pdynLinkPvt) recDynLinkClear(precDynLink);
	pdynLinkPvt = (dynLinkPvt *)calloc(1,sizeof(dynLinkPvt));
	if (!pdynLinkPvt) {
		printf("recDynLinkAddOutput can't allocate storage");
		epicsThreadSuspendSelf();
	}
	pdynLinkPvt->lock = epicsMutexMustCreate();
	precDynLink->pdynLinkPvt = pdynLinkPvt;
	pdynLinkPvt->pvname = pvname;
	pdynLinkPvt->dbrType = dbrType;
	pdynLinkPvt->searchCallback = searchCallback;
	pdynLinkPvt->io = ioOutput;
	pdynLinkPvt->scalar = (options&rdlSCALAR) ? TRUE : FALSE;
	pdynLinkPvt->state = stateStarting;
	cmd.data.precDynLink = precDynLink;
	cmd.cmd = cmdSearch;
	if (epicsMessageQueueTrySend(recDynLinkOutMsgQ, (void *)&cmd, sizeof(cmd))) {
		errMessage(0,"recDynLinkAddOutput: epicsMessageQueueTrySend error");
	}
	epicsEventSignal(wakeUpEvt);
	return(0);
}

long recDynLinkClear(recDynLink *precDynLink)
{
	dynLinkPvt	*pdynLinkPvt;
	msgQCmd	cmd;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (!pdynLinkPvt) {
		printf("recDynLinkClear: recDynLinkSearch was never called\n");
		epicsThreadSuspendSelf();
	}
	if (pdynLinkPvt->chid) ca_set_puser(pdynLinkPvt->chid, NULL);
	cmd.data.pdynLinkPvt = pdynLinkPvt;
	cmd.cmd = cmdClear;
	if (pdynLinkPvt->io==ioInput) {
		if (epicsMessageQueueTrySend(recDynLinkInpMsgQ, (void *)&cmd, sizeof(cmd))) {
			errMessage(0,"recDynLinkClear: epicsMessageQueueTrySend error");
		}
	} else {
		if (epicsMessageQueueTrySend(recDynLinkOutMsgQ, (void *)&cmd, sizeof(cmd))) {
			errMessage(0,"recDynLinkClear: epicsMessageQueueTrySend error");
		}
	}
	precDynLink->pdynLinkPvt = NULL;
	precDynLink->status = 0;
	return(0);
}

long recDynLinkConnectionStatus(recDynLink *precDynLink)
{
	dynLinkPvt	*pdynLinkPvt;
	long		status;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	status = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
	return(status);
}

long recDynLinkGetNelem(recDynLink *precDynLink,size_t *nelem)
{
	dynLinkPvt  *pdynLinkPvt;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (ca_state(pdynLinkPvt->chid)!=cs_conn) return(-1);
	*nelem = ca_element_count(pdynLinkPvt->chid);
	return(0);
}

long recDynLinkGetControlLimits(recDynLink *precDynLink,
	double *low,double *high)
{
	dynLinkPvt	*pdynLinkPvt;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->state!=stateConnected) return(-1);
	if (low) *low = pdynLinkPvt->controlLow;
	if (high) *high = pdynLinkPvt->controlHigh;
	return(0);
}

long recDynLinkGetGraphicLimits(recDynLink *precDynLink,
	double *low,double *high)
{
	dynLinkPvt	*pdynLinkPvt;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->state!=stateConnected) return(-1);
	if (low) *low = pdynLinkPvt->graphicLow;
	if (high) *high = pdynLinkPvt->graphHigh;
	return(0);
}

long recDynLinkGetPrecision(recDynLink *precDynLink,int *prec)
{
	dynLinkPvt	*pdynLinkPvt;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->state!=stateConnected) return(-1);
	if (prec) *prec = pdynLinkPvt->precision;
	return(0);
}

long recDynLinkGetUnits(recDynLink *precDynLink,char *units,int maxlen)
{
	dynLinkPvt	*pdynLinkPvt;
    int			maxToCopy;

	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->state!=stateConnected) return(-1);
	maxToCopy = MAX_UNITS_SIZE;
	if (maxlen<maxToCopy) maxToCopy = maxlen;
	strncpy(units,pdynLinkPvt->units,maxToCopy);
	if (maxToCopy<maxlen) units[maxToCopy] = '\0';
	return(0);
}

long recDynLinkGet(recDynLink *precDynLink,void *pbuffer,size_t *nRequest,
	TS_STAMP *timestamp,short *status,short *severity)
{
	dynLinkPvt	*pdynLinkPvt;
	long		caStatus;

	precDynLink->status = 0;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	caStatus = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
	if (caStatus) goto all_done;
	if (*nRequest > pdynLinkPvt->nRequest) {
		*nRequest = pdynLinkPvt->nRequest;
	}
	epicsMutexMustLock(pdynLinkPvt->lock);
	memcpy(pbuffer,pdynLinkPvt->pbuffer,
		(*nRequest * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
	if (timestamp) *timestamp = pdynLinkPvt->timestamp; /*array copy*/
	if (status) *status = pdynLinkPvt->status;
	if (severity) *severity = pdynLinkPvt->severity;
	epicsMutexUnlock(pdynLinkPvt->lock);

all_done:
	return(caStatus);
}

/* for backward compatibility with recDynLink in base */
long recDynLinkPut(recDynLink *precDynLink,void *pbuffer,size_t nRequest)
{
	return(recDynLinkPutCallback(precDynLink, pbuffer, nRequest, NULL));
}

long recDynLinkPutCallback(recDynLink *precDynLink,void *pbuffer,size_t nRequest,
	recDynCallback notifyCallback)
{
	dynLinkPvt	*pdynLinkPvt;
	long		status;
	msgQCmd	cmd;

	precDynLink->status = 0;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->io!=ioOutput || pdynLinkPvt->state!=stateConnected) {
		status = -1;
	} else {
		status = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
	}
	if (status) goto all_done;
	if (notifyCallback) {
		if (pdynLinkPvt->notifyInProgress) return(NOTIFY_IN_PROGRESS);
		pdynLinkPvt->notifyCallback = notifyCallback;
	}
	if (pdynLinkPvt->scalar) nRequest = 1;
	if (nRequest>ca_element_count(pdynLinkPvt->chid))
	nRequest = ca_element_count(pdynLinkPvt->chid);
	pdynLinkPvt->nRequest = nRequest;
	memcpy(pdynLinkPvt->pbuffer,pbuffer,
		(nRequest * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
	cmd.data.precDynLink = precDynLink;
	cmd.cmd = notifyCallback ? cmdPutCallback : cmdPut;
	if (epicsMessageQueueTrySend(recDynLinkOutMsgQ, (void *)&cmd, sizeof(cmd))) {
		errMessage(0,"recDynLinkPut: epicsMessageQueueTrySend error");
		status = RINGBUFF_PUT_ERROR;
	}
	epicsEventSignal(wakeUpEvt);

all_done:
	return(status);
}

LOCAL void recDynLinkStartInput(void)
{
	recDynLinkInpMsgQ = epicsMessageQueueCreate(recDynLinkQsize, sizeof(msgQCmd));
	if (recDynLinkInpMsgQ == NULL) {
		errMessage(0,"recDynLinkStart failed");
		exit(1);
	}
	inpTaskId = epicsThreadCreate("recDynINP",epicsThreadPriorityCAServerHigh+3,
		20000, (EPICSTHREADFUNC)recDynLinkInp,0);
	if (inpTaskId==NULL) {
		errMessage(0,"recDynLinkStartInput: taskSpawn Failure\n");
	}
}

LOCAL void recDynLinkStartOutput(void)
{
	wakeUpEvt = epicsEventCreate(epicsEventEmpty);
	if (wakeUpEvt == 0)
		errMessage(0, "epicsEventCreate failed in recDynLinkStartOutput");
	recDynLinkOutMsgQ = epicsMessageQueueCreate(recDynLinkQsize, sizeof(msgQCmd));
	if (recDynLinkOutMsgQ == NULL) {
		errMessage(0,"recDynLinkStartOutput failed");
		exit(1);
	}
	outTaskId = epicsThreadCreate("recDynOUT",epicsThreadPriorityCAServerHigh+3,
		20000, (EPICSTHREADFUNC)recDynLinkOut,0);
	if (outTaskId == NULL) {
		errMessage(0,"recDynLinkStart: taskSpawn Failure\n");
	}
}

LOCAL void connectCallback(struct connection_handler_args cha)
{
	chid		chid = cha.chid;
	recDynLink	*precDynLink;
	dynLinkPvt	*pdynLinkPvt;
    
	precDynLink = (recDynLink *)ca_puser(cha.chid);
	if (!precDynLink) return;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (ca_state(chid) == cs_conn) {
		pdynLinkPvt->state = stateGetting;
		SEVCHK(ca_get_callback(DBR_CTRL_DOUBLE,chid,getCallback,precDynLink),
			"ca_get_callback");
    } else {
		if (pdynLinkPvt->searchCallback)
			(pdynLinkPvt->searchCallback)(precDynLink);
	}
}

LOCAL void getCallback(struct event_handler_args eha)
{
	struct dbr_ctrl_double	*pdata = (struct dbr_ctrl_double *)eha.dbr;
	recDynLink				*precDynLink;
	dynLinkPvt				*pdynLinkPvt;
	size_t					nRequest;
    
	precDynLink = (recDynLink *)ca_puser(eha.chid);
	if (!precDynLink) return;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	pdynLinkPvt -> graphicLow = pdata->lower_disp_limit;
	pdynLinkPvt -> graphHigh = pdata->upper_disp_limit;
	pdynLinkPvt -> controlLow = pdata->lower_ctrl_limit;
	pdynLinkPvt -> controlHigh = pdata->upper_ctrl_limit;
	pdynLinkPvt -> precision = pdata->precision;
	strncpy(pdynLinkPvt->units,pdata->units,MAX_UNITS_SIZE);
	if (pdynLinkPvt->scalar) {
		pdynLinkPvt->nRequest = 1;
	} else {
		pdynLinkPvt->nRequest = ca_element_count(pdynLinkPvt->chid);
	}
	nRequest = pdynLinkPvt->nRequest;
	pdynLinkPvt->pbuffer = calloc(nRequest,
		dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]);
	pdynLinkPvt->state = stateConnected;
	if (pdynLinkPvt->searchCallback) (pdynLinkPvt->searchCallback)(precDynLink);
	if (pdynLinkPvt->io==ioInput) {
		SEVCHK(ca_add_array_event(
			dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]),
			pdynLinkPvt->nRequest,
			pdynLinkPvt->chid,monitorCallback,precDynLink,
			0.0,0.0,0.0,
			&pdynLinkPvt->evid),"ca_add_array_event");
	}
}

LOCAL void monitorCallback(struct event_handler_args eha)
{
	recDynLink	*precDynLink;
	dynLinkPvt	*pdynLinkPvt;
	long		count = eha.count;
	const void	*pbuffer = eha.dbr;
	struct dbr_time_string	*pdbr_time_string;
	void		*pdata;
	short		timeType;
    
	precDynLink = (recDynLink *)ca_puser(eha.chid);
	if (!precDynLink) return;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->pbuffer) {
		epicsMutexMustLock(pdynLinkPvt->lock);
		if (count>=pdynLinkPvt->nRequest) count = pdynLinkPvt->nRequest;
		pdbr_time_string = (struct dbr_time_string *)pbuffer;
		timeType = dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]);
		pdata = (void *)((char *)pbuffer + dbr_value_offset[timeType]);
		pdynLinkPvt->timestamp = pdbr_time_string->stamp; /*array copy*/
		pdynLinkPvt->status = pdbr_time_string->status;
		pdynLinkPvt->severity = pdbr_time_string->severity;
		memcpy(pdynLinkPvt->pbuffer,pdata,
			(count * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
		epicsMutexUnlock(pdynLinkPvt->lock);
	}
	if (pdynLinkPvt->monitorCallback)
		(*pdynLinkPvt->monitorCallback)(precDynLink);
}

LOCAL void notifyCallback(struct event_handler_args eha)
{
	recDynLink	*precDynLink;
	dynLinkPvt	*pdynLinkPvt;
    
	precDynLink = (recDynLink *)ca_puser(eha.chid);
	if (!precDynLink) return;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt->notifyCallback) {
		pdynLinkPvt->notifyInProgress = 0;
		(pdynLinkPvt->notifyCallback)(precDynLink);
	}
}
    
LOCAL void recDynLinkInp(void)
{
	int			status, n, s = sizeof(msgQCmd);
	recDynLink	*precDynLink;
	dynLinkPvt	*pdynLinkPvt;
	msgQCmd	cmd;

	taskwdInsert(epicsThreadGetIdSelf(),NULL,NULL);
	SEVCHK(ca_context_create(ca_enable_preemptive_callback),"ca_context_create");
	while(TRUE) {
		while (epicsMessageQueuePending(recDynLinkInpMsgQ) && interruptAccept) {
			DEBUG(5,"epicsMessageQueuePending(recDynLinkInpMsgQ)=%d\n", 
				epicsMessageQueuePending(recDynLinkInpMsgQ));
			n = epicsMessageQueueReceive(recDynLinkInpMsgQ, (void *)&cmd,sizeof(cmd));
			if (n != s) {
				printf("recDynLinkInpTask: got %d bytes, expected %d\n", n, s);
				continue;
			}
			if (cmd.cmd==cmdClear) {
				pdynLinkPvt = cmd.data.pdynLinkPvt;
				if (pdynLinkPvt->chid)
					SEVCHK(ca_clear_channel(pdynLinkPvt->chid),"ca_clear_channel");
				free(pdynLinkPvt->pbuffer);
				epicsMutexDestroy(pdynLinkPvt->lock);
				free((void *)pdynLinkPvt);
				continue;
			}
			precDynLink = cmd.data.precDynLink;
			pdynLinkPvt = precDynLink->pdynLinkPvt;
			switch (cmd.cmd) {
			case (cmdSearch) :
				SEVCHK(ca_create_channel(pdynLinkPvt->pvname,
					connectCallback,precDynLink, 10 ,&pdynLinkPvt->chid),
				"ca_create_channel");
				break;
			default:
				epicsPrintf("Logic error statement in recDynLinkTask\n");
			}
		}
		if (recDynINPCallPend) {
			status = ca_pend_event(.1);
			if (status!=ECA_NORMAL && status!=ECA_TIMEOUT)
			SEVCHK(status,"ca_pend_event");
		}
	}
}

LOCAL void recDynLinkOut(void)
{
	int			status, n, s = sizeof(msgQCmd);
	recDynLink	*precDynLink;
	dynLinkPvt	*pdynLinkPvt;
	msgQCmd		cmd;
	int			caStatus;

	taskwdInsert(epicsThreadGetIdSelf(),NULL,NULL);
	SEVCHK(ca_context_create(ca_enable_preemptive_callback),"ca_context_create");
	while(TRUE) {
		epicsEventWaitWithTimeout(wakeUpEvt,1.0);
		while (epicsMessageQueuePending(recDynLinkOutMsgQ) && interruptAccept) {
			DEBUG(5,"epicsMessageQueuePending(recDynLinkOutMsgQ)=%d\n", 
				epicsMessageQueuePending(recDynLinkOutMsgQ));
			n = epicsMessageQueueReceive(recDynLinkOutMsgQ, (void *)&cmd,
				sizeof(msgQCmd));
			DEBUG(5,"recDynLinkOut: got message of size %d\n", n); 
			if (n != s) {
				printf("recDynLinkOutTask: got %d bytes, expected %d\n", n, s);
				continue;
			}
			if (cmd.cmd==cmdClear) {
				pdynLinkPvt = cmd.data.pdynLinkPvt;
				if (pdynLinkPvt->chid)
					SEVCHK(ca_clear_channel(pdynLinkPvt->chid),
						"ca_clear_channel");
				free(pdynLinkPvt->pbuffer);
				epicsMutexDestroy(pdynLinkPvt->lock);
				free((void *)pdynLinkPvt);
				continue;
			}
			precDynLink = cmd.data.precDynLink;
			pdynLinkPvt = precDynLink->pdynLinkPvt;
			switch (cmd.cmd) {
			case (cmdSearch):
				SEVCHK(ca_create_channel(pdynLinkPvt->pvname,
					connectCallback,precDynLink, 10 ,&pdynLinkPvt->chid),
					"ca_create_channel");
				break;
			case (cmdPut):
				caStatus = ca_array_put(
					mapNewToOld[pdynLinkPvt->dbrType],
					pdynLinkPvt->nRequest,pdynLinkPvt->chid,
					pdynLinkPvt->pbuffer);
				if (caStatus!=ECA_NORMAL) {
					epicsPrintf("recDynLinkTask pv=%s CA Error %s\n",
					pdynLinkPvt->pvname,ca_message(caStatus));
				}
				break;
			case (cmdPutCallback):
				pdynLinkPvt->notifyInProgress = 1;
				caStatus = ca_array_put_callback(
					mapNewToOld[pdynLinkPvt->dbrType],
					pdynLinkPvt->nRequest,pdynLinkPvt->chid,
					pdynLinkPvt->pbuffer, notifyCallback, precDynLink);
				if (caStatus!=ECA_NORMAL) {
					epicsPrintf("recDynLinkTask pv=%s CA Error %s\n",
					pdynLinkPvt->pvname,ca_message(caStatus));
					/* error indicates user won't get a callback */
					pdynLinkPvt->notifyInProgress = 0;
					precDynLink->status = FATAL_ERROR;
					(pdynLinkPvt->notifyCallback)(precDynLink);
				}
				break;
			default:
				epicsPrintf("Logic error statement in recDynLinkTask\n");
			}
		}
		if (recDynOUTCallPend) {
			status = ca_pend_event(.00001);
			if (status!=ECA_NORMAL && status!=ECA_TIMEOUT)
			SEVCHK(status,"ca_pend_event");
		}
	}
}
