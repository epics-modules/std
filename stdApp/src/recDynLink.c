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
 *
 */
#include <vxWorks.h>
#include <taskLib.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <rngLib.h>
#include <vxLib.h>
#include <semLib.h>
#include <sysLib.h>

#include <dbDefs.h>
#include <epicsPrint.h>
#include <taskwd.h>
#include <fast_lock.h>
#include <db_access.h>
#include <cadef.h>
#include <caerr.h>
#include <caeventmask.h>
#include <tsDefs.h>
#include <task_params.h>
#include "recDynLink.h"

volatile int recDynINPCallPendEvent = 1;
volatile int recDynINPCallPendEventTime_ms = 100;
volatile int recDynINPCallPendIoTime_ms = 100;
volatile int recDynOUTCallPend = 0;
volatile int recDynOUTCallFlush = 1;

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

/* Definitions to map between old and new database access*/
/* because we are using CA must include db_access.h*/
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

extern int interruptAccept;

int   recDynLinkQsize = 256;

LOCAL int	inpTaskId=0;
LOCAL int	outTaskId=0;
LOCAL RING_ID	inpRingQ;
LOCAL RING_ID	outRingQ;
LOCAL SEM_ID	wakeUpSem;

typedef enum{cmdSearch,cmdClear,cmdPut,cmdPutCallback,cmdGetCallback} cmdType;
char commands[5][15] = {"Search","Clear","Put","PutCallback","GetCallback"};
typedef enum{ioInput,ioOutput}ioType;
typedef enum{stateStarting,stateSearching,stateGetting,stateConnected}stateType;

typedef struct dynLinkPvt{
    FAST_LOCK		lock;
    char		*pvname;
    chid		chid;
    evid		evid;
    recDynCallback	searchCallback;
    recDynCallback	monitorCallback;
    recDynCallback	notifyCallback;
	recDynCallback	userGetCallback;
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

/*For cmdClear data is chid. For all other commands precDynLink*/
typedef struct {
    union {
		recDynLink	*precDynLink;
		dynLinkPvt	*pdynLinkPvt;
    } data;
    cmdType	cmd;
} ringBufCmd;

LOCAL void recDynLinkStartTasks(void);
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
    ringBufCmd		cmd;
    
	DEBUG(10,"recDynLinkAddInput: precDynLink=%p\n", precDynLink); 

    if (options&rdlDBONLY  && db_name_to_addr(pvname,&dbaddr))return(-1);
	if (!inpTaskId) recDynLinkStartTasks();
	if (precDynLink->pdynLinkPvt) {
		DEBUG(10,"recDynLinkAddInput: clearing old pdynLinkPvt\n"); 
		recDynLinkClear(precDynLink);
	}
    pdynLinkPvt = (dynLinkPvt *)calloc(1,sizeof(dynLinkPvt));
    if(!pdynLinkPvt) {
	    printf("recDynLinkAddInput can't allocate storage");
	    taskSuspend(0);
    }
    FASTLOCKINIT(&pdynLinkPvt->lock);
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
	precDynLink->onQueue++;
    if (rngBufPut(inpRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd))
		errMessage(0,"recDynLinkAddInput: rngBufPut error");
    return(0);
}

long recDynLinkAddOutput(recDynLink *precDynLink,char *pvname,
	short dbrType, int options, recDynCallback searchCallback)
{
    dynLinkPvt		*pdynLinkPvt;
    struct dbAddr	dbaddr;
    ringBufCmd		cmd;
    
	DEBUG(10,"recDynLinkAddOutput: precDynLink=%p\n", precDynLink); 

    if (options&rdlDBONLY  && db_name_to_addr(pvname,&dbaddr)) return(-1);
	if (!outTaskId) recDynLinkStartTasks();
	if (precDynLink->pdynLinkPvt) {
		DEBUG(10,"recDynLinkAddOutput: clearing old pdynLinkPvt\n"); 
		recDynLinkClear(precDynLink);
	}
    pdynLinkPvt = (dynLinkPvt *)calloc(1,sizeof(dynLinkPvt));
    if (!pdynLinkPvt) {
	    printf("recDynLinkAddOutput can't allocate storage");
	    taskSuspend(0);
    }
    FASTLOCKINIT(&pdynLinkPvt->lock);
    precDynLink->pdynLinkPvt = pdynLinkPvt;
    pdynLinkPvt->pvname = pvname;
    pdynLinkPvt->dbrType = dbrType;
    pdynLinkPvt->searchCallback = searchCallback;
    pdynLinkPvt->io = ioOutput;
    pdynLinkPvt->scalar = (options&rdlSCALAR) ? TRUE : FALSE;
    pdynLinkPvt->state = stateStarting;
    cmd.data.precDynLink = precDynLink;
    cmd.cmd = cmdSearch;
	precDynLink->onQueue++;
    if (rngBufPut(outRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd)) {
		errMessage(0,"recDynLinkAddOutput: rngBufPut error");
		precDynLink->onQueue--;
	}
    semGive(wakeUpSem);
    return(0);
}

long recDynLinkClear(recDynLink *precDynLink)
{
    dynLinkPvt	*pdynLinkPvt;
    ringBufCmd	cmd;

	DEBUG(10,"recDynLinkClear: precDynLink=%p\n", precDynLink);
    pdynLinkPvt = precDynLink->pdynLinkPvt;
    if (!pdynLinkPvt) {
		printf("recDynLinkClear. recDynLinkSearch was never called\n");
		taskSuspend(0);
    }
    if (pdynLinkPvt->chid) ca_puser(pdynLinkPvt->chid) = NULL;
    cmd.data.pdynLinkPvt = pdynLinkPvt;
    cmd.cmd = cmdClear;
	if (precDynLink->onQueue) {
		DEBUG(1,"recDynLinkClear: waiting for queued action on %s\n", pdynLinkPvt->pvname);
		while (precDynLink->onQueue) semGive(wakeUpSem);
	}
    if (pdynLinkPvt->io==ioInput) {
		if (rngBufPut(inpRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd))
		    errMessage(0,"recDynLinkClear: rngBufPut error");
    } else {
		if(rngBufPut(outRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd))
		    errMessage(0,"recDynLinkClear: rngBufPut error");
    }
    precDynLink->pdynLinkPvt = NULL;
    precDynLink->status = 0;
    return(0);
}

long recDynLinkConnectionStatus(recDynLink *precDynLink)
{
    dynLinkPvt  *pdynLinkPvt;
    long	status;

	if (precDynLink == NULL) return(-1);
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if ((pdynLinkPvt == NULL) || (pdynLinkPvt->chid == NULL)) return(-1);
    status = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
    return(status);
}

long recDynLinkGetNelem(recDynLink *precDynLink,size_t *nelem)
{
    dynLinkPvt  *pdynLinkPvt;

	if (precDynLink == NULL) return(-1);
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if ((pdynLinkPvt == NULL) || (pdynLinkPvt->chid == NULL)) return(-1);
    if (ca_state(pdynLinkPvt->chid)!=cs_conn) return(-1);
    *nelem = ca_element_count(pdynLinkPvt->chid);
    return(0);
}

long recDynLinkGetControlLimits(recDynLink *precDynLink,
	double *low,double *high)
{
    dynLinkPvt  *pdynLinkPvt;

    pdynLinkPvt = precDynLink->pdynLinkPvt;
    if (pdynLinkPvt->state!=stateConnected) return(-1);
    if (low) *low = pdynLinkPvt->controlLow;
    if (high) *high = pdynLinkPvt->controlHigh;
    return(0);
}

long recDynLinkGetGraphicLimits(recDynLink *precDynLink,
	double *low,double *high)
{
    dynLinkPvt  *pdynLinkPvt;

    pdynLinkPvt = precDynLink->pdynLinkPvt;
    if (pdynLinkPvt->state!=stateConnected) return(-1);
    if (low) *low = pdynLinkPvt->graphicLow;
    if (high) *high = pdynLinkPvt->graphHigh;
    return(0);
}

long recDynLinkGetPrecision(recDynLink *precDynLink,int *prec)
{
    dynLinkPvt  *pdynLinkPvt;

    pdynLinkPvt = precDynLink->pdynLinkPvt;
    if (pdynLinkPvt->state!=stateConnected) return(-1);
    if (prec) *prec = pdynLinkPvt->precision;
    return(0);
}

long recDynLinkGetUnits(recDynLink *precDynLink,char *units,int maxlen)
{
    dynLinkPvt  *pdynLinkPvt;
    int		maxToCopy;

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
	long		caStatus, save_nRequest = *nRequest;

	if (precDynLink == NULL) return(-1);
	precDynLink->status = 0;
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if ((pdynLinkPvt == NULL) || (pdynLinkPvt->chid == NULL)) return(-1);
    caStatus = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
    if (caStatus) goto all_done;
    if (*nRequest > pdynLinkPvt->nRequest) {
		*nRequest = pdynLinkPvt->nRequest;
    }
    FASTLOCK(&pdynLinkPvt->lock);
    memcpy(pbuffer,pdynLinkPvt->pbuffer,
		(*nRequest * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
	DEBUG(5,"recDynLinkGet: PV=%s, user asked for=%ld, got %d\n", pdynLinkPvt->pvname,
		save_nRequest, *nRequest);
    if (timestamp) *timestamp = pdynLinkPvt->timestamp; /*array copy*/
    if (status) *status = pdynLinkPvt->status;
    if (severity) *severity = pdynLinkPvt->severity;
    FASTUNLOCK(&pdynLinkPvt->lock);
all_done:
    return(caStatus);
}

long recDynLinkGetCallback(recDynLink *precDynLink, size_t *nRequest,
	recDynCallback userGetCallback)
{
	dynLinkPvt	*pdynLinkPvt;
	long		status;
    ringBufCmd	cmd;

	if (precDynLink == NULL) return(-1);
	precDynLink->status = 0;
	precDynLink->getCallbackInProgress = 1;
	pdynLinkPvt = precDynLink->pdynLinkPvt;
	if ((pdynLinkPvt == NULL) || (pdynLinkPvt->chid == NULL)) return(-1);
	if (pdynLinkPvt->io!=ioInput || pdynLinkPvt->state!=stateConnected) {
		status = -1;
	} else {
		status = (ca_state(pdynLinkPvt->chid)==cs_conn) ? 0 : -1;
	}
	if (status) goto all_done;
	if (userGetCallback) pdynLinkPvt->userGetCallback = userGetCallback;
	if (*nRequest>ca_element_count(pdynLinkPvt->chid))
		*nRequest = ca_element_count(pdynLinkPvt->chid);
	pdynLinkPvt->nRequest = *nRequest;
	cmd.data.precDynLink = precDynLink;
	cmd.cmd = cmdGetCallback;
	precDynLink->onQueue++;
	DEBUG(5,"recDynLinkGetCallback: PV=%s, nRequest=%d\n", pdynLinkPvt->pvname,
		pdynLinkPvt->nRequest); 
	/*
	* Both the input and output tasks support the command 'cmdGetCallback'.
	* The output task is better suited to performing the command, but the
	* input task did the ca_search and maintains the monitor.  In 3.14, the
	* tasks share a CA context, so it's ok to have the output task do the
	* getCallback.  In 3.13, I don't know precisely what the rules are, but it
	* seems most reasonable that the input task should do the getCallback.
	*/
    if (rngBufPut(inpRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd)) {
		errMessage(0,"recDynLinkGetCallback: rngBufPut error");
		status = RINGBUFF_PUT_ERROR;
		precDynLink->onQueue--;
	}
    semGive(wakeUpSem);

all_done:
	return(status);
}

/* for backward compatibility with recDynLink in base */
long recDynLinkPut(recDynLink *precDynLink,void *pbuffer,size_t nRequest)
{
	return(recDynLinkPutCallback(precDynLink, pbuffer, nRequest, NULL));
}

long recDynLinkPutCallback(recDynLink *precDynLink,void *pbuffer,size_t nRequest, recDynCallback notifyCallback)
{
    dynLinkPvt	*pdynLinkPvt;
    long	status;
    ringBufCmd	cmd;

	if (precDynLink == NULL) return(-1);
	precDynLink->status = 0;
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt == NULL) return(-1);
    if (pdynLinkPvt->io!=ioOutput || pdynLinkPvt->state!=stateConnected) {
		status = -1;
    } else {
		if (pdynLinkPvt->chid == NULL) return(-1);
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
	precDynLink->onQueue++;
    if (rngBufPut(outRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd)) {
	    errMessage(0,"recDynLinkPut: rngBufPut error");
		status = RINGBUFF_PUT_ERROR;
		precDynLink->onQueue--;
	}
    semGive(wakeUpSem);
all_done:
    return(status);
}

LOCAL void recDynLinkStartTasks(void)
{
    if ((inpRingQ = rngCreate(sizeof(ringBufCmd) * recDynLinkQsize)) == NULL) {
		errMessage(0,"recDynLinkStart failed");
		exit(1);
    }
    inpTaskId = taskSpawn("recDynINP",CA_CLIENT_PRI-1,VX_FP_TASK,
	CA_CLIENT_STACK,(FUNCPTR)recDynLinkInp,0,0,0,0,0,0,0,0,0,0);
    if (inpTaskId==ERROR) {
		errMessage(0,"recDynLinkStartInput: taskSpawn Failure\n");
    }
    if ((wakeUpSem=semBCreate(SEM_Q_FIFO,SEM_EMPTY))==NULL)
		errMessage(0,"semBcreate failed in recDynLinkStart");
    if ((outRingQ = rngCreate(sizeof(ringBufCmd) * recDynLinkQsize)) == NULL) {
		errMessage(0,"recDynLinkStartOutput failed");
		exit(1);
    }
    outTaskId = taskSpawn("recDynOUT",CA_CLIENT_PRI-1,VX_FP_TASK,
		CA_CLIENT_STACK,(FUNCPTR)recDynLinkOut,0,0,0,0,0,0,0,0,0,0);
    if (outTaskId==ERROR) {
		errMessage(0,"recDynLinkStart: taskSpawn Failure\n");
    }
}

LOCAL void connectCallback(struct connection_handler_args cha)
{
    chid	chid = cha.chid;
    recDynLink	*precDynLink;
    dynLinkPvt	*pdynLinkPvt;
    
    precDynLink = (recDynLink *)ca_puser(cha.chid);
    if (!precDynLink) return;
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (pdynLinkPvt == NULL) return;
	if (chid && (ca_state(chid) == cs_conn)) {
		pdynLinkPvt->state = stateGetting;
		SEVCHK(ca_get_callback(DBR_CTRL_DOUBLE,chid,getCallback,precDynLink),
			"ca_get_callback");
    } else {
		if(pdynLinkPvt->searchCallback)
			(pdynLinkPvt->searchCallback)(precDynLink);
    }
}

LOCAL void getCallback(struct event_handler_args eha)
{
    struct dbr_ctrl_double	*pdata = (struct dbr_ctrl_double *)eha.dbr;
    recDynLink			*precDynLink;
    dynLinkPvt			*pdynLinkPvt;
    size_t			nRequest;
    
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
		if (recDynLinkDebug >= 5)
			printf("recDynLink:getCallback: array of %d elements\n", pdynLinkPvt->nRequest);
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
    long	count = eha.count;
    const void	*pbuffer = eha.dbr;
    struct dbr_time_string *pdbr_time_string;
    void	*pdata;
    short	timeType;
	char		*pchar;
	short		*pshort;
	long		*plong;
	float		*pfloat;
	double		*pdouble;
    
    precDynLink = (recDynLink *)ca_puser(eha.chid);
    if (!precDynLink) return;
    pdynLinkPvt = precDynLink->pdynLinkPvt;
	if (recDynLinkDebug >= 5) {
		printf("recDynLink:monitorCallback:  PV=%s, nRequest=%d\n",
			pdynLinkPvt->pvname, pdynLinkPvt->nRequest);
	}
    if (pdynLinkPvt->pbuffer) {
		FASTLOCK(&pdynLinkPvt->lock);
		if(count>=pdynLinkPvt->nRequest)
			count = pdynLinkPvt->nRequest;
		pdbr_time_string = (struct dbr_time_string *)pbuffer;
		timeType = dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]);
		pdata = (void *)((char *)pbuffer + dbr_value_offset[timeType]);
		pdynLinkPvt->timestamp = pdbr_time_string->stamp; /*array copy*/
		pdynLinkPvt->status = pdbr_time_string->status;
		pdynLinkPvt->severity = pdbr_time_string->severity;
		memcpy(pdynLinkPvt->pbuffer,pdata,
			(count * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
		FASTUNLOCK(&pdynLinkPvt->lock);
		if ((count > 1) && (recDynLinkDebug >= 5)) {
			printf("recDynLink:monitorCallback: array of %d elements\n", pdynLinkPvt->nRequest);
			switch (mapNewToOld[pdynLinkPvt->dbrType]) {
			case DBF_STRING: case DBF_CHAR:
				pchar = (char *)pdata;
				printf("...char/string: %c, %c, %c...\n", pchar[0], pchar[1], pchar[2]);
				break;
			case DBF_SHORT: case DBF_ENUM:
				pshort = (short *)pdata;
				printf("...short: %d, %d, %d...\n", pshort[0], pshort[1], pshort[2]);
				break;
			case DBF_LONG:
				plong = (long *)pdata;
				printf("...long: %ld, %ld, %ld...\n", plong[0], plong[1], plong[2]);
				break;
			case DBF_FLOAT:
				pfloat = (float *)pdata;
				printf("...float: %f, %f, %f...\n", pfloat[0], pfloat[1], pfloat[2]);
				break;
			case DBF_DOUBLE:
				pdouble = (double *)pdata;
				printf("...double: %f, %f, %f...\n", pdouble[0], pdouble[1], pdouble[2]);
				break;
			default:
				pchar = (char *)pdata;
				printf("...unknown type: %x, %x, %x...\n", pchar[0], pchar[1], pchar[2]);
				break;
			}
		}
    }
    if (pdynLinkPvt->monitorCallback)
		(*pdynLinkPvt->monitorCallback)(precDynLink);
}

LOCAL void userGetCallback(struct event_handler_args eha)
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
	if (recDynLinkDebug >= 5) {
		printf("recDynLink:userGetCallback:  PV=%s, nRequest=%d\n",
			pdynLinkPvt->pvname, pdynLinkPvt->nRequest);
	}
	if (pdynLinkPvt->pbuffer) {
		FASTLOCK(&pdynLinkPvt->lock);
		if (count>=pdynLinkPvt->nRequest) count = pdynLinkPvt->nRequest;
		pdbr_time_string = (struct dbr_time_string *)pbuffer;
		timeType = dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]);
		pdata = (void *)((char *)pbuffer + dbr_value_offset[timeType]);
		pdynLinkPvt->timestamp = pdbr_time_string->stamp; /*array copy*/
		pdynLinkPvt->status = pdbr_time_string->status;
		pdynLinkPvt->severity = pdbr_time_string->severity;
		memcpy(pdynLinkPvt->pbuffer,pdata,
			(count * dbr_size[mapNewToOld[pdynLinkPvt->dbrType]]));
		FASTUNLOCK(&pdynLinkPvt->lock);
	}
	if (pdynLinkPvt->userGetCallback)
		(*pdynLinkPvt->userGetCallback)(precDynLink);
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
    int		status;
    recDynLink	*precDynLink;
    dynLinkPvt	*pdynLinkPvt;
    ringBufCmd	cmd;
	int			didGetCallback=0;

    taskwdInsert(taskIdSelf(),NULL,NULL);
    SEVCHK(ca_task_initialize(),"ca_task_initialize");
    while(TRUE) {
		didGetCallback = 0;
		while (rngNBytes(inpRingQ)>=sizeof(cmd) && interruptAccept){
		    if (rngBufGet(inpRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd)) {
				errMessage(0,"recDynLinkTask: rngBufGet error");
				continue;
		    }
		    if (cmd.cmd==cmdClear) {
				pdynLinkPvt = cmd.data.pdynLinkPvt;
				if(pdynLinkPvt->chid)
				    SEVCHK(ca_clear_channel(pdynLinkPvt->chid),
					"ca_clear_channel");
				free(pdynLinkPvt->pbuffer);
				free((void *)pdynLinkPvt);
				continue;
		    }
		    precDynLink = cmd.data.precDynLink;
		    pdynLinkPvt = precDynLink->pdynLinkPvt;
			DEBUG(5,"recDynLinkInp: precDynLink=%p", precDynLink); 
			if (pdynLinkPvt==NULL) {
				printf("\n***ERROR***: pdynLinkPvt=%p\n", pdynLinkPvt);
				precDynLink->onQueue--;
				continue;
			} else {
				DEBUG(5,", pvname=%s\n", pdynLinkPvt->pvname);
			}
		    switch(cmd.cmd) {
		    case(cmdSearch) :
				SEVCHK(ca_search_and_connect(pdynLinkPvt->pvname,
				    &pdynLinkPvt->chid, connectCallback,precDynLink),
				    "ca_search_and_connect");
				precDynLink->onQueue--;
				break;
			case (cmdGetCallback):
				didGetCallback = 1;
				status = ca_array_get_callback(
					dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]),
					pdynLinkPvt->nRequest,pdynLinkPvt->chid, userGetCallback, precDynLink);
				if (status!=ECA_NORMAL) {
					epicsPrintf("recDynLinkTask pv=%s CA Error %s\n",
						pdynLinkPvt->pvname,ca_message(status));
					/* error indicates user won't get a callback, so we do it */
					precDynLink->status = FATAL_ERROR;
					(pdynLinkPvt->userGetCallback)(precDynLink);
				}
				precDynLink->onQueue--;
				break;
		    default:
				epicsPrintf("Logic error statement in recDynLinkTask\n");
				precDynLink->onQueue--;
		    }
		}
		if (didGetCallback) {
			status = ca_pend_io(recDynINPCallPendIoTime_ms/1000.);
			if (status!=ECA_NORMAL && status!=ECA_TIMEOUT)
			SEVCHK(status,"ca_pend_io");
		} else if (recDynINPCallPendEvent) {
			status = ca_pend_event(recDynINPCallPendEventTime_ms/1000. + 1.e-5);
			if (status!=ECA_NORMAL && status!=ECA_TIMEOUT)
			SEVCHK(status,"ca_pend_event");
		}
    }
}

/*
 * Note that we're a lower priority process than any expected caller,
 * so if caller should rapidly queue a bunch of cmdGetCallback's, then we
 * normally will not start processing them until the whole bunch has been
 * queued, and we'll stay in the "while (epicsMessageQueuePending)" loop
 * until the whole bunch has been dispatched, so caller needn't worry
 * about packaging messages into a group.
 */
 LOCAL void recDynLinkOut(void)
{
    int		status;
    recDynLink	*precDynLink;
    dynLinkPvt	*pdynLinkPvt;
    ringBufCmd	cmd;
    int		caStatus;

	taskwdInsert(taskIdSelf(),NULL,NULL);
	SEVCHK(ca_task_initialize(),"ca_task_initialize");
	while(TRUE) {
		semTake(wakeUpSem,sysClkRateGet());
		while (rngNBytes(outRingQ)>=sizeof(cmd) && interruptAccept) {
			if (rngBufGet(outRingQ,(void *)&cmd,sizeof(cmd)) != sizeof(cmd)) {
				errMessage(0,"recDynLinkTask: rngBufGet error");
				continue;
			}
			if (cmd.cmd==cmdClear) {
				pdynLinkPvt = cmd.data.pdynLinkPvt;
				if (pdynLinkPvt->chid)
					SEVCHK(ca_clear_channel(pdynLinkPvt->chid),
						"ca_clear_channel");
				free(pdynLinkPvt->pbuffer);
				free((void *)pdynLinkPvt);
				continue;
			}
			precDynLink = cmd.data.precDynLink;
			pdynLinkPvt = precDynLink->pdynLinkPvt;
			switch(cmd.cmd) {
			case (cmdSearch):
				SEVCHK(ca_search_and_connect(pdynLinkPvt->pvname,
				&pdynLinkPvt->chid, connectCallback,precDynLink),
				"ca_search_and_connect");
				precDynLink->onQueue--;
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
				precDynLink->onQueue--;
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
					/* error indicates user won't get a callback, so we do it */
					pdynLinkPvt->notifyInProgress = 0;
					precDynLink->status = FATAL_ERROR;
					(pdynLinkPvt->notifyCallback)(precDynLink);
				}
				precDynLink->onQueue--;
				break;
			case (cmdGetCallback):
				DEBUG(5,"recDynLinkOut: GetCallback PV=%s, nRequest=%d\n",
					pdynLinkPvt->pvname, pdynLinkPvt->nRequest); 

				status = ca_array_get_callback(
					dbf_type_to_DBR_TIME(mapNewToOld[pdynLinkPvt->dbrType]),
					pdynLinkPvt->nRequest,pdynLinkPvt->chid, userGetCallback, precDynLink);
				if (status!=ECA_NORMAL) {
					epicsPrintf("recDynLinkTask pv=%s CA Error %s\n",
						pdynLinkPvt->pvname,ca_message(status));
					/* error indicates user won't get a callback, so we do it */
					precDynLink->status = FATAL_ERROR;
					(pdynLinkPvt->userGetCallback)(precDynLink);
				}
				precDynLink->onQueue--;
				break;
			default:
				epicsPrintf("Logic error statement in recDynLinkTask\n");
				precDynLink->onQueue--;
			}
		}
		if (recDynOUTCallPend) {
			status = ca_pend_event(.00001);
			if(status!=ECA_NORMAL && status!=ECA_TIMEOUT)
			SEVCHK(status,"ca_pend_event");
		}
    }
}

