/* devHeidVRZ460.c */

/* Device Suport Routines for Heidenhain encoder */
/*
 *      Original Author: Greg Nawrocki
 *      Code Abused By: Dave Reid
 *      Date: 4/22/93 
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *****************************************************************
 *                         COPYRIGHT NOTIFICATION
 *****************************************************************

 * THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
 * AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
 * AND IN ALL SOURCE LISTINGS OF THE CODE.

 * (C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

 * Argonne National Laboratory (ANL), with facilities in the States of
 * Illinois and Idaho, is owned by the United States Government, and
 * operated by the University of Chicago under provision of a contract
 * with the Department of Energy.

 * Portions of this material resulted from work developed under a U.S.
 * Government contract and are subject to the following license:  For
 * a period of five years from March 30, 1993, the Government is
 * granted for itself and others acting on its behalf a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, and perform
 * publicly and display publicly.  With the approval of DOE, this
 * period may be renewed for two additional five year periods.
 * Following the expiration of this period or periods, the Government
 * is granted for itself and others acting on its behalf, a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, distribute copies
 * to the public, perform publicly and display publicly, and to permit
 * others to do so.

 *****************************************************************
 *                               DISCLAIMER
 *****************************************************************

 * NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
 * THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
 * MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
 * LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
 * USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
 * DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
 * OWNED RIGHTS.

 *****************************************************************
 * LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
 * DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
 *****************************************************************

 * Modification Log:
 * -----------------
 * .01  11-18-93        gjn     initialized
 * .02  04-22-94	ddr	modified for heidenhain encoder
 *      ...
 */



#include	<vxWorks.h>
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<string.h>
#include	<math.h>

#include	<alarm.h>
#include        <callback.h>
#include	<dbRecType.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<fast_lock.h>
#include        <recSup.h>
#include	<devSup.h>
#include	<drvSup.h>
#include	<dbScan.h>
#include	<special.h>
#include        <link.h>
#include	<module_types.h>
#include	<eventRecord.h>
#include	<drvBitBusInterface.h>

#include	<aiRecord.h>
#include	<biRecord.h>
#include	<boRecord.h>

/* commands */
#define HEIDCOMMAND    0x60 

/* types */
#define BI		0x01
#define BO		0x02
#define AI		0x04

/* Create the dsets for devHeidVRZ460 */
static long gen_init();
static long init_ai();
static long init_bo();
static long init_bi();
static double disp_to_deg();	/* function to convert 1st 2 bytes to degrees */
static double disp_to_min();	/* function to convert 2nd 2 bytes to minutes */
static double disp_to_sec();	/* function to convert 3rd 2 bytes to seconds */
static double mesformat();	/* function to decide format of messages      */
static long get_value();	/* get the values			      */

/* The following is the structure of the record support dsets */
typedef struct {
	long		number;
	DEVSUPFUN	resignal;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_heid_info;
	DEVSUPFUN	get_heid;
} IODSET;

/* Example dset */
/* xxDSET devXxxx={ 5,resignal,init,init_rec,get_newport_info,read_write}; */

/* Create the dsets for devNewport to use BI, BO, and AI records */
IODSET devBiHeidVRZ460 = { 5, NULL, NULL, init_bi, NULL, get_value };
IODSET devAiHeidVRZ460 = { 6, NULL, NULL, init_ai, NULL, get_value };

/* The following is the structure of the Parm Table */
typedef struct {
        char*   parmName;
        char*   parmString;
        double  (*parmFunct)(void*, int, int);
	int     parmFlg1;
	int     parmFlg2;
        } PARMTABLE;

/*                 NOTE: The parm table is in the following form                         */

/* {"Identification parameter", "String to send", Conversion function to call            */
/*   (NULL for none) Integer parameter 1 to pass into conversion function (0 for none),  */
/*   Integer parameter 2 to pass into conversion function (0 for none)}                  */   

/******************************* Parm Table *************************************/

PARMTABLE HeidVRZ460parm[] = {
                          {"format", "\x02", mesformat,   0, 0}, 
			  {"deg", "\x02", disp_to_deg, 0, 0},
			  {"min", "\x02", disp_to_min, 0, 0},
			  {"sec", "\x02", disp_to_sec, 0, 0},
			  NULL
                          };

/********************************************************************************/

/************************* devNewportDebug information **************************/

unsigned char     devHeidVRZ460Debug = 0x00;

/* Note: Bit 0 is the right most bit, bit 7 is the left most bit */

/** devHeidVRZ460Debug Bit 0 -- initialization information **/
/** devHeidVRZ460Debug Bit 1 -- errors **/
/** devHeidVRZ460Debug Bit 2 -- messages sent **/
/** devHeidVRZ460Debug Bit 3 -- messages received **/
/** devHeidVRZ460Debug Bit 4 -- conversion function information **/
/** devHeidVRZ460Debug Bit 5 -- calls to process **/

/********************************************************************************/

/* forward references */
static void get_data();
static void send_cntl_trans();
extern struct drvBitBusEt drvBitBus;

struct dprivate {
	struct dpvtBitBusHead bitbus;
	char type;
	unsigned char cmd;
	struct dbCommon	*precord; /* at end for callback to get it */
	int parmIndex;
};

/*************************** I/O Callback Function ******************************/

static int io_callback(struct dprivate *pcallback)
{
	struct dbCommon *precord;
	struct rset     *prset;

	precord=pcallback->precord;
	prset=(struct rset *)(precord->rset);

	dbScanLock(precord);
	(*prset->process)(precord);
	dbScanUnlock(precord);
}

/********************************************************************************/

/************************ Generalized Init Function *****************************/

static long gen_init (struct dbCommon *io, struct link* link, int recordCode)
{
   struct bitbusio *pbitbusio = (struct bitbusio *)(&link->value);
   struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
   int i;
   int length = 0;
   int messageCnt = 0;

        /* type must be an BITBUS_IO */
        if(link->type!=BITBUS_IO)
        {

                if (devHeidVRZ460Debug & 0x02)
                   printf("devHeidVRZ460.c : Illegal Bus Type\n");
                 
		recGblRecordError(S_dev_badBus,(void *)io,
		    "devHeidVRZ460 (init_record) Illegal IN Bus Type");
		return(S_dev_badBus);
	}

        /********** Find the proper parameter **********/

        for(i=0; HeidVRZ460parm[i].parmName && strcmp(HeidVRZ460parm[i].parmName, pbitbusio->parm); i++);

        if (!(HeidVRZ460parm[i].parmName))
        {
          if (devHeidVRZ460Debug & 0x02)
              printf("devHeidVRZ460.c : Illegal Parameter Type\n");
           recGblRecordError(S_dev_badBus,(void *)io, "devHeidVRZ460 (init_record) Illegal Parameter Type");
           return(S_dev_badBus);
        } 

        /**** If a proper parameter build the device private structure ****/

        my_dpvt=(struct dprivate *)(malloc(sizeof(struct dprivate)));

	/**** Set the parameter index ****/
        my_dpvt->parmIndex = i;

	/**** Pack the BITBUS message (not the data yet) ****/
        my_dpvt->bitbus.finishProc = io_callback;
        my_dpvt->bitbus.psyncSem = (SEM_ID *)NULL;
        my_dpvt->bitbus.priority = io->prio;
        my_dpvt->bitbus.link = pbitbusio->link;
        my_dpvt->bitbus.ageLimit = 120;
        my_dpvt->bitbus.txMsg.route = 0x40;
        my_dpvt->bitbus.txMsg.node = pbitbusio->node;
        my_dpvt->bitbus.txMsg.tasks = 1;
        my_dpvt->bitbus.txMsg.data = (unsigned char *)malloc(BB_MAX_DAT_LEN);
	my_dpvt->bitbus.txMsg.cmd = (HEIDCOMMAND | (pbitbusio->port & 0x01)); 
	my_dpvt->type = recordCode;

        my_dpvt->bitbus.rxMaxLen=BB_MAX_MSG_LENGTH;
        my_dpvt->bitbus.rxMsg.data=(unsigned char *)malloc(BB_MAX_DAT_LEN);

        if (devHeidVRZ460Debug & 0x01)
           printf("devHeidVRZ460.c : Successful Initialization  Record ->%s<-\n", io->name);

        /**** Pack the BITBUS message data and set the message length ****/

        for(i = 0; i <= 12; i++)
        {
           my_dpvt->bitbus.txMsg.data[messageCnt++] = HeidVRZ460parm[my_dpvt->parmIndex].parmString[i];
           if (HeidVRZ460parm[my_dpvt->parmIndex].parmString[i] == 0x02)
              break;
        }

        my_dpvt->bitbus.txMsg.length = messageCnt+7;

        if (devHeidVRZ460Debug & 0x01)
        {
           printf("devHeidVRZ460.c : Initialization: BITBUS message packed\n");
           for(i = 0; i < messageCnt; i++)
              printf("  BITBUS byte %d of sending message ->0x%02.2X<- ->%s<-\n", i, (unsigned char)my_dpvt->bitbus.txMsg.data[i], io->name);
           printf("  BITBUS data string length ->%d<- ->%s<-\n", messageCnt, io->name);
        }

        io->dpvt = my_dpvt;

	return(0);
}

/********************************************************************************/

/***************** Init functions specific to record types  *********************/

static long init_ai(struct aiRecord* ai){gen_init((struct dbCommon*)ai, &ai->inp, AI);}
static long init_bi(struct biRecord* bi){gen_init((struct dbCommon*)bi, &bi->inp, BI);}

/********************************************************************************/

/****************************** Call to process *********************************/

static long get_value(struct dbCommon *io)
{
	struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
	struct dpvtBitBusHead *pbitbus;

        if (devHeidVRZ460Debug & 0x20)
           printf("devHeidVRZ460.c : get_value entered\n");

	/*** If no initialization of variable, don't process ***/
	if(!io->dpvt) 
        {
           if (devHeidVRZ460Debug & 0x02)
              printf("devHeidVRZ460.c : io->dpvt = 0\n");
           return(S_dev_NoInit);
        }

	pbitbus=&(my_dpvt->bitbus);

	if(io->pact==TRUE)
	{
 
		/*** A transaction to BITBUS device has been completed ****/
		switch(pbitbus->status)
		{
		case BB_OK:
		   if (devHeidVRZ460Debug & 0x02)
		   {

	              printf("devHeidVRZ460.c: pbitbus->rxMsg.cmd is ->0x%02.2X<- ->%s<-\n", (unsigned char)my_dpvt->bitbus.rxMsg.cmd, io->name);
		      printf("devHeidVRZ460.c: pbitbus->rxMsg.length (data only) is ->%d<- ->%s<-\n",
			 (unsigned char)my_dpvt->bitbus.rxMsg.length - 7, io->name);
                   }

                   if (pbitbus->rxMsg.cmd == 0x20)
	           {
		      if (devHeidVRZ460Debug & 0x08)
		         printf("devHeidVRZ460.c : calling get_data\n");
                      get_data(io);

                      /**** Call conversion function ***/ 
		      return((*HeidVRZ460parm[my_dpvt->parmIndex].parmFunct)(io, (HeidVRZ460parm[my_dpvt->parmIndex].parmFlg1),
                               (HeidVRZ460parm[my_dpvt->parmIndex].parmFlg2)));
                   }
                   else
                   {
                      if (devHeidVRZ460Debug & 0x02)
		         printf("devHeidVRZ460.c : setting alarms\n");
                      switch(my_dpvt->type)
                      {
                         case AI:
                         case BI:
                            recGblSetSevr(io,READ_ALARM,INVALID_ALARM);
                            break;
                         default:
                            recGblSetSevr(io,WRITE_ALARM,INVALID_ALARM); 
                            break; 
                      }
		      return(0);
                   }

                case BB_TIMEOUT:
                        if (devHeidVRZ460Debug & 0x02)
                           printf("devIObug.c: BB_TIMEOUT, resending transaction \n");
                        send_cntl_trans(io);
                        break;

                default:
                        recGblSetSevr(io,WRITE_ALARM,MAJOR_ALARM);
                        recGblRecordError(S_dev_badBus,(void *)io,
                           "devHeidVRZ460.c (init_record) Initial BITBUS message failed");
                        if (devHeidVRZ460Debug & 0x02)
                           printf("devHeidVRZ460.c: ********** Unknown BITBUS Error **********\n");
                        return(S_dev_badBus);
                }
	}
        
        /*** A transaction to BITBUS device needs to be sent out ***/
        else
        {
           if (devHeidVRZ460Debug & 0x04)
              printf("devHeidVRZ460.c: calling send_cntl_trans \n");
           send_cntl_trans(io);
           io->pact=TRUE;
        }

        /*** Queue the command ***/
        if((*drvBitBus.qReq)(pbitbus,BB_Q_LOW)<0)
        {
                recGblSetSevr(io,WRITE_ALARM,MAJOR_ALARM);
                recGblRecordError(S_dev_badBus,(void *)io,
                   "devHeidVRZ460.c (init_record) Initial BITBUS message failed");
                if (devHeidVRZ460Debug & 0x04)
                   printf("devHeidVRZ460.c: Initial BITBUS message failed\n");
                return(S_dev_badBus);
        }

	return(0);
}

/********************************************************************************/

/******************* Function to receive a BITBUS message ***********************/

static void get_data(struct dbCommon *io)
{
        struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
        struct dpvtBitBusHead *pbitbus;
	struct boRecord *bo;
	struct biRecord *bi;
	struct aiRecord *ai;
        int    messageCnt = 0;
        int    errorFlg = 0;
        int    i;
 
        pbitbus=&(my_dpvt->bitbus);

        if (devHeidVRZ460Debug & 0x20)
           {
	   printf("devHeidVRZ460.c: get_data entered\n");

        for(i = 0; i < my_dpvt->bitbus.rxMsg.length - 7; i++)
	   printf("devHeidVRZ460.c: BITBUS byte %d of receiving message ->0x%02.2X<- ->%s<-\n", i, (unsigned char)my_dpvt->bitbus.rxMsg.data[i], io->name);
           }	   
	return;
}

/********************************************************************************/

/******************* Function to send out BITBUS message ************************/

static void send_cntl_trans(struct dbCommon *io)
{
        struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
        struct dpvtBitBusHead *pbitbus;
	struct boRecord *bo;
	struct aoRecord *ao;
        int i;
 
	int messageCnt = 0;    

        pbitbus=&(my_dpvt->bitbus);

       if (devHeidVRZ460Debug & 0x20)
	   printf("devHeidVRZ460.c: send_cntl_trans entered\n");

	pbitbus->finishProc=io_callback;
	pbitbus->psyncSem=(SEM_ID *)NULL;
	pbitbus->priority=io->prio;
	my_dpvt->precord=(struct dbCommon *)io;

	if (devHeidVRZ460Debug & 0x04)
	   printf("devHeidVRZ460.c: BITBUS message sent ->%s<-\n", io->name);

	return;
}

/********************************************************************************/

/************ Function to convert readback into degrees **************/

static double disp_to_deg(struct aiRecord *io, int flg1, int flg2)
{
	struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
	struct dpvtBitBusHead *pbitbus;

/*	printf("deg.mm.ss entered\n");     */
        if (my_dpvt->bitbus.rxMsg.data[8] == 0x2E)
	   {
	   printf("deg.mm.ss entered\n");
	   printf("2 = %x",my_dpvt->bitbus.rxMsg.data[2]);
           printf("3 = %x",my_dpvt->bitbus.rxMsg.data[3]);
           printf("4 = %x",my_dpvt->bitbus.rxMsg.data[4]);

	   io->val = (double)((((my_dpvt->bitbus.rxMsg.data[2]) - 0x30) * 100.0) +
		(((my_dpvt->bitbus.rxMsg.data[3])-0x30) *10.0) +
		((my_dpvt->bitbus.rxMsg.data[4])-0x30)); 	
		}

else
           {

           if(my_dpvt->bitbus.rxMsg.data[2] == 0x20)
              my_dpvt->bitbus.rxMsg.data[2] = 0x30;
           if(my_dpvt->bitbus.rxMsg.data[3] == 0x20)
              my_dpvt->bitbus.rxMsg.data[3] = 0x30;
              {

      io->val = (double)((((my_dpvt->bitbus.rxMsg.data[2]) - 0x30) * 100.0) +
           	(((my_dpvt->bitbus.rxMsg.data[3]) - 0x30) * 10.0) +
           	((my_dpvt->bitbus.rxMsg.data[4]) - 0x30) +
           	(((my_dpvt->bitbus.rxMsg.data[6]) - 0x30) / 10.0) +
           	(((my_dpvt->bitbus.rxMsg.data[7]) - 0x30) / 100.0) +
           	(((my_dpvt->bitbus.rxMsg.data[8]) - 0x30) / 1000.0) +
		(((my_dpvt->bitbus.rxMsg.data[9]) - 0x30) / 10000.0));
              }
           }
           if(my_dpvt->bitbus.rxMsg.data[0] != 0x2B)
              io->val = -(io->val); 


	/*** Return a 2 so record support does not recalculate io->val value ***/
        return(2);  
}



/************ Function to convert readback to minutes *************/ 

static double disp_to_min(struct aiRecord *io, int flg1, int flg2)
{
        struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
        struct dpvtBitBusHead *pbitbus;

        if (devHeidVRZ460Debug & 0x20)
           printf("devHeidVRZ460.c: disp_to_sec.c entered\n");

        if (my_dpvt->bitbus.rxMsg.data[5] == 0x2E)
           io->val = (double)((((my_dpvt->bitbus.rxMsg.data[7]) - 0x30) * 10.0)
             +   (((my_dpvt->bitbus.rxMsg.data[6])-0x30)))  ;
	else
	   io->val = (double)(0);

        if (devHeidVRZ460Debug & 0x10)
           printf("devHeidVRZ460.c: io->val is ->%7.3f<- for ->%s<-\n", io->val,
		io->name);

        /*** Return a 2 so record support does not recalculate io->val value ***/
       return(2);
}


/************ Function to convert readback to seconds *************/ 

static double disp_to_sec(struct aiRecord *io, int flg1, int flg2)
{
        struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
        struct dpvtBitBusHead *pbitbus;

        if (devHeidVRZ460Debug & 0x20)
           printf("devHeidVRZ460.c: disp_to_sec.c entered\n");

        if (my_dpvt->bitbus.rxMsg.data[5] == 0x2E)
           io->val = (double)((((my_dpvt->bitbus.rxMsg.data[4]) - 0x30) * 10.0)
             +   (((my_dpvt->bitbus.rxMsg.data[3])-0x30)))  ;
	else
	   io->val = (double)(0);

        if (devHeidVRZ460Debug & 0x10)
           printf("devHeidVRZ460.c: io->val is ->%7.3f<- for ->%s<-\n", io->val,
                io->name);

        /*** Return a 2 so record support does not recalculate io->val value ***/
      return(2);
}

/************ Function to decide format of message  *************/

static double mesformat(struct aiRecord *io, int flg1, int flg2)
{
        struct dprivate *my_dpvt=(struct dprivate *)io->dpvt;
        struct dpvtBitBusHead *pbitbus;

        if (devHeidVRZ460Debug & 0x20)
           printf("devHeidVRZ460.c: mesformat.c entered\n");

        if (my_dpvt->bitbus.rxMsg.data[5] == 0x2E)
           io->val = (double)(1);	/* put 1 in to show deg.mm.ss format */
	else
	   io->val = (double)(0);	/* put 0 in to show deg.dec format */		


        if (devHeidVRZ460Debug & 0x10)
           printf("devHeidVRZ460.c: io->val is ->%7.3f<- for ->%s<-\n", io->val,
                io->name);

        /*** Return a 2 so record support does not recalculate io->val value ***/
     return(2);
}


/********************************************************************************/


