/************************************************************************/
/*
 *      Original Author: Eric Boucher
 *      Date:            04-09-98
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * .01 04-01-98  erb  Initial development
 * .02 04-15-98  erb  development of the request file format for 
 *                    easy customisation
 * .03 04-28-98  erb  Work on the archive file format.
 * .04 05-25-98  erb  added hand-shaking customisation thru request file
 * .05 05-27-98  erb  added ioc prefix customisation in the request file
 *                    for archive naming convention.
 * .06 07-14-98  erb  Added real-time 1D data aquisition
 *                    - changed the file format
 *                    - added debug_saveDataCpt
 *                      0: wait forever for space in message queue, then send message
 *                      1: send message only if queue is not full
 *                      2: send message only if queue is not full and specified
 *                         time has passed (SetCptWait() sets this time
 *                      3: if specified time has passed, wait for space in queue,
 *                         then send message
 *                      else: don't send message
 * .07 08-03-98  erb  Added environment pv's
 *                    saveData will request data for those env pv's at the 
 *                    begining of the scan, but will not wait for answers
 *                    and save the env data at the end of the scan.
 * .08 08-09-98  erb  bug fixes: time-stamp
 *                    still some work to do: the time stamp is not availlable
 *                    when the scan start.
 *
 * .09 09-01-98  erb  First release: version 1.0
 *
 * .10 09-08-98  tmm  rename debug_saveDataCpt -> saveData_MessagePolicy
 * .11 09-09-98  erb  Clarification of the messages send to the user
 * .12 09-12-98  erb  Modification of saveData_active
 *                    -rename saveData_active to saveData_status
 *                     0: saveData is not active
 *                     1: saveData is active and ready to save scans
 *                     2: saveData is active but unable to save scan
 * .13 09-20-98  erb  work on the file-system and sub-directory
 *                    -check if file system is correctly mounted
 *                    -check R/W permission of the target directory
 *                    -Prevent user from starting a scan if something is wrong
 * .14 10-02-98  erb  Modification of the file format to improve "weird" scan storage
 *		                VERSION 1.1
 * .15 05-06-99  erb  Change EXSC field to DATA and reverse the logic
 *                    Save time stamp when scan starts
 * .16 07-15-99  erb  -Fix the trigger bug for 2D and higer scans.
 *                    -Change version to 1.2 to avoid confusion with 
 *                     "invalid 1.1" files.
 *                    -Add the saveData_Version function.
 *                    -Add the saveData_CVS function.
 * .17 11-30-99  tmm  v1.3 Added 70 new detectors, and two new triggers
 * .18 03-23-00  erb  -fix filename bug.
 *                    -Check that txpv is a registered scan pv AND that txcd=1.0
 *                     to link scans together.
 *                    -suspend calling task (shell) until all connections are made.
 * .19 12-19-00  tmm  v1.5 check file open to see if it really succeeded.
 * .20 01-09-01  tmm  v1.6 fold in Ron Sluiter's mods to 1.4 that made it work
 *                    with the powerpC processor.
 * .20 03-08-01  tmm  v1.7 Don't handshake if file system can't be mounted, but do
 *                    continue to enable and disable puts to filesystem PV's.
 * .21 05-01-01  tmm  v1.8 Don't allow punctuation characters in filename.
 *                    Extension changed to ".mda".
 *                    e.g., tmm:scan1_0000.scan  --> tmm_scan1_000.mda
 * .21 02-27-02  tmm  v1.9 Support filename PV, so clients can easily determine
 *                    the current data file.  Write char array with xdr_vector()
 *                    instead of xdr_bytes(), which wasn't working.
 */


#define FILE_FORMAT_VERSION (float)1.3
#define SAVE_DATA_VERSION   "1.9.0"


#include "req_file.h"


#include <stdioLib.h>
#include <ctype.h>

#include <string.h>
#include <usrLib.h>
#include <ioLib.h>
#include <nfsDrv.h>
#include <time.h>


#include <cadef.h>

#include <taskLib.h>
#include <tickLib.h>
#include <msgQLib.h>
#include "xdr_lib.h"
#include "xdr_stdio.h"


#include <dbEvent.h>
#include <special.h>


/************************************************************************/
/*                           MACROS                                     */

#define PRIORITY 150	/* The saveTask priority			*/

/*#define NODEBUG*/

#define LOCAL static

volatile int debug_saveData = 0;
volatile int debug_saveDataMsg = 0;
volatile int saveData_MessagePolicy = 0;

#ifdef NODEBUG 
#define Debug0(d,s) ;
#define Debug1(d,s,p) ;
#define Debug2(d,s,p1,p2) ;
#define Debug3(d,s,p1,p2,p3) ;
#define DebugMsg0(d,s) ;
#define DebugMsg1(d,s,p) ;
#define DebugMsg2(d,s,p1,p2) ;
#define DebugMsg3(d,s,p1,p2,p3) ;
#else
#define Debug0(d,s)       { if(d<=debug_saveData) { \
				printf(s); } }
#define Debug1(d,s,p)     { if(d<=debug_saveData) { \
				printf(s,p); } }
#define Debug2(d,s,p1,p2) { if(d<=debug_saveData) { \
				printf(s,p1,p2); } }
#define Debug3(d,s,p1,p2,p3) { if(d<=debug_saveData) { \
				printf(s,p1,p2,p3); } }

#define DebugMsg0(d,s)       { if(d<=debug_saveDataMsg) { \
				printf(s); } }
#define DebugMsg1(d,s,p)     { if(d<=debug_saveDataMsg) { \
				printf(s,p); } }
#define DebugMsg2(d,s,p1,p2) { if(d<=debug_saveDataMsg) { \
				printf(s,p1,p2); } }
#define DebugMsg3(d,s,p1,p2,p3) { if(d<=debug_saveDataMsg) { \
				printf(s,p1,p2,p3); } }
#endif


/************************************************************************/
/*---------- STATIC DECLARATIONS TO MATCH SCAN RECORD FIELDS -----------*/

#define SCAN_NBP	4     /* # of scan positioners        */
#define SCAN_NBD	85    /* # of scan detectors          */
#define SCAN_NBT	4     /* # of scan triggers           */

LOCAL char *pxnv[SCAN_NBP]= {
  "P1NV",  "P2NV",  "P3NV",  "P4NV" 
};
LOCAL char *pxpv[SCAN_NBP]= {
  "P1PV",  "P2PV",  "P3PV",  "P4PV"
};
LOCAL char *pxsm[SCAN_NBP]= {
  "P1SM",  "P2SM",  "P3SM",  "P4SM"
};
LOCAL char *rxnv[SCAN_NBP]= {
  "R1NV",  "R2NV",  "R3NV",  "R4NV"
};
LOCAL char *rxpv[SCAN_NBP]= { 
  "R1PV",  "R2PV",  "R3PV",  "R4PV"
};
LOCAL char *pxra[SCAN_NBP]= {
  "P1RA",  "P2RA",  "P3RA",  "P4RA"
};
LOCAL char *rxcv[SCAN_NBP]= {
  "R1CV",  "R2CV",  "R3CV",  "R4CV"
};
LOCAL char *dxnv[SCAN_NBD]= {
  "D1NV",  "D2NV",  "D3NV",  "D4NV",
  "D5NV",  "D6NV",  "D7NV",  "D8NV",
  "D9NV",  "DANV",  "DBNV",  "DCNV",
  "DDNV",  "DENV",  "DFNV",
  "D01NV", "D02NV","D03NV","D04NV","D05NV","D06NV","D07NV","D08NV","D09NV","D10NV",
  "D11NV", "D12NV","D13NV","D14NV","D15NV","D16NV","D17NV","D18NV","D19NV","D20NV",
  "D21NV", "D22NV","D23NV","D24NV","D25NV","D26NV","D27NV","D28NV","D29NV","D30NV",
  "D31NV", "D32NV","D33NV","D34NV","D35NV","D36NV","D37NV","D38NV","D39NV","D40NV",
  "D41NV", "D42NV","D43NV","D44NV","D45NV","D46NV","D47NV","D48NV","D49NV","D50NV",
  "D51NV", "D52NV","D53NV","D54NV","D55NV","D56NV","D57NV","D58NV","D59NV","D60NV",
  "D61NV", "D62NV","D63NV","D64NV","D65NV","D66NV","D67NV","D68NV","D69NV","D70NV"
};
LOCAL char *dxpv[SCAN_NBD]= {
  "D1PV",  "D2PV",  "D3PV",  "D4PV",
  "D5PV",  "D6PV",  "D7PV",  "D8PV",
  "D9PV",  "DAPV",  "DBPV",  "DCPV",
  "DDPV",  "DEPV",  "DFPV",
  "D01PV", "D02PV","D03PV","D04PV","D05PV","D06PV","D07PV","D08PV","D09PV","D10PV",
  "D11PV", "D12PV","D13PV","D14PV","D15PV","D16PV","D17PV","D18PV","D19PV","D20PV",
  "D21PV", "D22PV","D23PV","D24PV","D25PV","D26PV","D27PV","D28PV","D29PV","D30PV",
  "D31PV", "D32PV","D33PV","D34PV","D35PV","D36PV","D37PV","D38PV","D39PV","D40PV",
  "D41PV", "D42PV","D43PV","D44PV","D45PV","D46PV","D47PV","D48PV","D49PV","D50PV",
  "D51PV", "D52PV","D53PV","D54PV","D55PV","D56PV","D57PV","D58PV","D59PV","D60PV",
  "D61PV", "D62PV","D63PV","D64PV","D65PV","D66PV","D67PV","D68PV","D69PV","D70PV"
};
LOCAL char *dxda[SCAN_NBD]= {
  "D1DA",  "D2DA",  "D3DA",  "D4DA",
  "D5DA",  "D6DA",  "D7DA",  "D8DA",
  "D9DA",  "DADA",  "DBDA",  "DCDA",
  "DDDA",  "DEDA",  "DFDA",
  "D01DA", "D02DA","D03DA","D04DA","D05DA","D06DA","D07DA","D08DA","D09DA","D10DA",
  "D11DA", "D12DA","D13DA","D14DA","D15DA","D16DA","D17DA","D18DA","D19DA","D20DA",
  "D21DA", "D22DA","D23DA","D24DA","D25DA","D26DA","D27DA","D28DA","D29DA","D30DA",
  "D31DA", "D32DA","D33DA","D34DA","D35DA","D36DA","D37DA","D38DA","D39DA","D40DA",
  "D41DA", "D42DA","D43DA","D44DA","D45DA","D46DA","D47DA","D48DA","D49DA","D50DA",
  "D51DA", "D52DA","D53DA","D54DA","D55DA","D56DA","D57DA","D58DA","D59DA","D60DA",
  "D61DA", "D62DA","D63DA","D64DA","D65DA","D66DA","D67DA","D68DA","D69DA","D70DA"
};
LOCAL char *dxcv[SCAN_NBD]= {
  "D1CV",  "D2CV",  "D3CV",  "D4CV",
  "D5CV",  "D6CV",  "D7CV",  "D8CV",
  "D9CV",  "DACV",  "DBCV",  "DCCV",
  "DDCV",  "DECV",  "DFCV",
  "D01CV", "D02CV","D03CV","D04CV","D05CV","D06CV","D07CV","D08CV","D09CV","D10CV",
  "D11CV", "D12CV","D13CV","D14CV","D15CV","D16CV","D17CV","D18CV","D19CV","D20CV",
  "D21CV", "D22CV","D23CV","D24CV","D25CV","D26CV","D27CV","D28CV","D29CV","D30CV",
  "D31CV", "D32CV","D33CV","D34CV","D35CV","D36CV","D37CV","D38CV","D39CV","D40CV",
  "D41CV", "D42CV","D43CV","D44CV","D45CV","D46CV","D47CV","D48CV","D49CV","D50CV",
  "D51CV", "D52CV","D53CV","D54CV","D55CV","D56CV","D57CV","D58CV","D59CV","D60CV",
  "D61CV", "D62CV","D63CV","D64CV","D65CV","D66CV","D67CV","D68CV","D69CV","D70CV"
};
LOCAL char *txnv[SCAN_NBT]= {
  "T1NV", "T2NV", "T3NV", "T4NV"
};
LOCAL char *txpv[SCAN_NBT]= {
  "T1PV", "T2PV", "T3PV", "T4PV"
};
LOCAL char *txcd[SCAN_NBT]= {
  "T1CD", "T2CD", "T3CD", "T4CD"
};



/************************************************************************/
/*----- STRUCT TO CONNECT A SCAN RECORD AND RETRIEVE DATA FROM IT ------*/

LOCAL const float file_format_version=FILE_FORMAT_VERSION;
LOCAL const char  save_data_version[]=SAVE_DATA_VERSION;

#define NOT_CONNECTED	0	/* SCAN not connected			*/
#define CONNECTED	1	/* SCAN is correctly connected		*/
#define INITIALIZED	2	/* SCAN buffers are initialized		*/

#define XXNV_OK     0
#define XXNV_BADPV  1
#define XXNV_NOPV   2

#define HANDSHAKE_BUSY 1
#define HANDSHAKE_DONE 0

typedef struct scan {
  /*========================= PRIVATE FIELDS ===========================*/
  short        state;		/* state of the structure		*/
  char	       name[29];	/* name of the scan			*/
  short        scan_dim;	/* dimension of this scan		*/
  char         fname[100];	/* filename				*/
  char	       ffname[100];	/* full filename			*/
  int	       first_scan;	/* true if this is the first scan	*/
  struct scan* nxt;		/* link to the inner scan		*/
  long         offset;		/* where to store this scan's offset	*/
  long         offset_extraPV;  /* where to store the extra pv's offset */
  long	       counter;		/* current scan				*/
  long	       dims_offset;
  long	       regular_offset;
  short	       old_npts;

  /*=======================SCAN RECORD FIELDS ==========================*/
  short    data;       	/* scan execution				*/
  chid	   cdata;
  char     stamp[40];	/* scan's start time stamp			*/
  long	   time_fpos;
  
  short	  mpts;		/* max # of points				*/
  chid    cmpts;
  short	  npts;		/* # of points					*/
  chid    cnpts;
  short   cpt;		/* current point				*/
  chid    ccpt;
  long    cpt_fpos;	/* where to store data of the current point	*/
  ULONG   cpt_ticks;	/* ticks of the last cpt monitor		*/
  int     cpt_monitored;
  evid    cpt_evid;
  int     all_pts;	/* true if we were able to get all points	*/

  int     nb_pos;	/* # of pos to save				*/
  int     nb_det;	/* # of det to save				*/
  int     nb_trg;	/* # of trg to save				*/

  short   handShake_state;
  chid    chandShake;

  /*========================== POSITIONERS =============================*/

  short   pxnv[SCAN_NBP];	/* positioner X valid PV		*/
  chid    cpxnv[SCAN_NBP];
  char    pxpv[SCAN_NBP][40];	/* positioner X PV name			*/
  chid    cpxpv[SCAN_NBP];
  char    pxds[SCAN_NBP][40];	/* positioner X description field	*/
  chid    cpxds[SCAN_NBP];
  struct dbr_ctrl_double  pxeu[SCAN_NBP];/* positioner X eng unit	*/
  chid    cpxeu[SCAN_NBP];
  char    pxsm[SCAN_NBP][40];	/* positioner step mode			*/
  chid    cpxsm[SCAN_NBP];

  /*========================== READBACKS ===============================*/

  short	  rxnv[SCAN_NBP];	/* readback X valid PV			*/
  chid    crxnv[SCAN_NBP];
  char    rxpv[SCAN_NBP][40];	/* readback X PV name			*/
  chid    crxpv[SCAN_NBP];
  char    rxds[SCAN_NBP][40];	/* readback X description field		*/
  chid    crxds[SCAN_NBP];
  struct dbr_ctrl_double  rxeu[SCAN_NBP];/* readback X eng unit	*/
  chid    crxeu[SCAN_NBP];
  double* pxra[SCAN_NBP];       /* readback X steps array		*/
  chid	  cpxra[SCAN_NBP];
  long    pxra_fpos[SCAN_NBP];
  double  rxcv[SCAN_NBP];	/* current readback value		*/
  chid    crxcv[SCAN_NBP];

  /*========================== DETECTORS ===============================*/

  short	  dxnv[SCAN_NBD];	/* detector X valid PV			*/
  chid	  cdxnv[SCAN_NBD];
  char	  dxpv[SCAN_NBD][40];	/* detector X PV name			*/
  chid	  cdxpv[SCAN_NBD];
  char	  dxds[SCAN_NBD][40];	/* detector X description field		*/
  chid    cdxds[SCAN_NBD];
  struct dbr_ctrl_float  dxeu[SCAN_NBD];/* detector X eng unit		*/
  chid    cdxeu[SCAN_NBD];
  float*  dxda[SCAN_NBD];	/* detector X steps array		*/
  chid	  cdxda[SCAN_NBD];
  long    dxda_fpos[SCAN_NBD];
  float   dxcv[SCAN_NBD];	/* current readback value		*/
  chid    cdxcv[SCAN_NBD];

  /*========================== TRIGGERS ================================*/
  
  short	txnv[SCAN_NBT];		/* trigger X valid pv			*/
  short txsc[SCAN_NBT];		/* trigger X linked to a scan		*/
  chid  ctxnv[SCAN_NBT];
  char  txpv[SCAN_NBT][40];	/* trigger X pv name			*/
  chid  ctxpv[SCAN_NBT];
  float txcd[SCAN_NBT];		/* trigger X command			*/
  chid  ctxcd[SCAN_NBT];

} SCAN;		/****** end of structure SCAN ******/


/************************************************************************/
/*---------------------- saveScan task messages ------------------------*/

#define MAX_MSG		1000 /*200*/	/* max # of messages in saveTask queue	*/
#define MAX_SIZE	80	/* max size in byte of the messages	*/

#define MSG_SCAN_DATA	1	/* save scan        			*/
#define MSG_SCAN_NPTS	2	/* NPTS changed        			*/
#define MSG_SCAN_PXNV   3	/* positioner pv name changed        	*/
#define MSG_SCAN_PXSM   4	/* positioner step mode changed        	*/
#define MSG_SCAN_RXNV	5	/* readback pv name changed         	*/
#define MSG_SCAN_DXNV  	6	/* detector pv name changed		*/
#define MSG_SCAN_TXNV	7	/* trigger pv name changed		*/
#define MSG_SCAN_TXCD   8	/* trigger command changed		*/
#define MSG_SCAN_CPT	9	/* the current point changed		*/

#define MSG_DESC	10
#define MSG_EGU		11

#define MSG_FILE_SYSTEM	20
#define MSG_FILE_SUBDIR	21
#define MSG_REALTIME1D  22


/* Message structures							*/
typedef struct scan_short_msg {
  int   type;
  ULONG time;
  SCAN* pscan;
  short val;
} SCAN_SHORT_MSG;

#define SCAN_SHORT_SIZE (sizeof(SCAN_SHORT_MSG)<MAX_SIZE? \
			 sizeof(SCAN_SHORT_MSG):MAX_SIZE)

#define sendScanShortMsg(t, s, v, w) { \
    SCAN_SHORT_MSG msg; \
    msg.type= t; msg.pscan=s; msg.val= v;\
    msg.time= tickGet(); \
    msgQSend(msg_queue, (char*)&msg, \
	     SCAN_SHORT_SIZE, w, MSG_PRI_NORMAL); }


typedef struct scan_ts_short_msg {
  int   type;
  ULONG time;
  SCAN* pscan;
  TS_STAMP stamp;
  short    val;
} SCAN_TS_SHORT_MSG;

#define SCAN_TS_SHORT_SIZE (sizeof(SCAN_TS_SHORT_MSG)<MAX_SIZE? \
			    sizeof(SCAN_TS_SHORT_MSG):MAX_SIZE)

#define sendScanTSShortMsg(t, s, q, v, w) { \
    SCAN_TS_SHORT_MSG msg; \
    msg.type= t; msg.pscan=s; msg.val= v;\
    msg.stamp.secPastEpoch= q.secPastEpoch; \
    msg.stamp.nsec= q.nsec; \
    msg.time= tickGet(); \
    msgQSend(msg_queue, (char*)&msg, \
	     SCAN_TS_SHORT_SIZE, w, MSG_PRI_NORMAL); }



typedef struct scan_index_msg {
  int   type;
  ULONG time;
  SCAN* pscan;
  int   index;
  double val;
} SCAN_INDEX_MSG;

#define SCAN_INDEX_SIZE (sizeof(SCAN_INDEX_MSG)<MAX_SIZE? \
			 sizeof(SCAN_INDEX_MSG):MAX_SIZE)

#define sendScanIndexMsg(t,s,i,v,w) { \
    SCAN_INDEX_MSG msg; \
    msg.type=t; msg.pscan=s; msg.index=i; msg.val= (double)v; \
    msg.time= tickGet(); \
    msgQSend(msg_queue, (char*)&msg, \
	     SCAN_INDEX_SIZE, w, MSG_PRI_NORMAL); }

typedef struct string_msg {
  int  type;
  ULONG time;
  char* pdest;
  char  string[40];
} STRING_MSG;

#define STRING_SIZE (sizeof(STRING_MSG)<MAX_SIZE? \
		     sizeof(STRING_MSG):MAX_SIZE)

#define sendStringMsg(t,d,s,w) { \
    STRING_MSG msg; \
    msg.type=t; msg.pdest=(char*)d; strncpy(msg.string, s, 40); \
    msg.time= tickGet(); \
    msgQSend(msg_queue, (char*)&msg, \
	     STRING_SIZE, w, MSG_PRI_NORMAL); }

typedef struct integer_msg {
  int type;
  ULONG time;
  int val;
} INTEGER_MSG;

#define INTEGER_SIZE (sizeof(INTEGER_MSG)<MAX_SIZE? \
		      sizeof(INTEGER_MSG):MAX_SIZE)

#define sendIntegerMsg(t,v,w) { \
    INTEGER_MSG msg; \
    msg.type=t; msg.val=v; \
    msg.time= tickGet(); \
    msgQSend(msg_queue,(char*)&msg, \
	     INTEGER_SIZE, w, MSG_PRI_NORMAL); }

/************************************************************************/
/*--------------------- list of scan to be saved -----------------------*/

typedef struct scan_node {
  SCAN              scan;
  struct scan_node* nxt;
} SCAN_NODE;

typedef union db_access_val DBR_VAL;

typedef struct pv_node {
  SEM_ID   lock;
  chid     channel;
  chid     desc_chid;
  char     name[29];
  char     desc[40];
  int      dbr_type;
  long     count;
  DBR_VAL* pval;
  struct pv_node* nxt;
} PV_NODE;


LOCAL char  req_file[40];
LOCAL char  req_macros[40];

LOCAL char  server_pathname[80];
LOCAL char* server_subdir;
LOCAL char  local_pathname[80];
LOCAL char* local_subdir;

#define STATUS_INACTIVE		0
#define STATUS_ACTIVE_OK	1
#define STATUS_ACTIVE_FS_ERROR	2
#define STATUS_ERROR	3
LOCAL chid  save_status_chid;
LOCAL short save_status= STATUS_INACTIVE;
LOCAL chid  message_chid;
LOCAL chid  filename_chid;
LOCAL chid  full_pathname_chid;

#define FS_NOT_MOUNTED	0
#define FS_MOUNTED	1
LOCAL chid  file_system_chid;
LOCAL chid  file_system_disp_chid;
LOCAL int   file_system_state= FS_NOT_MOUNTED;

LOCAL chid  file_subdir_chid;
LOCAL chid  file_subdir_disp_chid;

LOCAL int   realTime1D= 1;
LOCAL chid  realTime1D_chid;

LOCAL long  counter;		/* data file counter			*/
LOCAL chid  counter_chid;
LOCAL char  ioc_prefix[10];

LOCAL int          task_id     =ERROR;/* saveScan task id		*/
LOCAL MSG_Q_ID     msg_queue   =NULL; /* saveScan task's message queue	*/

LOCAL ULONG        cpt_wait_ticks;
LOCAL int          nb_scan_running=0; /* # of scan currently running	*/
LOCAL SCAN_NODE*   list_scan   =NULL; /* list of scan to be saved	*/
LOCAL PV_NODE*     list_pv= NULL;    /* list of pvs to be saved with each scan */
LOCAL int          nb_extraPV= 0;

/************************************************************************/
/*                      PROTOTYPES FUNCTIONS				*/

/*----------------------------------------------------------------------*/
/* Try to establish the connections with a scanRecord ,allocate		*/
/* memory for the buffers and add it to the list of scans		*/
/* name: the name of the scan record.   				*/
/* return 0 if successful.						*/
/*        -1 otherwise.							*/
LOCAL int connectScan(char* name, char* handShake);

/*----------------------------------------------------------------------*/
/* Disconnect a SCAN from the scan record				*/
/* ei clear all ca connections and events and free all buffers.		*/
/* pscan: a pointer to a SCAN structure					*/
/* return 0 if successful						*/
/*       -1 otherwise							*/
LOCAL int deconnectScan(SCAN* pscan);

/*----------------------------------------------------------------------*/
/* start the monitoring of a scan record.             			*/
/* pscan: a pointer to the SCAN structure connected to the scan record.	*/
/* return 0 if all monitors have been successfuly added.           	*/
/*        -1 otherwise.							*/
LOCAL int monitorScan(SCAN* pscan);

/*----------------------------------------------------------------------*/
/* monitor all SCAN of the list.                                        */
LOCAL void monitorScans();

/*----------------------------------------------------------------------*/
/* search for a scan in the SCAN list.                                  */
/* name: the name of the scan to look for.                              */
/* return a pointer to the SCAN structure if found                      */
/*        NULL otherwise.                                               */
LOCAL SCAN* searchScan(char* name);

/*----------------------------------------------------------------------*/
LOCAL int  scan_getDim(SCAN* pscan);

/*----------------------------------------------------------------------*/
/* print debug info on a SCAN structure.                                */
LOCAL void infoScan(SCAN* pscan);

/*----------------------------------------------------------------------*/
/* DATA field monitor.                                                  */
/*                                                                      */
LOCAL void dataMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* NPTS  fields monitor                                       		*/
/*                                                			*/
LOCAL void nptsMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* CPT  fields monitor                                       		*/
/*                                                			*/
LOCAL void cptMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* PXNV  fields monitor                                       		*/
/*                                                			*/
LOCAL void pxnvMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* PXSM  fields monitor                                       		*/
/*                                                			*/
LOCAL void pxsmMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* RXNV  fields monitor                                       		*/
/*                                                			*/
LOCAL void rxnvMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* DXNV  fields monitor                                       		*/
/*                                                			*/
LOCAL void dxnvMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* TXNV field monitor.                                        		*/
/*                                                			*/
LOCAL void txnvMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* TXCD field monitor.                                        		*/
/*                                                			*/
LOCAL void txcdMonitor(struct event_handler_args eha);

/*----------------------------------------------------------------------*/
/* The task in charge of updating and saving scans           		*/
/*                                                			*/
LOCAL int saveDataTask(int itd,int p1,int p2,int p3,int p4,int p5,int p6,int p7,int p8,int p9);



/*----------------------------------------------------------------------*/
/* test the status of a file			           		*/
/* fname: the file							*/
/* return OK if the file exist						*/
/*	  ERROR otherwise						*/
/*                                                			*/
LOCAL int fileStatus(char* fname)
{
  struct stat status;

  return stat(fname, &status);
}

/*----------------------------------------------------------------------*/
/* test for Read/Write permission in a directory           		*/
/* path: the pathname							*/
/* return OK if R/W allowed   						*/
/*	  ERROR otherwise						*/
/*                                                			*/
LOCAL int checkRWpermission(char* path) {
  /* Quick and dirty way to check for R/W permission			*/
  int  file;
  char tmpfile[100];

  strcpy(tmpfile, path);
  strcat(tmpfile, "/rix:_");

  while(fileStatus(tmpfile)==OK && strlen(tmpfile)<100) {
    strcat(tmpfile, "_");
  }

  if(fileStatus(tmpfile)==OK) {
    return ERROR;
  }

  file= creat(tmpfile, O_RDWR);

  if(fileStatus(tmpfile)!=OK) {
    return ERROR;
  }

  close(file);
  remove(tmpfile);

  return  OK;
}

LOCAL void sendUserMessage(char* msg) {
  /*  printf("%s\n", msg); */
  if(message_chid) {
    ca_array_put(DBR_STRING, 1, message_chid, msg);
  }
}

/************************************************************************/
/*                        TESTS FUNCTIONS    				*/

int saveData_Init(char* fname, char* macros)
{
  strncpy(req_file, fname, 39);
  strncpy(req_macros, macros, 39);

  if(msg_queue==NULL) {
    msg_queue= msgQCreate(MAX_MSG, MAX_SIZE, MSG_Q_FIFO);
    if(msg_queue==NULL) {
      Debug0(1, "Unable to create message queue\n");
      return -1;
    }
    printf("saveData: message queue created\n");
    if(taskSpawn("saveDataTask", PRIORITY, VX_FP_TASK, 5000, (FUNCPTR)saveDataTask, 
                       taskIdSelf(),0,0,0,0,0,0,0,0,0)==ERROR) {
      Debug0(1, "Unable to create saveDataTask\n");
      msgQDelete(msg_queue);
      return -1;
    } else {
      taskSuspend(0);
    }
  }
  return 0;
}  

int saveData_PrintScanInfo(char* name)
{
  SCAN* pscan;
  
  pscan= searchScan(name);
  if(pscan) {
    infoScan(pscan);
  }

  return 0;
}

void saveData_Priority(int p)
{
  taskPrioritySet(task_id, p);
}

void saveData_SetCptWait(float s)
{
  cpt_wait_ticks= (ULONG) (s*vxTicksPerSecond);
}

void saveData_Version()
{
  printf("saveData Version: %s\n", save_data_version);
}

void saveData_CVS() 
{
  printf("saveData CVS: $Id: saveData.c,v 1.2 2002-02-27 17:22:07 bcda Exp $\n");
}

void saveData_Info() {
  SCAN_NODE* pnode;
  SCAN* scan;
  SCAN* cur;
  pnode= list_scan;
  printf("saveData: scan info:\n");
  while(pnode) {
    scan= &pnode->scan;
    printf("scan   : %s\n", scan->name);
    printf("  rank : %d\n", scan_getDim(scan));
    printf("  links:");
    cur= scan;
    while(cur) {
      printf(cur->name);
      cur= cur->nxt;
      if(cur) printf("->");
    }
    printf("\n");
    pnode= pnode->nxt;
  }
}

/************************************************************************/
/*                        IMPLEMENTATION     				*/

/*----------------------------------------------------------------------*/
/* Try to establish the connections with a scanRecord, allocate		*/
/* memory for the buffers and add it to the list of scans		*/
/* name: the name of the scan record.   				*/
/* return 0 if successful.						*/
/*        -1 otherwise.							*/
LOCAL int connectScan(char* name, char* handShake)
{
  SCAN_NODE* pnode;
  SCAN_NODE* pcur;
  SCAN* pscan;
  int   i, ok, nc;
  char  pvname[80];
  char* field;

  Debug1(1, "connectScan(%s)...\n", name);

  pnode= (SCAN_NODE*) malloc(sizeof(SCAN_NODE));
  if(!pnode) {
    printf("saveData: Not enough space to connect %s\n", name);
    return -1;
  }
  pnode->nxt= NULL;
  pscan= &pnode->scan;

  /* initialize critical fields						*/
  memset((void*)pscan, 0, sizeof(SCAN));

  pscan->data= -1;

  pscan->first_scan= TRUE;
  pscan->scan_dim= 1;
  strncpy(pscan->name, name, 28);
  pscan->name[28]='\0';
  pscan->nxt= NULL;
  pscan->cpt_ticks=0;
  pscan->cpt_monitored= FALSE;


  /* try to connect to the hand shake variable				*/
  pscan->chandShake= NULL;
  if(handShake && (*handShake!='\0')) {
    ca_search(handShake, &(pscan->chandShake));
    ca_pend_io(1.0);
  }

  /* try to connect to the scan record					*/
  strcpy(pvname, pscan->name);
  strcat(pvname, ".");
  field= &pvname[strlen(pvname)];

  strcpy(field, "DATA");
  ca_search_and_connect(pvname, &(pscan->cdata), NULL, (void*)pscan);

  strcpy(field, "MPTS");
  ca_search_and_connect(pvname, &(pscan->cmpts), NULL, (void*)pscan);

  strcpy(field, "NPTS");
  ca_search_and_connect(pvname, &(pscan->cnpts), NULL, (void*)pscan);

  strcpy(field, "CPT");
  ca_search_and_connect(pvname, &(pscan->ccpt), NULL, (void*)pscan);
  

  if(ca_pend_io(5.0)!= ECA_NORMAL) {
    printf("saveData: Unable to connect to %s\n", pscan->name);
    deconnectScan(pscan);
    free(pnode);
    return -1;
  }


  /*---------------------- POSITIONERS & READBACKS ---------------------*/
  for(i=0; i<SCAN_NBP; i++) {
    pscan->pxnv[i]= XXNV_NOPV;
    strcpy(field, pxnv[i]);
    ca_search_and_connect(pvname, &(pscan->cpxnv[i]), NULL, (void*)pscan);
    strcpy(field, pxpv[i]);
    ca_search_and_connect(pvname, &(pscan->cpxpv[i]), NULL, (void*)pscan);
    strcpy(field, pxsm[i]);
    ca_search_and_connect(pvname, &(pscan->cpxsm[i]), NULL, (void*)pscan);
    pscan->rxnv[i]= XXNV_NOPV;
    strcpy(field, rxnv[i]);
    ca_search_and_connect(pvname, &(pscan->crxnv[i]), NULL, (void*)pscan);
    strcpy(field, rxpv[i]);
    ca_search_and_connect(pvname, &(pscan->crxpv[i]), NULL, (void*)pscan);
    strcpy(field, pxra[i]);
    ca_search_and_connect(pvname, &(pscan->cpxra[i]), NULL, (void*)pscan);
    strcpy(field, rxcv[i]);
    ca_search_and_connect(pvname, &(pscan->crxcv[i]), NULL, (void*)pscan);
  }

  /*------------------------- DETECTORS --------------------------------*/
  for(i=0; i<SCAN_NBD; i++) {
    pscan->dxnv[i]= XXNV_NOPV;
    strcpy(field, dxnv[i]);
    ca_search_and_connect(pvname, &(pscan->cdxnv[i]), NULL, (void*)pscan);
    strcpy(field, dxpv[i]);
    ca_search_and_connect(pvname, &(pscan->cdxpv[i]), NULL, (void*)pscan);
    strcpy(field, dxda[i]);
    ca_search_and_connect(pvname, &(pscan->cdxda[i]), NULL, (void*)pscan);
    strcpy(field, dxcv[i]);
    ca_search_and_connect(pvname, &(pscan->cdxcv[i]), NULL, (void*)pscan);
  }

  /*------------------------- TRIGGERS ---------------------------------*/
  for(i=0; i<SCAN_NBT; i++) {
    pscan->txnv[i]= XXNV_NOPV;
    pscan->txsc[i]= 1;
    strcpy(field, txnv[i]);
    ca_search_and_connect(pvname, &(pscan->ctxnv[i]), NULL, (void*)pscan);
    strcpy(field, txpv[i]);
    ca_search_and_connect(pvname, &(pscan->ctxpv[i]), NULL, (void*)pscan);
    strcpy(field, txcd[i]);
    ca_search_and_connect(pvname, &(pscan->ctxcd[i]), NULL, (void*)pscan);
  }

  if(ca_pend_io(5.0)!= ECA_NORMAL) {
    nc=0;
    for(i=0; i<SCAN_NBP; ++i) {
      if(ca_state(pscan->cpxnv[i])!=cs_conn ||
         ca_state(pscan->cpxpv[i])!=cs_conn ||
         ca_state(pscan->cpxsm[i])!=cs_conn ||
         ca_state(pscan->crxnv[i])!=cs_conn ||
         ca_state(pscan->crxpv[i])!=cs_conn ||
         ca_state(pscan->cpxra[i])!=cs_conn ||
         ca_state(pscan->crxcv[i])!=cs_conn) {
        ++nc;
        ca_clear_channel(pscan->cpxnv[i]);
        pscan->cpxnv[i]=NULL;
        pscan->pxnv[i]= XXNV_NOPV;
        ca_clear_channel(pscan->cpxpv[i]);
        pscan->cpxpv[i]= NULL;
        pscan->pxpv[i][0]= '\0';
        ca_clear_channel(pscan->cpxsm[i]);
        pscan->cpxsm[i]= NULL;
        pscan->pxsm[i][0]= '\0';
        ca_clear_channel(pscan->crxnv[i]);
        pscan->crxnv[i]= NULL;
        pscan->rxnv[i]= XXNV_NOPV;
        ca_clear_channel(pscan->crxpv[i]);
        pscan->crxpv[i]= NULL;
        pscan->rxpv[i][0]= '\0';
        ca_clear_channel(pscan->cpxra[i]);
        pscan->cpxra[i]= NULL;
        pscan->pxra[i]= NULL;
        ca_clear_channel(pscan->crxcv[i]);
        pscan->crxcv[i]= NULL;
      }
    }
    if(nc>0) printf("saveDataTask warning: %s: %d positioner(s) not connected\n", pscan->name, nc);
    
    nc=0;
    for(i=0; i<SCAN_NBD; ++i) {
      if(ca_state(pscan->cdxnv[i])!=cs_conn ||
         ca_state(pscan->cdxpv[i])!=cs_conn ||
         ca_state(pscan->cdxda[i])!=cs_conn ||
         ca_state(pscan->cdxcv[i])!=cs_conn) {
        ++nc;
        ca_clear_channel(pscan->cdxnv[i]);
        pscan->cdxnv[i]=NULL;
        pscan->dxnv[i]= XXNV_NOPV;
        ca_clear_channel(pscan->cdxpv[i]);
        pscan->cdxpv[i]= NULL;
        pscan->dxpv[i][0]= '\0';
        ca_clear_channel(pscan->cdxda[i]);
        pscan->cdxda[i]= NULL;
        pscan->dxda[i]= NULL;
        ca_clear_channel(pscan->cdxcv[i]);
        pscan->cdxcv[i]= NULL;
      }
    }
    if(nc>0) printf("saveDataTask warning: %s: %d detector(s) not connected\n", pscan->name, nc);

    nc=0;
    for(i=0; i<SCAN_NBT; ++i) {
      if(ca_state(pscan->ctxnv[i])!=cs_conn ||
         ca_state(pscan->ctxpv[i])!=cs_conn ||
         ca_state(pscan->ctxcd[i])!=cs_conn) {
        ++nc;
        ca_clear_channel(pscan->ctxnv[i]);
        pscan->ctxnv[i]=NULL;
        pscan->txnv[i]= XXNV_NOPV;
        ca_clear_channel(pscan->ctxpv[i]);
        pscan->ctxpv[i]= NULL;
        pscan->txpv[i][0]= '\0';
        ca_clear_channel(pscan->ctxcd[i]);
        pscan->ctxcd[i]= NULL;
      }
    }
    if(nc>0) printf("saveDataTask warning: %s: %d trigger(s) not connected\n", pscan->name, nc);
  }
  
  /* get the max number of points of the scan to allocate the buffers	*/
  if(ca_array_get(DBR_SHORT, 1, pscan->cmpts, &(pscan->mpts))!=ECA_NORMAL) {
    if(ca_pend_io(5.0)==ECA_TIMEOUT) {
      printf("saveData: %s: Unable to read MPTS. Aborting connection", pscan->name);
      deconnectScan(pscan);
      free(pnode);
      return -1;
    }
  }

  /* Try to allocate memory for the buffers				*/
  ok= 1;
  for(i=0; ok && i<SCAN_NBP; i++) {
    if(pscan->cpxra[i]!=NULL) {
      ok= (pscan->pxra[i]= (double*) calloc(pscan->mpts, sizeof(double)))!= NULL;
    }
  }

  for(i=0; ok && i<SCAN_NBD; i++) {
    if(pscan->cdxda[i]!=NULL) {
      ok= (pscan->dxda[i]= (float*) calloc(pscan->mpts, sizeof(float)))!= NULL;
    }
  }

  if(!ok) {
    printf("saveData: %s: Memory allocation failled\n", pscan->name);
    deconnectScan(pscan);
    free(pnode);
    return -1;
  }
  
  if(!list_scan) {
    list_scan= pnode;
  } else {
    pcur= list_scan;
    while(pcur->nxt!=NULL) pcur= pcur->nxt;    
    pcur->nxt= pnode;
  }

  Debug1(1, "connectScan(%s) OK\n", pscan->name);

  return 0;
}


/*----------------------------------------------------------------------*/
/* Disconnect a SCAN from the scan record				*/
/* ei clear all ca connections and events and free all buffers.		*/
/* pscan: a pointer to a SCAN structure					*/
/* return 0 if successful						*/
/*       -1 otherwise							*/
LOCAL int deconnectScan(SCAN* pscan)
{
  int i;

  Debug1(1, "deconnectScan(%s)...\n", pscan->name);

  Debug1(2, "Deconnect %s\n", ca_name(pscan->chandShake));
  if(pscan->chandShake) ca_clear_channel(pscan->chandShake);

  /* deconnect fields							*/
  Debug1(2, "Deconnect %s.DATA\n", pscan->name);
  if(pscan->cdata) ca_clear_channel(pscan->cdata);
  Debug1(2, "Deconnect %s.MPTS\n", pscan->name);
  if(pscan->cmpts) ca_clear_channel(pscan->cmpts);
  Debug1(2, "Deconnect %s.NPTS\n", pscan->name);
  if(pscan->cnpts) ca_clear_channel(pscan->cnpts);
  Debug1(2, "Deconnect %s.CPT\n", pscan->name);
  if(pscan->ccpt) ca_clear_channel(pscan->ccpt);

  for(i=0; i<SCAN_NBP; i++) {
    Debug2(2, "Deconnect %s.PXNV[%d]\n", pscan->name, i); 
    if(pscan->cpxnv[i]) ca_clear_channel(pscan->cpxnv[i]);
    Debug2(2, "Deconnect %s.PXPV[%d]\n", pscan->name, i); 
    if(pscan->cpxpv[i]) ca_clear_channel(pscan->cpxpv[i]);
    Debug2(2, "Deconnect %s.PXDS[%d]\n", pscan->name, i); 
    if(pscan->cpxds[i]) ca_clear_channel(pscan->cpxds[i]);
    Debug2(2, "Deconnect %s.PXEU[%d]\n", pscan->name, i); 
    if(pscan->cpxeu[i]) ca_clear_channel(pscan->cpxeu[i]);
    Debug2(2, "Deconnect %s.PXSM[%d]\n", pscan->name, i); 
    if(pscan->cpxsm[i]) ca_clear_channel(pscan->cpxsm[i]);

    Debug2(2, "Deconnect %s.RXNV[%d]\n", pscan->name, i); 
    if(pscan->crxnv) ca_clear_channel(pscan->crxnv[i]);
    Debug2(2, "Deconnect %s.RXPV[%d]\n", pscan->name, i); 
    if(pscan->crxpv[i]) ca_clear_channel(pscan->crxpv[i]);
    Debug2(2, "Deconnect %s.RXDS[%d]\n", pscan->name, i); 
    if(pscan->crxds[i]) ca_clear_channel(pscan->crxds[i]);
    Debug2(2, "Deconnect %s.RXEU[%d]\n", pscan->name, i); 
    if(pscan->crxeu[i]) ca_clear_channel(pscan->crxeu[i]);
    Debug2(2, "Deconnect %s.PXRA[%d]\n", pscan->name, i); 
    if(pscan->cpxra[i]) ca_clear_channel(pscan->cpxra[i]);
    Debug2(2, "Deconnect %s.RXCV[%d]\n", pscan->name, i); 
    if(pscan->crxcv[i]) ca_clear_channel(pscan->crxcv[i]);
  }

  for(i=0; i<SCAN_NBD; i++) {
    Debug2(2, "Deconnect %s.DXNV[%d]\n", pscan->name, i); 
    if(pscan->cdxnv[i]) ca_clear_channel(pscan->cdxnv[i]);
    Debug2(2, "Deconnect %s.DXPV[%d]\n", pscan->name, i); 
    if(pscan->cdxpv[i]) ca_clear_channel(pscan->cdxpv[i]);
    Debug2(2, "Deconnect %s.DXDS[%d]\n", pscan->name, i);     
    if(pscan->cdxds[i]) ca_clear_channel(pscan->cdxds[i]);
    Debug2(2, "Deconnect %s.PXEU[%d]\n", pscan->name, i); 
    if(pscan->cdxeu[i]) ca_clear_channel(pscan->cdxeu[i]);
    Debug2(2, "Deconnect %s.DXDA[%d]\n", pscan->name, i); 
    if(pscan->cdxda[i]) ca_clear_channel(pscan->cdxda[i]);
    Debug2(2, "Deconnect %s.DXCV[%d]\n", pscan->name, i); 
    if(pscan->cdxcv[i]) ca_clear_channel(pscan->cdxcv[i]);
  }

  for(i=0; i<SCAN_NBT; i++) {
    Debug2(2, "Deconnect %s.TXNV[%d]\n", pscan->name, i); 
    if(pscan->ctxnv[i]) ca_clear_channel(pscan->ctxnv[i]);
    Debug2(2, "Deconnect %s.TXPV[%d]\n", pscan->name, i); 
    if(pscan->ctxpv[i]) ca_clear_channel(pscan->ctxpv[i]);
    Debug2(2, "Deconnect %s.TXCD[%d]\n", pscan->name, i); 
    if(pscan->ctxcd[i]) ca_clear_channel(pscan->ctxcd[i]);
  }

  /* free buffers							*/
  for(i=0; i<SCAN_NBP; i++) {
    Debug1(2, "Free pxra[%d]\n", i);
    if(pscan->pxra[i]) free(pscan->pxra[i]);
  }

  for(i=0; i<SCAN_NBD; i++) {
    Debug1(2, "Free dxda[%d]\n", i);
    if(pscan->dxda[i]) free(pscan->dxda[i]);
  }

  Debug1(1, "deconnectScan(%s) OK\n", pscan->name);
  
  return 0;
}



/*----------------------------------------------------------------------*/
/* start the monitoring of a scan record.             			*/
/* pscan: a pointer to the SCAN structure connected to the scan record.	*/
/* return 0 if all monitors have been successfuly added.           	*/
/*        -1 otherwise.							*/
LOCAL int monitorScan(SCAN* pscan)
{  
  int i;

  Debug1(1, "monitorScan(%s)...\n", pscan->name);

  if(ca_add_event(DBR_SHORT, pscan->cnpts, 
                  nptsMonitor, (void*)NULL, 0)!=ECA_NORMAL) {
    Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->cnpts));
    return -1;
  }
  
  for(i=0; i<SCAN_NBP; i++) { 
    if(pscan->cpxnv[i]!=NULL &&
       pscan->cpxsm[i]!=NULL &&
       pscan->crxnv[i]!=NULL) {
      if(ca_add_event(DBR_SHORT, pscan->cpxnv[i], pxnvMonitor, 
                      (void*)i, 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->cpxnv[i]));
        return -1;
      }
      if(ca_add_event(DBR_STRING, pscan->cpxsm[i], pxsmMonitor, 
                      (void*)pscan->pxsm[i], 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->cpxsm[i]));
        return -1;
      }
      if(ca_add_event(DBR_SHORT, pscan->crxnv[i], rxnvMonitor, 
                      (void*)i, 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->crxnv[i]));
        return -1;
      }
    }
  }
  
  for(i=0; i<SCAN_NBD; i++) {
    if(pscan->cdxnv[i]!=NULL) {
      if(ca_add_event(DBR_SHORT, pscan->cdxnv[i], dxnvMonitor, 
                      (void*)i, 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->cdxnv[i]));
        return -1;
      }
    }
  }

  for(i=0; i<SCAN_NBT; i++) {
    if(pscan->ctxnv[i]!=NULL &&
       pscan->ctxcd[i]!=NULL) {
      if(ca_add_event(DBR_SHORT, pscan->ctxnv[i], txnvMonitor, 
                      (void*)i, 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->ctxnv[i]));
        return -1;
      }
      if(ca_add_event(DBR_FLOAT, pscan->ctxcd[i], txcdMonitor, 
                      (void*)i, 0)!=ECA_NORMAL) {
        Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->ctxcd[i]));
        return -1;
      }
    }
  }



  if(ca_add_event(DBR_TIME_SHORT, pscan->cdata, 
                  dataMonitor, (void*)NULL, 0)!=ECA_NORMAL) {
    Debug1(2, "Unable to post monitor on %s\n", ca_name(pscan->cdata));
    return -1;
  }

  Debug1(1, "monitorScan(%s) OK\n", pscan->name);

  return 0;
}



LOCAL void updateScan(SCAN* pscan)
{
  int i;

  pscan->nxt=0;
  for(i=0; i<SCAN_NBT; i++) {
    if(pscan->txsc[i]==0 && pscan->txcd[i]!=0) {
      pscan->nxt= searchScan(pscan->txpv[i]);
      if(pscan->nxt) break;
    }
  }
  if(!(pscan->nxt) && (realTime1D==0)) {
    if(pscan->cpt_monitored==TRUE) {
      ca_clear_event(pscan->cpt_evid);
      pscan->cpt_monitored= FALSE;
      pscan->all_pts= FALSE;
    }
  } else {
    if(pscan->cpt_monitored==FALSE) {
      ca_add_event(DBR_SHORT, pscan->ccpt, cptMonitor, NULL, &pscan->cpt_evid);
      pscan->cpt_monitored=TRUE;
      pscan->all_pts= FALSE;
    }
  }
}

LOCAL void updateScans()
{
  SCAN_NODE* pnode;

  pnode= list_scan;
  while(pnode) {
    updateScan(&pnode->scan);
    pnode= pnode->nxt;
  }
}


/*----------------------------------------------------------------------*/
/* monitor all SCAN of the list.                             		*/
LOCAL void monitorScans()
{
  SCAN_NODE* pnode;

  pnode= list_scan;
  while(pnode) {
    monitorScan(&pnode->scan);
    pnode= pnode->nxt;
  }
  updateScans();
}

/*----------------------------------------------------------------------*/
/* search for a scan in the SCAN list.                       		*/
/* name: the name of the scan to look for.				*/
/* return a pointer to the SCAN structure if found			*/
/*        NULL otherwise.						*/
LOCAL SCAN* searchScan(char* name)
{
  SCAN_NODE* current;

  current= list_scan;
  while(current) {
    if(strcmp(current->scan.name, name)==0) {
      return &current->scan;
    }
    current= current->nxt;
  }
  return NULL;
}


/*----------------------------------------------------------------------*/
/* print debug info on a SCAN structure.                     		*/

LOCAL char* cs[]= {
  "valid chid, IOC not found                 ",
  "valid chid, IOC was found but unavailable ",
  "valid chid, IOC was found, still available",
  "invalid chid                              "
};

LOCAL void infoScan(SCAN* pscan)
{
  int i;

  printf("scan name: %s\n", pscan->name);

  if(pscan->chandShake) {
    printf("hand shake: %s\n", ca_name(pscan->chandShake));
  } else {
    printf(" No hand shake\n");
  }

  printf("%s.DATA[%s]= %d\n", pscan->name, 
         cs[ca_state(pscan->cdata)], pscan->data);
  printf("%s.MPTS[%s]= %d\n", pscan->name,
         cs[ca_state(pscan->cmpts)], pscan->mpts);
  printf("%s.NPTS[%s]= %d\n", pscan->name,
         cs[ca_state(pscan->cnpts)], pscan->npts);
  printf("%s.CPT [%s]= %d\n", pscan->name,
         cs[ca_state(pscan->ccpt)], pscan->cpt);
   
  for(i=0; i<SCAN_NBP; i++)
    if(pscan->cpxnv[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, pxnv[i], cs[ca_state(pscan->cpxnv[i])], pscan->pxnv[i]);
  for(i=0; i<SCAN_NBP; i++) {
    if(pscan->cpxpv[i]!=NULL) {
      printf("%s.%s[%s]= %s\n", pscan->name, pxpv[i], cs[ca_state(pscan->cpxpv[i])], pscan->pxpv[i]);
      printf("  DESC: %s\n", pscan->pxds[i]);
      printf("  EGU : %s\n", pscan->pxeu[i].units);
    }
  }
  for(i=0; i<SCAN_NBP; i++)
    if(pscan->cpxsm[i]!=NULL) printf("%s.%s[%s]= %s\n", pscan->name, pxsm[i], cs[ca_state(pscan->cpxsm[i])], pscan->pxsm[i]);
  for(i=0; i<SCAN_NBP; i++)
    if(pscan->crxnv[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, rxnv[i], cs[ca_state(pscan->crxnv[i])], pscan->rxnv[i]);
  for(i=0; i<SCAN_NBP; i++) {
    if(pscan->crxpv[i]!=NULL) {
      printf("%s.%s[%s]= %s\n", pscan->name, rxpv[i],
             cs[ca_state(pscan->crxpv[i])], pscan->rxpv[i]);
      printf("  DESC: %s\n", pscan->rxds[i]);
      printf("  EGU : %s\n", pscan->rxeu[i].units);
    }
  }
  for(i=0; i<SCAN_NBP; i++)
    if(pscan->cpxra[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, pxra[i], cs[ca_state(pscan->cpxra[i])], (int)pscan->pxra[i]);
  for(i=0; i<SCAN_NBP; i++)
    if(pscan->crxcv[i]!=NULL) printf("%s.%s[%s]= %f\n", pscan->name, rxcv[i], cs[ca_state(pscan->crxcv[i])], pscan->rxcv[i]);

  
  for(i=0; i<SCAN_NBD; i++)
    if(pscan->cdxnv[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, dxnv[i], cs[ca_state(pscan->cdxnv[i])], pscan->dxnv[i]);
  for(i=0; i<SCAN_NBD; i++) {
    if(pscan->cdxpv[i]!=NULL) {
      printf("%s.%s[%s]= %s\n", pscan->name, dxpv[i],
             cs[ca_state(pscan->cdxpv[i])], pscan->dxpv[i]);
      printf("  DESC: %s\n", pscan->dxds[i]);
      printf("  EGU : %s\n", pscan->dxeu[i].units);
    }
  }
  for(i=0; i<SCAN_NBD; i++)
    if(pscan->cdxda[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, dxda[i], cs[ca_state(pscan->cdxda[i])], (int)pscan->dxda[i]);
  for(i=0; i<SCAN_NBD; i++)
    if(pscan->cdxcv[i]!=NULL) printf("%s.%s[%s]= %f\n", pscan->name, dxcv[i], cs[ca_state(pscan->cdxcv[i])], pscan->dxcv[i]);
  
  for(i=0; i<SCAN_NBT; i++)
    if(pscan->ctxnv[i]!=NULL) printf("%s.%s[%s]= %d\n", pscan->name, txnv[i], cs[ca_state(pscan->ctxnv[i])], pscan->txnv[i]);
  for(i=0; i<SCAN_NBT; i++)
    if(pscan->ctxpv[i]!=NULL) printf("%s.%s[%s]= %s\n", pscan->name, txpv[i], cs[ca_state(pscan->ctxpv[i])], pscan->txpv[i]);
  for(i=0; i<SCAN_NBT; i++)
    if(pscan->ctxcd[i]!=NULL) printf("%s.%s[%s]= %f\n", pscan->name, txcd[i], cs[ca_state(pscan->ctxcd[i])], pscan->txcd[i]);
}


/*----------------------------------------------------------------------*/
/* DATA field monitor.                                                  */
/*                                                                      */
LOCAL void dataMonitor(struct event_handler_args eha)
{
  struct dbr_time_short *pval;
  SCAN* pscan;
  short sval;
  char  disp;

  pscan= (SCAN*)ca_puser(eha.chid);
  pval = (struct dbr_time_short *) eha.dbr;
  sval= pval->value;
  if((pscan->data!=-1) && (sval==0)) {

    if(pscan->chandShake) {
      if ((save_status != STATUS_INACTIVE) && (save_status != STATUS_ACTIVE_FS_ERROR)) {
        /* hand shake busy						*/
        sval= HANDSHAKE_BUSY;
        ca_array_put(DBR_SHORT, 1, pscan->chandShake, &sval);
      }
    }
    if(nb_scan_running++ == 0) {
      /* disable put to filesystem and subdir				*/
      disp=(char)1;
      ca_array_put(DBR_CHAR, 1, file_system_disp_chid, &disp);
      disp=(char)1;
      ca_array_put(DBR_CHAR, 1, file_subdir_disp_chid, &disp);
      disp=(char)0;
      ca_array_put(DBR_STRING, 1, message_chid, &disp);
    }
    Debug1(2,"\n nb_scan_running=%d\n", nb_scan_running);
  }

  tsStampToText(&pval->stamp, TS_TEXT_MONDDYYYY, pscan->stamp);
  if(strlen(pscan->stamp)<31) {
    memset(pscan->stamp+strlen(pscan->stamp), '0', 31-strlen(pscan->stamp));
  }
  pscan->stamp[31]='\0';

  sendScanShortMsg(MSG_SCAN_DATA, (SCAN*)ca_puser(eha.chid),
                   pval->value, WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* NPTS field monitor.                                        		*/
/*                                                			*/
LOCAL void nptsMonitor(struct event_handler_args eha)
{
    sendScanShortMsg(MSG_SCAN_NPTS, (SCAN *) ca_puser(eha.chid), *((short *) eha.dbr), WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* CPT field monitor.                                        		*/
/*                                                			*/
LOCAL void cptMonitor(struct event_handler_args eha)
{
  SCAN* pscan;
  ULONG ticks;

  switch(saveData_MessagePolicy) {
  case 0:
    sendScanShortMsg(MSG_SCAN_CPT, (SCAN *) ca_puser(eha.chid), *((short *) eha.dbr), WAIT_FOREVER);
    break;
  case 1:
    sendScanShortMsg(MSG_SCAN_CPT, (SCAN *) ca_puser(eha.chid), *((short *) eha.dbr), NO_WAIT);
    break;
  case 2:
    pscan = (SCAN *) ca_puser(eha.chid);
    ticks= tickGet();
    if(ticks-pscan->cpt_ticks>=cpt_wait_ticks) {
      pscan->cpt_ticks= ticks;
      sendScanShortMsg(MSG_SCAN_CPT, (SCAN *) ca_puser(eha.chid), *((short *) eha.dbr), NO_WAIT);
    }
    break;
  case 3:
    pscan = (SCAN *) ca_puser(eha.chid);
    ticks= tickGet();
    if(ticks-pscan->cpt_ticks>=cpt_wait_ticks) {
      pscan->cpt_ticks= ticks;
      sendScanShortMsg(MSG_SCAN_CPT, (SCAN *) ca_puser(eha.chid), *((short *) eha.dbr), WAIT_FOREVER);
    }
    break;
  }
}

/*----------------------------------------------------------------------*/
/* PXNV field monitor.                                        		*/
/*                                                			*/
LOCAL void pxnvMonitor(struct event_handler_args eha)
{
    sendScanIndexMsg(MSG_SCAN_PXNV, (SCAN *) ca_puser(eha.chid), (int) eha.usr, *((short *) eha.dbr), WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* PXSM field monitor.                                        		*/
/*                                                			*/
LOCAL void pxsmMonitor(struct event_handler_args eha)
{
    sendStringMsg(MSG_SCAN_PXSM, eha.usr, eha.dbr, WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* RXNV field monitor.                                        		*/
/*                                                			*/
LOCAL void rxnvMonitor(struct event_handler_args eha)
{
    sendScanIndexMsg(MSG_SCAN_RXNV, (SCAN *) ca_puser(eha.chid), (int) eha.usr, *((short *) eha.dbr), WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* DXNV field monitor.                                        		*/
/*                                                			*/
LOCAL void dxnvMonitor(struct event_handler_args eha)
{
    sendScanIndexMsg(MSG_SCAN_DXNV, (SCAN *) ca_puser(eha.chid), (int) eha.usr, *((short *) eha.dbr), WAIT_FOREVER);
}


/*----------------------------------------------------------------------*/
/* TXNV field monitor.                                        		*/
/*                                                			*/
LOCAL void txnvMonitor(struct event_handler_args eha)
{
    sendScanIndexMsg(MSG_SCAN_TXNV, (SCAN *) ca_puser(eha.chid), (int) eha.usr, *((short *) eha.dbr), WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* TXCD field monitor.                                        		*/
/*                                                			*/
LOCAL void txcdMonitor(struct event_handler_args eha)
{
    sendScanIndexMsg(MSG_SCAN_TXCD, (SCAN *) ca_puser(eha.chid), (int) eha.usr, *((short *) eha.dbr), WAIT_FOREVER);
}

/*----------------------------------------------------------------------*/
/* DESC field monitor.                                        		*/
/*                                                			*/
LOCAL void descMonitor(struct event_handler_args eha)
{
    sendStringMsg(MSG_DESC, eha.usr, eha.dbr, WAIT_FOREVER);
}


LOCAL void fileSystemMonitor(struct event_handler_args eha)
{
    sendStringMsg(MSG_FILE_SYSTEM, NULL, eha.dbr, WAIT_FOREVER);
}

LOCAL void fileSubdirMonitor(struct event_handler_args eha)
{
    sendStringMsg(MSG_FILE_SUBDIR, NULL, eha.dbr, WAIT_FOREVER);
}


LOCAL void realTime1DMonitor(struct event_handler_args eha)
{
    sendIntegerMsg(MSG_REALTIME1D, *((int *) eha.dbr), WAIT_FOREVER);
}


LOCAL int connectCounter(char* name)
{
  counter= 0;

  ca_search(name, &counter_chid);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    Debug1(1, "Unable to connect counter %s\n", 
           name);
    return -1;
  }
  return 0;
}

LOCAL int connectFileSystem(char* fs)
{
  char fs_disp[80];
  file_system_state= FS_NOT_MOUNTED;

  sprintf(fs_disp, "%s.DISP", fs);

  ca_search(fs, &file_system_chid);
  ca_search(fs_disp, &file_system_disp_chid);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    printf("saveData: Unable to connect %s\nsaveDataTask not initialized\n", 
           fs);
    return -1;
  } else {
    if(ca_add_event(DBR_STRING, file_system_chid, 
                    fileSystemMonitor, NULL, NULL)!=ECA_NORMAL) {
      printf("saveData: Unable to post monitor on %s\nsaveDataTask not initialized\n"
             , fs);
      ca_clear_channel(file_system_chid);
      return -1;
    }
  }
  return 0;
}

LOCAL int connectSubdir(char* sd)
{
  char sd_disp[80];

  sprintf(sd_disp, "%s.DISP", sd);
  ca_search(sd, &file_subdir_chid);
  ca_search(sd_disp, &file_subdir_disp_chid);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    printf("saveData: Unable to connect %s\nsaveDataTask not initialized\n", 
           sd);
    return -1;
  } else {
    if(ca_add_event(DBR_STRING, file_subdir_chid, 
                    fileSubdirMonitor, NULL, NULL)!=ECA_NORMAL) {
      printf("saveData: Unable to post monitor on %s\nsaveDataTask not initialized\n"
             , sd);
      ca_clear_channel(file_subdir_chid);
      return -1;
    }
  }
  return 0;
}

LOCAL int connectRealTime1D(char* rt)
{
  ca_search(rt, &realTime1D_chid);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    printf("saveData: Unable to connect %s\n", rt);
    return -1;
  } else {
    if(ca_add_event(DBR_SHORT, realTime1D_chid,
                    realTime1DMonitor, NULL, NULL)!= ECA_NORMAL) {
      printf("saveData: Unable to post monitor on %s\n", rt);
      ca_clear_channel(realTime1D_chid);
      return -1;
    }
  }
  return 0;
}


LOCAL void extraValCallback(struct event_handler_args eha)
{
  PV_NODE * pnode = eha.usr;
  long type = eha.type;
  long count = eha.count;
  DBR_VAL * pval = eha.dbr;

  size_t size=0;

  semTake(pnode->lock, WAIT_FOREVER);

  switch(type) {
  case DBR_STRING:
    size= strlen((char*)pval);
    break;
  case DBR_CTRL_CHAR:
    size= dbr_size[DBR_CTRL_CHAR]+(count-1);
    break;
  case DBR_CTRL_SHORT:
    size= dbr_size[DBR_CTRL_SHORT]+(count-1)*sizeof(short);
    break;
  case DBR_CTRL_LONG:
    size= dbr_size[DBR_CTRL_LONG]+(count-1)*sizeof(long);
    break;
  case DBR_CTRL_FLOAT:
    size= dbr_size[DBR_CTRL_FLOAT]+(count-1)*sizeof(float);
    break;
  case DBR_CTRL_DOUBLE:
    size= dbr_size[DBR_CTRL_DOUBLE]+(count-1)*sizeof(double);
    break;
  default:
    printf("saveDta: unsuported dbr_type %d\n", (int)type);
    semGive(pnode->lock);
    return;
  }

  memcpy(pnode->pval, pval, size);
  pnode->count= count;

  semGive(pnode->lock);
}

LOCAL void extraDescCallback(struct event_handler_args eha)
{ 
  PV_NODE * pnode = eha.usr;
  DBR_VAL * pval = eha.dbr;

  semTake(pnode->lock, WAIT_FOREVER);

  strcpy(pnode->desc, pval);
  ca_clear_channel(pnode->desc_chid);

  semGive(pnode->lock);
}


LOCAL int connectPV(char* pv, char* desc)
{
  PV_NODE* pnode;
  PV_NODE* pcur;
  char buff[40];
  long  count;
  int   type;
  int   len;
  long  size;

  /* allocate space for the new pv */
  pnode= (PV_NODE*) malloc(sizeof(PV_NODE));
  if(!pnode) {
    printf("saveData: Unable to add %s\n", pv);
    return -1;
  }
  
  /* set everything to 0 */
  memset(pnode, 0, sizeof(PV_NODE));

  /* try to connect the pv */
  ca_search(pv, &pnode->channel);
  if(ca_pend_io(10)!=ECA_NORMAL) {
    /* Unable to connect in 10 seconds. Discard the pv ! */
    printf("saveData: Unable to connect to %s\n", pv);
    ca_clear_channel(pnode->channel);
    free(pnode);
    return -1;
  }

  strncpy(pnode->name, pv, 28);
  pnode->name[28]= '\0';

  /* Allocate space for the pv's value */
  size= 0;
  type= ca_field_type(pnode->channel);
  count= ca_element_count(pnode->channel);
  switch(type) {
  case DBR_STRING:
    pnode->dbr_type= DBR_STRING;
    count= 1;
    size= 40;
    break;
  case DBR_CHAR:
    pnode->dbr_type= DBR_CTRL_CHAR;
    size= dbr_size[DBR_CTRL_CHAR]+ (count-1);
    break;
  case DBR_SHORT:
    pnode->dbr_type= DBR_CTRL_SHORT;
    size= dbr_size[DBR_CTRL_SHORT]+ (count-1)*sizeof(short);
    break;
  case DBR_ENUM:
    pnode->dbr_type= DBR_STRING;
    count= 1;
    size= 40;
    break;
  case DBR_LONG:
    pnode->dbr_type= DBR_CTRL_LONG;
    size= dbr_size[DBR_CTRL_LONG]+ (count-1) * sizeof(long);
    break;
  case DBR_FLOAT:
    pnode->dbr_type= DBR_CTRL_FLOAT;
    size= dbr_size[DBR_CTRL_FLOAT]+ (count-1) * sizeof(float);
    break;
  case DBR_DOUBLE:
    pnode->dbr_type= DBR_CTRL_DOUBLE;
    size= dbr_size[DBR_CTRL_DOUBLE]+ (count-1) * sizeof(double);
    break;
  default:
    printf("saveData: %s has an unsuported type\n", pv);
    ca_clear_channel(pnode->channel);
    free(pnode);
    return -1;
  }

  pnode->pval= malloc(size);
  memset(pnode->pval, 0, size);

  pnode->lock= semBCreate(SEM_Q_FIFO, SEM_FULL);
  /* Get a first image of the pv's value */
  ca_array_get_callback(pnode->dbr_type, count,
                        pnode->channel, extraValCallback, (void*)pnode);

  /* Get the pv's description */
  if(!desc || (*desc=='\0')) {
    /* The description was not given in the req file. */
    /* Get it from the .DESC field */
    len= strcspn(pv, ".");
    strncpy(buff, pv, len);
    buff[len]='\0';
    strcat(buff, ".DESC");
    ca_search(buff, &pnode->desc_chid);
    pnode->desc[0]='\0';
    if(ca_pend_io(10)!=ECA_NORMAL) {
      printf("saveData: Unable to connect to %s\n", buff);
      ca_clear_channel(pnode->desc_chid);
    } else {
      ca_array_get_callback(DBR_STRING, 1, pnode->desc_chid, 
                            extraDescCallback, (void*)pnode);
    }
  } else {
    /* Copy the description from the req file. */
    strncpy(pnode->desc, desc, 39);
    pnode->desc[39]='\0';
  }

  /* Append the pv to the list */
  if(!list_pv) {
    list_pv= pnode;
  } else {
    pcur= list_pv;
    while(pcur->nxt) pcur=pcur->nxt;
    pcur->nxt= pnode;
  }

  /* Increase the number of extra pvs */
  nb_extraPV++;

  return 0;
}

LOCAL int initSaveDataTask() 
{
  REQ_FILE* rf;
  char  buff1[41];
  char  buff2[41];
  int i;

  server_pathname[0]= '\0';
  server_subdir= server_pathname;
  strcpy(local_pathname, "/data/");
  local_subdir= &local_pathname[strlen(local_pathname)];

  rf= req_open_file(req_file, req_macros);
  if(!rf) {
    Debug1(1, "Unable to open \"%s\". saveDataTask aborted\n", req_file);
    return -1;
  }

  /* get the IOC prefix							*/
  ioc_prefix[0]='\0';
  if(req_gotoSection(rf, "prefix")==0) {
    req_readMacId(rf, ioc_prefix, 9);
  }
  for (i=0; i<10 && ioc_prefix[i]; i++) {
    if (ispunct((int)ioc_prefix[i])) ioc_prefix[i] = '_';
  }

  /* Connect to saveData_active						*/
  if(req_gotoSection(rf, "status")!=0) {
    printf("saveData: section [status] not found\n");
    return -1;
  }
  if(req_readMacId(rf, buff1, 40)==0) {
    printf("saveData: status pv name not defined\n");
    return -1;
  }
  ca_search(buff1, &save_status_chid);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    printf("saveData: Unable to connect %s\n", buff1);
    return -1;
  }

  /* Connect to saveData_message					*/
  message_chid= NULL;
  if(req_gotoSection(rf, "message")!=0) {
    printf("saveData: section [message] not found\n");
  } else {
    if(req_readMacId(rf, buff1, 40)==0) {
      printf("saveData: message pv name not defined\n");
    } else {
      ca_search(buff1, &message_chid);
      ca_pend_io(0.5);
    }
  }

  /* Connect to saveData_filename					*/
  filename_chid= NULL;
  if(req_gotoSection(rf, "filename")!=0) {
    printf("saveData: section [filename] not found\n");
  } else {
    if(req_readMacId(rf, buff1, 40)==0) {
      printf("saveData: filename pv name not defined\n");
    } else {
      ca_search(buff1, &filename_chid);
      ca_pend_io(0.5);
    }
  }

  /* Connect to saveData_fullPathName					*/
  full_pathname_chid= NULL;
  if(req_gotoSection(rf, "fullPathName")!=0) {
    printf("saveData: section [fullPathName] not found\n");
  } else {
    if(req_readMacId(rf, buff1, 40)==0) {
      printf("saveData: fullPathName pv name not defined\n");
    } else {
      ca_search(buff1, &full_pathname_chid);
      ca_pend_io(0.5);
    }
  }

  /* Connect to saveData_scanNumber					*/
  if(req_gotoSection(rf, "counter")!=0) {
    printf("saveData: section [counter] not found\n");
    return -1;
  }
  if(req_readMacId(rf, buff1, 40)==0) {
    printf("saveData: counter pv name not defined\n");
    return -1;
  }
  if(connectCounter(buff1)==-1) return -1;
    
  
  /* Connect to saveData_fileSystem					*/
  if(req_gotoSection(rf, "fileSystem")!=0) {
    printf("saveData: section [fileSystem] not found\n");
    return -1;
  }
  if(req_readMacId(rf, buff1, 40)==0) {
    printf("saveData: fileSystem pv name not defined\n");
    return -1;
  }
  if(connectFileSystem(buff1)==-1) return -1;


  /* Connect to saveData_baseName					*/
  if(req_gotoSection(rf, "subdir")!=0) {
    printf("saveData: section [subdir] not found\n");
    return -1;
  }
  if(req_readMacId(rf, buff1, 40)==0) {
    printf("saveData: baseName pv name not defined\n");
    return -1;
  }
  if(connectSubdir(buff1)==-1) return -1;

  /* Connect all scan records.      					*/
  if(req_gotoSection(rf, "scanRecord")==0) {
    while(!eos(rf)) {
      req_readMacId(rf, buff1, 40);
      if(current(rf)==',') {
        req_readChar(rf);
        req_readMacId(rf, buff2, 40);
      } else {
        buff2[0]= '\0';
      }
      connectScan(buff1, buff2);
    }
  }

  /* Connect all extra pvnames      					*/
  nb_extraPV= 0;
  if(req_gotoSection(rf, "extraPV")==0) {
    while(!eos(rf)) {
      req_readMacId(rf, buff1, 40);
      if(current(rf)=='"') {
        req_readString(rf, buff2, 40);
      } else {
        buff2[0]= '\0';
      }
      connectPV(buff1, buff2);
    }
  }


  /* Connect to saveData_realTime1D					*/
  if(req_gotoSection(rf, "realTime1D")!=0) {
    printf("saveData: section [realTime1D] not found\n");
  } else {
    if(req_readMacId(rf, buff1, 40)==0) {
      printf("saveData: realTime1D pv name not defined\n");
    } else {
      connectRealTime1D(buff1);
    }
  }

  req_close_file(rf);
  
  monitorScans();

  return 0;
}

LOCAL int  scan_getDim(SCAN* pscan)
{
  int   dim= 0;
  SCAN* cur= pscan;

  while(cur) {
    dim++;
    cur= cur->nxt;
  }
  return dim;
}

LOCAL void getExtraPV()
{
  PV_NODE* pcur;
  chid     channel;

  pcur= list_pv;
  while(pcur) {
    channel= pcur->channel;
    ca_array_get_callback(pcur->dbr_type, ca_element_count(channel),
                          channel, extraValCallback, (void*)pcur);
    pcur= pcur->nxt;
  }
}

LOCAL void saveExtraPV(XDR* pxdrs)
{
  PV_NODE* pcur;
  chid     channel;
  int      type;
  DBR_VAL* pval;
  long     count;
  char*    cptr;

  /* number of pv saved */
  xdr_int(pxdrs, &nb_extraPV);

  if(nb_extraPV>0) {

    pcur= list_pv;
    while(pcur) {
      semTake(pcur->lock, WAIT_FOREVER);

      channel= pcur->channel;
      pval= pcur->pval;
      
      cptr= pcur->name;
      xdr_counted_string(pxdrs, &cptr);

      cptr= pcur->desc;
      xdr_counted_string(pxdrs, &cptr);
      
      type= pcur->dbr_type;
      xdr_int(pxdrs, &type);

      if(type!=DBR_STRING) {
        count= pcur->count;
        xdr_long(pxdrs, &count);
      }

      switch(type) {
      case DBR_STRING:
        xdr_counted_string(pxdrs, (char**)&pval);
        break;
      case DBR_CTRL_CHAR:
        cptr= pval->cchrval.units;
        xdr_counted_string(pxdrs, &cptr);
        /* xdr_bytes(pxdrs,(char**)&pval->cchrval.value,&count, count); */
        xdr_vector(pxdrs,(char*)&pval->cchrval.value,count,sizeof(char),xdr_char);
        break;
      case DBR_CTRL_SHORT:
        cptr= pval->cshrtval.units;
        xdr_counted_string(pxdrs, &cptr);
        xdr_vector(pxdrs,(char*)&pval->cshrtval.value,count,sizeof(short),xdr_short);
        break;
      case DBR_CTRL_LONG:
        cptr= pval->clngval.units;
        xdr_counted_string(pxdrs, &cptr);
        xdr_vector(pxdrs,(char*)&pval->clngval.value,count, sizeof(long),xdr_long);
        break;
      case DBR_CTRL_FLOAT:
        cptr= pval->cfltval.units;
        xdr_counted_string(pxdrs, &cptr);
        xdr_vector(pxdrs,(char*)&pval->cfltval.value,count, sizeof(float),xdr_float);
        break;
      case DBR_CTRL_DOUBLE:
        cptr= pval->cdblval.units;
        xdr_counted_string(pxdrs, &cptr);
        xdr_vector(pxdrs,(char*)&pval->cdblval.value,count,
                   sizeof(double),xdr_double);
        break;
      }

      semGive(pcur->lock);
      pcur= pcur->nxt;
    }
  }
}

LOCAL void reset_old_npts(SCAN *pscan) {
  pscan->old_npts= -1;
  if(pscan->nxt) reset_old_npts(pscan->nxt);
}

LOCAL void proc_scan_data(SCAN_SHORT_MSG* pmsg)
{
  char  msg[200];
  char* cptr;
  SCAN* pscan;
  FILE* fd;
  XDR   xdrs;
  int   i;

  char  cval;
  short sval;
  int   ival, duplicate_scan_number;
  long  lval;
  long  scan_offset;
  long  data_size;

  pscan= pmsg->pscan;  

  if (pscan->data ==-1) {
    /* this should never happen */
    pscan->data = pmsg->val;
    Debug1(2,"!!!%s pscan->data == -1!!!\n", pscan->name);
    return;
  }

  if ((save_status == STATUS_INACTIVE) || (save_status == STATUS_ACTIVE_FS_ERROR)) {
    if (pmsg->val==0) {
      /* The file system is not mounted.  Warn user and punt */
      sendUserMessage("Scan not being saved !!!!!");
    } else {
      /* Scan is over.  Enable file system record					*/
      if(--nb_scan_running==0) {
        cval=(char)0;
        ca_array_put(DBR_CHAR, 1, file_system_disp_chid, &cval);
        cval=(char)0;
        ca_array_put(DBR_CHAR, 1, file_subdir_disp_chid, &cval);
      }
      Debug1(2,"(save_status inactive) nb_scan_running=%d\n", nb_scan_running);
    }
    pscan->data= pmsg->val;
    return;
  }

  if (pmsg->val==0) {
    Debug1(2, "scan started: %s\n", pscan->name);
    /* the scan is started, update counter and lower scan		*/
    pscan->data=0;

    /* initialize the scan */
    pscan->nb_pos=0;
    Debug0(3, "Checking number of valid positioner\n");
    for(i=0; i<SCAN_NBP; i++) {
      if((pscan->pxnv[i]==XXNV_OK) || (pscan->rxnv[i]==XXNV_OK)) {
        pscan->nb_pos++;
        /* request ctrl info for the positioner (unit) */
        if(pscan->cpxeu[i]) ca_array_get(DBR_CTRL_DOUBLE, 1, pscan->cpxeu[i], &pscan->pxeu[i]);
        /* request ctrl info for the readback (unit) */
        if(pscan->crxeu[i]) ca_array_get(DBR_CTRL_DOUBLE, 1, pscan->crxeu[i], &pscan->rxeu[i]);
        /* compute the number of valid pos/rdb in the scan */
      }
    }
    Debug0(3, "Checking number of valid detector\n");
    pscan->nb_det=0;
    for(i= 0; i<SCAN_NBD; i++) {
      if(pscan->dxnv[i]==XXNV_OK) {
        pscan->nb_det++;
        /* request ctrl info for the detector (unit) */
        if(pscan->cdxeu) ca_array_get(DBR_CTRL_FLOAT, 1, pscan->cdxeu[i], &pscan->dxeu[i]);
        /* compute the number of valid det in the scan */
      }
    }
    pscan->nb_trg=0;
    Debug0(3, "Checking number of valid trigger\n");
    for(i= 0; i<SCAN_NBT; i++) {
      /* compute the number of valid triger in the scan */
      if(pscan->txnv[i]==XXNV_OK) {
        pscan->nb_trg++;
      }
    }
    
    pscan->all_pts= FALSE;
    pscan->cpt= 0;
      
    /* make sure all request for units are completed */
    if(ca_pend_io(2.0)!=ECA_NORMAL) {
      printf("saveData: Unable to get all pos/rdb/det units\n");
    }

    /* Attempt to open data file */
    if (pscan->first_scan) {
      /* We're processing the outermost of a possibly multidimensional scan */
      Debug0(3, "Outermost scan\n");
        
      /* Get number for this scan */
      ca_array_get(DBR_LONG, 1, counter_chid, &counter);
      if(ca_pend_io(0.5)!=ECA_NORMAL) {
        /* error !!! */
        printf("saveData: unable to get scan number !!!\n");
      } else {
        pscan->counter = counter;
      }

      /* Make file name */
      sprintf(pscan->fname, "%s%.4d.mda", ioc_prefix, (int)pscan->counter);
      sprintf(pscan->ffname, "%s%s", local_pathname, pscan->fname);
      cptr= &pscan->ffname[strlen(pscan->ffname)];
      duplicate_scan_number = 0;
      while(fileStatus(pscan->ffname)==OK) {
        sprintf(cptr, "_%.2d", ++duplicate_scan_number);
      }
      if (duplicate_scan_number) {
        sprintf(&pscan->fname[strlen(pscan->fname)], "_%.2d", duplicate_scan_number);
      }

      Debug1(3, "Open file: %s\n", pscan->ffname);
      fd= fopen(pscan->ffname, "wb+");

    } else {
      Debug1(3, "Open file: %s\n", pscan->ffname);
      fd= fopen(pscan->ffname, "rb+");
      if (fd != NULL) fseek(fd, 0, SEEK_END);
    }

    if ((fd == NULL) || (fileStatus(pscan->ffname) == ERROR)) {
      printf("saveData:proc_scan_data: can't open data file!!\n");
      sprintf(msg, "Warning!! can't open file '%s'", pscan->fname);
      msg[39]= '\0';
      sendUserMessage(msg);
      save_status = STATUS_ERROR;
      ca_array_put(DBR_SHORT, 1, save_status_chid, &save_status);
      return;
    } else if (save_status == STATUS_ERROR) {
      save_status = STATUS_ACTIVE_OK;
      ca_array_put(DBR_SHORT, 1, save_status_chid, &save_status);
    }

    xdrstdio_create(&xdrs, fd, XDR_ENCODE);

    if(pscan->first_scan) {
      /* Tell user what we're doing */
      sprintf(msg, "saving: %s", pscan->fname);	  
      msg[39]= '\0';
      sendUserMessage(msg);

      /* Write file name where client can easily find it. */
      ca_array_put(DBR_LONG, 1, counter_chid, &counter);
      if (filename_chid) {
        ca_array_put(DBR_STRING, 1, filename_chid, pscan->fname);
      }

      /* increment scan number and write it to the PV */
      counter = pscan->counter + 1;
      ca_array_put(DBR_LONG, 1, counter_chid, &counter);
        
      /* Get all values of the extra pvs */
      getExtraPV();
        
      pscan->scan_dim= scan_getDim(pscan);
      reset_old_npts(pscan);

        
      /*----------------------------------------------------------------*/
      /* Write the file header */
      Debug0(3, "Writing file header\n");
      xdr_float(&xdrs, &file_format_version);       /* file version	*/
      xdr_long(&xdrs, &pscan->counter); /* scan number */
      xdr_short(&xdrs, &pscan->scan_dim);/* rank of the data */
      pscan->dims_offset= xdr_getpos(&xdrs);
      ival=-1;
      for(i=0; i<pscan->scan_dim; i++) {
        xdr_int(&xdrs, &ival);
      }
      pscan->regular_offset= xdr_getpos(&xdrs);
      ival=1;				  /* regular (rectangular array) = true */
      xdr_int(&xdrs, &ival);
        
      /* offset to the extraPVs */
      pscan->offset_extraPV= xdr_getpos(&xdrs);
      lval= 0;
      xdr_long(&xdrs, &lval);
      Debug0(3, "File Header written\n");
    }

    /* If an inner scan dimension exists, set it up. */
    if(pscan->nxt) {
      pscan->nxt->first_scan= FALSE;
      pscan->nxt->scan_dim= pscan->scan_dim-1;
      pscan->nxt->dims_offset= pscan->dims_offset+sizeof(int);
      pscan->nxt->regular_offset= pscan->regular_offset;
      strcpy(pscan->nxt->fname, pscan->fname);
      strcpy(pscan->nxt->ffname, pscan->ffname);
    }
      
    /* the offset of this scan						*/
    scan_offset= xdr_getpos(&xdrs);
      
    /*------------------------------------------------------------------------*/
    /* Scan header								*/
    Debug0(3, "Writing per-scan header\n");
    xdr_short(&xdrs, &pscan->scan_dim);	/* scan dimension		*/
    ival= (int)pscan->npts;
    xdr_int(&xdrs, &ival);		/* # of pts			*/
    pscan->cpt_fpos= xdr_getpos(&xdrs);
    ival= (int)pscan->cpt;		/* last valid point		*/
    xdr_int(&xdrs, &ival);    
      
    if(pscan->scan_dim>1) {			/* index of lower scans		*/
      lval= xdr_getpos(&xdrs);
      pscan->nxt->offset= lval;
      xdr_setpos(&xdrs, lval+pscan->npts*sizeof(long));
    }
      
    /*------------------------------------------------------------------------*/
    /* Scan info								*/
    Debug0(3, "Save scan info\n");
    cptr= pscan->name;
    xdr_counted_string(&xdrs, &cptr);	/* scan name		        */
    cptr= pscan->stamp;
    pscan->time_fpos= xdr_getpos(&xdrs);
    /*      strcpy(cptr, "------- NOT INITIALIZED -------");  */
    xdr_counted_string(&xdrs, &cptr);  /* time stamp			*/
      
    xdr_int(&xdrs, &pscan->nb_pos);	/* # of positioners		*/
    xdr_int(&xdrs, &pscan->nb_det);	/* # of detectors		*/
    xdr_int(&xdrs, &pscan->nb_trg);	/* # of detectors		*/
      
    if(pscan->nb_pos) {
      for(i=0; i<SCAN_NBP; i++) {
        if((pscan->rxnv[i]==XXNV_OK) || (pscan->pxnv[i]==XXNV_OK)) {
          Debug1(3, "Pos[%d] info\n", i);
          xdr_int(&xdrs, &i);		   /* positioner number		*/
          cptr= pscan->pxpv[i];
          xdr_counted_string(&xdrs, &cptr);/* positioner name		*/
          cptr= pscan->pxds[i];
          xdr_counted_string(&xdrs, &cptr);/* positioner desc		*/
          cptr= pscan->pxsm[i];
          xdr_counted_string(&xdrs, &cptr);/* positioner step mode	*/
          cptr= pscan->pxeu[i].units;
          xdr_counted_string(&xdrs, &cptr);/* positioner unit		*/
          cptr= pscan->rxpv[i];
          xdr_counted_string(&xdrs, &cptr);/* readback name		*/
          cptr= pscan->rxds[i];
          xdr_counted_string(&xdrs, &cptr);/* readback description	*/
          cptr= pscan->rxeu[i].units;
          xdr_counted_string(&xdrs, &cptr);/* readback unit		*/
        }
      }
    }
    if(pscan->nb_det) {
      for(i=0; i<SCAN_NBD; i++) {
        if(pscan->dxnv[i]==XXNV_OK) {
          Debug1(3, "Det[%d] info\n", i);
          xdr_int(&xdrs, &i);		   /* detector number		*/
          cptr= pscan->dxpv[i];
          xdr_counted_string(&xdrs, &cptr);/* detector name  		*/
          cptr= pscan->dxds[i];
          xdr_counted_string(&xdrs, &cptr);/* detector description   	*/
          cptr= pscan->dxeu[i].units;
          xdr_counted_string(&xdrs, &cptr);/* detector unit		*/
        }
      }
    }
    if(pscan->nb_trg) {
      for(i=0; i<SCAN_NBT; i++) {
        if(pscan->txnv[i]==XXNV_OK) {
          Debug1(3, "Trg[%d] info\n", i);
          xdr_int(&xdrs, &i);		    /* trigger number		*/
          cptr= pscan->txpv[i];
          xdr_counted_string(&xdrs, &cptr); /* trigger name  		*/
          xdr_float(&xdrs, &pscan->txcd[i]);/* trigger command		*/
        }
      }
    }
      
    data_size=0;
    lval= xdr_getpos(&xdrs);
    if(pscan->nb_pos) {
      /* allocate space for nb_pos positioners				*/
      for(i=0; i<SCAN_NBP; i++) {
        if((pscan->rxnv[i]==XXNV_OK) || (pscan->pxnv[i]==XXNV_OK)) {	
          Debug1(3, "Allocate space for Pos[%d]\n", i);
          pscan->pxra_fpos[i]= lval+data_size;
          data_size+= pscan->npts*sizeof(double);
        }
      }
    }
    if(pscan->nb_det) {
      /* allocate space for nb_det detectors				*/
      for(i=0; i<SCAN_NBD; i++) {
        if(pscan->dxnv[i]==XXNV_OK) {
          Debug1(3, "Allocate space for Det[%d]\n", i);
          pscan->dxda_fpos[i]= lval+data_size;
          data_size+=pscan->npts*sizeof(float);
        }
      }
    }

    if(data_size>0) {
      /* reserve space for data */
      fseek(fd, data_size-1, SEEK_CUR);
      cval=0;
      fwrite((void*)&cval, 1,1, fd);
    }
      
    if(pscan->old_npts < pscan->npts) {
      xdr_setpos(&xdrs, pscan->dims_offset);
      ival= (int)pscan->npts;
      xdr_int(&xdrs, &ival);
    }

    if(pscan->old_npts!=-1 && pscan->old_npts!=pscan->npts) {
      ival= 0;  /*regular= FALSE */
      xdr_setpos(&xdrs, pscan->regular_offset);
      xdr_int(&xdrs, &ival);
    }
    pscan->old_npts= pscan->npts;


    if(pscan->first_scan==FALSE) {
      xdr_setpos(&xdrs, pscan->offset);
      xdr_long(&xdrs, &scan_offset);
      pscan->offset= xdr_getpos(&xdrs);
    }

    xdr_destroy(&xdrs);
    fclose(fd);
    Debug0(3, "scan Header written\n");
      
    DebugMsg2(2, "%s MSG_SCAN_DATA(0)= %f\n", pscan->name,
              ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);

  } else if((pscan->data==0) && (pmsg->val==1)) {
    /* scan ended */
    pscan->data=1;

    /* process the message */

    fd= fopen(pscan->ffname, "rb+");
    if ((fd == NULL) || (fileStatus(pscan->ffname) == ERROR)) {
      printf("saveData:proc_scan_data: can't open data file!!\n");
      sprintf(msg, "Warning!! can't open file '%s'", pscan->fname);
      msg[39]= '\0';
      sendUserMessage(msg);
      return;
    }

    xdrstdio_create(&xdrs, fd, XDR_ENCODE);

    if((!pscan->all_pts) || (pscan->cpt!=pscan->npts)) {
	
      /* The scan just finished. update buffers and save scan	*/
      Debug2(3, "writing %s to %s\n", pscan->name, pscan->fname);

      /* get the current point */
      ca_array_get(DBR_SHORT, 1, pscan->ccpt, &pscan->cpt);
      if(ca_pend_io(0.1)!=ECA_NORMAL) {
        pscan->cpt= pscan->npts;
      }

      /*------------------------------------------------------------*/
      /* Get all valid arrays					*/
      if(pscan->nb_pos) {
        for(i=0; i<SCAN_NBP; i++) {
          if((pscan->pxnv[i]==XXNV_OK) || (pscan->rxnv[i]==XXNV_OK)) {
            ca_array_get(DBR_DOUBLE, pscan->cpt, 
                         pscan->cpxra[i], pscan->pxra[i]);
          }
        }
      }
      if(pscan->nb_det) {
        for(i= 0; i<SCAN_NBD; i++) {
          if(pscan->dxnv[i]==XXNV_OK) {
            ca_array_get(DBR_FLOAT, pscan->cpt, 
                         pscan->cdxda[i], pscan->dxda[i]);
          }
        }
      }
      if(ca_pend_io(1.0)!=ECA_NORMAL) {
        Debug0(3, "unable to get all valid arrays \n");
        sprintf(msg, "Warning!! can't get data");
        msg[39]= '\0';
        sendUserMessage(msg);
        return;
      }

      /* current point */
      xdr_setpos(&xdrs, pscan->cpt_fpos);
      ival= (int)pscan->cpt;
      xdr_int(&xdrs, &ival);
      /*--------------------------------------------------------------*/
      /* Save the positioners arrays				  */
      if(pscan->nb_pos) {
        for(i=0; i<SCAN_NBP; i++) {
          if((pscan->rxnv[i]==XXNV_OK) || (pscan->pxnv[i]==XXNV_OK)) {
            xdr_setpos(&xdrs, pscan->pxra_fpos[i]);
            xdr_vector(&xdrs, (char*)pscan->pxra[i], pscan->npts, 
                       sizeof(double), xdr_double);
          }
        }
      }
      /*------------------------------------------------------------*/
      /* Save the detectors arrays					*/
      if(pscan->nb_det) {
        for(i=0; i<SCAN_NBD; i++) {
          if(pscan->dxnv[i]==XXNV_OK) {
            xdr_setpos(&xdrs, pscan->dxda_fpos[i]);
            xdr_vector(&xdrs, (char*)pscan->dxda[i], pscan->npts,
                       sizeof(float), xdr_float);
          }
        }
      }
    } else {
      /* current point */
      xdr_setpos(&xdrs, pscan->cpt_fpos);
      ival= (int)pscan->cpt;
      xdr_int(&xdrs, &ival); 
    }

    if(pscan->first_scan) {
      /* Save extra pvs						*/
      fseek(fd, 0, SEEK_END);
	
      lval= xdr_getpos(&xdrs);
      saveExtraPV(&xdrs);
      xdr_setpos(&xdrs, pscan->offset_extraPV);
      xdr_long(&xdrs, &lval);

      sprintf(msg,"Scan saved: %s", pscan->fname);
      sendUserMessage(msg);
    }

    xdr_destroy(&xdrs);
    fclose(fd);

    if(pscan->nxt) {
      pscan->nxt->first_scan=TRUE;
    }

    /* hand shaking notify						*/
    if(pscan->chandShake) {
      sval= HANDSHAKE_DONE;
      ca_array_put(DBR_SHORT, 1, pscan->chandShake, &sval);
    }

    /* enable file system record					*/
    if(--nb_scan_running==0) {
      cval=(char)0;
      ca_array_put(DBR_CHAR, 1, file_system_disp_chid, &cval);
      cval=(char)0;
      ca_array_put(DBR_CHAR, 1, file_subdir_disp_chid, &cval);
    }
    Debug1(2,"(save_status active) nb_scan_running=%d\n", nb_scan_running);

    DebugMsg2(2, "%s MSG_SCAN_DATA(1)= %f\n", pscan->name, 
              (float)(tickGet()-pmsg->time)/vxTicksPerSecond);

  }
}

LOCAL void proc_scan_npts(SCAN_SHORT_MSG* pmsg)
{
  SCAN* pscan;

  pscan= pmsg->pscan;
  pscan->npts= pmsg->val;
  DebugMsg3(2, "%s MSG_SCAN_NPTS(%d)= %f\n", pscan->name, pscan->npts, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_scan_cpt(SCAN_SHORT_MSG* pmsg)
{
  int   i, ival;
  SCAN* pscan;
  FILE* fd;
  XDR   xdrs;

  pscan= pmsg->pscan;

  if ((save_status == STATUS_INACTIVE) || (save_status == STATUS_ACTIVE_FS_ERROR)) {
    return;
  }

  /* process the message */

  if(pmsg->val==0) pscan->all_pts= TRUE;
  else pscan->all_pts= ((pscan->all_pts) && (pscan->cpt==pmsg->val-1));

  pscan->cpt= pmsg->val;

  /* is the scan running ? */
  if((pscan->data!=0) || (pscan->cpt==0)) return;

  Debug3(2, "saving %s[%d] to %s\n", pscan->name, pscan->cpt-1, pscan->fname);
    
  for(i=0; i<SCAN_NBP; i++) {
    if((pscan->rxnv[i]==XXNV_OK) || (pscan->pxnv[i]==XXNV_OK))
      ca_array_get(DBR_DOUBLE, 1, pscan->crxcv[i], &pscan->rxcv[i]);
  }
  for(i=0; i<SCAN_NBD; i++) {
    if(pscan->dxnv[i]==XXNV_OK)
      ca_array_get(DBR_FLOAT, 1, pscan->cdxcv[i], &pscan->dxcv[i]);
  }
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    /* error !!! */
    printf("saveData: unable to get current values !!!\n");
    pscan->all_pts= FALSE;
  } else {
      
    fd= fopen(pscan->ffname, "rb+");
    xdrstdio_create(&xdrs, fd, XDR_ENCODE);
      
    /* point number		*/
    xdr_setpos(&xdrs, pscan->cpt_fpos);
    ival= (int)pscan->cpt;
    xdr_int(&xdrs, &ival);
    /* positioners and detectors values */
    if(pscan->nb_pos)
      for(i=0; i<SCAN_NBP; i++) {
        if((pscan->rxnv[i]==XXNV_OK) || (pscan->pxnv[i]==XXNV_OK)) {
          xdr_setpos(&xdrs, pscan->pxra_fpos[i]+(pscan->cpt-1)*sizeof(double));
          xdr_double(&xdrs, &pscan->rxcv[i]);
        }
      }
    if(pscan->nb_det)
      for(i=0; i<SCAN_NBD; i++) {
        if(pscan->dxnv[i]==XXNV_OK) {
          xdr_setpos(&xdrs, pscan->dxda_fpos[i]+(pscan->cpt-1)*sizeof(float));
          xdr_float(&xdrs, &pscan->dxcv[i]);
        }
      }
      
    xdr_destroy(&xdrs);
    fclose(fd);
  }

  DebugMsg3(2, "%s MSG_SCAN_CPT(%d)= %f\n", pscan->name, pscan->cpt, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}


LOCAL void proc_scan_pxnv(SCAN_INDEX_MSG* pmsg)
{
  SCAN* pscan;
  int   i;
  short val;
  char  buff[40];
  int   len;

  pscan= pmsg->pscan;
  i= pmsg->index;
  val= (short)pmsg->val;
  /* get PxNV value						*/
  pscan->pxnv[i]= val;
  pscan->pxpv[i][0]='\0';
  pscan->pxds[i][0]='\0';
  pscan->pxeu[i].units[0]='\0';

  /* clear previous desc monitors */
  if(pscan->cpxds[i]) {
    ca_clear_channel(pscan->cpxds[i]);
    pscan->cpxds[i]= NULL;
  }
  /* clear previous unit channel */
  if(pscan->cpxeu[i]) {
    ca_clear_channel(pscan->cpxeu[i]);
    pscan->cpxeu[i]= NULL;
  }

  if(val==XXNV_OK) {
    /* the pvname is valid, get it.		      		*/
    ca_array_get(DBR_STRING, 1, pscan->cpxpv[i], pscan->pxpv[i]);
    if(ca_pend_io(2.0)!=ECA_NORMAL) {
      Debug2(2, "Unable to get %s.%s\n", pscan->name, pxpv[i]);
      strcpy(pscan->pxpv[i], "ERROR");
    } else {
      /* Try to connect the positioner DESC field */
      len= strcspn(pscan->pxpv[i], ".");
      strncpy(buff, pscan->pxpv[i], len);
      buff[len]='\0';
      strcat(buff, ".DESC");
      ca_search(buff, &pscan->cpxds[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", buff);
        ca_clear_channel(pscan->cpxds[i]);
        pscan->cpxds[i]=NULL;
      } else {
        ca_add_array_event(DBR_STRING, 1, pscan->cpxds[i], 
                           descMonitor, pscan->pxds[i],
                           (float)0,(float)0,(float)0, NULL);
      }

      /* Try to connect the positioner */
      ca_search(pscan->pxpv[i], &pscan->cpxeu[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", pscan->pxpv[i]);
        ca_clear_channel(pscan->cpxeu[i]);
        pscan->cpxeu[i]=NULL;
      } else {
        ca_array_get(DBR_CTRL_DOUBLE, 1, pscan->cpxeu[i], &pscan->pxeu[i]);
        ca_pend_io(2.0);
      }
    }
  }
  DebugMsg3(2, "%s MSG_SCAN_PXNV(%d)= %f\n", pscan->name, val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_scan_pxsm(STRING_MSG* pmsg)
{

  strncpy(pmsg->pdest, pmsg->string, 39);
  pmsg->pdest[39]='\0';

  DebugMsg2(2, "MSG_SCAN_PXSM(%s)= %f\n", pmsg->string, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_scan_rxnv(SCAN_INDEX_MSG* pmsg)
{
  SCAN* pscan;
  int   i;
  short val;
  char  buff[40];
  int   len;

  pscan= pmsg->pscan;
  i= pmsg->index;
  val= (short)pmsg->val;
  /* Get RxNV value						*/
  pscan->rxnv[i]= val;
  pscan->rxpv[i][0]='\0';
  pscan->rxds[i][0]='\0';
  pscan->rxeu[i].units[0]='\0';

  /* clear previous desc monitors */
  if(pscan->crxds[i]) {
    ca_clear_channel(pscan->crxds[i]);
    pscan->crxds[i]= NULL;
  }
  /* clear previous unit channel*/
  if(pscan->crxeu[i]) {
    ca_clear_channel(pscan->crxeu[i]);
    pscan->crxeu[i]= NULL;
  }

  /* Get the readback pvname				       	*/
  ca_array_get(DBR_STRING, 1, pscan->crxpv[i], pscan->rxpv[i]);
  if(ca_pend_io(0.5)!=ECA_NORMAL) {
    Debug2(2, "Unable to get %s.%s\n", pscan->name, rxpv[i]);
    strcpy(pscan->rxpv[i], "ERROR");
  } else {
    if(val==XXNV_OK) {
      /* The pvname is valid					*/
      /* Try to connect the readback DESC field			*/
      len= strcspn(pscan->rxpv[i], ".");
      strncpy(buff, pscan->rxpv[i], len);
      buff[len]='\0';
      strcat(buff, ".DESC");
      ca_search(buff, &pscan->crxds[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", buff);
        ca_clear_channel(pscan->crxds[i]);
        pscan->crxds[i]=NULL;
      } else {
        ca_add_array_event(DBR_STRING, 1, pscan->crxds[i], 
                           descMonitor, pscan->rxds[i],
                           (float)0,(float)0,(float)0, NULL);
      }
      /* Try to connect the readback */
      ca_search(pscan->rxpv[i], &pscan->crxeu[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", pscan->rxpv[i]);
        ca_clear_channel(pscan->crxeu[i]);
        pscan->crxeu[i]=NULL;
      } else {
        ca_array_get(DBR_CTRL_DOUBLE, 1, pscan->crxeu[i], &pscan->rxeu[i]);
        ca_pend_io(2.0);
      }
    } else {
      /* the pvname is not valid					*/
      /* Check for time or TIME					*/
      if((strcmp(pscan->rxpv[i], "time")==0) || 
         (strcmp(pscan->rxpv[i], "TIME")==0)) {
        pscan->rxnv[i]=XXNV_OK;
        strcpy(pscan->rxeu[i].units, "second");
      } else {
        pscan->rxpv[i][0]='\0';
      }
    }
  }
  DebugMsg3(2, "%s MSG_SCAN_RXNV(%d)= %f\n", pscan->name, val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}


LOCAL void proc_scan_dxnv(SCAN_INDEX_MSG* pmsg)
{
  SCAN* pscan;
  int   i;
  short val;
  char  buff[40];
  int   len;

  pscan= pmsg->pscan;
  i= pmsg->index;
  val= (short)pmsg->val;

  pscan->dxnv[i]= val;
  pscan->dxpv[i][0]='\0';
  pscan->dxds[i][0]='\0';
  pscan->dxeu[i].units[0]='\0';

  /* clear previous desc monitors */
  if(pscan->cdxds[i]) {
    ca_clear_channel(pscan->cdxds[i]);
    pscan->cdxds[i]= NULL;
  }
  /* clear previous unit channel*/
  if(pscan->cdxeu[i]) {
    ca_clear_channel(pscan->cdxeu[i]);
    pscan->cdxeu[i]= NULL;
  }

  if(val==XXNV_OK) {
    ca_array_get(DBR_STRING, 1, pscan->cdxpv[i], pscan->dxpv[i]);
    if(ca_pend_io(1.0)!=ECA_NORMAL) {
      Debug2(2, "Unable to get %s.%s\n", pscan->name, dxpv[i]);
      strcpy(pscan->dxpv[i], "ERROR");
    } else {
      /* Try to connect the detector DESC field			*/
      len= strcspn(pscan->dxpv[i], ".");
      strncpy(buff, pscan->dxpv[i], len);
      buff[len]='\0';
      strcat(buff, ".DESC");
      ca_search(buff, &pscan->cdxds[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", buff);
        ca_clear_channel(pscan->cdxds[i]);
        pscan->cdxds[i]=NULL;
      } else {
        ca_add_array_event(DBR_STRING, 1, pscan->cdxds[i], 
                           descMonitor, pscan->dxds[i],
                           (float)0,(float)0,(float)0, NULL);
      }
      /* Try to connect the detector */
      ca_search(pscan->dxpv[i], &pscan->cdxeu[i]);
      if(ca_pend_io(2.0)!=ECA_NORMAL) {
        Debug1(2, "Unable to connect %s\n", pscan->dxpv[i]);
        ca_clear_channel(pscan->cdxeu[i]);
        pscan->cdxeu[i]=NULL;
      } else {
        ca_array_get(DBR_CTRL_FLOAT, 1, pscan->cdxeu[i], &pscan->dxeu[i]);
        ca_pend_io(2.0);
      }
    }
  }
  DebugMsg3(2, "%s MSG_SCAN_DXNV(%d)= %f\n", pscan->name, val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}


LOCAL void proc_scan_txnv(SCAN_INDEX_MSG* pmsg)
{
  SCAN* pscan;
  int   i;
  short val;
  int   len;

  pscan= pmsg->pscan;
  i= pmsg->index;
  val= (short)pmsg->val;

  pscan->txsc[i]= 1;
  pscan->txnv[i]= val;
  pscan->txpv[i][0]= '\0';

  if(val==XXNV_OK) {
    ca_array_get(DBR_STRING, 1, pscan->ctxpv[i], pscan->txpv[i]);
    if(ca_pend_io(2.0)!=ECA_NORMAL) {
      Debug2(2, "Unable to get %s.%s\n", pscan->name, txpv[i]);
      pscan->txpv[i][0]='\0';
    } else {
      len= strcspn(pscan->txpv[i], ".");
      pscan->txsc[i]= strncmp(&pscan->txpv[i][len], ".EXSC", 6);
      pscan->txpv[i][len]='\0';
    }
  }

  updateScan(pscan);

  DebugMsg3(2, "%s MSG_SCAN_TXNV(%d)= %f\n", pscan->name, val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_scan_txcd(SCAN_INDEX_MSG* pmsg)
{
  SCAN* pscan;
  int   i;

  pscan= pmsg->pscan;
  i= pmsg->index;

  pscan->txcd[i]= (float)pmsg->val;

  updateScan(pscan);

  DebugMsg3(2, "%s MSG_SCAN_TXCD(%f)= %f\n", pscan->name, (float)pmsg->val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_desc(STRING_MSG* pmsg)
{
  strncpy(pmsg->pdest, pmsg->string, 39);
  pmsg->pdest[39]= '\0';

  DebugMsg2(2, "MSG_DESC(%s)= %f\n", pmsg->string, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_egu(STRING_MSG* pmsg)
{
  strncpy(pmsg->pdest, pmsg->string, 15);
  pmsg->pdest[15]= '\0';

  DebugMsg2(2, "MSG_EGU(%s)= %f\n", pmsg->string, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}


LOCAL void proc_file_system(STRING_MSG* pmsg)
{
  char  msg[40];
  char  hostname[40];
  char  *filesystem;
  char* cout;

  /* make sure string is null terminated */
  pmsg->string[39]='\0';

  /* unmount previous data directory */
  nfsUnmount("/data");

  file_system_state= FS_NOT_MOUNTED;
  save_status= STATUS_ACTIVE_FS_ERROR;

  /* reset subdirectory to "" */
  if(*local_subdir!='\0') {
    *local_subdir='\0';
    ca_array_put(DBR_STRING, 1, file_subdir_chid, local_subdir);
  }
  server_pathname[0]='\0';
  server_subdir= server_pathname;

  filesystem= pmsg->string;
  if((*(filesystem++)!='/') || (*(filesystem++)!='/')) {
    strcpy(msg, "Invalid file system !!!");
  } else {
    /* extract the host name */
    cout= hostname;
    while((*filesystem!='\0') && (*filesystem!='/'))
      *(cout++)= *(filesystem++);
    *cout='\0';
    
    /* Mount the new file system */
    if(nfsMount(hostname, filesystem, "/data")==ERROR) {
      strcpy(msg, "Unable to mount file system !!!!");
    } else {
      strcpy(server_pathname, pmsg->string);
      strcat(server_pathname, "/");
      server_subdir= &server_pathname[strlen(server_pathname)];
      
      file_system_state= FS_MOUNTED;

      if(checkRWpermission(local_pathname)!=OK) {
        strcpy(msg, "RW permission denied !!!");
      } else {
        strcpy(msg, "saveData OK");
        save_status= STATUS_ACTIVE_OK;
      }
    }
  }

  if(full_pathname_chid) {
    ca_array_put(DBR_CHAR, strlen(server_pathname)+1,
                 full_pathname_chid, server_pathname);
  }
  sendUserMessage(msg);
  ca_array_put(DBR_SHORT, 1, save_status_chid, &save_status);

  DebugMsg2(2, "MSG_FILE_SYSTEM(%s)= %f\n", pmsg->string, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_file_subdir(STRING_MSG* pmsg)
{
  char msg[40];
  int fd;
  char* cin;
  char* server;
  char* local;

  if(file_system_state==FS_MOUNTED) {

    save_status= STATUS_ACTIVE_FS_ERROR;
    cin= pmsg->string;

    /* the new directory should be different from the previous one */
    if(strcmp(cin, local_subdir)==0) return;

    server= server_subdir;
    local= local_subdir;

    *server= *local= '\0';

    while((*cin!='\0') && (*cin=='/')) cin++;
    
    while(*cin!='\0') {
      while((*cin!='\0') && (*cin!='/')) {
        *server= *local= *cin;
        if(*cin==' ') *server=*local= '_';
        server++;
        local++;
        cin++;
      }
      /* NULL terminate file_subdirectory */
      *server= *local= '\0';
      /* skip all trailling '/' */
      while((*cin!='\0') && (*cin=='/')) cin++;
      /* create directory */
      fd = open (local_pathname, O_RDWR | O_CREAT, 
                 FSTAT_DIR | DEFAULT_DIR_PERM | 0775);
      if(fd!=ERROR) close(fd);
      /* append '/' */
      *(server++)= *(local++)= '/';
      *(server)= *(local)= '\0';    
    }

    if(fileStatus(local_pathname)!=OK) {
      strcpy(msg, "Invalid directory !!!");
      *server_subdir=*local_subdir= '\0';
    } else if(checkRWpermission(local_pathname)!=OK) {
      strcpy(msg, "RW permission denied !!!");
      *server_subdir=*local_subdir= '\0';
    } else {
      strcpy(msg, "saveData OK");
      save_status= STATUS_ACTIVE_OK;
    }

    if(full_pathname_chid) {
      ca_array_put(DBR_CHAR, strlen(server_pathname)+1,
                   full_pathname_chid, server_pathname);
    }
    sendUserMessage(msg);
    ca_array_put(DBR_SHORT, 1, save_status_chid, &save_status);
  }
  DebugMsg2(2, "MSG_FILE_SUBDIR(%s)= %f\n", pmsg->string, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

LOCAL void proc_realTime1D(INTEGER_MSG* pmsg)
{
  if(realTime1D!= pmsg->val) {
    realTime1D= pmsg->val;
    updateScans();
  }
  DebugMsg2(2, "MSG_REALTIME1D(%d)= %f\n", pmsg->val, 
            ((float)(tickGet()-pmsg->time))/vxTicksPerSecond);
}

/*----------------------------------------------------------------------*/
/* The task in charge of updating and saving scans           		*/
/*                                                			*/
LOCAL int saveDataTask(int tid,int p1,int p2,int p3,int p4,int p5,int p6,int p7,int p8,int p9)
{
  char*    pmsg;
  int*     ptype;

  Debug0(1, "Task saveDataTask running...\n");

  cpt_wait_ticks= vxTicksPerSecond/10;

  if(initSaveDataTask()==-1) {
    printf("saveData: Unable to configure saveDataTask\n");
    if(taskIsSuspended(tid)) taskResume(tid);
    return -1;
  }

  pmsg= (char*) malloc(MAX_SIZE);
  ptype= (int*)pmsg;
  if(!pmsg) {
    printf("saveData: Not enough memory to allocate message buffer\n");
    if(taskIsSuspended(tid)) taskResume(tid);
    return -1;
  }

  Debug0(1, "saveDataTask waiting for messages\n");
  if(taskIsSuspended(tid)) taskResume(tid);

  while(1) {
    
    /* waiting for messages						*/
    if(msgQReceive(msg_queue, pmsg, MAX_SIZE,
                   WAIT_FOREVER)==S_objLib_OBJ_DELETED) {
      break;
    }

    switch(*ptype) {

      /*--------------------------------------------------------------*/
      /* DATA changed							*/
    case MSG_SCAN_DATA:
      Debug0(1, "saveDataTask: message MSG_SCAN_DATA\n");
      proc_scan_data((SCAN_SHORT_MSG*)pmsg);
      break;
      /*--------------------------------------------------------------*/
      /* NPTS has changed						*/
    case MSG_SCAN_NPTS:
      proc_scan_npts((SCAN_SHORT_MSG*)pmsg);
      break;
      /*--------------------------------------------------------------*/
      /* CPT has changed						*/
    case MSG_SCAN_CPT:
      proc_scan_cpt((SCAN_SHORT_MSG*)pmsg);
      break;
      /*--------------------------------------------------------------*/
      /* PxNV has changed						*/
    case MSG_SCAN_PXNV:
      proc_scan_pxnv((SCAN_INDEX_MSG*)pmsg);
      break;
      /*--------------------------------------------------------------*/
      /* PxSM has changed						*/
    case MSG_SCAN_PXSM:
      proc_scan_pxsm((STRING_MSG*)pmsg);
      break;

      /*--------------------------------------------------------------*/
      /* RxNV has changed						*/
    case MSG_SCAN_RXNV:
      proc_scan_rxnv((SCAN_INDEX_MSG*)pmsg);
      break;

      /*--------------------------------------------------------------*/
      /* DxNV has changed						*/
    case MSG_SCAN_DXNV:
      proc_scan_dxnv((SCAN_INDEX_MSG*)pmsg);
      break;

      /*--------------------------------------------------------------*/
      /* TxNV has changed						*/
    case MSG_SCAN_TXNV:
      proc_scan_txnv((SCAN_INDEX_MSG*)pmsg);
      break;
      /*--------------------------------------------------------------*/
      /* TxCD has changed						*/
    case MSG_SCAN_TXCD:
      proc_scan_txcd((SCAN_INDEX_MSG*)pmsg);
      break;

    case MSG_DESC:
      proc_desc((STRING_MSG*)pmsg);
      break;
    case MSG_EGU:
      proc_egu((STRING_MSG*)pmsg);
      break;


    case MSG_FILE_SYSTEM:
      proc_file_system((STRING_MSG*)pmsg);
      break;

    case MSG_FILE_SUBDIR:
      proc_file_subdir((STRING_MSG*)pmsg);
      break;

    case MSG_REALTIME1D:
      proc_realTime1D((INTEGER_MSG*)pmsg);
      break;

    default: 
      Debug1(2, "Unknow message: #%d", *ptype);
    }
  }
  return 0;
}    


