/* 	devA32Vme.c	*/

/*****************************************************************
 *
 *      Author :                     Ned D. Arnold
 *      Date:                        11/21/97
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
 * 01-23-98   nda       initially functional
 * 02-25-98   mr        modified ai and ao to support 2's complement.
 * 07-22-98   mr        Fixed Param field to accomadate both i`uni and bi polar
 * 			Inputs and outputs (AI, AO records)..
 * 10-06-98   nda       fixed a bug with li,lo,ai,ao where sum of bit+
 *                      numbits > MAX_ACTIVE_BITS
*              
 */


/*To Use this device support, Include the following before iocInit */
/* devA32VmeConfig(card,a32base,nreg,iVector,iLevel)  */
/*    card    = card number                           */
/*    a32base = base address of card                  */
/*    nreg    = number of A32 registers on this card  */
/*    iVector = interrupt vector (MRD100 ONLY !!)     */
/*    iLevel  = interrupt level  (MRD100 ONLY !!)     */
/* For Example					      */
/* devA32VmeConfig(0, 0x80000000, 44, 0x3e, 5)        */


 /**********************************************************************/
 /** Brief Description of device support                              **/
 /**						    	              **/
 /** This device support allows access to any register of a VME       **/
 /** module found in the A32/D32 VME space. The bit field of interest **/
 /** is described in the PARM field of the INP or OUT link.           **/
 /** This allows a generic driver to be used without hard-coding      **/
 /** register numbers within the software.                            **/
 /**						    	              **/
 /** Record type     Signal #           Parm Field                    **/
 /**                                                                  **/
 /**    ai          reg_offset     lsb, width, type                   **/
 /**    ao          reg_offset     lsb, width, type                   **/
 /**    bi          reg_offset     bit #                              **/
 /**    bo          reg_offset     bit #                              **/
 /**    longin      reg_offset     lsb, width                         **/
 /**    longout     reg_offset     lsb, width                         **/
 /**    mbbi        reg_offset     lsb, width                         **/
 /**    mbbo        reg_offset     lsb, width                         **/
 /**                                                                  **/
 /** reg_offset is specified by the register number (0,1,2,3, etc)    **/
 /** Parm field must be provided, no defaults are assumed ...         **/
 /** In ai and ao type is either 0 - unipolar, 1 -bipolar             **/
 /**                                                                  **/
 /**                                                                  **/
 /**********************************************************************/

#include	<vxWorks.h>
#include	<vxLib.h>
#include	<sysLib.h>
#include	<vme.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<stdlib.h>
#include	<intLib.h>
#include	<string.h>
#include	<math.h>
#include	<iv.h>
#include	<rebootLib.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<recGbl.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<link.h>

#include	<epicsPrint.h>
#include	<epicsExport.h>

#include	<aoRecord.h>
#include	<aiRecord.h>
#include	<boRecord.h>
#include	<biRecord.h>
#include	<longinRecord.h>
#include	<longoutRecord.h>
#include	<mbboRecord.h>
#include	<mbbiRecord.h>

#include	<dbScan.h>

#define ERROR (-1)

static long init_ai(), read_ai();
static long init_ao(), write_ao();
static long init_bi(), read_bi();
static long init_bo(), write_bo();
static long init_li(), read_li();
static long init_lo(), write_lo();
static long init_mbbi(), read_mbbi();
static long init_mbbo(), write_mbbo();
static long checkCard(), write_card(), read_card();
static long get_bi_int_info();
static void devA32_isr();
static void devA32RebootFunc();

static long devA32VmeReport();

int  devA32VmeDebug = 0; 

/***** devA32VmeDebug *****/

/** devA32VmeDebug == 0 --- no debugging messages **/
/** devA32VmeDebug >= 5 --- hardware initialization information **/
/** devA32VmeDebug >= 10 -- record initialization information **/
/** devA32VmeDebug >= 15 -- write commands **/
/** devA32VmeDebug >= 20 -- read commands **/


#define MAX_NUM_CARDS    4
#define MAX_A32_ADDRESS  0xf0000000
#define MAX_ACTIVE_REGS  64   /* largest register number allowed */
#define MAX_ACTIVE_BITS  32   /* largest bit # expected */

#define IVEC_REG           29   /* Interrupt Vector Register (MRD100 ONLY) */
#define IVEC_ENA_REG       28   /* Interrupt Enable Register (MRD100 ONLY) */
#define IVEC_MASK          0xff /* Interrupt Vector Mask     (MRD100 ONLY) */
#define IVEC_ENA_MASK      0x10 /* Interrupt Enable Mask     (MRD100 ONLY) */
#define IVEC_REENABLE_MASK 0xf  /* Interrupt Re-enable Mask  (MRD100 ONLY) */
#define IVEC_REENABLE_REG  27   /* Interrupt Re-enable Reg   (MRD100 ONLY) */

/* Register layout */
typedef struct a32Reg {
  unsigned long reg[MAX_ACTIVE_REGS];
}a32Reg;

typedef struct ioCard {  /* Unique for each card */
  volatile a32Reg  *base;    /* address of this card's registers */
  int               nReg;    /* Number of registers on this card */
  epicsMutexId      lock;    /* semaphore */
  IOSCANPVT         ioscanpvt; /* records to process upon interrupt */
}ioCard;

static struct ioCard cards[MAX_NUM_CARDS]; /* array of card info */

typedef struct a32VmeDpvt { /* unique for each record */
  unsigned short  reg;   /* index of register to use (determined by signal #*/
  unsigned short  lbit;  /* least significant bit of interest */
  unsigned short  nbit;  /* no of significant bits */
  unsigned short  type;  /* Type either 0 or 1 for uni, bi polar */
  unsigned long   mask;  /* mask to use ...  */
}a32VmeDpvt;

/* Define the dset for A32VME */
typedef struct {
	long		number;
	DEVSUPFUN	report;		/* used by dbior */
	DEVSUPFUN	init;	
	DEVSUPFUN	init_record;	/* called 1 time for each record */
	DEVSUPFUN	get_ioint_info;	
	DEVSUPFUN	read_write;
        DEVSUPFUN       special_linconv;
} A32VME_DSET;

A32VME_DSET devAiA32Vme =   {6, NULL, NULL, init_ai, NULL, read_ai,  NULL};
A32VME_DSET devAoA32Vme =   {6, NULL, NULL, init_ao, NULL, write_ao, NULL};
A32VME_DSET devBiA32Vme =   {5, devA32VmeReport,NULL,init_bi, get_bi_int_info, 
                             read_bi,  NULL};
A32VME_DSET devBoA32Vme =   {5, NULL, NULL, init_bo, NULL, write_bo, NULL};
A32VME_DSET devLiA32Vme =   {5, NULL, NULL, init_li, NULL, read_li,  NULL};
A32VME_DSET devLoA32Vme =   {5, NULL, NULL, init_lo, NULL, write_lo, NULL};
A32VME_DSET devMbbiA32Vme = {5, NULL, NULL, init_mbbi, NULL, read_mbbi,  NULL};
A32VME_DSET devMbboA32Vme = {5, NULL, NULL, init_mbbo, NULL, write_mbbo, NULL};

epicsExportAddress(A32VME_DSET, devAiA32Vme);
epicsExportAddress(A32VME_DSET, devAoA32Vme);
epicsExportAddress(A32VME_DSET, devBiA32Vme);
epicsExportAddress(A32VME_DSET, devBoA32Vme);
epicsExportAddress(A32VME_DSET, devLiA32Vme);
epicsExportAddress(A32VME_DSET, devLoA32Vme);
epicsExportAddress(A32VME_DSET, devMbbiA32Vme);
epicsExportAddress(A32VME_DSET, devMbboA32Vme);

/**************************************************************************
 **************************************************************************/
static long devA32VmeReport()
{
int             i;
int		cardNum = 0;
unsigned long   regData;

  for(cardNum=0; cardNum < MAX_NUM_CARDS; cardNum++) {
    if(cards[cardNum].base != NULL) {
      printf("  Card #%d at %p\n", cardNum, cards[cardNum].base);
      for(i=0; i < cards[cardNum].nReg; i++) {
          regData = cards[cardNum].base->reg[i];
          printf("    Register %d -> 0x%4.4lX (%ld)\n", i, regData, regData);
      }
    }
  }
return(0);
}


/**************************************************************************
*
* Initialization of A32/D32 Card
*
***************************************************************************/
int devA32VmeConfig(card,a32base,nregs,iVector,iLevel)
int	      card;
unsigned long a32base;
int	      nregs;
int	      iVector;
int	      iLevel;
{

  unsigned long probeVal;

  if((card >= MAX_NUM_CARDS) || (card < 0)) {
      epicsPrintf("devA32VmeConfig: Invalid Card # %d \n",card);
      return(ERROR);
  }

  if(a32base >= MAX_A32_ADDRESS) {
      epicsPrintf("devA32VmeConfig: Invalid Card Address %ld \n",a32base);
      return(ERROR);
  }

  if(sysBusToLocalAdrs(VME_AM_EXT_USR_DATA, (char *)a32base,
        (char **)&cards[card].base) != OK) {
       cards[card].base = NULL;
       epicsPrintf("devA32VmeConfig: A32 Address map failed for Card %d",card);
       return(ERROR);
  }

  if(vxMemProbe( (char *)cards[card].base, READ, 4, (char *)&probeVal) != OK ) {
       cards[card].base = NULL;
       epicsPrintf("devA32VmeConfig: vxMemProbe failed for Card %d",card);
       return(ERROR);
  }

  if(nregs > MAX_ACTIVE_REGS) {
      epicsPrintf("devA32VmeConfig: # of registers (%d) exceeds max\n",nregs);
      return(ERROR);
  }
  else {
      cards[card].nReg = nregs;
      cards[card].lock = epicsMutexMustCreate();
  }
 
  if(iVector) {
      scanIoInit(&cards[card].ioscanpvt);
      if(intConnect(INUM_TO_IVEC(iVector), (VOIDFUNCPTR)devA32_isr, 
                    (int)card) == OK)
      {
          cards[card].base->reg[IVEC_REG] = (unsigned long)iVector;
          write_card(card, IVEC_REG, IVEC_MASK, (unsigned long)iVector);
          write_card(card, IVEC_REENABLE_REG, IVEC_REENABLE_MASK, 
                     IVEC_REENABLE_MASK);
          write_card(card, IVEC_ENA_REG, IVEC_ENA_MASK, IVEC_ENA_MASK);
          sysIntEnable(iLevel);
      }
      else {
          epicsPrintf("devA32VmeConfig: Interrupt connect failed for card %d\n",
                          card);
      }
      rebootHookAdd((FUNCPTR)devA32RebootFunc);
  }       
  return(OK);
}

/**************************************************************************
 *
 * BI record interrupt routine
 *
 **************************************************************************/
static long get_bi_int_info(cmd, pbi, ppvt)
int                     cmd;
struct biRecord         *pbi;
IOSCANPVT               *ppvt;
{

   struct vmeio           *pvmeio = (struct vmeio *)(&pbi->inp.value);

   if(cards[pvmeio->card].ioscanpvt != NULL) {
       *ppvt = cards[pvmeio->card].ioscanpvt;
       return(OK);
   }
   else {
       return(ERROR);
   }
}


/**************************************************************************
 *
 * Interrupt service routine
 *
 **************************************************************************/
static void devA32_isr(int card)
{
   scanIoRequest(cards[card].ioscanpvt);
   write_card(card, IVEC_REENABLE_REG, IVEC_REENABLE_MASK, IVEC_REENABLE_MASK);
}


/******************************************************************************
 *
 * A function to disable interrupts in case we get a ^X style reboot.
 *
 ******************************************************************************/
static void devA32RebootFunc(void)
{
  int   card = 0;

  while (card < MAX_NUM_CARDS)
  {
    if (cards[card].ioscanpvt != NULL) {
        write_card(card, IVEC_ENA_REG, IVEC_ENA_MASK, 0);
    }
    card++;
  }
  return;
}


/**************************************************************************
 *
 * BO Initialization (Called one time for each BO MSLT card record)
 *
 **************************************************************************/
static long init_bo(pbo)
struct boRecord *pbo;
{
    long                status = 0;
    int                 card, args, bit;
    unsigned long 	rawVal;
    a32VmeDpvt         *pPvt;

    /* bo.out must be an VME_IO */
    switch (pbo->out.type) {
    case (VME_IO) :
 
      if(pbo->out.value.vmeio.card > MAX_NUM_CARDS) {
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pbo->out.value.vmeio.card , pbo->name);
        return(ERROR);
      }

      card = pbo->out.value.vmeio.card;

      if(cards[card].base == NULL) {
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
        	card, pbo->name);
        return(ERROR);
      }

      if (pbo->out.value.vmeio.signal >= cards[card].nReg) {
        pbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pbo->name);
        return(ERROR);
      }

      args = sscanf(pbo->out.value.vmeio.parm, "%d", &bit);
 
      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pbo->name);
        return(ERROR);
      }

      pbo->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pbo->dpvt;

      pPvt->reg =  pbo->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = 1 << pPvt->lbit;
      pbo->mask = pPvt->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == OK)
         {
         pbo->rbv = pbo->rval = rawVal;
         }
      else 
         {
         status = 2;
         }
      break;
         
    default :
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal OUT field ->%s<- \n", pbo->name);
        status = ERROR;
    }
    return(status);
}

/**************************************************************************
 *
 * BI Initialization (Called one time for each BI record)
 *
 **************************************************************************/
static long init_bi(pbi)
struct biRecord *pbi;
{
    long                status = 0;
    int                 card, args, bit;
    unsigned long       rawVal;
    a32VmeDpvt         *pPvt;
   

    /* bi.inp must be an VME_IO */
    switch (pbi->inp.type) {
    case (VME_IO) :

      if(pbi->inp.value.vmeio.card > MAX_NUM_CARDS) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pbi->inp.value.vmeio.card , pbi->name);
        return(ERROR);
      }

      card = pbi->inp.value.vmeio.card;

      if(cards[card].base == NULL) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
        	card, pbi->name);
        return(ERROR);
      }

      if (pbi->inp.value.vmeio.signal >= cards[card].nReg) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pbi->name);
        return(ERROR);
      }

      args = sscanf(pbi->inp.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pbi->name);
        return(ERROR);
      }
      pbi->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pbi->dpvt;

      pPvt->reg =  pbi->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = 1 << pPvt->lbit;
      pbi->mask = pPvt->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == OK)
         {
         pbi->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal INP field ->%s<- \n", pbi->name);
        status = ERROR;
    }
    return(status);
}


/**************************************************************************
 *
 * MBBO Initialization (Called one time for each MBBO record)
 *
 **************************************************************************/
static long init_mbbo(pmbbo)
struct mbboRecord   *pmbbo;
{
    long                status = 0;
    int                 card, args, bit;
    unsigned long       rawVal;
    a32VmeDpvt         *pPvt;

    /* mbbo.out must be an VME_IO */
    switch (pmbbo->out.type) {
    case (VME_IO) :
      if(pmbbo->out.value.vmeio.card > MAX_NUM_CARDS) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pmbbo->out.value.vmeio.card , pmbbo->name);
        return(ERROR);
      }

      card = pmbbo->out.value.vmeio.card;

      if(cards[card].base == NULL) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, pmbbo->name);
        return(ERROR);
      }

      if (pmbbo->out.value.vmeio.signal >= cards[card].nReg) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pmbbo->name);
        return(ERROR);
      }

      args = sscanf(pmbbo->out.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pmbbo->name);
        return(ERROR);
      }

      pmbbo->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pmbbo->dpvt;

      pPvt->reg =  pmbbo->out.value.vmeio.signal;
      pPvt->lbit = bit;

      /* record support determined .mask from .nobt, need to adjust */
      pmbbo->shft = pPvt->lbit;
      pmbbo->mask <<= pPvt->lbit;
      pPvt->mask = pmbbo->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == OK)
         {
         pmbbo->rbv = pmbbo->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal OUT field ->%s<- \n", pmbbo->name);
        status = ERROR;
    }
    return(status);
}


/**************************************************************************
 *
 * MBBI Initialization (Called one time for each MBBO record)
 *
 **************************************************************************/
static long init_mbbi(pmbbi)
struct mbbiRecord   *pmbbi;
{
    long                status = 0;
    int                 card, args, bit;
    unsigned long       rawVal;
    a32VmeDpvt         *pPvt;

    /* mbbi.inp must be an VME_IO */
    switch (pmbbi->inp.type) {
    case (VME_IO) :
      if(pmbbi->inp.value.vmeio.card > MAX_NUM_CARDS) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pmbbi->inp.value.vmeio.card , pmbbi->name);
        return(ERROR);
      }

      card = pmbbi->inp.value.vmeio.card;

      if(cards[card].base == NULL) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, pmbbi->name);
        return(ERROR);
      }

      if (pmbbi->inp.value.vmeio.signal >= cards[card].nReg) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pmbbi->name);
        return(ERROR);
      }

      args = sscanf(pmbbi->inp.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pmbbi->name);
        return(ERROR);
      }

      pmbbi->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pmbbi->dpvt;

      pPvt->reg =  pmbbi->inp.value.vmeio.signal;
      pPvt->lbit = bit;

      /* record support determined .mask from .nobt, need to adjust */
      pmbbi->shft = pPvt->lbit;
      pmbbi->mask <<= pPvt->lbit;
      pPvt->mask = pmbbi->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == OK)
         {
         pmbbi->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal INP field ->%s<- \n", pmbbi->name);
        status = ERROR;
    }
    return(status);
}


/**************************************************************************
 *
 * AI Initialization (Called one time for each AI record)
 *
 **************************************************************************/
static long init_ai(pai)
struct aiRecord   *pai;
{
    long                status = 0;
    int                 card, args, bit, numBits, twotype;
    a32VmeDpvt         *pPvt;

    /* ai.inp must be an VME_IO */
    switch (pai->inp.type) {
    case (VME_IO) :
      if(pai->inp.value.vmeio.card > MAX_NUM_CARDS) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pai->inp.value.vmeio.card , pai->name);
        return(ERROR);
      }

      card = pai->inp.value.vmeio.card;

      if(cards[card].base == NULL) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, pai->name);
        return(ERROR);
      }

      if (pai->inp.value.vmeio.signal >= cards[card].nReg) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pai->name);
        return(ERROR);
      }

      args = sscanf(pai->inp.value.vmeio.parm, "%d,%d,%d", 
      				&bit, &numBits, &twotype);

      if( (args != 3) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         	(bit + numBits > MAX_ACTIVE_BITS) ||
         	(twotype > 1) || (twotype < 0) ) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf
        ("devA32Vme: Invalid Bit #/Width/Type in parm field: ->%s<-\n",
                     pai->name);
        return(ERROR);
      }

      pai->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pai->dpvt;

      pPvt->reg =  pai->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->nbit = numBits;
      pPvt->type = twotype;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      pai->eslo = (pai->eguf - pai->egul)/(pow(2,numBits)-1);
      
/*  Shift Raw value if Bi-polar */
      if (pPvt->type ==1) 
         pai->roff = pow(2,(numBits-1));

      status = OK;

      break;
    default :
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal INP field ->%s<- \n", pai->name);
        status = ERROR;
    }
    return(status);
}



/**************************************************************************
 *
 * AO Initialization (Called one time for each AO record)
 *
 **************************************************************************/
static long init_ao(pao)
struct aoRecord   *pao;
{
    long                status = 0;
    unsigned long       rawVal;
    int                 card, args, bit, numBits, twotype;
    a32VmeDpvt         *pPvt;

    /* ao.out must be an VME_IO */
    switch (pao->out.type) {
    case (VME_IO) :
      if(pao->out.value.vmeio.card > MAX_NUM_CARDS) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pao->out.value.vmeio.card , pao->name);
        return(ERROR);
      }

      card = pao->out.value.vmeio.card;

      if(cards[card].base == NULL) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, pao->name);
        return(ERROR);
      }

      if (pao->out.value.vmeio.signal >= cards[card].nReg) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pao->name);
        return(ERROR);
      }

      args = sscanf(pao->out.value.vmeio.parm, "%d,%d,%d", 
      				&bit, &numBits, &twotype);

      if( (args != 3) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         	(bit + numBits > MAX_ACTIVE_BITS) ||
         	(twotype > 1) || (twotype < 0) ) {
        epicsPrintf
        ("devA32Vme: Invalid Bit #/Width/Type in parm field: ->%s<-\n",
                     pao->name);
        return(ERROR);
      }

      pao->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pao->dpvt;

      pPvt->reg =  pao->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->nbit = numBits;
      pPvt->type = twotype;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      pao->eslo = (pao->eguf - pao->egul)/(pow(2,numBits)-1);

/*  Shift Raw value if Bi-polar */
      if (pPvt->type == 1) 
         pao->roff = pow(2,(numBits-1));

      /* Init rval to current setting */ 
      if(read_card(card,pPvt->reg,pPvt->mask,&rawVal) == OK) {
        pao->rbv = rawVal>>pPvt->lbit;

/* here is where we do the sign extensions for Bipolar.... */        
        if (pPvt->type ==1) {
           if (pao->rbv & (2<<(pPvt->nbit-2)))
               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2 ;

	}

        pao->rval = pao->rbv;
      }

      status = OK;

      break;
    default :
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal OUT field ->%s<- \n", pao->name);
        status = ERROR;
    }
    return(status);
}


/**************************************************************************
 *
 * LI Initialization (Called one time for each LI record)
 *
 **************************************************************************/
static long init_li(pli)
struct longinRecord   *pli;
{
    long                status = 0;
    int                 card, args, bit, numBits;
    a32VmeDpvt         *pPvt;

    /* li.inp must be an VME_IO */
    switch (pli->inp.type) {
    case (VME_IO) :
      if(pli->inp.value.vmeio.card > MAX_NUM_CARDS) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	pli->inp.value.vmeio.card , pli->name);
        return(ERROR);
      }

      card = pli->inp.value.vmeio.card;

      if(cards[card].base == NULL) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, pli->name);
        return(ERROR);
      }

      if (pli->inp.value.vmeio.signal >= cards[card].nReg) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     pli->name);
        return(ERROR);
      }

      args = sscanf(pli->inp.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     pli->name);
        return(ERROR);
      }

      pli->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)pli->dpvt;

      pPvt->reg =  pli->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      status = OK;


      break;
    default :
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal INP field ->%s<- \n", pli->name);
        status = ERROR;
    }
    return(status);
}



/**************************************************************************
 *
 * Long Out Initialization (Called one time for each LO record)
 *
 **************************************************************************/
static long init_lo(plo)
struct longoutRecord   *plo;
{
    long                status = 0;
    unsigned long       rawVal;
    int                 card, args, bit, numBits;
    a32VmeDpvt         *pPvt;

    /* lo.out must be an VME_IO */
    switch (plo->out.type) {
    case (VME_IO) :
      if(plo->out.value.vmeio.card > MAX_NUM_CARDS) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d exceeds max: ->%s<- \n", 
        	plo->out.value.vmeio.card , plo->name);
        return(ERROR);
      }

      card = plo->out.value.vmeio.card;

      if(cards[card].base == NULL) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Card #%d not initialized: ->%s<-\n",
                     card, plo->name);
        return(ERROR);
      }

      if (plo->out.value.vmeio.signal >= cards[card].nReg) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Signal # exceeds registers: ->%s<-\n",
                     plo->name);
        return(ERROR);
      }

      args = sscanf(plo->out.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     plo->name);
        return(ERROR);
      }

      plo->dpvt = (void *)calloc(1, sizeof(struct a32VmeDpvt));
      pPvt = (a32VmeDpvt *)plo->dpvt;

      pPvt->reg =  plo->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      /* if .dol is NOT a constant, initialize .val field to readback value */
      if ((plo->dol.type == CONSTANT) && 
          (strlen(plo->dol.value.constantStr) == 0)) {
          if (read_card(card,pPvt->reg,pPvt->mask,&rawVal) == OK) {
              plo->val = rawVal>>pPvt->lbit;
              plo->udf = 0;
          }
      }

      status = OK;

      break;
    default :
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA32Vme: Illegal OUT field ->%s<- \n", plo->name);
        status = ERROR;
    }
    return(status);
}



/**************************************************************************
 *
 * Perform a write operation from a BO record
 *
 **************************************************************************/
static long write_bo(pbo)
struct boRecord *pbo;
{

  unsigned long 	rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pbo->dpvt;

  if (write_card(pbo->out.value.vmeio.card, pPvt->reg, pbo->mask, pbo->rval) 
        == OK)
  {
    if(read_card(pbo->out.value.vmeio.card, pPvt->reg, pbo->mask, &rawVal) 
        == OK)
    {
      pbo->rbv = rawVal;
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Perform a read operation from a BI record
 *
 **************************************************************************/
static long read_bi(pbi)
struct biRecord *pbi;
{

  unsigned long       rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pbi->dpvt;

  if (read_card(pbi->inp.value.vmeio.card, pPvt->reg, pbi->mask,&rawVal) == OK)
  {
     pbi->rval = rawVal;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pbi, READ_ALARM, INVALID_ALARM);
  return(2);
}


/**************************************************************************
 *
 * Perform a read operation from a MBBI record
 *
 **************************************************************************/
static long read_mbbi(pmbbi)
struct mbbiRecord *pmbbi;
{

  unsigned long       rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pmbbi->dpvt;

  if (read_card(pmbbi->inp.value.vmeio.card,pPvt->reg,pmbbi->mask,&rawVal) 
        == OK)
  {
     pmbbi->rval = rawVal;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pmbbi, READ_ALARM, INVALID_ALARM);
  return(2);
}



/**************************************************************************
 *
 * Perform a write operation from a MBBO record
 *
 **************************************************************************/
static long write_mbbo(pmbbo)
struct mbboRecord *pmbbo;
{

  unsigned long         rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pmbbo->dpvt;

  if (write_card(pmbbo->out.value.vmeio.card,pPvt->reg,
                      pmbbo->mask,pmbbo->rval) == OK)
  {
    if(read_card(pmbbo->out.value.vmeio.card,pPvt->reg,pmbbo->mask,&rawVal) 
       == OK)
    {
      pmbbo->rbv = rawVal;
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}


/**************************************************************************
 *
 * Perform a read operation from a AI record
 *
 **************************************************************************/
static long read_ai(pai)
struct aiRecord *pai;
{
  unsigned long         rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pai->dpvt;

  if (read_card(pai->inp.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal) == OK)
  {
     pai->rval = rawVal>>pPvt->lbit;

/* here is where we do the sign extensions for Bipolar....    */     
        if (pPvt->type ==1) {
           if (pai->rval & (2<<(pPvt->nbit-2))) 
               pai->rval |= ((2<<31) - (2<<(pPvt->nbit-2)))*2; 

	}

     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
  return(2);

}

/**************************************************************************
 *
 * Perform a write operation from a AO record
 *
 **************************************************************************/
static long write_ao(pao)
struct aoRecord *pao;
{

  unsigned long      rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pao->dpvt;

  if (write_card(pao->out.value.vmeio.card,pPvt->reg,
                 pPvt->mask,pao->rval<<pPvt->lbit) == OK)
  {
    if(read_card(pao->out.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal)
       == OK)
    {
      pao->rbv = rawVal>>pPvt->lbit;

/* here is where we do the sign extensions for Bipolar.... */        
        if (pPvt->type ==1) {
           if (pao->rbv & (2<<(pPvt->nbit-2)))
               pao->rbv |= ((2<<31) - (2<<(pPvt->nbit-2)))*2;

	}
      
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Perform a read operation from a LI record
 *
 **************************************************************************/
static long read_li(pli)
struct longinRecord *pli;
{

  unsigned long         rawVal;
  a32VmeDpvt           *pPvt = (a32VmeDpvt *)pli->dpvt;

  if (read_card(pli->inp.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal) == OK)
  {
     pli->val = rawVal>>pPvt->lbit;
     pli->udf = 0;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
  return(2);

}

/**************************************************************************
 *
 * Perform a write operation from a LO record
 *
 **************************************************************************/
static long write_lo(plo)
struct longoutRecord *plo;
{

  a32VmeDpvt           *pPvt = (a32VmeDpvt *)plo->dpvt;

  if (write_card(plo->out.value.vmeio.card,pPvt->reg,
                 pPvt->mask,plo->val<<pPvt->lbit) == OK)
  {
      return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}



/**************************************************************************
 *
 * Raw read a bitfield from the card
 *
 **************************************************************************/
static long read_card(card, reg, mask, value)
short           card;  
unsigned short  reg;
unsigned long   mask;   /* created in init_bo() */
unsigned long  *value; /* the value to return from the card */
{
  if (checkCard(card) == ERROR)
    return(ERROR);

  *value = cards[card].base->reg[reg] & mask;

  if (devA32VmeDebug >= 20)
    printf("devA32Vme: read 0x%4.4lX from card %d\n", *value, card);

  return(OK);
}



/**************************************************************************
 *
 * Write a bitfield to the card retaining the states of the other bits
 *
 **************************************************************************/
static long write_card(card, reg, mask, value)
short           card;
unsigned short  reg;
unsigned long   mask;
unsigned long   value;
{
  if (checkCard(card) == ERROR)
    return(ERROR);

  epicsMutexMustLock(cards[card].lock);
  cards[card].base->reg[reg] = ((cards[card].base->reg[reg] & ~mask) | 
                              (value & mask));
  epicsMutexUnlock(cards[card].lock);

  if (devA32VmeDebug >= 15)
    printf("devA32Vme: wrote 0x%4.4lX to card %d\n",
            cards[card].base->reg[reg], card);

  return(0);
}


/**************************************************************************
 *
 * Make sure card number is valid
 *
 **************************************************************************/
static long checkCard(card)
short   card;
{
  if ((card >= MAX_NUM_CARDS) || (cards[card].base == NULL))
    return(ERROR);
  else
    return(OK);
}

