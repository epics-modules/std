/* varoc_driver.c */
/* share/src/drv @(#)drvVaroc.c	1.0     6/7/93 */
/*
 * subroutines that are used to interface to the Varoc absolute encoder
 * interface on VMEbus      
 *
 * 	Author:      Karen J. Coulter
 * 	Date:        6-7-93
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *	Copyright 1991, the Regents of the University of California,
 *	and the University of Chicago Board of Governors.
 *
 *	This software was produced under  U.S. Government contracts:
 *	(W-7405-ENG-36) at the Los Alamos National Laboratory,
 *	and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *	Initial development by:
 *		The Controls and Automation Group (AT-8)
 *		Ground Test Accelerator
 *		Accelerator Technology Division
 *		Los Alamos National Laboratory
 *
 *	Co-developed with
 *		The Controls and Computing Group
 *		Accelerator Systems Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .00  06-07-93        kjc     original version, uses 0x0800 from ZIO085 card
 *                              assumes only one card (up to 16 encoders)
 *                              Modeled after drvVmi4100.c
 */

/*
 * Code Portions:
 *
 * varoc_drv_init  Finds and initializes varoc (currently assumes one card) 
 * varoc_driver    interfaces to varoc, writes cmd registers, reads data
 */

/* static char SccsId[] = "@(#)drvVaroc.c	1.0\t6/7/93";  */

#include <vxWorks.h>
#include <vme.h>
#include <dbDefs.h>
#include <drvSup.h>
#include <module_types.h>


static long varoc_io_report();
long varoc_drv_init();

struct {
        long    number;
        DRVSUPFUN       report;
        DRVSUPFUN       init;
} drvVaroc={
        2,
        varoc_io_report,
        varoc_drv_init};


union varoc_reg {
  unsigned short int data;       /* 16 bits of data */ 
  unsigned short int cmd;        /* word write accepted, MSB is don't care */
};

struct ai_varoc{
  volatile union varoc_reg reg0;  /* cmd0 and rd0 */
  volatile union varoc_reg reg1;  /* cmd1 and rd1 */
  char  endpad[4];       /* upper 4 bytes copy lower 4 bytes */
  char  nextaddrpad[8];  /* next addr=base+16 bytes total */
};


struct ai_varoc **pai_varoc;
struct ai_varoc  *varoc_addr;


#define  MAX_VAROC_CARDS  1
#define  VAROC_SIZE       16      /* 8 bytes occupied + 8 bytes skipped */
#define  ADDR             0x0800
#define  BUSY             0x8000
#define  CMD1_OPMODE      0x0000
#define  CMD1_INTLOOP     0x0060
#define  CMD1_EXTLOOP     0x0020
#define  CMD1_DEBUG       0x0010
#define  PARITY           0x0000  /* no parity assumed, should be rec field */
#define  GRAY             0x0020  /* gray code assumed, should be rec field */
#define  SLOW             0x0000  /* using slow mode; rec field?? */


/*
 * VAROC_DRV_INIT
 *
 * intialization for the varoc encoder interface
 */
long varoc_drv_init()
{
  int i,status,value;
  
  pai_varoc = (struct ai_varoc **) calloc(MAX_VAROC_CARDS,sizeof(pai_varoc));
  if (!pai_varoc) {
	printf("\n%s: pai_varoc = %d! (returning error)\n",__FILE__,pai_varoc);
    return ERROR;
  }
  
  status = sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, ADDR, &varoc_addr);
  if (status != 0){
    printf("%s: varoc short address base failure: status = %d \n",
	   __FILE__,status);
    return ERROR;
  }

  /* find each card that is present in IOC */
  for (i = 0; i < MAX_VAROC_CARDS; i++, varoc_addr+= VAROC_SIZE) {
    if (vxMemProbe(varoc_addr,VX_READ,sizeof(short int),&value) == 0) {
        pai_varoc[i] = varoc_addr;
	 printf("\a\a\a\n%s: card %d present at addr %X.\n",__FILE__,i,varoc_addr); 
      }
    else{
        pai_varoc[i] = 0;
      }
  }
      
  return (0);
}



long varoc_io_report(level)
  short int level;
 { 
   int i;

   for (i=0; i < MAX_VAROC_CARDS; i++) {
        if (pai_varoc[i]) {
	    printf("AI: VAROC:      card %d\n",i);
	  }
      }
   return(0);
 }


/*
 * VAROC_DRIVER
 *
 * interface to the Varoc Absolute Encoder board (16 channels)
 */
varoc_driver(card, chan, numbits, gray, prbval)
register unsigned short card;
register unsigned short chan;
int                     numbits;
int			gray;
register long           *prbval;
{
  long                   msb=0, lsb=0;
  unsigned short int     type;
  volatile unsigned short int   rd1,rd0;

/* ---------------------------------------------------------------------- */
/* if Varoc address is 0, the board isn't there, or it wasn't initialized */
/* check it, if it still returns a zero, it will error and say it doesn't */
/* exist.                                                                 */
/* ---------------------------------------------------------------------- */

	if(pai_varoc[card] == 0)
 		msb = varoc_drv_init();

/* ---------------------------------------------------------------------- */
/* verify card exists                                                     */
/* ---------------------------------------------------------------------- */
 
  if (card > 1 || !pai_varoc[card]){
    printf("\n %s: Varoc Board Does Not exist!\n",__FILE__);
    printf(" card = %d --- pai_varoc[card] = %d\n",card,pai_varoc[card]);
    return (-1);
  }

/* -------------------------------------------------------------------- */
/*  card exists, so, type = the number of bits used in the encoder.     */
/*  Taken from the DESC field. * should be changed to it's own field I  */
/*  think.  I guess that will come later.                               */
/* -------------------------------------------------------------------- */

  type = (unsigned short int)(numbits);   /* bits 4-0 are TYPE */
	 
/* --------------------------------------------------- */
/* write CMD registers to configure channel to be read */
/* CMD0:0-4   number of bits 0-25                      */
/* CMD0:5     Gray 1=on 0=off                          */
/* CMD0:6     NODUM 1=no dummy 0=dummy bit after data  */
/* CMD0:7     PARITY 1=parity on 0=parity off          */
/*                                                     */
/* CMD1:0-3   CHANNEL channel *signal number 0-16      */
/* CMD1:4     PPOL polarity of parity bit              */
/* CMD1:5-6   Speed 00-slow 10-fast                    */
/* CMD1:7     0 (unused)                               */
/* --------------------------------------------------- */

  pai_varoc[card]->reg0.cmd = (unsigned short int)(PARITY|gray|SLOW|type);
  pai_varoc[card]->reg1.cmd = (unsigned short int)(CMD1_OPMODE | chan);

/* ----------------------------------------------------------- */
/* read RD1 register until not busy; read RD1 and RD0 for data */
/* only waiting for register READY, not an asynch dev!!        */
/*                                                             */
/* RD1:0-8    Data MSB                                         */
/* RD1:9      Dummy bit                                        */
/* RD1:10-13  readback of Channel number                       */
/* RD1:14     ERROR in reading of encoder                      */
/* RD1:15     BUSY bit                                         */
/* ----------------------------------------------------------- */
  
  do {
    rd1 = (unsigned short int)pai_varoc[card]->reg1.data;
  } while (rd1 & BUSY);

/* ----------------------------------------------------------- */
/* Do some error checking to see if the value is ok.           */
/* ----------------------------------------------------------- */
#if 0
	if(rd1 & 0x4000) {
          printf("Varoc Error Bit set!\n");
	  return (-1);
	}
#endif
/* ----------------------------------------------------------- */
/*  Read the LSB from RD0.  Then, combine the MSByte from RD1  */
/*  to the LSByte of RD0.                                      */
/*                                                             */
/* RD0:0-15   DATA LSByte                                      */
/*                                                             */
/* ----------------------------------------------------------- */

  rd0 = (unsigned short int) pai_varoc[card]->reg0.data;
  msb = ((long) (rd1 & 0x00FF) )<<16;

  *prbval = msb | (long)rd0;

  return (0);
}
