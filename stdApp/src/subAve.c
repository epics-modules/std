/* @(#)subAve.c	1.2 4/27/95     */
/* subAve.c -  */
/*
 * Author:      Frank Lenkszus
 * Date:        9/29/93
 *
 *      Experimental Physics and Industrial Control System (EPICS)
*/
/*
*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
AND IN ALL SOURCE LISTINGS OF THE CODE.
 
(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO
 
Argonne National Laboratory (ANL), with facilities in the States of 
Illinois and Idaho, is owned by the United States Government, and
operated by the University of Chicago under provision of a contract
with the Department of Energy.

Portions of this material resulted from work developed under a U.S.
Government contract and are subject to the following license:  For
a period of five years from March 30, 1993, the Government is
granted for itself and others acting on its behalf a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, and perform
publicly and display publicly.  With the approval of DOE, this
period may be renewed for two additional five year periods. 
Following the expiration of this period or periods, the Government
is granted for itself and others acting on its behalf, a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, distribute copies
to the public, perform publicly and display publicly, and to permit
others to do so.

*****************************************************************
                                DISCLAIMER
*****************************************************************

NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
OWNED RIGHTS.  

*****************************************************************
LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
*/
/*
* Modification Log:
* -----------------
* .01  9-29-93  frl  initial
* .02  4-27-95  frl  added RESTART and MODE
*  03  9-05-02  tmm  If NUM_2_AVE (.A) is greater than allowed, set it
*                    to the maximum allowed number, so user can see it.
*                    Set restart field (.C) to zero after we use it.
*                    Report current sample via .E field.
*/

/*  subroutine to average data */
/*  F. Lenkszus */

#include <vxWorks.h>
#include <stdlib.h>
#include <stdio.h>
#include <dbDefs.h>
#include <dbEvent.h>
#include <devLib.h>
#include <alarm.h>
#include <recSup.h>
#include <subRecord.h>

#define	CIRBUFSIZE	10000
#define NO_ERR_RPT	-1

#define NUM_2_AVE	psub->a
#define INPUTVAL	psub->b
#define RESTART		psub->c
#define MODE		psub->d
#define FILL		psub->e

#define CONTINUOUS_MODE	0
#define STOPONNUM_MODE  1

int	debugSubAve = 0;

struct	fcirBuf {
	short	num;
	short	cur;
        short	fill;
	double  *wp;
	double  sum;
	double  ave;
	double	buf[CIRBUFSIZE];
};

long	initSubAve(
struct	subRecord *psub)
{
  char	*xname="initSubAve";
  struct  fcirBuf	*p;
  short	i;

  if ((psub->dpvt = malloc( sizeof(struct fcirBuf))) == NULL) {
        errPrintf(S_dev_noMemory, __FILE__, __LINE__,
		"%s: couldn't allocate memory for %s", xname, psub->name);
	return(S_dev_noMemory);
  }
  p = (struct fcirBuf *)psub->dpvt;
  if(debugSubAve)
	printf("%s: Init completed for Subroutine Record %s\n", xname, 
		psub->name);
  for ( i = 0 ; i < CIRBUFSIZE; i++)
	p->buf[i] = 0;
  p->num = 1;
  p->fill = p->cur  = 0;
  p->wp = p->buf;
  p->ave = p->sum = 0;
  return(OK);
}


long	SubAve(
struct	subRecord *psub)
{
  char	*xname="SubAve";
  long	num;
  short i;
  short restart;
  unsigned short monitor_mask;
  struct  fcirBuf	*p;

	if((p = (struct fcirBuf *)psub->dpvt) == NULL) {
		if(debugSubAve)
			errPrintf(S_dev_noMemory, __FILE__, __LINE__,
				"%s: dpvt in NULL for %s", xname, psub->name);
		return(ERROR);
	}
  num = (long)NUM_2_AVE;
  if ( num >  CIRBUFSIZE ) {
	if(debugSubAve)
		errPrintf(NO_ERR_RPT, __FILE__, __LINE__,
			"%s: Num to ave (%d) exceeds limit (%d) for PV %s",
			 xname, num, CIRBUFSIZE,  psub->name);
	num = CIRBUFSIZE;
	NUM_2_AVE = num;
	db_post_events(psub, &psub->a, DBE_VALUE);
  }
	
  restart = RESTART;
  if (RESTART) {
	RESTART=0;
	db_post_events(psub, &psub->c, DBE_VALUE);
  }

  if ( ((num != p->num) && (MODE == CONTINUOUS_MODE)) || restart ) {
  	for ( i=0; i < p->num; i++)
		p->buf[i]=0;
  	p->wp = p->buf;
	*p->wp = 0;
        p->fill = p->cur = 0;
	p->num = num;
	p->ave = p->sum = 0;
  }
  if( p->fill == p->num) {
	if( MODE == CONTINUOUS_MODE) {
		p->sum += INPUTVAL -  *p->wp;
		p->ave = p->sum/(double)num;
		*p->wp = INPUTVAL;
		if( ++(p->wp) >= p->buf + p->num) {
		    p->wp = p->buf;
			if(debugSubAve > 10) {
  			    for ( i=0; i < p->num; i++)
				printf("buf[%d] = %f\n", i, p->buf[i]);
			}
		}
	}
	monitor_mask = recGblResetAlarms(psub);
	db_post_events(psub, &psub->val, monitor_mask);
  } else {
	recGblSetSevr(psub, SOFT_ALARM, MAJOR_ALARM);
	p->sum += INPUTVAL;
	p->ave = p->sum/(double)(++p->fill);
	*p->wp = INPUTVAL;
	if( ++(p->wp) >= p->buf + p->num)
		p->wp = p->buf;
  }
  psub->val = p->ave;
  if(debugSubAve) {
      printf("%s: ave = %.3f, sum = %.3f, num = %d\n", xname, p->ave, p->sum,
		 p->fill);
  }
  psub->e = p->fill;
  db_post_events(psub, &psub->e, DBE_VALUE);

  return(OK);
}

