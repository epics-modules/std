/* $Id: aiCvtDouble.c,v 1.2 2003-05-28 20:09:18 bcda Exp $ */
#include <vxWorks.h>
#include <types.h>
#include <stdioLib.h>
#include <lstLib.h>
#include <string.h>

#include "dbDefs.h"
#include "alarm.h"
#include "cvtTable.h"
#include "dbAccess.h"
#include "dbScan.h"
#include "dbEvent.h"
#include "dbFldTypes.h"
#include "devSup.h"
#include "recSup.h"
#include "recGbl.h"
#include "special.h"
#include "menuConvert.h"
#include "aiRecord.h"

#include <aiCvtDouble.h>

/*
 * allow for conversion of the DOUBLE val for ai device support
 *
 * This is a copy of aiRecord.c's conversion routine
 *
 * T. Straumann (PTB, 1999)
 *
 * $Log: not supported by cvs2svn $
 * Revision 1.1.1.1  2001/07/03 20:05:23  sluiter
 * Creating
 *
 * Revision 1.4  1999/04/20 19:25:16  strauman
 * *** empty log message ***
 *
 * Revision 1.3  1999/04/20 10:52:08  strauman
 *  - added comment
 *
 */

/* DISCLAIMER: This software is provided `as is' and without _any_ kind of
 *             warranty. Use it at your own risk - I won't be responsible
 *			   if your dog drowns as a consequence of using my software blah...
 */


void aiCvtDouble(aiRecord *pai)
{
	double val;


	val = pai->val;
	/* adjust slope and offset */
	if(pai->aslo!=0.0) val*=pai->aslo;
	val+=pai->aoff;

	/* convert raw to engineering units and signal units */
	if(pai->linr == menuConvertNO_CONVERSION) {
		; /* do nothing*/
	}
	else if(pai->linr == menuConvertLINEAR) {
#if ( BASE_VERSION > 3 || BASE_REVISION > 13 || BASE_MODIFICATION > 0 || BASE_UPDATE_LEVEL > 11 )
		val = (val * pai->eslo) + pai->eoff;
#else
		val = (val * pai->eslo);
#endif
	}
	else { /* must use breakpoint table */
                if (cvtRawToEngBpt(&val,pai->linr,pai->init,(void *)&pai->pbrk,&pai->lbrk)!=0) {
                      recGblSetSevr(pai,SOFT_ALARM,INVALID_ALARM);
                }
	}

	/* apply smoothing algorithm */
	if (pai->smoo != 0.0){
	    if (pai->init) pai->val = val;	/* initial condition */
	    pai->val = val * (1.00 - pai->smoo) + (pai->val * pai->smoo);
	}else{
	    pai->val = val;
	}
	pai->udf = FALSE;
	return;
}
