/************************************************************************/
/*
 *      Original Author: Eric Boucher
 *      Date:            04-09-98
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
*
 * Modification Log:
 * .01 05-01-98  erb  Initial development
 * .02 10-10-00  tmm  Put local #include in quotes instead of brackets
 */


#include <stdlib.h>
#include "xdr_lib.h"


bool_t xdr_complex(XDR* xdrs, struct complex *p)     
{
  return(xdr_float(xdrs, &p->r) &&
         xdr_float(xdrs, &p->i));
}

bool_t xdr_counted_string(XDR* xdrs, char** p)
{
  int input = (xdrs->x_op == XDR_DECODE);
  short length;
  /* If writing, obtain the length */  
  if(!input) length = strlen(*p);

  /* Transfer the string length */  
  if (!xdr_short(xdrs, &length)) return(FALSE);

  /* If reading, obtain room for the string */  
  if (input) {
    *p = malloc((unsigned) (length + 1));
    (*p)[length] = '\0'; /* Null termination */  
  }
  /* If the string length is nonzero, transfer it */  
  return(length ? xdr_string(xdrs, p, length) : TRUE);
}
