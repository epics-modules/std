/* subroutine for checking the status of an IOC                
 *
 * This subroutine assumes the following use of the subroutine record inputs:
 *
 * VAL	: ioc health status: 1= ok;  0 = bad
 *   A  : if set, subroutine will clear bad status and reset A. (used if task is restarted)
 */

/* mods ...
 *
 * 06/17/94	nda	initial coding. Simply checks all tasks monitored by watchdog
 *
 */


#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdio.h>
#include        <vme.h>

#include	<dbDefs.h>
#include    <registryFunction.h>
#include	<subRecord.h>
#include    <epicsExport.h>
#include	<taskwd.h>

void taskFault();
long iocStatus;


long  iocCheckInit(psub)
    struct subRecord    *psub;
{
    long ret = 0;

    taskwdAnyInsert(NULL, taskFault, NULL);
    iocStatus = 1;                           /* assume OK during init */
    psub->val = iocStatus;

    return(ret);
}


long  iocCheck(psub)
    struct subRecord	*psub;
{
    short a;

    a=psub->a;

    /* if a is set, make status = OK, reset a */
    if(a) {
        iocStatus = 1;
        psub->a = 0;
    }

    psub->val = iocStatus;

    return(0);
}

void taskFault(userarg)
    VOID *userarg;
{
    iocStatus = 0;
}

static registryFunctionRef iocCheckRef[] = {
    {"iocCheckInit",(REGISTRYFUNCTION)iocCheckInit},
    {"iocCheck",(REGISTRYFUNCTION)iocCheck}
};

static void iocCheckRegister(void)
{
    registryFunctionRefAdd(iocCheckRef,NELEMENTS(iocCheckRef));
}
epicsExportRegistrar(iocCheckRegister);


