/* devEpidSoft.c */

/* devEpidSoft.c - "Soft" Device Support Routines for epid records */
/*
 *      Original Author: Bob Dalesio
 *      Current Author:  Mark Rivers
 *
 * Modification Log:
 * -----------------
 * .01  11-10-99        mlr     Split this code into a seperate device support
                                module, so that the epid record can be used to
                                communicate with hardware or fast software for
                                higher performance applications.
 */


/* A discrete form of the PID algorithm is as follows
 * M(n) = KP*(E(n) + KI*SUMi(E(i)*dT(i))
 *         + KD*(E(n) -E(n-1))/dT(n)
 * where
 *  M(n)    Value of manipulated variable at nth sampling instant
 *  KP,KI,KD Proportional, Integral, and Differential Gains
 *      NOTE: KI is inverse of normal definition of KI
 *  E(n)    Error at nth sampling instant
 *  SUMi    Sum from i=0 to i=n
 *  dT(n)   Time difference between n-1 and n
 */

#include    <vxWorks.h>
#include    <types.h>
#include    <stdioLib.h>
#include    <tickLib.h>
#include    <alarm.h>
#include    <dbDefs.h>
#include    <dbAccess.h>
#include    <dbEvent.h>
#include    <dbFldTypes.h>
#include    <errMdef.h>
#include    <recSup.h>
#include    <devSup.h>
#include    "epidRecord.h"

/* Create DSET */
static long init_record();
static long do_pid();
typedef struct { /* epid DSET */
    long            number;
    DEVSUPFUN       dev_report;
    DEVSUPFUN       init;
    DEVSUPFUN       init_record;
    DEVSUPFUN       get_ioint_info;
    DEVSUPFUN       do_pid;
} EPID_SOFT_DSET;

EPID_SOFT_DSET devEpidSoft = {
    5,
    NULL,
    NULL,
    init_record,
    NULL,
    do_pid
};

static long init_record(epidRecord *pepid)
{
    /* For now we do nothing in this function */
    return(0);
}

static long do_pid(epidRecord *pepid)
{
    unsigned long   ctp;    /*clock ticks previous  */
    unsigned long   ct;     /*clock ticks       */
    float           cval;   /*actual value      */
    float           setp;   /*setpoint          */
    float           dt;     /*delta time (seconds)  */
    float       kp,ki,kd;   /*gains        */
    float           di;     /*change in integral term */
    float           e;      /*error         */
    float           ep;     /*previous error    */
    float           de;     /*change in error   */
    float           oval;   /*new value of manip variable */
    float           p;      /*proportional contribution*/
    float           i;      /*integral contribution*/
    float           d;      /*derivative contribution*/

    /* fetch the controlled value */
    if (pepid->inp.type == CONSTANT) { /* nothing to control*/
        if (recGblSetSevr(pepid,SOFT_ALARM,INVALID_ALARM)) return(0);
    }
    if (dbGetLink(&pepid->inp,DBR_FLOAT,&pepid->cval,0,0)) {
       recGblSetSevr(pepid,LINK_ALARM,INVALID_ALARM);
       return(0);
    }
    
    setp = pepid->val;
    cval = pepid->cval;
    
    /* compute time difference and make sure it is large enough*/
    ctp = pepid->ct;
    ct = tickGet();
    if(ctp==0) {/*this happens the first time*/
        dt=0.0;
    } else {
        if(ctp<ct) {
            dt = (float)(ct-ctp);
        } else { /* clock has overflowed */
            dt = (unsigned long)(0xffffffff) - ctp;
            dt = dt + ct + 1;
        }
        dt = dt/vxTicksPerSecond;
        if(dt<pepid->mdt) return(1);
    }
    /* get the rest of values needed */
    kp = pepid->kp;
    ki = pepid->ki;
    kd = pepid->kd;
    ep = pepid->err;
    oval = pepid->oval;
    i = pepid->i;
    e = setp - cval;
    de = e - ep;
    p = kp*e;
    /* Sanity checks on integral term:
     * 1) Don't increase I if output >= DRVH
     * 2) Don't decrease I if output <= DRVL
     * 3) Limit the integral term to be in the range between DRLV and DRVH
     * 4) If KI is zero then set the sum to zero, to allow easily turning
     *    off the integral term for PID tuning.
     */
    di = kp*ki*e*dt;
    if (((oval > pepid->drvl) && (oval < pepid->drvh)) ||
        ((oval >= pepid->drvh) && ( di < 0.)) ||
        ((oval <= pepid->drvl)  && ( di > 0.))) {
        i = i + di;
        if (i < pepid->drvl) i = pepid->drvl;
        if (i > pepid->drvh) i = pepid->drvh;
    }
    if (ki == 0) i = 0;
    if(dt>0.0) d = kp*kd*(de/dt); else d = 0.0;
    oval = p + i + d;
    /* Limit output to range from DRLV to DRVH */
    if (oval > pepid->drvh) oval = pepid->drvh;
    if (oval < pepid->drvl) oval = pepid->drvl;
    /* update record*/
    pepid->ct  = ct;
    pepid->dt   = dt;
    pepid->err  = e;
    pepid->cval  = cval;
    pepid->oval  = oval;
    pepid->p  = p;
    pepid->i  = i;
    pepid->d  = d;
    
    /* If feedback is on, and output link is a PV_LINK then write the 
     * output link */
    if (pepid->fbon && (pepid->outl.type != CONSTANT)) {
        if (dbPutLink(&pepid->outl,DBR_FLOAT, &pepid->oval,1)) {
            recGblSetSevr(pepid,LINK_ALARM,INVALID_ALARM);
        }
    } 
    return(0);
}
