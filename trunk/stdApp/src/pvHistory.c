#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include <dbDefs.h>
#include <dbCommon.h>
#include <recSup.h>
#include <aSubRecord.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define NINT(f)  (int)((f)>0 ? (f)+0.5 : (f)-0.5)

volatile int pvHistoryDebug=0;

static long pvHistory_init(aSubRecord *pasub)
{
	double	*valb, *valc;
	double	*vald, *vale, *valf;
	long   	i, n;

	valb = (double *)pasub->valb;
	valc = (double *)pasub->valc;
	vald = (double *)pasub->vald;
	vale = (double *)pasub->vale;
	valf = (double *)pasub->valf;
	n = pasub->novb;
	for (i=0; i<n; i++) {
		valb[i] = 0.0;
		valc[i] = 0.0;
		vald[i] = 0.0;
		vale[i] = 0.0;
		valf[i] = 0.0;
	}
	return(0);
}
static long pvHistory(aSubRecord *pasub)
{
	double	*b, *valb, *valc;
	double	*d, *vald, *e, *vale, *f, *valf;
	long   	*a, i, n;

	n = pasub->novb;	/* max elements of arrays */
	a = (long *)pasub->a;
	b = (double *)pasub->b; /* current secsPastEpoch */
	d = (double *)pasub->d; /* current PV value */
	e = (double *)pasub->e; /* current PV value */
	f = (double *)pasub->f; /* current PV value */
	valb = (double *)pasub->valb; /* array of b values */
	valc = (double *)pasub->valc; /* (valb[] - curr_time), in hours */
	vald = (double *)pasub->vald; /* recorded PV value array */
	vale = (double *)pasub->vale; /* recorded PV value array */
	valf = (double *)pasub->valf; /* recorded PV value array */
	/* clear everything */
	if (*a) {
		for(i=0; i<n; i++) {
			valb[i] = *b;
			valc[i] = 0.0;
			vald[i] = 0.0;
			vale[i] = 0.0;
			valf[i] = 0.0;
		}
		*a = 0;
	} else {
		/* shift array contents to next higher index */
		for (i=n-1; i>0; i--) {
			valb[i] = valb[i-1];
			vald[i] = vald[i-1];
			vale[i] = vale[i-1];
			valf[i] = valf[i-1];
		}
		valb[0] = *b; /* record current secsPastEpoch */
		valc[0] = 0.0;
		vald[0] = *d;
		vale[0] = *e;
		valf[0] = *f;
		/* repeat last valid entry to end of array */
		for (i=1; i<n; i++) {
			if (valb[i] == 0) valb[i] = valb[i-1];
		}
		if (pvHistoryDebug) printf("pvHistory: secsPastEpoch=%f\n", *b);

		for (i=1; i<n; i++) {
			valc[i] = (valb[i]==0)?valc[i-1]:(valb[i] - valb[0])/3600;
		}
	}
	return(0);
}

#include <registryFunction.h>
#include <epicsExport.h>

epicsExportAddress(int, pvHistoryDebug);

static registryFunctionRef pvHistoryRef[] = {
	{"pvHistory_init", (REGISTRYFUNCTION)pvHistory_init},
	{"pvHistory", (REGISTRYFUNCTION)pvHistory}
};

static void pvHistoryRegister(void) {
	registryFunctionRefAdd(pvHistoryRef, NELEMENTS(pvHistoryRef));
}

epicsExportRegistrar(pvHistoryRegister);
