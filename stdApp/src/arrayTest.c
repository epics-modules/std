#include <stddef.h>
#include <stdlib.h>
/* #include <ctype.h> */
#include <math.h>
#include <stdio.h>

/* #include <dbEvent.h> */
#include <dbDefs.h>
#include <dbCommon.h>
#include <recSup.h>
#include <genSubRecord.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define NINT(f)  (int)((f)>0 ? (f)+0.5 : (f)-0.5)


volatile int arrayTestDebug=0;

long arrayTest_init(genSubRecord *pgsub)
{
	long *e = (long *)pgsub->e;

	if (*e == 0) *e = (long)pgsub->nova;
	return(0);
}

long arrayTest_do(genSubRecord *pgsub)
{
	double	*a, *valb, *vala;
	long	i, *e;

	a = (double *)pgsub->a;
	valb = (double *)pgsub->valb;
	vala = (double *)pgsub->vala;
	e = (long *)pgsub->e;
	if (*e > pgsub->nova) *e = (long)pgsub->nova;
	for (i=0; i<*e; i++) {
		vala[i] = *a+i;
		valb[i] = i;
		if (arrayTestDebug) printf("arrayTest: vala[%ld]=%f\n", i, vala[i]);
	}
	return(0);
}
