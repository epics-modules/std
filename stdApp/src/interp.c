#include <vxWorks.h>
#include <types.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdioLib.h>

#include <dbEvent.h>
#include <dbDefs.h>
#include <dbCommon.h>
#include <recSup.h>
#include <genSubRecord.h>

volatile int interpDebug=0;

long interp_init(genSubRecord *pgsub)
{
	double	*a, *b, *c;
	int i, n=100;
	long *e;

	a = (double *)pgsub->a;
	b = (double *)pgsub->b;
	c = (double *)pgsub->c;
	e = (long *)pgsub->e;
	*e = n;
	for (i=0; i<n; i++) {
		a[i] = 2*3.141592654*i/n;
		b[i] = sin(a[i]);
		c[i] = cos(a[i]);
	}
	return(0);
}

long interp_do(genSubRecord *pgsub)
{
	double	*a, *b, *c, *d, ix, p;
	double	*valb, *valc;
	int		hi, lo, n, mid;

	a = (double *)pgsub->a;
	b = (double *)pgsub->b;
	c = (double *)pgsub->c;
	d = (double *)pgsub->d;
	n = *(int *)pgsub->e;
	valb = (double *)pgsub->valb;
	valc = (double *)pgsub->valc;

	if (interpDebug) printf("x = %f\n", *d);
	/* if arrays haven't been set up yet, output is same as input */
	if (n <= 1) {
		*valb = *valc = *d;
		return(0);
	}
	if ((*d >= a[0]) && (*d <= a[n-1])) {
		/* binary search for index into a array */
		for (lo=0, hi=n-1, mid = (hi+lo)/2; abs(hi-lo)>1;) {
			if (*d > a[mid]) {
				lo = mid;
			} else {
				hi = mid;
			}
			mid = (hi+lo)/2;
		}
		ix = lo + (*d-a[lo])/(a[hi]-a[lo]);
		if (interpDebug) printf("index = %f\n", ix);

		/* linear interpolation for now */
		*valb = b[lo] + (ix-lo)*(b[hi]-b[lo]);
		*valc = c[lo] + (ix-lo)*(c[hi]-c[lo]);
		if (interpDebug) printf("valb = %f\n", *valb);
	}
	return(0);
}
