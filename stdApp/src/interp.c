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

#define MAXORDER 15
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define NINT(f)  (int)((f)>0 ? (f)+0.5 : (f)-0.5)
int polint(double *xa, double *ya, int n, double x, double *y);

#define MAXPOINTS 2000
double y2[MAXPOINTS+1], u[MAXPOINTS+1];
int spline(double *x, double *y, int n);
int splint(double *xa, double *ya, int n, double x, double *y);

volatile int interpDebug=0;

long interp_init(genSubRecord *pgsub)
{
	double	*a, *b, *c, *lo_lim, *hi_lim;
	int i, j, n=100, *order;
	long *e;

	a = (double *)pgsub->a;
	b = (double *)pgsub->b;
	c = (double *)pgsub->c;
	e = (long *)pgsub->e;
	lo_lim = (double *)pgsub->valf;
	hi_lim = (double *)pgsub->valg;
	order = (int *)pgsub->f;
	*e = n;
	for (i=0; i<n/2; i++) {
		a[i] = 4*3.141592654*i/(n-1);
		b[i] = sin(a[i]);
		c[i] = cos(a[i]);
	}
	for (i=n/2, j=0; i<n; i++, j++) {
		a[i] = 4*3.141592654*i/(n-1);
		b[i] = j<5 ? 0 : 1;
		c[i] = j>5 ? 0 : 1;
		if (j > 10) j = 0;
	}
	*lo_lim = 0;
	*hi_lim = 4*3.141592654;
	return(0);
}

long interp_do(genSubRecord *pgsub)
{
	double	*a, *b, *c, *d, *lo_lim, *hi_lim, ix;
	double	*valb, *valc;
	int    	hi, lo, n, mid, i, s=0, first, *order;

	a = (double *)pgsub->a;
	b = (double *)pgsub->b;
	c = (double *)pgsub->c;
	d = (double *)pgsub->d;
	n = *(int *)pgsub->e;
	valb = (double *)pgsub->valb;
	valc = (double *)pgsub->valc;
	lo_lim = (double *)pgsub->valf;
	hi_lim = (double *)pgsub->valg;
	order = (int *)pgsub->f;

	if (interpDebug) printf("interp: x=%f, ", *d);
	/* if arrays haven't been set up yet, output is same as input */
	if (n <= 1) {
		*valb = *valc = *d;
		return(0);
	}
	if (*d < a[0]) {*valb = b[0]; *valc = c[0]; return(-1);}
	if (*d > a[n-1]) {*valb = b[n-1]; *valc = c[n-1]; return(-1);}

	/* find limits of independent variable */
	*lo_lim = *hi_lim  = a[0];
	for (i=1; i<n; i++) {
		if (a[i] < *lo_lim) *lo_lim = a[i];
		if (a[i] > *hi_lim) *hi_lim = a[i];
	}
	/* binary search for indexes of a[] bracketing *d */
	for (lo=0, hi=n-1, mid = (hi+lo)/2; abs(hi-lo)>1;) {
		if (*d > a[mid]) {
			lo = mid;
		} else {
			hi = mid;
		}
		mid = (hi+lo)/2;
	}
	/* index at which *d would occur in a[] */
	ix = lo + (*d-a[lo])/(a[hi]-a[lo]);

	if (*order > 1) {
		/* polynomial */
		i = MIN(*order+1, n);	/* number of points required */
		/* arrange that we switch from one set of points to the next */
		/* when *d crosses a point, and not halfway between */
		first = MAX(0, MIN(n-i, (int)(ix - i/2)));
		s = polint(&a[first], &b[first], i, *d, valb);
		if (interpDebug >= 10) {
			printf("interp:polint: (O%d, first=%d) valb = %f\n",
				*order, first, *valb);
		}
		s = polint(&a[first], &c[first], i, *d, valc);
	} else if (*order < 0) {
		/* cubic spline */
		if (interpDebug) printf("interp:spline\n");
		s = spline(a, b, n);
		s = splint(a, b, n, *d, valb);
		s = spline(a, c, n);
		s = splint(a, c, n, *d, valc);
	} else {
		/* linear interpolation */
		*valb = b[lo] + (ix-lo)*(b[hi]-b[lo]);
		*valc = c[lo] + (ix-lo)*(c[hi]-c[lo]);
		if (interpDebug) printf("interp:linear: (%d:%f:%d) valb = %f\n",			lo, ix, hi, *valb);
	}
	if (interpDebug) {
		printf("\n");
		*valc = b[lo] + (ix-lo)*(b[hi]-b[lo]);
	}
	return(s);
}

int polint(double *xa, double *ya, int n, double x, double *y)
{
	int i,m,ns=1;
	double den,dif,dift,ho,hp,w, dy;
	double c[MAXORDER+2], d[MAXORDER+2];

	if (n > MAXORDER+1) n = MAXORDER+1; /* array bounds protection */
	xa--; ya--;	/* convert from c array to fortran array */

	if (interpDebug >= 20) {
		printf("interp:polint: x[1..5] = [ ");
		for (i=1; i<=5; i++) printf("%f ", xa[i]);
		printf("]\n");
	}
	dif=fabs(x-xa[1]);
	for (i=1;i<=n;i++) {
		if ( (dift=fabs(x-xa[i])) < dif) {
			ns=i;
			dif=dift;
		}
		c[i]=ya[i];
		d[i]=ya[i];
	}
	*y = ya[ns--];
	if (interpDebug >= 10) printf("ns=%d ", ns);
	for (m=1;m<n;m++) {
		for (i=1;i<=n-m;i++) {
			ho=xa[i]-x;
			hp=xa[i+m]-x;
			w=c[i+1]-d[i];
			if ( (den=ho-hp) == 0.0) {
				printf("Error in routine POLINT");
				*y = dy = 0;
				return(-1);
			}
			den=w/den;
			d[i]=hp*den;
			c[i]=ho*den;
		}
		if (2*ns < (n-m)) {
			dy = c[ns+1];
				if (interpDebug >= 10) printf("C");
		} else {
			dy = d[ns--];
			if (interpDebug >= 10) printf("D");
		}
		*y += dy;
	}
	return(0);
}

int spline(double *x, double *y, int n)
{
	int i, k;
	double p, qn, sig, un;

	/* convert from c array to fortran array */
	x--; y--;

	y2[1] = u[1] = 0.0;
	for (i=2; i<=n-1; i++) {
		sig = (x[i]-x[i-1])/(x[i+1]-x[i-1]);
		p = sig*y2[i-1]+2.0;
		y2[i] = (sig-1.0)/p;
		u[i] = (y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
		u[i] = (6.0*u[i]/(x[i+1]-x[i-1])-sig*u[i-1])/p;
	}
	qn = un = 0.0;
	y2[n] = (un-qn*u[n-1])/(qn*y2[n-1]+1.0);
	for (k=n-1; k>=1; k--)
		y2[k] = y2[k]*y2[k+1]+u[k];
	return(0);
}

int splint(double *xa, double *ya, int n, double x, double *y)
{
	int klo,khi,k;
	double h,b,a;

	/* convert from c array to fortran array */
	xa--; ya--;

	klo = 1;
	khi = n;
	while (khi-klo > 1) {
		k = (khi+klo) >> 1;
		if (xa[k] > x) khi = k;
		else klo = k;
	}
	h = xa[khi]-xa[klo];
	if (h == 0.0) {
		printf("Bad XA input to routine SPLINT");
		return(-1);
	}
	a = (xa[khi]-x)/h;
	b = (x-xa[klo])/h;
	*y = a*ya[klo]+b*ya[khi]+((a*a*a-a)*y2[klo]+(b*b*b-b)*y2[khi])*(h*h)/6.0;
	return(0);
}
