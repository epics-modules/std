#ifdef vxWorks
#include  <vxWorks.h>
#endif

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<ctype.h>
#include	<tickLib.h>
#include	"dbDefs.h"
#include	"sCalcPostfix.h"
#include	"sCalcPostfixPvt.h"	/* need USES_STRING for testing */
#include	"postfix.h"

long test_sCalcPostfix(char *pinfix)
{
	short i, error;
	long stat;
	char *p_postfix=NULL;
	char cbuf[1000];
	ULONG start;
	double parg[12], result;
	char *ppsarg[12], space[1200], sresult[100];

	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = postfix(pinfix, cbuf, &error);
	printf("%d calls to postfix() in 1/60 s: %f us per call\n",
		i, 1.e6/(i*sysClkRateGet()));

	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = calcPerform(parg, &result, cbuf);
	printf("%d calls to calcPerform() in 1/60 s: %f us per call\n",
		i, 1.e6/(i*sysClkRateGet()));

	for (i=0; i<12; i++) ppsarg[i] = &space[i*100];
	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = sCalcPostfix(pinfix, &p_postfix, &error);
	printf("%d calls to sCalcPostfix() in 1/60 s: %f us per call\n",
		i, 1.e6/(i*sysClkRateGet()));

	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = sCalcPerform(parg, 12, ppsarg, 12, &result, sresult, 100, p_postfix);
	printf("%d calls to sCalcPerform() in 1/60 s: %f us per call\n",
		i, 1.e6/(i*sysClkRateGet()));
	return(0);
}

long test_sCalcPerform(char *pinfix)
{
	int i;
	short error;
	long stat;
	char *p_postfix=NULL;
	char cbuf[1000];
	ULONG start;
	double parg[12], result;
	char *ppsarg[12], space[1200], sresult[100];

	for (i=0; i<12; i++) {
		parg[i] = (double)i;
		ppsarg[i] = &space[i*100];
		sprintf(ppsarg[i], "%d", i);
	}

	stat = postfix(pinfix, cbuf, &error);
	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = calcPerform(parg, &result, cbuf);
	printf(":calcPerform()                                 : %.2f us/call\n",
		1.e6/(i*sysClkRateGet()));

	stat = sCalcPostfix(pinfix, &p_postfix, &error);

	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = sCalcPerform(parg, 12, ppsarg, 12, &result, sresult, 100, p_postfix);
	printf(":sCalcPerform()                                : %.2f us/call\n",
		1.e6/(i*sysClkRateGet()));

	for (start = tickGet(); start==tickGet(); );
	for (i=0, start = tickGet(); start==tickGet(); i++)
		stat = sCalcPerform(parg, 12, NULL, 0, &result, NULL, 0, p_postfix);
	printf(":sCalcPerform(no-string-buf)                   : %.2f us/call\n",
		1.e6/(i*sysClkRateGet()));

	if (p_postfix) free(p_postfix);
	return(0);
}
