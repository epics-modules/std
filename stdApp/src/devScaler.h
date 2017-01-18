/* devScaler.h */
#include <devSup.h>

/* ----------------Device Support Entry Table for devScaler----------------- */

#define MAX_SCALER_CHANNELS 64

typedef struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	reset;
	DEVSUPFUN	read;
	DEVSUPFUN	write_preset;
	DEVSUPFUN	arm;
	DEVSUPFUN	done;
} SCALERDSET;

