/* tableRecord.c - Record Support Routines for 6-degree-of-freedom table */
/*
 *      Original Author: Tim Mooney
 *      Date:            8/2/94
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  8-02-94  tmm  initial development
 * .02  2-08-95  tmm  bug fixes, move to EPICS 3.12
 * .03  3-14-95  tmm  minimize db_post, ensure to poke a motor
 * .03  4-04-95  nda  added sset and suse field for easier burting
 * .04  8-25-95  tmm  added get_units()
 * .05  10-05-95 tmm  encoder inputs get same treatment as drive-value readbacks
 * .06  02-21-96 tmm  v2.0: added version (VERS) field
 * .07  06-06-96 tmm  v2.01: default get_precision to PREC field
 * .08  08-16-96 tmm  v3.0: conversion to EPICS 3.13
 * .09  10-31-97 tmm  v3.1: allow fewer than six motors, check links
 * .10  11/07/97 tmm  v3.2: make record support fully re-entrant
 * .11  11/26/97 tmm  v4.0: allow user to specify x location of M2 (.LX21)
 * .12  02/02/98 tmm  v4.1: fixed inverse matrix elements b[2][1,2,4]; support
 *                          GEOCARS geometry with GEOM switch.
 * .13  02/05/98 tmm  v4.2: added user-limit fields UH*, UL* which restrict
 *                          limits calculated from motor limits
 * .14  01/05/99 tmm  v5.0: exact solution
 * .15  02/05/99 tmm  v5.01: RX, RY, RZ get same treatment in special as SX...
 * .16  02/28/99 tmm  v5.02: Fixed typo in matrix determinant (left out a term).
 * .17  03/09/99 tmm  v5.10: Fixed GEOCARS-geometry error (took wrong root).
 * .18  10/07/01 tmm  v5.11: Add support for Newport 5-motor table geometry
 * .19  10/19/01 tmm  v5.12: Fix Newport 5&6-motor geometry
 * .20  01/10/02 tmm  v5.13: Add PNC geometry (SRI with M0 and M1 swapped)
 * .21  07/03/03 rls  v5.13: Port to R3.14
*/

#define VERSION 5.13

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<math.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbFldTypes.h>
#include	<dbEvent.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<recGbl.h>
#include	<special.h>
#define GEN_SIZE_OFFSET
#include	"tableRecord.h"
#undef GEN_SIZE_OFFSET
#include	"epicsExport.h"

double D2R;	/* set in init_record() */

#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define SMALL 1.e-6
#define LARGE 1.e9
#define X 0
#define Y 1
#define Z 2
#define AX_6 0
#define AY_6 1
#define AZ_6 2
#define X_6  3
#define Y_6  4
#define Z_6  5

#define M0X	0
#define M0Y	1
#define M1Y	2
#define M2X	3
#define M2Y	4
#define M2Z	5

struct trajectory {
	double user;
	double motor[6];
	short lvio;
};

static void     checkAlarms();
static void     monitor();
static void		PrintValues(tableRecord *ptbl);
static long     ProcessOutputLinks(tableRecord *ptbl,
					unsigned short motor_move_mask);
static long     SaveMotorSpeeds(tableRecord *ptbl, double *sv0x);
static long     RestoreMotorSpeeds(tableRecord *ptbl, double *sv0x);
static long     GetMotorLimits(tableRecord *ptbl);
static long     MotorLimitViol(tableRecord *ptbl);
static long     UserLimitViol(tableRecord *ptbl);
static long     GetReadback(tableRecord *ptbl, double *r);
static long		ReadEncoders(tableRecord *ptbl, double *e);
static void     InitGeometry(tableRecord *ptbl);
static void		NaiveMotorToPivotPointVector(tableRecord *ptbl, double *m,
					double *q0, double *q1, double *q2);
static void		MotorToPivotPointVector(tableRecord *ptbl, double *m,
					double *q0, double *q1, double *q2);
static void		PivotPointVectorToLocalUserAngles(tableRecord *ptbl,
					double *q0, double *q1, double *q2, double *u);
static void		MotorToLocalUserAngles(tableRecord *ptbl, double *m, double *u);
static void     MotorToUser(tableRecord *ptbl, double *m, double *u);
static void 	LocalUserToPivotPointVector(tableRecord *ptbl, double *user,
					double *pp0, double *pp1, double *pp2);
static void 	PivotPointVectorToMotor(tableRecord *ptbl,
					double *pp0, double *pp1, double *pp2, double *m, double *u);
static void     UserToMotor(tableRecord *ptbl, double *u, double *m);
static void		MakeRotationMatrix(tableRecord *ptbl, double *u);
static void     LabToLocal(tableRecord *ptbl, double *lab, double *local);
static void     LocalToLab(tableRecord *ptbl, double *local, double *lab);
static void		RotY(double *in_, double *out, double a);
static void     ZeroTable(tableRecord *ptbl);
static void		CalcLocalUserLimits(tableRecord *ptbl);
static void		UserLimits_LocalToLab(tableRecord *ptbl);

static void		InitTrajectory(struct trajectory *traj, int n);
static void		SortTrajectory(struct trajectory *traj, int n);
static int		polint(double *xa, double *ya, int n, double x, double *y,
					double *dy);
static int		FindLimit(tableRecord *ptbl, struct trajectory *traj, int n,
					double *limit);
static void     CalcUserLimits(tableRecord *ptbl);
static void     checkLinks(tableRecord *ptbl);

#ifdef NODEBUG
#define Debug0(l,FMT,V) ;
#define Debug(l,FMT,V) ;
#else
#define Debug0(l,FMT) {  if(l <= tableRecordDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= tableRecordDebug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif
volatile int    tableRecordDebug = 0;

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long     init_record(tableRecord *ptbl, int pass);
static long     process();
static long     special();
#define get_value NULL
static long     cvt_dbaddr();
#define get_array_info NULL
#define put_array_info NULL
static long		get_units(struct dbAddr * paddr, char *units);
static long     get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
static long     get_graphic_double(struct dbAddr * paddr,
					struct dbr_grDouble * pcd);
static long     get_control_double(struct dbAddr * paddr,
					struct dbr_ctrlDouble * pcd);
#define get_alarm_double NULL

struct rset     tableRSET = {
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};
epicsExportAddress(rset, tableRSET);

struct linkStatus {
	short can_RW_drive;				/* link .m0xl ... */
	short can_read_position;		/* inlink .e0xi ... */
	short can_read_limits;			/* link .h0xl ... */
	short can_RW_speed;				/* link .v0xl ... */
};

struct saveTable {
	double ax,ay,az,x,y,z;
	double axrb,ayrb,azrb,xrb,yrb,zrb;
	double hlax,hlay,hlaz,hlx,hly,hlz;
	double llax,llay,llaz,llx,lly,llz;
	double m0x,m0y,m1y,m2x,m2y,m2z;
	double e0x,e0y,e1y,e2x,e2y,e2z;
	double eax,eay,eaz,ex,ey,ez;
 	double ax0,ay0,az0,x0,y0,z0;
	double axl,ayl,azl,xl,yl,zl;
	short lvio;
};

struct private {
	struct linkStatus lnkStat[6];
	struct saveTable savTbl;
};


/* Save record values, so we know which must be db_post'ed. */
static void
SaveRecordValues(tableRecord *ptbl)
{
    struct private *p = (struct private *)ptbl->dpvt;
    struct saveTable *savTbl = (struct saveTable *)&(p->savTbl);
	int i;
	double *ax = &ptbl->ax;
	double *axrb = &ptbl->axrb;
	double *hlax = &ptbl->hlax;
	double *llax = &ptbl->llax;
	double *m0x = &ptbl->m0x;
	double *e0x = &ptbl->e0x;
	double *eax = &ptbl->eax;
	double *ax0 = &ptbl->ax0;
	double *axl = &ptbl->axl;
	double *Sax = &savTbl->ax;
	double *Saxrb = &savTbl->axrb;
	double *Shlax = &savTbl->hlax;
	double *Sllax = &savTbl->llax;
	double *Sm0x = &savTbl->m0x;
	double *Se0x = &savTbl->e0x;
	double *Seax = &savTbl->eax;
	double *Sax0 = &savTbl->ax0;
	double *Saxl = &savTbl->axl;

	for (i=0; i<6; i++) {
		*Sax++   = *ax++;
		*Saxrb++ = *axrb++;
		*Shlax++ = *hlax++;
		*Sllax++ = *llax++;
		*Sm0x++  = *m0x++;
		*Se0x++  = *e0x++;
		*Seax++  = *eax++;
		*Sax0++  = *ax0++;
		*Saxl++  = *axl++;
	}
	savTbl->lvio = ptbl->lvio;
}


static long 
init_record(tableRecord *ptbl, int pass)
{
	short  i;
	double *ax = &ptbl->ax;
	double *axrb = &ptbl->axrb;
	double *m0x = &ptbl->m0x;
	double *ax0 = &ptbl->ax0;
	double *axl = &ptbl->axl;
	double *r0x = &ptbl->r0x;

	Debug(5, "init_record: pass = %d\n", pass);

	if (pass == 0) {
		D2R = atan(1.0)/45.0;
		/*
		 * Allocate link-status structure and hang on dpvt.  We can get
		 * away with this because there's no possibility of device support.
		 */
		ptbl->dpvt = (void *)calloc(1, sizeof(struct private));
	}

	/* Save current values of selected fields. */
	SaveRecordValues(ptbl);

	if (pass == 0) {
		/* initialize version field */
		ptbl->vers = VERSION;

		/* Gotta have a .val field.  Make its value reproducible. */
		ptbl->val = 0;

		/* allocate contiguous memory for pivot-point vectors */
		ptbl->pp0 = (double *) calloc(18, sizeof(double));
		ptbl->pp1 = &(ptbl->pp0[3]);
		ptbl->pp2 = &(ptbl->pp0[6]);
		ptbl->ppo0 = &(ptbl->pp0[9]);
		ptbl->ppo1 = &(ptbl->pp0[12]);
		ptbl->ppo2 = &(ptbl->pp0[15]);

		/*
		 * Allocate matrices.  Elements must be contiguous so
		 * channel access can regard the 3x3 2D arrays as 9-element 1D
		 * arrays.  In addition, we maintain a set of three pointers to the
		 * rows of each array at the beginning of the allocated space.
		 */
		ptbl->a = (double **) calloc(1, 3*sizeof(double *) + 9*sizeof(double));
		ptbl->b = (double **) calloc(1, 3*sizeof(double *) + 9*sizeof(double));
		ptbl->a[0] = (double *) (ptbl->a + 3);
		ptbl->b[0] = (double *) (ptbl->b + 3);
		for (i = 1; i < 3; i++) {
			ptbl->a[i] = ptbl->a[i-1] + 3;
			ptbl->b[i] = ptbl->b[i-1] + 3;
		}

		/* initialize pivot-point vectors and transform matrices */
		InitGeometry(ptbl);

		/*
		 * Init user and internal motor values to zero.
		 * (Note that offsets may have been autorestored.)
		 */
		for (i = 0; i < 6; i++) {
			ax[i] = 0.0;
			axl[i] = ax0[i];
			m0x[i] = 0.0;
		}

		return (0);
	}

	checkLinks(ptbl);

	/* read motors and set initial user-coordinate values */
	(void) GetReadback(ptbl, &ptbl->r0x);
	MotorToUser(ptbl, &ptbl->r0x, &ptbl->ax);

	/* read encoders and set initial user-coordinate values */
	(void) ReadEncoders(ptbl, &ptbl->e0x);
	MotorToUser(ptbl, &ptbl->e0x, &ptbl->eax);

	/* Propagate readbacks */
	for (i = 0; i < 6; i++) {
		m0x[i] = r0x[i];
		axrb[i] = ax[i];
		axl[i] = ax[i] + ax0[i];
	}

	/* get motor limits and transform into user limits */
	(void) GetMotorLimits(ptbl);
	/* We should calculate user limits only if we think they've changed */
	CalcUserLimits(ptbl);

	monitor(ptbl);

	return (0);
}


static long 
process(tableRecord *ptbl)
{
	int               i;
	unsigned short	  motor_move_mask;
	long              err;
	double            move_max, velo;
	double            sm[6], sv0x[6];
    struct private    *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;
	double *ax = &ptbl->ax;		/* user-coordinate drive values (include offsets) */
	double *axrb = &ptbl->axrb;	/* user-coordinate readback values (include offsets) */
	double *m0x = &ptbl->m0x;	/* motor drive values */
	double *v0x = &ptbl->v0x;	/* motor speeds */
	double *ax0 = &ptbl->ax0;	/* user-coordinate offsets (e.g., via SET field) */
	double *axl = &ptbl->axl;	/* previous user-coordinate drive values (ignore offsets) */
	double *r0x = &ptbl->r0x;	/* motor readback values */

	Debug(5, "tableRecord(%s):process:entry\n", ptbl->name);

	ptbl->pact = TRUE;
	ptbl->udf = FALSE;

	checkLinks(ptbl);
	SaveRecordValues(ptbl);
	ptbl->lvio = 0;
	(void) GetMotorLimits(ptbl);

	if (ptbl->zero) {

		Debug0(5, "process: zero\n");
		ZeroTable(ptbl);
		ptbl->zero = 0;
		db_post_events(ptbl, &ptbl->zero, DBE_VALUE);

	} else if (ptbl->read) {

		/* read motors and set user-coord readback vals.  done below. */
		Debug0(5, "process: read\n");
		ptbl->read = 0;
		db_post_events(ptbl, &ptbl->read, DBE_VALUE);

	} else if (ptbl->sync) {

		Debug0(5, "process: sync\n");
		(void) GetReadback(ptbl, &ptbl->r0x);
		MotorToUser(ptbl, &ptbl->r0x, &ptbl->axrb);
		/*
		 * Get readback values (ptbl->axl) in offset user coords
		 * (ptbl->axl - ptbl->ax0).
		 * Set drive (ptbl->ax) and readback (ptbl->axrb) to these values.
		 * Sync motors to readbacks.
		 */
		for (i=0; i<6; i++) {
			m0x[i] = r0x[i];
			ax[i] = axrb[i];
			axl[i] = ax[i] + ax0[i];
		}

		ptbl->sync = 0;
		db_post_events(ptbl, &ptbl->sync, DBE_VALUE);

		if (tableRecordDebug) {
			printf("process: sync done\n");
			PrintValues(ptbl);
		}
	} else if (ptbl->init) {

		Debug0(5, "process: init\n");
		/** read motors and set initial user-coordinate values **/
		for (i=0; i<6; i++) {ax0[i] = 0;}
		(void) GetReadback(ptbl, &ptbl->r0x);
		MotorToUser(ptbl, &ptbl->r0x, &ptbl->axrb);
		/* accept these values for drive and readback fields; zero offsets. */
		for (i=0; i<6; i++) {
			m0x[i] = r0x[i];
			ax[i] = axrb[i];
			axl[i] = ax[i] + ax0[i];
		}
		ptbl->init = 0;
		db_post_events(ptbl, &ptbl->init, DBE_VALUE);

	} else if (ptbl->set) {

		Debug0(5, "process: set\n");
		for (i=0; i<6; i++) {ax0[i] = axl[i] - ax[i];}

	} else {

		Debug0(5, "process: calc&move\n");

		/* save current motor positions */
		for (i=0; i<6; i++) sm[i] = m0x[i];

		/*** Do calculations. ***/
		UserToMotor(ptbl, &ptbl->ax, &ptbl->m0x);

		if (UserLimitViol(ptbl) || MotorLimitViol(ptbl)) {
			/*** moving would cause a limit violation ***/
			ptbl->lvio = 1;
			/* restore user- and motor-coordinate fields */
			for (i=0; i<6; i++) {
				m0x[i] = sm[i];
				ax[i] = axl[i] - ax0[i];
			}
		} else {
			/*** move motors ***/
			err = SaveMotorSpeeds(ptbl, sv0x);
			/* find which motors must move, get max distance and speed */
			motor_move_mask = 0;
			velo = 0;
			move_max = 0;
			for (i=0; i<6; i++) {
				/*
				 * Find largest motion and highest speed. Restrict attention
				 * to motors that are moving an appreciable distance, and for
				 * which we have links to speed fields.
				 */
				if (lnkStat[i].can_RW_speed && (fabs(m0x[i] - sm[i]) > SMALL)) {
					motor_move_mask |= 1<<i;
					move_max = MAX(move_max, fabs(m0x[i] - sm[i]));
					velo = MAX(velo, sv0x[i]);
				}
			}
			if (move_max > SMALL) {
				double speed_ratio = LARGE;
				/* calculate motor speeds so all stop at once */
				for (i=0; i<6; i++) {
					if (lnkStat[i].can_RW_speed) {
						v0x[i] = velo * fabs(m0x[i] - sm[i]) / move_max;
						speed_ratio = MIN(speed_ratio, sv0x[i]/v0x[i]);
					}
				}
				/* don't move any motor faster than its saved speed */
				if (speed_ratio < 1.) {
					for (i=0; i<6; i++) v0x[i] *= speed_ratio;
				}
			}
			else {
				/* use saved motor speeds */
				for (i=0; i<6; i++) {v0x[i] = sv0x[i];}
			}

			/* set new motor speeds and positions */
			err = ProcessOutputLinks(ptbl, motor_move_mask);
			err = RestoreMotorSpeeds(ptbl, sv0x);
			if (!RTN_SUCCESS(err))
				return (err);
		}
	}

	/* save user-coordinate values (ignore offsets) */
	for (i=0; i<6; i++) {axl[i] = ax[i] + ax0[i];}

	/* Read motor limit values and transform to offset user coordinates */
	CalcUserLimits(ptbl);

	/* Read motor drive values and transform to offset user coordinates */
	(void) GetReadback(ptbl, &ptbl->r0x);
	MotorToUser(ptbl, &ptbl->r0x, &ptbl->axrb);

	/* Read encoders and transform to offset user coordinates */
	(void) ReadEncoders(ptbl, &ptbl->e0x);
	MotorToUser(ptbl, &ptbl->e0x, &ptbl->eax);

	recGblGetTimeStamp(ptbl);
	checkAlarms(ptbl);
	monitor(ptbl);
	recGblFwdLink(ptbl);
	ptbl->pact = FALSE;
	Debug0(5, "process: exit at bottom\n");
	return (0);
}


static long
special(struct dbAddr *paddr, int after)
{
	tableRecord *ptbl = (tableRecord *) (paddr->precord);
    int fieldIndex = dbGetFieldIndex(paddr);

	Debug(5, "special: after = %d\n", after);
	if (ptbl->pact) {
		printf("%s(%d): special: Record is active.\n",__FILE__,__LINE__);
		return(0);
	}

	if (after && (fieldIndex >= tableRecordLX) &&
			(fieldIndex <= tableRecordRZ)) {
		/* re-initialize pivot-point vectors */
		InitGeometry(ptbl);
		ptbl->sync = 1;
	} else if (after && (fieldIndex == tableRecordGEOM)) {
		/* re-initialize pivot-point vectors */
		InitGeometry(ptbl);
		ptbl->sync = 1;
	} else if (fieldIndex == tableRecordYANG) {
		/* new local-to-lab and lab-to-local transforms */
		if (!after) {
			LabToLocal(ptbl, &ptbl->ax0, &ptbl->ax0);
		} else {
			LocalToLab(ptbl, &ptbl->ax0, &ptbl->ax0);
			ptbl->sync = 1;
		}
	} else if (after) {
		switch (fieldIndex) {
		case tableRecordSSET:
 	       /* Set to SET mode  */
            ptbl->set = 1;
            db_post_events(ptbl,&ptbl->set,DBE_VALUE);
			break;
		case tableRecordSUSE:
	        /* Set to USE mode  */
            ptbl->set = 0; db_post_events(ptbl,&ptbl->set,DBE_VALUE);
 			break;
		case tableRecordSYNC:
 	      /* Let a put of any value to the SYNC field cause a sync. */
			ptbl->sync = 1; db_post_events(ptbl,&ptbl->sync,DBE_VALUE);
			break;
		case tableRecordINIT:
	        /* Let a put of any value to the INIT field cause an init. */
			ptbl->init = 1; db_post_events(ptbl,&ptbl->init,DBE_VALUE);
			break;
		case tableRecordZERO:
	        /* Let a put of any value to the ZERO field cause a zero. */
			ptbl->zero = 1; db_post_events(ptbl,&ptbl->zero,DBE_VALUE);
			break;
		case tableRecordREAD:
	        /* Let a put of any value to the READ field cause a read. */
			ptbl->read = 1; db_post_events(ptbl,&ptbl->read,DBE_VALUE);
			break;
		default:
			break;
		} /* switch (fieldIndex) */
	}
	return (0);
}


static long 
cvt_dbaddr(struct dbAddr *paddr)
{
	tableRecord *ptbl = (tableRecord *) paddr->precord;
	double      **pfield = paddr->pfield;
	double      **pp0 = &(ptbl->pp0);
    int         fieldIndex = dbGetFieldIndex(paddr);

	Debug(5, "cvt_dbaddr: paddr->pfield = %p\n", (void *)paddr->pfield);
	if (fieldIndex == tableRecordA) {
		paddr->pfield = ptbl->a[0];
		paddr->no_elements = 9;
	} else if (fieldIndex == tableRecordB) {
		paddr->pfield = ptbl->b[0];
		paddr->no_elements = 9;
	} else {
		paddr->pfield = pp0[pfield - pp0];
		paddr->no_elements = 3;
	}
	paddr->field_type = DBF_DOUBLE;
	paddr->field_size = sizeof(double);
	paddr->dbr_field_type = DBR_DOUBLE;
	return (0);
}


static long
get_units(struct dbAddr *paddr, char *units)
{
	tableRecord *ptbl = (tableRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

	switch (fieldIndex) {
	case tableRecordAX:    case tableRecordAY:   case tableRecordAZ:
	case tableRecordAX0:   case tableRecordAY0:  case tableRecordAZ0:
	case tableRecordAXL:   case tableRecordAYL:  case tableRecordAZL:
	case tableRecordAXRB:  case tableRecordAYRB: case tableRecordAZRB:
	case tableRecordEAX:   case tableRecordEAY:  case tableRecordEAZ:
	case tableRecordHLAX:  case tableRecordHLAY: case tableRecordHLAZ:
	case tableRecordLLAX:  case tableRecordLLAY: case tableRecordLLAZ:
	case tableRecordYANG:
		strncpy(units, ptbl->aegu, sizeof(ptbl->aegu));
		break;
	default:
		strncpy(units, ptbl->legu, sizeof(ptbl->legu));
		break;
	}

	return (0);
}


static long 
get_graphic_double(struct dbAddr *paddr, struct dbr_grDouble *pgd)
{
	tableRecord *ptbl = (tableRecord *) paddr->precord;
    int i, fieldIndex = dbGetFieldIndex(paddr);

	i = fieldIndex - tableRecordAX;
	if (i >= 0 && i < 6) {
		pgd->upper_disp_limit = (&ptbl->hlax)[i];
		pgd->lower_disp_limit = (&ptbl->llax)[i];
	} else {
		recGblGetGraphicDouble(paddr, pgd);
	}
	return (0);
}


static long 
get_control_double(struct dbAddr *paddr, struct dbr_ctrlDouble *pcd)
{
	tableRecord *ptbl = (tableRecord *) paddr->precord;
    int i, fieldIndex = dbGetFieldIndex(paddr);

	i = fieldIndex - tableRecordAX;
	if (i >= 0 && i < 6) {
		pcd->upper_ctrl_limit = (&ptbl->hlax)[i];
		pcd->lower_ctrl_limit = (&ptbl->llax)[i];
	} else {
		recGblGetControlDouble(paddr, pcd);
	}

	return (0);
}


static long 
get_precision(struct dbAddr *paddr, long *precision)
{
	tableRecord *ptbl = (tableRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

	*precision = ptbl->prec;
	if (fieldIndex == tableRecordVERS) {
		*precision = 2;
	} else if (fieldIndex >= tableRecordVAL) {
		*precision = ptbl->prec;
	} else {
		recGblGetPrec(paddr, precision);	/* Field is in dbCommon */
	}
	return (0);
}


static void 
checkAlarms(tableRecord *ptbl)
{
	if (ptbl->udf == TRUE) {
		recGblSetSevr(ptbl, UDF_ALARM, INVALID_ALARM);
		return;
	}
	return;
}


static void 
monitor(tableRecord *ptbl)
{
	unsigned short  monitor_mask;
	struct private *p = (struct private *)ptbl->dpvt;
	struct saveTable *savTbl = (struct saveTable *)&(p->savTbl);
	int i;
	double *ax = &ptbl->ax;
	double *axrb = &ptbl->axrb;
	double *hlax = &ptbl->hlax;
	double *llax = &ptbl->llax;
	double *m0x = &ptbl->m0x;
	double *e0x = &ptbl->e0x;
	double *eax = &ptbl->eax;
	double *ax0 = &ptbl->ax0;
	double *axl = &ptbl->axl;
	double *Sax = &savTbl->ax;
	double *Saxrb = &savTbl->axrb;
	double *Shlax = &savTbl->hlax;
	double *Sllax = &savTbl->llax;
	double *Sm0x = &savTbl->m0x;
	double *Se0x = &savTbl->e0x;
	double *Seax = &savTbl->eax;
	double *Sax0 = &savTbl->ax0;
	double *Saxl = &savTbl->axl;

	Debug0(5, "monitor: entry\n");

	monitor_mask = recGblResetAlarms(ptbl);
	monitor_mask |= DBE_VALUE;

	for (i=0; i<6; i++) {
		if (*Sax++ != *ax) db_post_events(ptbl, ax, monitor_mask);
		ax++;
		if (*Saxrb++ != *axrb) db_post_events(ptbl, axrb, monitor_mask);
		axrb++;
		if (*Shlax++ != *hlax) db_post_events(ptbl, hlax, monitor_mask);
		hlax++;
		if (*Sllax++ != *llax) db_post_events(ptbl, llax, monitor_mask);
		llax++;
		if (*Sm0x++ != *m0x) db_post_events(ptbl, m0x, monitor_mask);
		m0x++;
		if (*Se0x++ != *e0x) db_post_events(ptbl, e0x, monitor_mask);
		e0x++;
		if (*Seax++ != *eax) db_post_events(ptbl, eax, monitor_mask);
		eax++;
		if (*Sax0++ != *ax0) db_post_events(ptbl, ax0, monitor_mask);
		ax0++;
		if (*Saxl++ != *axl) db_post_events(ptbl, axl, monitor_mask);
		axl++;
	}

	if (savTbl->lvio != ptbl->lvio)
		db_post_events(ptbl, &ptbl->lvio, monitor_mask);

	Debug0(5, "monitor: exit\n");
	return;
}


static long 
ProcessOutputLinks(tableRecord *ptbl, unsigned short motor_move_mask)
{
	long err;
	struct link *v0xl = &ptbl->v0xl, *m0xl = &ptbl->m0xl;
	double *v0x = &ptbl->v0x, *m0x = &ptbl->m0x;
	short i, motors_moved=0;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	/*** Process output links. ***/
	/* set speeds */
	for (i=0; i<6; i++) {
		if ((motor_move_mask & 1<<i) && lnkStat[i].can_RW_speed) {
			err = dbPutLink(&v0xl[i], DBR_DOUBLE, &v0x[i], 1);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbPutLink (set speed) failed for motor %d\n", i);
				return (err);
			}
		}
	}
	/* move motors */
	for (i=0; i<6; i++) {
		if ((motor_move_mask & 1<<i) && lnkStat[i].can_RW_drive) {
			err = dbPutLink(&m0xl[i], DBR_DOUBLE, &m0x[i], 1);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbPutLink (drive motor) failed for motor %d\n", i);
				return (err);
			}
			motors_moved++;
		}
	}
	/* write to at least one motor, so a .dmov flag gets toggled. */
	/* (Kludge to support the old scan software's completion detector.) */
	for (i=0; i<6 && !motors_moved; i++) {
		if (lnkStat[i].can_RW_drive) {
			err = dbPutLink(&m0xl[i], DBR_DOUBLE, &m0x[i], 1);
			motors_moved++;
		}
	}
	return (0);
}


static long 
SaveMotorSpeeds(tableRecord *ptbl, double *sv0x)
{
	long            err;
	struct link *v0xi = &ptbl->v0xi;
	short i;
	struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	for (i=0; i<6; i++) {
		if (lnkStat[i].can_RW_speed) {
			err = dbGetLink(&v0xi[i], DBR_DOUBLE, &sv0x[i], NULL, NULL);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbGetLink (read speed) failed for motor %d\n", i);
				return (err);
			}
		}
	}
	return(0);
}


static long 
RestoreMotorSpeeds(tableRecord *ptbl, double *sv0x)
{
	long            err;
	struct link *v0xl = &ptbl->v0xl;
	short i;
	struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	for (i=0; i<6; i++) {
		if (lnkStat[i].can_RW_speed) {
			err = dbPutLink(&v0xl[i], DBR_DOUBLE, &sv0x[i], 1);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbPutLink (write speed) failed for motor %d\n", i);
				return (err);
			}
		}
	}
	return(0);
}


static long 
GetMotorLimits(tableRecord *ptbl)
{
	long err;
	struct link *h0xl = &ptbl->h0xl, *l0xl = &ptbl->l0xl;
	double *h0x = &ptbl->h0x, *l0x = &ptbl->l0x;
	short i;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	for (i=0; i<6; i++) {
		if (lnkStat[i].can_read_limits) {
			err = dbGetLink(&h0xl[i], DBR_DOUBLE, &h0x[i], NULL, NULL);
			if (err) {
				Debug(5,"dbGetLink (read hilimit) failed for motor %d\n", i);
				h0x[i] = 0;
			}
			err = dbGetLink(&l0xl[i], DBR_DOUBLE, &l0x[i], NULL, NULL);
			if (err) {
				Debug(5,"dbGetLink (read lolimit) failed for motor %d\n", i);
				l0x[i] = 0;
			}
		}
	}
	return(0);
}


static long 
MotorLimitViol(tableRecord *ptbl)
{
	double *h0x = &ptbl->h0x, *l0x = &ptbl->l0x;
	double *m0x = &ptbl->m0x;
	short i;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	for (i=0; i<6; i++, h0x++, l0x++, m0x++) {
		if ((lnkStat[i].can_read_limits) && (lnkStat[i].can_RW_drive)) {
			if ((*h0x != 0.) || (*l0x != 0.)) {
				if ((*m0x > *h0x) || (*m0x < *l0x)) {
					if (tableRecordDebug >= 1)
						printf("MotorLimitViol: motor[%d]=%f, l=%f, h=%f\n",
							i, *m0x, *l0x, *h0x);
					return(1);
				}
			}
		}
	}
	return (0);
}


static long 
UserLimitViol(tableRecord *ptbl)
{
	short	i;
	double	*hlax = &ptbl->hlax, *llax = &ptbl->llax;
	double	*ax = &ptbl->ax, *ax0 = &ptbl->ax0;
	double	u;

	/* check against user's limits */
	for (i=0; i<6; i++, hlax++, llax++, ax++, ax0++) {
		if ((*hlax != 0.0) || (*llax != 0.0)) {
			u = *ax + *ax0;
			if ((u < *llax) || (u > *hlax)) {
				if (tableRecordDebug >= 1)
					printf("MotorLimitViol: user[%d]=%f, l=%f, h=%f\n",
						i, u, *llax, *hlax);
				return(1);
			}
		}
	}
	return(0);
}


static long 
GetReadback(tableRecord *ptbl, double *r)
{
	long err;
	struct link *r0xi = &ptbl->r0xi;
	short i;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	Debug(5, "tableRecord(%s):GetReadback:entry\n", ptbl->name);
	for (i=0; i<6; i++) {
		r[i] = 0.0;
		if (lnkStat[i].can_RW_drive) {
			err = dbGetLink(&r0xi[i], DBR_DOUBLE, &r[i], NULL, NULL);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbGetLink (read drive) failed for motor %d\n", i);
				return (err);
			}
		}
	}
	return(0);
}


static long 
ReadEncoders(tableRecord *ptbl, double *e)
{
	long err;
	struct link *e0xi = &ptbl->e0xi;
	short i;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	Debug(5, "tableRecord(%s):ReadEncoders:entry\n", ptbl->name);
	for (i=0; i<6; i++) {
		e[i] = 0.;
		if (lnkStat[i].can_read_position) {
			err = dbGetLink(&e0xi[i], DBR_DOUBLE, &e[i], NULL, NULL);
			if (!RTN_SUCCESS(err)) {
				Debug(5,"dbGetLink (position RBV) failed for motor %d\n", i);
				return (err);
			}
		}
	}
	return(0);
}


static void 
InitGeometry(tableRecord *ptbl)
{
	double **bb = ptbl->b;
	double *pp0=ptbl->pp0, *pp1=ptbl->pp1, *pp2=ptbl->pp2;
	double *ppo0=ptbl->ppo0, *ppo1=ptbl->ppo1, *ppo2=ptbl->ppo2;
	double sx = ptbl->sx, sy = ptbl->sy, sz = ptbl->sz;
	double lx = ptbl->lx, lz = ptbl->lz;
	double fx, fy, fz;
	double a, b, c, d, e, f, g, h, i, det;

	Debug(5, "tableRecord(%s):InitGeometry:entry\n", ptbl->name);
	fx = ptbl->rx + sx;
	fy = ptbl->ry + sy;
	fz = ptbl->rz + sz;
	/* Vectors from the 'fixed point' to each of the table's pivot points */
	switch (ptbl->geom) {
	case tableGEOM_GEOCARS:
		pp0[X] = ppo0[X] = -fx;
		pp0[Y] = ppo0[Y] = -fy;
		pp0[Z] = ppo0[Z] = lz/2 - fz;
		pp1[X] = ppo1[X] = lx - fx;
		pp1[Y] = ppo1[Y] = -fy;
		pp1[Z] = ppo1[Z] = lz - fz;
		pp2[X] = ppo2[X] = lx - fx;
		pp2[Y] = ppo2[Y] = -fy;
		pp2[Z] = ppo2[Z] = -fz;
		break;

	case tableGEOM_NEWPORT:
		pp0[X] = ppo0[X] = lx - fx;
		pp0[Y] = ppo0[Y] = -fy;
		pp0[Z] = ppo0[Z] = -fz;
		pp1[X] = ppo1[X] = -fx;
		pp1[Y] = ppo1[Y] = -fy;
		pp1[Z] = ppo1[Z] = lz/2 - fz;
		pp2[X] = ppo2[X] = lx - fx;
		pp2[Y] = ppo2[Y] = -fy;
		pp2[Z] = ppo2[Z] = lz - fz;
		break;

	case tableGEOM_PNC:
		pp1[X] = ppo1[X] = lx - fx;
		pp1[Y] = ppo1[Y] = -fy;
		pp1[Z] = ppo1[Z] = -fz;
		pp0[X] = ppo0[X] = -fx;
		pp0[Y] = ppo0[Y] = -fy;
		pp0[Z] = ppo0[Z] = -fz;
		pp2[X] = ppo2[X] = lx/2 - fx;
		pp2[Y] = ppo2[Y] = -fy;
		pp2[Z] = ppo2[Z] = lz - fz;
		break;

	case tableGEOM_SRI:
	default:
		pp0[X] = ppo0[X] = lx - fx;
		pp0[Y] = ppo0[Y] = -fy;
		pp0[Z] = ppo0[Z] = -fz;
		pp1[X] = ppo1[X] = -fx;
		pp1[Y] = ppo1[Y] = -fy;
		pp1[Z] = ppo1[Z] = -fz;
		pp2[X] = ppo2[X] = lx/2 - fx;
		pp2[Y] = ppo2[Y] = -fy;
		pp2[Z] = ppo2[Z] = lz - fz;
		break;
	}

	/*
	 * Get three vectors that are independent of the user coordinates (x,y,z),
	 * in the space spanned by the pivot-point vectors, and make the matrix
	 *		(a b c)
	 *		(d e f)
	 *		(g h i)
	 * out of them.  The vectors are:
	 * ppo1 - ppo0
	 * ppo2 - ppo1
	 * (ppo1-ppo0) X (ppo2-ppo1)  (X means cross product)
	 */
	a = ppo1[X] - ppo0[X]; b = ppo1[Y] - ppo0[Y]; c = ppo1[Z] - ppo0[Z];
	d = ppo2[X] - ppo1[X]; e = ppo2[Y] - ppo1[Y]; f = ppo2[Z] - ppo1[Z];
	g = b*f - c*e;         h = c*d - a*f;         i = a*e - b*d;

	/*
	 * Get inverse of the matrix.  Later, we'll apply this matrix to
	 * rotated/translated pivot-point vectors to get the merely rotated
	 * unit vectors.  This will allow us to determine the rotation angles
	 * in the SRI, GEOCARS, and PNC geometries.
	 */
	det = a*(e*i-h*f) + b*(f*g-i*d) + c*(d*h-g*e);
	bb[0][0] = (e*i - f*h) / det;
	bb[0][1] = (c*h - b*i) / det;
	bb[0][2] = (b*f - c*e) / det;
	bb[1][0] = (f*g - d*i) / det;
	bb[1][1] = (a*i - c*g) / det;
	bb[1][2] = (c*d - a*f) / det;
	bb[2][0] = (d*h - e*g) / det;
	bb[2][1] = (b*g - a*h) / det;
	bb[2][2] = (a*e - b*d) / det;
}

/*
 * If a pivot-point vector component is driven by a motor, it's easy to 
 * calculate it's value.  Can't use this routine for the NEWPORT geometry.
 */
static void 
NaiveMotorToPivotPointVector(tableRecord *ptbl, double *m, double *q0,
	double *q1, double *q2)
{
	double *p0 = ptbl->ppo0, *p1 = ptbl->ppo1, *p2 = ptbl->ppo2;
	double norm[3], **a = ptbl->a;

	Debug(5, "tableRecord(%s):NaiveMotorToPivotPointVector:entry\n", ptbl->name);
	switch (ptbl->geom) {
	case tableGEOM_SRI:
	case tableGEOM_PNC:
	case tableGEOM_GEOCARS:
	default:
		q0[X] = p0[X] + m[M0X];
		q0[Y] = p0[Y] + m[M0Y];
	/*	q0[Z] = ? */

	/*	q1[X] = ? */
		q1[Y] = p1[Y] + m[M1Y];
	/*	q1[Z] = ? */

		q2[X] = p2[X] + m[M2X];
		q2[Y] = p2[Y] + m[M2Y];
		q2[Z] = p2[Z] + m[M2Z];
		break;

	case tableGEOM_NEWPORT:
		/* Note that for the Newport geometry, the following calculations
		 * require that the rotation matrix be known.  Therefore, this
		 * routine may not be used by MotorToUser(), as it is for the other
		 * geometries.
		 * Also note that for the Newoport we are not calculating the locations
		 * of the actual pivot points.  Instead, we're pretending the pivot points
		 * move vertically -- as they do in the other geometries -- to maintain
		 * a fixed distance from the point about which the table rotates.
		 */
		MakeRotationMatrix(ptbl, &ptbl->ax);
		norm[X] = a[X][Y];	/* unit vector normal to table */
		norm[Y] = a[Y][Y];
		norm[Z] = a[Z][Y];
		q0[X] = p0[X] + m[M0X] + norm[X] * m[M0Y];
		q0[Y] = p0[Y] + norm[Y] * m[M0Y];
	/*	q0[Z] = have to calculate this */

	/*	q1[X] = have to calculate this */
		q1[Y] = p1[Y] + norm[Y] * m[M1Y];
	/*	q1[Z] = have to calculate this */

		q2[X] = p2[X] + m[M2X] + norm[X] * m[M2Y];
		q2[Y] = p2[Y] + norm[Y] * m[M2Y];
		q2[Z] = p2[Z] + m[M2Z] + norm[Z] * m[M2Y];
		break;
	}
}

/*
 * Calculate pivot-point vectors from motors.
 * Can't use this routine for the NEWPORT geometry.
 */
static void 
MotorToPivotPointVector(tableRecord *ptbl, double *m, double *q0, double *q1,
	double *q2)
{
	short i;
	double *p0 = ptbl->ppo0, *p1 = ptbl->ppo1, *p2 = ptbl->ppo2;
	double q1z_m;
	double s, t, p10p20, p10p10, alpha;
	double dx, dy, dz, d0x, d0y, d0z;

	Debug(5, "tableRecord(%s):MotorToPivotPointVector:entry\n", ptbl->name);
	/*
	 * Get what information about the rotated/translated pivot-point vectors we can
	 * get straight from the motors.  We'll have to calculate the rest.
	 */
	NaiveMotorToPivotPointVector(ptbl, m, q0, q1, q2);

	/* get q0[Z] from |q0-q2| == |p0-p2| */
	d0x = p2[X]-p0[X]; d0y = p2[Y]-p0[Y]; d0z = p2[Z]-p0[Z];
	 dx = q2[X]-q0[X];  dy = q2[Y]-q0[Y];  dz = q2[Z]-q0[Z];
	switch (ptbl->geom) {
	case tableGEOM_GEOCARS:
		/* choose the root that yields (q2[Z] < q0[Z]) */
		q0[Z] = q2[Z] + sqrt(d0x*d0x + d0y*d0y + d0z*d0z - (dx*dx + dy*dy));
		break;
	case tableGEOM_SRI:
	case tableGEOM_PNC:
	default:
		/* choose the root that yields (q2[Z] > q0[Z]) */
		q0[Z] = q2[Z] - sqrt(d0x*d0x + d0y*d0y + d0z*d0z - (dx*dx + dy*dy));
		break;
	}

	/*
	 * get q1[X] and q1[Z] from (q1-q0).(q2-q0) == (p1-p0).(p2-p0)
	 * and |q1-q0| == |p1-p0|
	 */
 
	/* p10p20 = (p1-p0).(p2-p0) */
	for (i=X, p10p20=0; i<=Z; i++) {p10p20 += (p1[i]-p0[i])*(p2[i]-p0[i]);}

	/*
	 * Define s, t for clarity:
	 * q1[X] = (-p10p20 + q0[X]*(q0[X] - q2[X]) +
	 *			(q0[Y] - q1[Y])*(q0[Y] - q2[Y]) +
	 *			(q0[Z] - q1[Z])*(q0[Z] - q2[Z])) / (q0[X] - q2[X])
	 *       = s*q1[Z] + t
	 */

	s = -(q0[Z] - q2[Z])/(q0[X] - q2[X]);
	t = (-p10p20 + q0[X]*(q0[X] - q2[X]) + (q0[Y] - q1[Y])*(q0[Y] - q2[Y]) +
		q0[Z]*(q0[Z] - q2[Z])) / (q0[X] - q2[X]);

	/* p10p10 = (p1-p0).(p1-p0) */
	for (i=X, p10p10=0; i<=Z; i++) {p10p10 += (p1[i]-p0[i])*(p1[i]-p0[i]);}
	alpha = sqrt((2*s*t - 2*s*q0[X] - 2*q0[Z]) * (2*s*t - 2*s*q0[X] - 2*q0[Z]) -
		4*(1 + s*s)*(t*t - p10p10 - 2*t*q0[X] + q0[X]*q0[X] + q0[Y]*q0[Y] +
		q0[Z]*q0[Z] - 2*q0[Y]*q1[Y] + q1[Y]*q1[Y]));
	/* two roots */
	q1[Z] = (-2*s*t + 2*s*q0[X] + 2*q0[Z] + alpha) / (2*(1 + s*s));
	q1z_m = (-2*s*t + 2*s*q0[X] + 2*q0[Z] - alpha) / (2*(1 + s*s));
	/* take the root that represents the smaller motion */
	if (fabs(q1[Z] - p1[Z]) > fabs(q1z_m - p1[Z])) q1[Z] = q1z_m;
	q1[X] = s*q1[Z] + t;
	if (tableRecordDebug>29) {
		printf("MotorToPivotPointVector: q1z_p=%f, q1z_m=%f, p1[Z]=%f\n",
			q1[Z],q1z_m,p1[Z]);
		printf("MotorToPivotPointVector: q0=(%f,%f,%f)\n", q0[X],q0[Y],q0[Z]);
		printf("MotorToPivotPointVector: q1=(%f,%f,%f)\n", q1[X],q1[Y],q1[Z]);
		printf("MotorToPivotPointVector: q2=(%f,%f,%f)\n", q2[X],q2[Y],q2[Z]);
	}
}


/*
 * Forget about translations (x,y,z) and calculate just the angles
 * from the rotated pivot-point vectors.  We need this only for the SRI,
 * GEOCARS, and PNC geometries.
 */
static void 
PivotPointVectorToLocalUserAngles(tableRecord *ptbl, double *q0, double *q1,
	double *q2, double *u)
{
	double **bb = ptbl->b;
	double a, b, c, d, e, f, g, h, i;
	double /*ip[3],*/ jp[3], kp[3];

	Debug(5, "tableRecord(%s):PivotPointVectorToLocalUserAngles:entry\n", ptbl->name);
	/*
	 * Get three vectors independent of the user coordinates (x,y,z), in the
	 * space spanned by the transformed pivot-point vectors, and make the matrix
	 *		(a b c)
	 *		(d e f)
	 *		(g h i)
	 * out of them.  The vectors are:
	 * q1 - q0
	 * q2 - q1
	 * (q1-q0) X (q2-q1)  (X means cross product)
	 */
	a = q1[X] - q0[X]; b = q1[Y] - q0[Y]; c = q1[Z] - q0[Z];
	d = q2[X] - q1[X]; e = q2[Y] - q1[Y]; f = q2[Z] - q1[Z];
	g = b*f - c*e;     h = c*d - a*f;     i = a*e - b*d;

	/*
	 * Get unit vectors in rotated frame from the matrix (a...i), by applying
	 * the inverse of a matrix we calculated (in InitGeometry()) from the
	 * pristine pivot-point vectors.
	 */
	/* rotated x axis */
/*	ip[X] = bb[0][0]*a + bb[0][1]*d + bb[0][2]*g; don't need */
/*	ip[Y] = bb[0][0]*b + bb[0][1]*e + bb[0][2]*h; don't need */
/*	ip[Z] = bb[0][0]*c + bb[0][1]*f + bb[0][2]*i; don't need */
	/* rotated y axis */
	jp[X] = bb[1][0]*a + bb[1][1]*d + bb[1][2]*g;
/*	jp[Y] = bb[1][0]*b + bb[1][1]*e + bb[1][2]*h; don't need */
/*	jp[Z] = bb[1][0]*c + bb[1][1]*f + bb[1][2]*i; don't need */
	/* rotated z axis */
	kp[X] = bb[2][0]*a + bb[2][1]*d + bb[2][2]*g;
	kp[Y] = bb[2][0]*b + bb[2][1]*e + bb[2][2]*h;
/*	kp[Z] = bb[2][0]*c + bb[2][1]*f + bb[2][2]*i; don't need */

	/*
	 * Rotation-matrix elements are the dot products of rotated and unrotated
	 * unit vectors.  We only need three of them to get the angles, if we
	 * restrict angles to the range of the arcsine function ([-90 ... 90]
	 * degrees).  Convert angles to degrees.
	 *
	 * Note that we're assuming the functional form of the rotation matrix
	 * elements (a[0][2], a[1][2], and a[0][1]) here.  If MakeRotationMatrix()
	 * changes, the next three lines may also have to change.
	 */
	u[AY_6] = asin(-kp[X]); /* a[X][Z] */
	u[AX_6] = asin(kp[Y]/cos(u[AY_6])) / D2R; /* a[Y][Z] */
	u[AZ_6] = asin(jp[X]/cos(u[AY_6])) / D2R; /* a[X][Y] */
	u[AY_6] /= D2R;	/* user angles are in degrees, not radians */
}


/*
 * Calculate angles (and whatever else we get as a byproduct)
 * from the motor positions.  We only need this for the Newport geometry.
 */
static void 
MotorToLocalUserAngles(tableRecord *ptbl, double *m, double *u)
{

	double *p0 = ptbl->ppo0, *p1 = ptbl->ppo1, *p2 = ptbl->ppo2;
	double p0x = p0[X], p0y = p0[Y], p0z = p0[Z];
	double p1x = p1[X], p1y = p1[Y], p1z = p1[Z];
	double p2x = p2[X], p2y = p2[Y], p2z = p2[Z];
	double p10x = p1[X]-p0[X], p10y = p1[Y]-p0[Y], p10z = p1[Z]-p0[Z];
	double p01x = p0[X]-p1[X], p01y = p0[Y]-p1[Y], p01z = p0[Z]-p1[Z];
	double p20x = p2[X]-p0[X], p20y = p2[Y]-p0[Y], p20z = p2[Z]-p0[Z];
	double p02x = p0[X]-p2[X], p02y = p0[Y]-p2[Y], p02z = p0[Z]-p2[Z];
	double p02x_2 = p02x * p02x, p02y_2 = p02y * p02y, p02z_2 = p02z * p02z;
	double L0 = m[M0Y], L1 = m[M1Y], L2 = m[M2Y];
	double L10 = m[M1Y]-m[M0Y], L20 = m[M2Y]-m[M0Y], L02 = m[M0Y]-m[M2Y];
	double L02_2 = L02 * L02;
	double Ryx, Ryy, Ryz, Ryx_2, Ryy_2, Ryz_2;
	double Rxx, Rxy, Rxz;
	double n0x = m[M0X], n2x = m[M2X], n02x = n0x - n2x, n02x_2 = n02x * n02x;
	double a, b, c, tmp, tmp1;
	double npx, npy, npz;
	
	Debug(5, "tableRecord(%s):MotorToLocalUserAngles:entry\n", ptbl->name);
	/*
	 * From "upside-down" analysis (regard table top as horizontal and calculate how much
	 * the floor is tilted) to find y component of table's normal vector.  (It's easier this
	 * way because the legs are normal to the table top.)
	 *
	 * ->     ->   ^        ->   ^          ->   ^        ->   ^
	 * np = ((p1 - j L1) - (p0 - j L0)) X ((p2 - j L2) - (p0 - j L0))
	 *
	 *       ->    ^         ->    ^        ->    ->        ^   ->        ^   ->
	 *    = (p10 - j L10) X (p20 - j L20) = p10 X p20 - L10 j X p20 + L20 j X p10
	 *
	 * npy = p10z p20x - p20z p10x - L10 
	 */
	npx = p10y * p20z - p10z * p20y - p20z * L10 + p10z * L20;
	npy = p10z * p20x - p10x * p20z;
	npz = p10x * p20y - p10y * p20x + p20x * L10 - p10x * L20;

	Ryy = npy / sqrt(npx*npx + npy*npy + npz*npz);
	Ryy_2 = Ryy * Ryy;

	/*
	 * Now we can uncouple the y equations from the rest of this awful mess,
	 * and get y as an unavoidable byproduct of solving for the Ry* matrix elements.
	 * Mathematica input:
	 * Solve[{Ryx p0x + (Ryy-1) p0y + Ryz p0z + y - Ryy L0 == 0,
	 *        Ryx p1x + (Ryy-1) p1y + Ryz p1z + y - Ryy L1 == 0,
	 *        Ryx p2x + (Ryy-1) p2y + Ryz p2z + y - Ryy L2 == 0},
	 *        {Ryx,Ryz,y}]//FullSimplify
	 */
 
	Ryx = (p1z * p2y - p1y * p2z + p0y * (p1z - p2z) * (-1 + Ryy) - 
	 (p1z * (L0 - L2 + p2y) - (L0 - L1 + p1y) * p2z) * Ryy + 
	 p0z * (p1y - p2y + (L1 - L2 - p1y + p2y) * Ryy)) / 
	 (p0z * (p1x - p2x) + p1z * p2x - p1x * p2z + p0x * (-p1z + p2z));
	Ryx_2 = Ryx * Ryx;

	Ryz = (p1y * p2x - p1x * p2y - p0y * (p1x - p2x) * (-1 + Ryy) + 
	 (L0 * p1x - L2 * p1x - L0 * p2x + L1 * p2x - p1y * p2x + p1x * p2y) * Ryy + 
	 p0x * (-p1y + p2y + (-L1 + L2 + p1y - p2y) * Ryy)) / 
	 (p0z * (p1x - p2x) + p1z * p2x - p1x * p2z + p0x * (-p1z + p2z));
 	Ryz_2 = Ryz * Ryz;

	u[Y_6] = (-(p0x * p1z * p2y) + p0x * p1y * p2z - 
	 p0y * (p1z * p2x - p1x * p2z) * (-1 + Ryy) - L2 * p0x * p1z * Ryy + 
	 L0 * p1z * p2x * Ryy + p0x * p1z * p2y * Ryy + L1 * p0x * p2z * Ryy - 
	 L0 * p1x * p2z * Ryy - p0x * p1y * p2z * Ryy + 
	 p0z * (p1y * p2x * (-1 + Ryy) - (L1 * p2x + p1x * p2y) * Ryy + 
	 p1x * (p2y + L2 * Ryy))) / 
	 (p0z * (p1x - p2x) + p1z * p2x - p1x * p2z + p0x * (-p1z + p2z));

	if (tableRecordDebug >= 5) printf("Ryx=%f, Ryy=%f, Ryz=%f, y=%f\n", Ryx, Ryy, Ryz, u[Y_6]);

	/* Thus far, we've been pretending rotation-matrix elements are independent, because Mathematica
	 * chokes on the real problem.  Now the x equations, together with some rotation-matrix identities,
	 * can be solved for enough of the remaining matrix elements to nail down all three user angles.
	 * Mathematica input:
	 * Solve[Rxx^2 + Rxy^2 + Rxz^2 == 1, 
	 *    Rxx == (n02x + p02x + (L02 - p02y) Rxy - p02z Rxz)/p02x, 
	 *    Rxx Ryx + Rxy Ryy + Rxz Ryz == 0, Rxx, Rxy, Rxz] // FullSimplify
	 */

	a = (n02x + p02x) * (L02 * Ryx * Ryy - p02y * Ryx * Ryy - p02z * Ryx * Ryz + p02x * (Ryy_2 + Ryz_2));
	b = -p02x_2 * Ryx_2 + p02y_2 * Ryx_2 + p02z_2 * Ryx_2 - 2 * p02x * p02y * Ryx * Ryy + p02z_2 * Ryy_2 -
		2 * p02z * (p02x * Ryx + p02y * Ryy) * Ryz + p02y_2 * Ryz_2 + L02_2 * (Ryx_2 + Ryz_2) -
		n02x_2 * (Ryx_2 + Ryy_2 + Ryz_2) - 2 * n02x * p02x * (Ryx_2 + Ryy_2 + Ryz_2) -
		2 * L02 * (-Ryy * (p02x * Ryx + p02z * Ryz) + p02y * (Ryx_2 + Ryz_2));
	c = L02_2 * Ryx_2 - 2 * L02 * p02y * Ryx_2 + p02y_2 * Ryx_2 + p02z_2 * Ryx_2 +
		2 * L02 * p02x * Ryx * Ryy - 2 * p02x * p02y * Ryx * Ryy + p02x_2 * Ryy_2 + p02z_2 * Ryy_2 -
		2 * p02z * (p02x * Ryx + (-L02 + p02y) * Ryy) * Ryz + (p02x_2 + (L02 - p02y)*(L02 - p02y)) * Ryz_2;
	Rxx = (a - (p02z * Ryy + (L02 - p02y) * Ryz) * sqrt(b)) / c;
	tmp = (a + (p02z * Ryy + (L02 - p02y) * Ryz) * sqrt(b)) / c;
	if (tableRecordDebug >= 6) printf("MotorToLocalUserAngles: Rxx = %f or %f\n", Rxx, tmp);
	/* angles are small, so we should be close to unit matrix, for which Rxx = 1; */
	if (fabs(tmp-1) < fabs(Rxx-1)) Rxx = tmp;

	a = (n02x + p02x) * (p02z * (Ryx_2 + Ryy_2) - (p02x * Ryx + (-L02 + p02y) * Ryy) * Ryz);
	Rxz = (a + (L02 * Ryx - p02y * Ryx + p02x * Ryy) * sqrt(b)) / c;
	tmp = (a - (L02 * Ryx - p02y * Ryx + p02x * Ryy) * sqrt(b)) / c;
	if (tableRecordDebug >= 6) printf("MotorToLocalUserAngles: Rxz = %f or %f\n", Rxz, tmp);
	/* Plus sign seems always to be the right choice (in 500,000 random simulations).  I'm just
	 * too lazy to figure out why.
	 */

	a = -(n02x + p02x) * (Ryx * (L02 * Ryx - p02y * Ryx + p02x * Ryy) +
		p02z * Ryy * Ryz + (L02 - p02y) * Ryz_2);
	Rxy = (a + (p02z * Ryx - p02x * Ryz) * sqrt(b)) / c;
	tmp = (a - (p02z * Ryx - p02x * Ryz) * sqrt(b)) / c;
	/* Use rotation-matrix identity to choose the right root. */
	tmp1 = sqrt(1 - (Rxx*Rxx + Rxz*Rxz));
	if (tableRecordDebug >= 6) printf("MotorToLocalUserAngles: Rxy = %f or %f (abs val should be near %f)\n",
		Rxy, tmp, tmp1);
	if (fabs(fabs(tmp)-tmp1) < fabs(fabs(Rxy)-tmp1)) Rxy = tmp;

	if (tableRecordDebug >= 5) printf("Rxx=%f, Rxy=%f, Rxz=%f\n", Rxx, Rxy, Rxz);

	/* Now we have the right matrix elements to pick out user angles easily. */
	u[AY_6] = asin(-Rxz);
	u[AX_6] = asin(Ryz/cos(u[AY_6])) / D2R;
	u[AZ_6] = asin(Rxy/cos(u[AY_6])) / D2R;
	u[AY_6] /= D2R;
}


/*
 * From motor positions, calculate lab coordinate system user values, including user offsets.
 */
static void 
MotorToUser(tableRecord *ptbl, double *m, double *u)
{
	short i, j, k;
	double *ax0 = &ptbl->ax0;
	double *ppo0 = ptbl->ppo0, *ppo1 = ptbl->ppo1, *ppo2 = ptbl->ppo2;
	double q0[3], q1[3], q2[3];
	double **aa = ptbl->a;
	double pp0[3], pp1[3], pp2[3], m_try[6];
    struct private    *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	Debug(5, "tableRecord(%s):MotorToUser:entry\n", ptbl->name);
	switch (ptbl->geom) {
	case tableGEOM_SRI:
	case tableGEOM_GEOCARS:
	case tableGEOM_PNC:
	default:
		MotorToPivotPointVector(ptbl, m, q0, q1, q2);
		PivotPointVectorToLocalUserAngles(ptbl, q0, q1, q2, u);
		break;

	case tableGEOM_NEWPORT:
		MotorToLocalUserAngles(ptbl, m, u);
		break;
	}

	/*
	 * Now we have enough information to recover the rotation matrix, so we can compare
	 * rotated pivot-point vectors with motor positions, and get the (x, y, z) translations.
	 */
	if (tableRecordDebug >= 5) printf("MotorToUser: get translations\n");
	MakeRotationMatrix(ptbl, u);
	/* Rotate pivot point 2, because it has all three translations specified. */
	for (j=X; j<=Z; j++) {
		pp0[j] = pp1[j] = pp2[j] = 0;
		for (k=X; k<=Z; k++) {
			pp0[j] += ppo0[k] * aa[j][k];
			pp1[j] += ppo1[k] * aa[j][k];
			pp2[j] += ppo2[k] * aa[j][k];
		}
	}
	if (ptbl->geom == tableGEOM_NEWPORT) {
		/* We already have u[Y_6].  Use it. */
		pp0[Y] += u[Y_6]; pp1[Y] += u[Y_6]; pp2[Y] += u[Y_6];
	}
	/* Compare rotated, y-translated pivot point with motors to get translations. */
	PivotPointVectorToMotor(ptbl, pp0, pp1, pp2, m_try, u);
	if (tableRecordDebug >= 5) printf("m[] = 0x:%f, 0y:%f, 1y:%f, 2x:%f, 2y:%f, 2z:%f\n",
		m_try[M0X], m_try[M0Y], m_try[M1Y], m_try[M2X], m_try[M2Y], m_try[M2Z]);
	/* Check special case of 5-motor table */
	if ((ptbl->geom == tableGEOM_NEWPORT) && !(lnkStat[M2X].can_RW_drive)) {
		; /* PivotPointVectorToMotor() calculated u[X_6] in local coordinate system */
	} else {
		u[X_6] = m[M2X] - m_try[M2X];
	}
	/* For Newport, we got u[Y_6] for free in MotorToLocalUserAngles(). */
	if (ptbl->geom != tableGEOM_NEWPORT) u[Y_6] = m[M2Y] - m_try[M2Y];
	if (!(lnkStat[M2Z].can_RW_drive)) {
		; /* PivotPointVectorToMotor() calculated u[Z_6] in local coordinate system */
	} else {
		u[Z_6] = m[M2Z] - m_try[M2Z];
	}

	LocalToLab(ptbl, u, u);
	/* subtract off user offsets */
	for (i=0; i<6; i++) {u[i] -= ax0[i];}

	if (tableRecordDebug > 25) {
	    printf("MotorToUser: m[]="); for (j=0;j<6;j++) printf("%f, ",m[j]);
	    printf("\nMotorToUser: u[]="); for (j=0;j<6;j++) printf("%f, ",u[j]);
		printf("\n");
	}
}


/*
 * Go from (local-table) user coordinates to rotated, translated pivot-point
 * vectors.
 */
static void 
LocalUserToPivotPointVector(tableRecord *ptbl, double *u, double *pp0,
	double *pp1, double *pp2)
{
	short i, j, k;
	double *ppo0 = ptbl->ppo0, *ppo1 = ptbl->ppo1, *ppo2 = ptbl->ppo2;
	double **a = ptbl->a, m2x, m2y;
    struct private    *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	Debug(5, "tableRecord(%s):LocalUserToPivotPointVector:entry\n", ptbl->name);
	MakeRotationMatrix(ptbl, u);

	/* transform */
	for (i=X, k=X_6; i<=Z; i++, k++) {
		pp0[i] = 0.; pp1[i] = 0.; pp2[i] = 0.;
		/* rotate */
		for (j=X; j<=Z; j++) {
			pp0[i] += ppo0[j] * a[i][j];
			pp1[i] += ppo1[j] * a[i][j];
			pp2[i] += ppo2[j] * a[i][j];
		}
		/* translate */
		pp0[i] += u[k]; pp1[i] += u[k]; pp2[i] += u[k];
	}
}


/*
 * Calculate motor positions from rotated, translated pivot-point vectors (in
 * local table coordinates).  Enforce constraints imposed by any missing motors.
 */
static void 
PivotPointVectorToMotor(tableRecord *ptbl, double *pp0, double *pp1,
	double *pp2, double *m, double *u)
{
	double *ppo0 = ptbl->ppo0, *ppo1 = ptbl->ppo1, *ppo2 = ptbl->ppo2;
	double norm[3], **a = ptbl->a;
    struct private    *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;

	Debug(5, "tableRecord(%s):PivotPointVectorToMotor:entry\n", ptbl->name);

	switch (ptbl->geom) {
	case tableGEOM_SRI:
	case tableGEOM_GEOCARS:
	case tableGEOM_PNC:
	default:
		m[M0X] = pp0[X] - ppo0[X];
		m[M0Y] = pp0[Y] - ppo0[Y];
/*		m[M0Z] = pp0[Z] - ppo0[Z]; motion is free */

/*		m[M1X] = pp1[X] - ppo1[X]; motion is free */
		m[M1Y] = pp1[Y] - ppo1[Y];
/*		m[M1Z] = pp1[Z] - ppo1[Z]; motion is free */

		m[M2X] = pp2[X] - ppo2[X];
		m[M2Y] = pp2[Y] - ppo2[Y];
		if (lnkStat[M2Z].can_RW_drive) {
			m[M2Z] = pp2[Z] - ppo2[Z];
		} else {
			u[Z_6] = -(a[Z][X] * ppo2[X] + a[Z][Y] * ppo2[Y] + (a[Z][Z] - 1) * ppo2[Z]);
			m[M2Z] = 0.0;
		}

		break;

	case tableGEOM_NEWPORT:
		/* Note that we need the rotation matrix for this geometry */
		norm[X] = a[X][Y];	/* unit vector normal to table */
		norm[Y] = a[Y][Y];
		norm[Z] = a[Z][Y];

		m[M0Y] = (pp0[Y] - ppo0[Y]) / norm[Y];
		m[M1Y] = (pp1[Y] - ppo1[Y]) / norm[Y];
		m[M2Y] = (pp2[Y] - ppo2[Y]) / norm[Y];

/*		m[M0Z] = (pp0[Z] - ppo0[Z]) - norm[Z] * m[M0Y]; motion is free */
/*		m[M1Z] = (pp1[Z] - ppo1[Z]) - norm[Z] * m[M1Y]; motion is free */
		m[M2Z] = (pp2[Z] - ppo2[Z]) - norm[Z] * m[M2Y];

		if (lnkStat[M2X].can_RW_drive) {
			/* 6-motor table */
			m[M0X] = (pp0[X] - ppo0[X]) - norm[X] * m[M0Y];
/*			m[M1X] = (pp1[X] - ppo1[X]) - norm[X] * m[M1Y]; motion is free */
			m[M2X] = (pp2[X] - ppo2[X]) - norm[X] * m[M2Y];
		} else {
			/* 5-motor table */
			/* x (in local coordinate system) is constrained by the missing motor */
			u[X_6] = -((a[X][X] - 1) * ppo2[X] + a[X][Y] * ppo2[Y] + a[X][Z] * ppo2[Z] - norm[X] * m[M2Y]);
			m[M0X] = (a[X][X] - 1) * ppo0[X] + a[X][Y] * ppo0[Y] + a[X][Z] * ppo0[Z] -
				norm[X] * m[M0Y] + u[X_6];
/*			m[M1X] = (a[X][X] - 1) * ppo1[X] + a[X][Y] * ppo1[Y] + a[X][Z] * ppo1[Z] -
				norm[X] * m[M1Y] + u[X_6]; free */
			m[M2X] = 0.0;	/* no motor */
		}
		break;

	}
}


static void 
UserToMotor(tableRecord *ptbl, double *user, double *m)
{
	double u[6];
	double *ax0 = &ptbl->ax0;
	int i;

	Debug(5, "tableRecord(%s):UserToMotor:entry\n", ptbl->name);
	/* Get user coordinates into local coordinate system */
	for (i=0; i<6; i++) {u[i] = user[i] + ax0[i];}
	LabToLocal(ptbl, u, u);

	LocalUserToPivotPointVector(ptbl, u, ptbl->pp0, ptbl->pp1, ptbl->pp2);
	PivotPointVectorToMotor(ptbl, ptbl->pp0, ptbl->pp1, ptbl->pp2, m, u);
}


static void 
MakeRotationMatrix(tableRecord *ptbl, double *u)
{
	double **a = ptbl->a;
	double cx, cy, cz, sx, sy, sz;

	cx = cos(D2R * u[AX_6]); sx = sin(D2R * u[AX_6]);
	cy = cos(D2R * u[AY_6]); sy = sin(D2R * u[AY_6]);
	cz = cos(D2R * u[AZ_6]); sz = sin(D2R * u[AZ_6]);

	/* Make rotation matrix */
	a[0][0] = cy*cz;            a[0][1] = cy*sz;            a[0][2] = -sy;
	a[1][0] = sx*sy*cz - cx*sz; a[1][1] = sx*sy*sz + cx*cz; a[1][2] = sx*cy;
	a[2][0] = cx*sy*cz + sx*sz; a[2][1] = cx*sy*sz - sx*cz; a[2][2] = cx*cy;

	if (tableRecordDebug >= 5) {
		printf("    [%9.6f %9.6f %9.6f]\n", a[0][0], a[0][1], a[0][2]);
		printf("a = [%9.6f %9.6f %9.6f]\n", a[1][0], a[1][1], a[1][2]);
		printf("    [%9.6f %9.6f %9.6f]\n", a[2][0], a[2][1], a[2][2]);
	}
}


static void 
LabToLocal(tableRecord *ptbl, double *lab, double *local)
{
	RotY(lab, local, ptbl->yang*D2R);
}


static void 
LocalToLab(tableRecord *ptbl, double *local, double *lab)
{
	RotY(local, lab, -ptbl->yang*D2R);
}


static void
RotY(double *in_, double *out, double a)
{
	short i;
	double in[6];

	/* copy, in case in_ and out point to same place */
	for (i=0; i<6; i++) in[i] = in_[i];

	out[X_6]  = in[X_6]  * cos(a)  +  in[Z_6]  * sin(a);
	out[AX_6] = in[AX_6] * cos(a)  +  in[AZ_6] * sin(a);
	out[Z_6]  = in[X_6]  * -sin(a) +  in[Z_6]  * cos(a);
	out[AZ_6] = in[AX_6] * -sin(a) +  in[AZ_6] * cos(a);
	out[Y_6]  = in[Y_6];
	out[AY_6] = in[AY_6];
}


static void 
ZeroTable(tableRecord *ptbl)
{
	short i;
	double *ax = &ptbl->ax, *ax0 = &ptbl->ax0, *axl = &ptbl->axl;

	/* Make current rotation/translation appear as zero. */
	for (i=0; i<6; i++) {
		*ax++ = 0;
		*ax0++ = *axl++;
	}
}


static void
PrintTrajectory(struct trajectory *t, int n)
{
	short i, j;
	printf("Trajectory:\n");
	for (i=0; i<n; i++, t++) {
		printf("   user=%8.3f, ", t->user);
		printf("lvio=%1d, m = [", t->lvio);
		for (j=0; j<5; j++) {printf("%8.3f, ", t->motor[j]);}
		printf("%f]\n", t->motor[5]);
	}
}


static void
InitTrajectory(struct trajectory *t, int n)
{
	short i, j;
	for (i=0; i<n; i++, t++) {
		t->user = 0;
		t->lvio = 0;
		for (j=0; j<6; j++) {t->motor[j] = 0;}
	}
}


/* Sort array of struct trajectory according to the user field.
 * Caller guarantees that first element is already in its correct place,
 * and this fact determines the sort order.
 */
static void
SortTrajectory(struct trajectory *t, int n)
{
	int i, j;
	int ascending = t[1].user > t[0].user;
	struct trajectory this;

	if (n<3) return;
	for (j=2; j<n; j++) {
		memcpy((void *)&this, (void *)&t[j], sizeof(struct trajectory));
		i = j-1;
		while (i >= 0 && ((t[i].user > this.user) == ascending)) {
			memcpy((void *)&t[i+1], (void *)&t[i], sizeof(struct trajectory));
			i--;
		}
		memcpy((void *)&t[i+1], (void *)&this, sizeof(struct trajectory));
	}
}

/* From Numerical Recipes in C */
#define NTRAJ 10
static int
polint(double *xa, double *ya, int n, double x, double *y, double *dy)
{
	int i,m,ns=1;
	double den,dif,dift,ho,hp,w;
	double room[NTRAJ*2], *c, *d;

	if (n>NTRAJ) n = NTRAJ;
	c = &(room[-1]); d = &(room[n-1]);

	dif=fabs(x-xa[1]);
	for (i=1;i<=n;i++) {
		if ( (dift=fabs(x-xa[i])) < dif) {
			ns=i;
			dif=dift;
		}
		c[i]=ya[i];
		d[i]=ya[i];
	}
	*y=ya[ns--];
	for (m=1;m<n;m++) {
		for (i=1;i<=n-m;i++) {
			ho=xa[i]-x;
			hp=xa[i+m]-x;
			w=c[i+1]-d[i];
			if ( (den=ho-hp) == 0.0) return(1); /* Error in routine POLINT*/
			den=w/den;
			d[i]=hp*den;
			c[i]=ho*den;
		}
		*y += (*dy=(2*ns < (n-m) ? c[ns+1] : d[ns--]));
	}
	if (tableRecordDebug >= 5) {
			printf("polint: interpolation yields %f\n", *y);
	}
	return(0);
}


static int
FindLimit(tableRecord *ptbl, struct trajectory *t, int n, double *userLimit)
{
	int i, j, failed;
	double	*hm = &ptbl->h0x, *lm = &ptbl->l0x;
	double user[NTRAJ], motor[NTRAJ], limit, error;
    struct	private *p = (struct private *)ptbl->dpvt;
    struct	linkStatus *lnkStat = p->lnkStat;

	SortTrajectory(t, n);
	if (tableRecordDebug >= 10) PrintTrajectory(t, n);

	/* Make sure a limit violation occurred */
	for (i=0; i<n; i++) {if (t[i].lvio) break;}
	if (i == n) {
		if (tableRecordDebug >= 5) {
			printf("FindLimit: no limit was reached\n");
		}
		return(1);	/* failure */
	}

	for (i=0; i<n; i++) {user[i] = t[i].user;}
	*userLimit = user[n-1];

	failed = 1;
	for (i=M0X; i<=M2Z; i++) {
		if (lnkStat[i].can_read_limits) {
			for (j=0; j<n; j++) {motor[j] = t[j].motor[i];}
			if ((hm[i] > motor[0]) != (hm[i] > motor[n-1])) {
				if (tableRecordDebug >= 10)
					printf("FindLimit:motor %d high limit was crossed\n", i);
				/* At what user valuewas the motor limit hm[i] crossed? */
				if (polint(&(motor[-1]), &(user[-1]), n, hm[i], &limit, &error)
						 == 0) {
					failed = 0;
					if (fabs(limit-user[0]) < fabs(*userLimit-user[0]))
						*userLimit = limit;
				}
			}
			if ((lm[i] > motor[0]) != (lm[i] > motor[n-1])) {
				if (tableRecordDebug >= 10)
					printf("FindLimit:motor %d low limit was crossed\n", i);
				/* At what user value was the motor limit lm[i] crossed? */
				if (polint(&(motor[-1]), &(user[-1]), n, lm[i], &limit, &error)
						== 0) {
					failed = 0;
					if (fabs(limit-user[0]) < fabs(*userLimit-user[0]))
						*userLimit = limit;
				}
			}
		}
	}
	if (tableRecordDebug >= 5 && failed)
		printf("FindLimit:motor %d NO limit was crossed\n", i);
	return(failed);
}


#define DELTA_INIT 5
static void
CalcLocalUserLimits(tableRecord *ptbl)
{
	unsigned short  i, j, k, ii, limitCrossings;
	double	*hm = &ptbl->h0x, *lm = &ptbl->l0x;
	double	*hu = &ptbl->hlax, *lu = &ptbl->llax, *lim;
	double	pp0h[3], pp1h[3], pp2h[3];
	double	pp0l[3], pp1l[3], pp2l[3];
	double	*pp0 = ptbl->pp0, *pp1 = ptbl->pp1, *pp2 = ptbl->pp2;
	double	*ax = &ptbl->ax, *ax0= &ptbl->ax0, *m0x = &ptbl->m0x;
	double	save, delta, limit;
	double	u[6];
	struct	trajectory t[NTRAJ]; 
    struct	private *p = (struct private *)ptbl->dpvt;
    struct	linkStatus *lnkStat = p->lnkStat;

	UserToMotor(ptbl, ax, m0x);

	/*** get limits for user translations ***/
	NaiveMotorToPivotPointVector(ptbl, hm, pp0h, pp1h, pp2h);
	NaiveMotorToPivotPointVector(ptbl, lm, pp0l, pp1l, pp2l);

	for (i=0; i<6; i++) {
		u[i] = ax[i] + ax0[i];
		hu[i] = LARGE; lu[i] = -LARGE;
	}
	LabToLocal(ptbl, u, u);

	if (lnkStat[M0X].can_read_limits) {
		hu[X_6] = MIN(hu[X_6], u[X_6] + pp0h[X]-pp0[X]);
		lu[X_6] = MAX(lu[X_6], u[X_6] + pp0l[X]-pp0[X]);
	}
	if (lnkStat[M2X].can_read_limits) {
		hu[X_6] = MIN(hu[X_6], u[X_6] + pp2h[X]-pp2[X]);
		lu[X_6] = MAX(lu[X_6], u[X_6] + pp2l[X]-pp2[X]);
	}
	if (lnkStat[M0Y].can_read_limits) {
		hu[Y_6] = MIN(hu[Y_6], u[Y_6] + pp0h[Y]-pp0[Y]);
		lu[Y_6] = MAX(lu[Y_6], u[Y_6] + pp0l[Y]-pp0[Y]);
	}
	if (lnkStat[M1Y].can_read_limits) {
		hu[Y_6] = MIN(hu[Y_6], u[Y_6] + pp1h[Y]-pp1[Y]);
		lu[Y_6] = MAX(lu[Y_6], u[Y_6] + pp1l[Y]-pp1[Y]);
	}
	if (lnkStat[M2Y].can_read_limits) {
		hu[Y_6] = MIN(hu[Y_6], u[Y_6] + pp2h[Y]-pp2[Y]);
		lu[Y_6] = MAX(lu[Y_6], u[Y_6] + pp2l[Y]-pp2[Y]);
	}
	if (lnkStat[M2Z].can_read_limits) {
		hu[Z_6] = MIN(hu[Z_6], u[Z_6] + pp2h[Z]-pp2[Z]);
		lu[Z_6] = MAX(lu[Z_6], u[Z_6] + pp2l[Z]-pp2[Z]);
	}
	for (i=X_6; i<=Z_6; i++) {
		if (hu[i] == LARGE) hu[i] = u[i] + SMALL;
		if (lu[i] == -LARGE) lu[i] = u[i] - SMALL;
	}

	/*** get limits for user rotations ***/
	for (i=AX_6; i<=AZ_6; i++) {
		/* save user coordinate */
		save = ax[i];

		/* try to find a legal value for this coordinate */
		if (MotorLimitViol(ptbl)) {
			ax[i] = 0;
			UserToMotor(ptbl, ax, m0x);
			if (MotorLimitViol(ptbl)) {
				hu[i] = lu[i] = ax[i] = save;
				UserToMotor(ptbl, ax, m0x);
				continue;
			}
		}
		for (ii=0, lim=&hu[i], delta=DELTA_INIT;
				ii<=1;
				ii++, lim=&lu[i], delta = -DELTA_INIT) {
			InitTrajectory(t, NTRAJ);
			limitCrossings = 0;
			for (j=0; j<NTRAJ && limitCrossings<2; j++) {
				t[j].user = ax[i];
				for (k=0; k<6; k++) {t[j].motor[k] = m0x[k];}
				t[j].lvio = MotorLimitViol(ptbl);
				if (j) {
					if (t[j].lvio != t[j-1].lvio) {
						limitCrossings++;
						delta = -delta;
					}
					if (limitCrossings) delta *= 0.5;
				}
				ax[i] += delta;
				UserToMotor(ptbl, ax, m0x);
			}
			if (tableRecordDebug >= 10) {
				printf("CalcLocalUserLimits: userCoord %d, limitCrossings=%d\n",
					i, limitCrossings);
			}
			if (limitCrossings && !(FindLimit(ptbl, t, j, &limit))) {
				*lim = limit;
			} else {
				*lim = save;
			}
			/* restore user coordinate */
			ax[i] = save;
			UserToMotor(ptbl, ax, m0x);
		}
	}
}


static void 
UserLimits_LocalToLab(tableRecord *ptbl)
{
	unsigned short quadrant;
	double	*hu = &ptbl->hlax, *lu = &ptbl->llax;
	double	hi_lab[6], lo_lab[6];
	double	sa = sin(ptbl->yang*D2R), ca = cos(ptbl->yang*D2R);

	/* cvt translation limits from local user coord's to lab user coord's */
	quadrant  = (sa >= 0) ? 0x10 : 0x00;
	quadrant |= (ca >= 0) ? 0x01 : 0x00;
	switch (quadrant) {
	/*   0xSC */
	case 0x11:
		/* local's +z is in lab's (-x,+z) quadrant */
		hi_lab[X_6] = ca*hu[X_6] - sa*lu[Z_6];
		lo_lab[X_6] = ca*lu[X_6] - sa*hu[Z_6];
		hi_lab[Z_6] = sa*hu[X_6] + ca*hu[Z_6];
		lo_lab[Z_6] = sa*lu[X_6] + ca*lu[Z_6];
		break;
	case 0x10:
		/* local's +z is in lab's (-x,-z) quadrant */
		hi_lab[X_6] = ca*lu[X_6] - sa*lu[Z_6];
		lo_lab[X_6] = ca*hu[X_6] - sa*hu[Z_6];
		hi_lab[Z_6] = sa*hu[X_6] + ca*lu[Z_6];
		lo_lab[Z_6] = sa*lu[X_6] + ca*hu[Z_6];
		break;
	case 0x00:
		/* local's +z is in lab's (+x,-z) quadrant */
		hi_lab[X_6] = ca*lu[X_6] - sa*hu[Z_6];
		lo_lab[X_6] = ca*hu[X_6] - sa*lu[Z_6];
		hi_lab[Z_6] = sa*lu[X_6] + ca*lu[Z_6];
		lo_lab[Z_6] = sa*hu[X_6] + ca*hu[Z_6];
		break;
	case 0x01:
		/* local's +z is in lab's (+x,+z) quadrant */
		hi_lab[X_6] = ca*hu[X_6] - sa*hu[Z_6];
		lo_lab[X_6] = ca*lu[X_6] - sa*lu[Z_6];
		hi_lab[Z_6] = sa*lu[X_6] + ca*hu[Z_6];
		lo_lab[Z_6] = sa*hu[X_6] + ca*lu[Z_6];
		break;
	}
	hu[X_6] = hi_lab[X_6];
	lu[X_6] = lo_lab[X_6];
	hu[Z_6] = hi_lab[Z_6];
	lu[Z_6] = lo_lab[Z_6];
}

static void
PrintValues(tableRecord *ptbl)
{
	double	*p, **a;

	p = ptbl->pp0;
	printf("pp0 = [%9.6f %9.6f %9.6f]\n", p[X], p[Y], p[Z]);
	p = ptbl->pp1;
	printf("pp1 = [%9.6f %9.6f %9.6f]\n", p[X], p[Y], p[Z]);
	p = ptbl->pp2;
	printf("pp2 = [%9.6f %9.6f %9.6f]\n", p[X], p[Y], p[Z]);

	p = &ptbl->m0x;
	printf(" m = [%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f]\n",
		p[M0X], p[M0Y], p[M1Y], p[M2X], p[M2Y], p[M2Z]);

	p = &ptbl->ax;
	printf(" u = [%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f]\n",
		p[AX_6], p[AY_6], p[AZ_6], p[X_6], p[Y_6], p[Z_6]);

	p = &ptbl->axl;
	printf("ul = [%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f]\n",
		p[AX_6], p[AY_6], p[AZ_6], p[X_6], p[Y_6], p[Z_6]);

	p = &ptbl->ax0;
	printf("u0 = [%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f]\n",
		p[AX_6], p[AY_6], p[AZ_6], p[X_6], p[Y_6], p[Z_6]);

	a = ptbl->a;
	printf("    [%9.6f %9.6f %9.6f]\n", a[0][0], a[0][1], a[0][2]);
	printf("a = [%9.6f %9.6f %9.6f]\n", a[1][0], a[1][1], a[1][2]);
	printf("    [%9.6f %9.6f %9.6f]\n", a[2][0], a[2][1], a[2][2]);

	a = ptbl->b;
	printf("    [%9.6f %9.6f %9.6f]\n", a[0][0], a[0][1], a[0][2]);
	printf("b = [%9.6f %9.6f %9.6f]\n", a[1][0], a[1][1], a[1][2]);
	printf("    [%9.6f %9.6f %9.6f]\n", a[2][0], a[2][1], a[2][2]);
}


static void 
CalcUserLimits(tableRecord *ptbl)
{
	unsigned short  i;
	double	*hu = &ptbl->hlax, *lu = &ptbl->llax;
	double	*uhax = &ptbl->uhax, *ulax = &ptbl->ulax;
	double	*ax0 = &ptbl->ax0;
	int saveTableRecordDebug = tableRecordDebug;

	Debug(5, "tableRecord(%s):CalcUserLimits:entry\n", ptbl->name);
	if (tableRecordDebug >= 10) {
		PrintValues(ptbl);
	} else {
		tableRecordDebug=0;	/* CalcUserLimits produces too much output */
	}

	CalcLocalUserLimits(ptbl);
	UserLimits_LocalToLab(ptbl);

	/* Enforce user's limits; subtract offsets */
	for (i=0; i<6; i++) {
		if ((uhax[i] != 0.0) || (ulax[i] != 0.0)) {
			hu[i] = MIN(hu[i], uhax[i]);
			lu[i] = MAX(lu[i], ulax[i]);
		}
		hu[i] -= ax0[i];
		lu[i] -= ax0[i];
	}
	if (tableRecordDebug > 10) {
		printf("CalcUserLimits:done\n");
		PrintValues(ptbl);
	}

	tableRecordDebug = saveTableRecordDebug;
}


static void
checkLinks(tableRecord *ptbl)
{
	struct link *ml = &ptbl->m0xl, *rl = &ptbl->r0xi;
	struct link *vl = &ptbl->v0xl, *vi = &ptbl->v0xi;
	struct link *el = &ptbl->e0xi;
	struct link *hl = &ptbl->h0xl, *ll = &ptbl->l0xl;
    struct private *p = (struct private *)ptbl->dpvt;
    struct linkStatus *lnkStat = p->lnkStat;
    int i;

    Debug(5, "checkLinks() for table record at %p\n", (void *)ptbl);

    for (i=0; i<6; i++) {
		/* Can move? (need to read and write motor's drive field) */
		lnkStat[i].can_RW_drive = 1;
		if ((ml[i].type == CONSTANT) || (rl[i].type == CONSTANT))
			lnkStat[i].can_RW_drive = 0;
		if (ml[i].type == CA_LINK)
			lnkStat[i].can_RW_drive = dbCaIsLinkConnected(&(ml[i]));
		if ((rl[i].type == CA_LINK) && !dbCaIsLinkConnected(&(rl[i])))
			lnkStat[i].can_RW_drive = 0;

		/* Can read limits? (need both) */
		lnkStat[i].can_read_limits = 1;
		if ((hl[i].type == CONSTANT) || (ll[i].type == CONSTANT))
			lnkStat[i].can_read_limits = 0;
		if (hl[i].type == CA_LINK)
			lnkStat[i].can_read_limits = dbCaIsLinkConnected(&(hl[i]));
		if ((ll[i].type == CA_LINK) && !dbCaIsLinkConnected(&(ll[i])))
			lnkStat[i].can_read_limits = 0;

		/* Can read position? */ 
		lnkStat[i].can_read_position = 1;
		if (el[i].type == CONSTANT)
			lnkStat[i].can_read_position = 0;
		if (el[i].type == CA_LINK)
			lnkStat[i].can_read_position = dbCaIsLinkConnected(&(el[i]));

		/* Can set speed? (need to R/W speed field) */
		lnkStat[i].can_RW_speed = 1;
		if ((vl[i].type == CONSTANT) || (vi[i].type == CONSTANT))
			lnkStat[i].can_RW_speed = 0;
		if (vl[i].type == CA_LINK)
			lnkStat[i].can_RW_speed = dbCaIsLinkConnected(&(vl[i]));
		if ((vi[i].type == CA_LINK) && !dbCaIsLinkConnected(&(vi[i])))
			lnkStat[i].can_RW_speed = 0;
    }
}
