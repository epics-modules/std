/* $Id: sCalcPostfix.c,v 1.3 2003-05-28 22:17:33 bcda Exp $
 * Subroutines used to convert an infix expression to a postfix expression
 *
 *      Author:          Bob Dalesio
 *      Date:            12-12-86
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  01-11-89	lrd	added right shift and left shift operations
 * .02  01-13-89	lrd	modified to load into IOCs
 * .03  02-01-89	lrd	added trigonometric functions
 * .04  04-05-89	lrd	fixed the order of some operations in the
 *						element table and added a warning label
 * .05  11-26-90	lrd	fix SINH, COSH, TANH
 * .06	02-20-92	rcz	fixed for vxWorks build
 * .07  02-24-92	jba	add EXP and fixed trailing blanks in expression
 * .08  03-03-92	jba	added MAX_VAL and MIN_VAL and comma(like close paren)
 * .09  03-06-92	jba	added multiple conditional expressions ?
 * .10  04-01-92	jba	allowed floating pt constants in expression
 * .11  05-01-92	jba	flt pt constant string replaced with double in postfix
 * .12  08-21-92	jba	ANSI c changes
 * .13  08-21-92	jba	initialized *ppostfix: needed when calc expr not defined
 * .14  12-11-92	mrk	Removed include for stdioLib.h
 * .15  11-03-93	jba	Added test for extra close paren at end of expression
 * .16  01-24-94	jba	Changed seperator test to catch invalid commas
 * .17  05-11-94	jba	Added support for CONST_PI, CONST_R2D, and CONST_D2R
 *						and conversion of infix expression to uppercase
 * .18  01-22-98	tmm	Changed name postfix() to calcPostfix().  Changed arg list
 *						from pointer to postfix expression to address of pointer to
 *						postfix expression.  calcPostfix() frees old expression, allocates
 *						space sufficient to hold new expression, and returns pointer to it.
 *						Added S2R, R2S (conversions between arc-seconds and radians).  Changed
 *						CONSTANT to LITERAL to avoid conflict with link.h.  Support 26 vars (A-Z).
 * .19  03-18-98	tmm	Added string operators
 * .20  11-12-98	tmm	v4.2 Added array operators @, @@.  Changed ICP of
 *					[ and { operators from 8 to 7.
 * .21  11-27-01	mlr	Added BYTE unary function
 * .22  03-20-03    tmm Added support for variable # of args (VARG_FUNC MAXV_VAL, MINV_VAL),
 *                      <? and >? operators, sCalcCheck(), FETCH_*
 * .23  04-09-03    tmm Changed arg list: char * instead of char ** for postfix buffer.  User
 *                      should provide 240-char buffer (for 40-char infix string).
 */

/* 
 * Subroutines
 *
 *	Public
 *
 * sCalcPostfix		convert an algebraic expression to symbolic postfix
 *	args
 *		pinfix		the algebraic expression
 *		p_postfix	symbolic postfix expression
 *      perror      error information
 *	returns
 *		0		successful
 *		-1		not successful
 * Private routines for calcPostfix
 *
 * find_element		finds a symbolic element in the expression element tbl
 *	args
 *		pbuffer		pointer to the infix expression element
 *		pelement	pointer to the expression element table entry
 *		pno_bytes	pointer to the size of this element
 *		parg		pointer to arg (used for fetch)
 *	returns
 *		TRUE		element found
 *		FALSE		element not found
 *
 * get_element		finds the next expression element in the infix expr
 *	args
 *		pinfix		pointer into the infix expression
 *		pelement	pointer to the expression element table
 *		pno_bytes	size of the element in the infix expression
 *		parg		pointer to argument (used for fetch)
 *	returns
 *		FINE		found an expression element
 *		VARIABLE	found a database reference
 *		UNKNOWN_ELEMENT	unknown element found in the infix expression
 */

#ifdef vxWorks
#include  <vxWorks.h>
#endif

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<ctype.h>
#include	"dbDefs.h"
#define epicsExportSharedSymbols
#include	"sCalcPostfix.h"
#include	"sCalcPostfixPvt.h"


long test_sCalcPostfix(char *pinfix, int n);
long test_sCalcPerform(char *pinfix, int n);

#define DEBUG 0
volatile int sCalcPostfixDebug=0;

/* declarations for postfix */
/* element types */
#define	OPERAND			0
#define UNARY_OPERATOR	1
#define	BINARY_OPERATOR	2
#define	EXPR_TERM		3
#define	COND			4
#define	CLOSE_PAREN		5
#define	CONDITIONAL		6
#define	ELSE			7
#define	SEPARATOR		8
#define	TRASH			9
#define	FLOAT_PT_CONST	10
#define	MINUS_OPERATOR	11
#define	STRING_CONST	12
#define	CLOSE_BRACKET	13
#define	CLOSE_CURLY		14
#define	VARG_FUNC		15

/* parsing return values */
#define	FINE		0
#define	UNKNOWN_ELEMENT	-1
#define	END		-2

/*
 * element table
 *
 * structure of an element
 */
struct	expression_element{
	char	element[10];	/* character representation of an element */
	char	in_stack_pri;	/* priority in translation stack */
	char	in_coming_pri;	/* priority when first checking */
	char	type;		/* element type */
	char	code;		/* postfix representation */
};

/*
 * NOTE: DO NOT CHANGE WITHOUT READING THIS NOTICE !!!!!!!!!!!!!!!!!!!!
 * Because the routine that looks for a match in this table takes the first 
 * match it finds, elements whose designations are contained in other elements
 * MUST come first in this list. (e.g. ABS will match A if A preceeds ABS and
 * then try to find BS therefore ABS must be first in this list
 */
#define UNARY_MINUS_I_S_P  8
#define UNARY_MINUS_I_C_P  9
#define UNARY_MINUS_CODE   UNARY_NEG
#define BINARY_MINUS_I_S_P 5
#define BINARY_MINUS_I_C_P 5
#define BINARY_MINUS_CODE  SUB

static struct expression_element elements[] = {
/*
element    i_s_p i_c_p type_element     internal_rep */
{"ABS",    8,    9,    UNARY_OPERATOR,  ABS_VAL},   /* absolute value */
{"NOT",    8,    9,    UNARY_OPERATOR,  UNARY_NEG},   /* unary negate */
{"-",      8,    9,    MINUS_OPERATOR,  UNARY_NEG},   /* unary negate (or binary op) */
{"SQRT",   8,    9,    UNARY_OPERATOR,  SQU_RT},      /* square root */
{"SQR",    8,    9,    UNARY_OPERATOR,  SQU_RT},      /* square root */
{"EXP",    8,    9,    UNARY_OPERATOR,  EXP},         /* exponential function */
{"LOGE",   8,    9,    UNARY_OPERATOR,  LOG_E},       /* log E */
{"LN",     8,    9,    UNARY_OPERATOR,  LOG_E},       /* log E */
{"LOG",    8,    9,    UNARY_OPERATOR,  LOG_10},      /* log 10 */
{"ACOS",   8,    9,    UNARY_OPERATOR,  ACOS},        /* arc cosine */
{"ASIN",   8,    9,    UNARY_OPERATOR,  ASIN},        /* arc sine */
{"ATAN2",  8,    9,    UNARY_OPERATOR,  ATAN2},       /* arc tangent */
{"ATAN",   8,    9,    UNARY_OPERATOR,  ATAN},        /* arc tangent */
{"MAX",    8,    9,    VARG_FUNC,       MAXV_VAL},    /* variable # of args */
{"MIN",    8,    9,    VARG_FUNC,       MINV_VAL},    /* variable # of args */
{"CEIL",   8,    9,    UNARY_OPERATOR,  CEIL},        /* smallest integer >= */
{"FLOOR",  8,    9,    UNARY_OPERATOR,  FLOOR},       /* largest integer <=  */
{"NINT",   8,    9,    UNARY_OPERATOR,  NINT},        /* nearest integer */
{"INT",    8,    9,    UNARY_OPERATOR,  NINT},        /* nearest integer */
{"COSH",   8,    9,    UNARY_OPERATOR,  COSH},        /* hyperbolic cosine */
{"COS",    8,    9,    UNARY_OPERATOR,  COS},         /* cosine */
{"SINH",   8,    9,    UNARY_OPERATOR,  SINH},        /* hyperbolic sine */
{"SIN",    8,    9,    UNARY_OPERATOR,  SIN},         /* sine */
{"TANH",   8,    9,    UNARY_OPERATOR,  TANH},        /* hyperbolic tangent*/
{"TAN",    8,    9,    UNARY_OPERATOR,  TAN},         /* tangent */
{"!=",     4,    4,    BINARY_OPERATOR, NOT_EQ},      /* not equal */
{"!",      8,    9,    UNARY_OPERATOR,  REL_NOT},     /* not */
{"~",      8,    9,    UNARY_OPERATOR,  BIT_NOT},     /* bitwise not */
{"DBL",    8,    9,    UNARY_OPERATOR,  TO_DOUBLE},   /* convert to double */
{"STR",    8,    9,    UNARY_OPERATOR,  TO_STRING},   /* convert to string */
{"$P",     8,    9,    UNARY_OPERATOR,  PRINTF},      /* formatted print to string */
{"PRINTF", 8,    9,    UNARY_OPERATOR,  PRINTF},      /* formatted print to string */
{"BYTE",   8,    9,    UNARY_OPERATOR,  BYTE},        /* string[0] to byte */
{"$S",     8,    9,    UNARY_OPERATOR,  SSCANF},      /* scan string argument */
{"SSCANF", 8,    9,    UNARY_OPERATOR,  SSCANF},      /* scan string argument */
{"@@",     8,    9,    UNARY_OPERATOR,  A_SFETCH},    /* fetch string argument */
{"@",      8,    9,    UNARY_OPERATOR,  A_FETCH},     /* fetch numeric argument */
{"RNDM",   0,    0,    OPERAND,         RANDOM},      /* Random Number */
{"OR",     1,    1,    BINARY_OPERATOR, BIT_OR},      /* or */
{"AND",    2,    2,    BINARY_OPERATOR, BIT_AND},     /* and */
{"XOR",    1,    1,    BINARY_OPERATOR, BIT_EXCL_OR}, /* exclusive or */
{"PI",     0,    0,    OPERAND,         CONST_PI},    /* pi */
{"D2R",    0,    0,    OPERAND,         CONST_D2R},   /* pi/180 */
{"R2D",    0,    0,    OPERAND,         CONST_R2D},   /* 180/pi */
{"S2R",    0,    0,    OPERAND,         CONST_S2R},   /* arc-sec to radians: pi/(180*3600) */
{"R2S",    0,    0,    OPERAND,         CONST_R2S},   /* radians to arc-sec: (180*3600)/pi */
{"0",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"1",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"2",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"3",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"4",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"5",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"6",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"7",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"8",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"9",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{".",      0,    0,    FLOAT_PT_CONST,  LITERAL},     /* flt pt constant */
{"\"",     0,    0,    STRING_CONST,    SLITERAL},    /* string constant */
{"'",      0,    0,    STRING_CONST,    SLITERAL},    /* string constant */
{"?",      0,    0,    CONDITIONAL,     COND_IF},     /* conditional */
{":",      0,    0,    CONDITIONAL,     COND_ELSE},   /* else */
{"(",      0,    9,    UNARY_OPERATOR,  PAREN},       /* open paren */
{"[",      0,    8,    BINARY_OPERATOR, SUBRANGE},    /* string subrange */
{"{",      0,    8,    BINARY_OPERATOR, REPLACE},     /* string replace */
{"^",      7,    7,    BINARY_OPERATOR, EXPON},       /* exponentiation */
{"**",     7,    7,    BINARY_OPERATOR, EXPON},       /* exponentiation */
{"+",      5,    5,    BINARY_OPERATOR, ADD},         /* addition */
#if 0 /* "-" operator is overloaded; may be unary or binary */
{"-",      5,    5,    BINARY_OPERATOR, SUB},         /* subtraction */
#endif
{"*",      6,    6,    BINARY_OPERATOR, MULT},        /* multiplication */
{"/",      6,    6,    BINARY_OPERATOR, DIV},         /* division */
{"%",      6,    6,    BINARY_OPERATOR, MODULO},      /* modulo */
{",",      0,    0,    SEPARATOR,       COMMA},       /* comma */
{")",      0,    0,    CLOSE_PAREN,     PAREN},       /* close paren */
{"]",      0,    0,    CLOSE_BRACKET,   SUBRANGE},    /* close bracket */
{"]",      0,    0,    CLOSE_BRACKET,   SUBRANGE},    /* close bracket */
{"}",      0,    0,    CLOSE_CURLY,     REPLACE},     /* close curly bracket */
{"||",     1,    1,    BINARY_OPERATOR, REL_OR},      /* logical or */
{"|",      1,    1,    BINARY_OPERATOR, BIT_OR},      /* bitwise or */
{"&&",     2,    2,    BINARY_OPERATOR, REL_AND},     /* logical and */
{"&",      2,    2,    BINARY_OPERATOR, BIT_AND},     /* bitwise and */
{">?",     3,    3,    BINARY_OPERATOR, MAX_VAL},     /* maximum of 2 args */
{">>",     2,    2,    BINARY_OPERATOR, RIGHT_SHIFT}, /* right shift */
{">=",     4,    4,    BINARY_OPERATOR, GR_OR_EQ},    /* greater or equal*/
{">",      4,    4,    BINARY_OPERATOR, GR_THAN},     /* greater than */
{"<?",     3,    3,    BINARY_OPERATOR, MIN_VAL},     /* minimum of 2 args */
{"<<",     2,    2,    BINARY_OPERATOR, LEFT_SHIFT},  /* left shift */
{"<=",     4,    4,    BINARY_OPERATOR, LESS_OR_EQ},  /* less or equal to*/
{"<",      4,    4,    BINARY_OPERATOR, LESS_THAN},   /* less than */
{"#",      4,    4,    BINARY_OPERATOR, NOT_EQ},      /* not equal */
{"==",     4,    4,    BINARY_OPERATOR, EQUAL},       /* equal */
{"=",      4,    4,    BINARY_OPERATOR, EQUAL},       /* equal */
{""}
};

/*
 * Element-table entry for "fetch" operation.  This element is used for all
 * named variables.  Currently, letters A-Z (double) and AA-ZZ (string) are
 * allowed.  Lower and upper case letters mean the same thing.
 */
static struct expression_element	fetch_element = {
"A",		0,	0,	OPERAND,	FETCH,   /* fetch var */
};

static struct expression_element	fetch_string_element = {
"AA",		0,	0,	OPERAND,	SFETCH,   /* fetch var */
};

static int strncasecmp(char *s1, char *s2, size_t n)
{
	short i;
	for (i=0; i<(short)n && (*s1 || *s2); i++, s1++, s2++) {
		if (toupper((int)*s1) > toupper((int)*s2)) return(1);
		if (toupper((int)*s1) < toupper((int)*s2)) return(-1);
	}
	return(0);
}


/*
 * sCalcCheck()
 * The implementation of a variable number of arguments (for MAX and MIN)
 * has sCalcPerform() putting a NaN on its result stack to mark the end of
 * arguments.  sCalcPostfix() does not check its work thoroughly enough to ensure
 * that a pathological expression, such as "tan(max(a),b)" (enough total
 * arguments, but too many for "tan" and not enough for "max") gets flagged
 * as an error.  So, in the example above, the code that implements "max" will
 * see a NaN after only one argument--before it has begun to look for the
 * end-of-arguments marker--and will treat it as a value to be operated on.
 * We don't want to spend the time on every run of calcPerform() to check for
 * this sort of thing, so we check now.
 * Note that the only kind of problem we're capable of finding is an operator
 * encountering an unexpected NaN that was placed on the stack by the
 * VARG_TERM operator.  Pathological expressions not involving MAX or MIN
 * will sail on by, as they always have.
 */
#define STACKSIZE 30
#define checkStackElement(ps) {if ((ps)->s == NULL && isnan((ps)->d)) return(-1);}
long sCalcCheck(char *post, int forks_checked, int dir_mask)
{
	struct stackElement stack[STACKSIZE], *top;
	struct stackElement *ps;
	int					i, this_fork = 0;
	double				dir;
	short 				got_if=0;
	DOUBLE_LONG			*pu;
	char				*post_top = post;
#if DEBUG
	char				debug_prefix[10]="";

	if (sCalcPostfixDebug) {
		for (i=0; i<=forks_checked; i++)
			strcat(debug_prefix, (dir_mask&(1<<i))?"T":"F");
		printf("sCalcCheck: entry: forks_checked=%d, dir_mask=0x%x\n",
			forks_checked, dir_mask);
	}
	stack[0].d = 1.23456;	/* stack telltale */
	stack[0].s = NULL;
#endif

	top = ps = &stack[1];
	ps--;  /* Expression handler assumes ps is pointing to a filled element */

	/* string expressions and values handled */
	while (*post != END_STACK) {
#if DEBUG
		if (sCalcPostfixDebug) printf("sCalcCheck: %s *post=%d\n", debug_prefix, *post);
#endif

		switch (*post) {

		case FETCH_A: case FETCH_B: case FETCH_C: case FETCH_D:
		case FETCH_E: case FETCH_F: case FETCH_G: case FETCH_H:
		case FETCH_I: case FETCH_J: case FETCH_K: case FETCH_L:
			ps++;
			ps->s = NULL;
			ps->d = 0;
			break;

		case FETCH:
			ps++;
			++post;
			ps->s = NULL;
			ps->d = 0;
			break;

		case SFETCH:	/* fetch from string variable */
			ps++;
			++post;
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			break;

		case CONST_PI:	case CONST_D2R:	case CONST_R2D:	case CONST_S2R:
		case CONST_R2S:	case RANDOM:
			ps++;
			ps->s = NULL;
			ps->d = 0;
			break;

		case ADD:			case SUB:		case MAX_VAL:	case MIN_VAL:
		case MULT:			case DIV:		case EXPON:		case MODULO:
		case REL_OR:		case REL_AND:	case BIT_OR:	case BIT_AND:
		case BIT_EXCL_OR:	case GR_OR_EQ:	case GR_THAN:	case LESS_OR_EQ:
		case LESS_THAN:		case NOT_EQ:	case EQUAL:		case RIGHT_SHIFT:
		case LEFT_SHIFT:	case ATAN2:
			checkStackElement(ps);
			ps--;
			checkStackElement(ps);
			ps->d = 0;
			ps->s = NULL;
			break;

	
		case ABS_VAL:	case UNARY_NEG:	case SQU_RT:	case EXP:
		case LOG_10:	case LOG_E:		case ACOS:		case ASIN:
		case ATAN:		case COS:		case SIN:		case TAN:
		case COSH:		case SINH:		case TANH:		case CEIL:
		case FLOOR:		case NINT:		case REL_NOT:	case BIT_NOT:
		case A_FETCH:	case TO_DOUBLE:	case BYTE:
			checkStackElement(ps);
			ps->d = 0;
			ps->s = NULL;
			break;

		case A_SFETCH:
			checkStackElement(ps);
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			break;

		case LITERAL:
			ps++;
			++post;
			if (post == NULL) {
				++post;
			} else {
				post += 7;
			}
			ps->d = 0;
			ps->s = NULL;
			break;

		case SLITERAL:
			ps++;
			++post;
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			while (*post) post++;
			break;

		case TO_STRING:
			checkStackElement(ps);
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			break;

 		case PRINTF:
 		case SSCANF:
			checkStackElement(ps);
			ps--;
			checkStackElement(ps);
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			ps->d = 0;
			break;

		case SUBRANGE:
		case REPLACE:
			checkStackElement(ps);
			ps--;
			checkStackElement(ps);
			ps--;
			checkStackElement(ps);
			ps->s = &(ps->local_string[0]);
			ps->s[0] = '\0';
			ps->d = 0;
			break;
 
		case MAXV_VAL:
		case MINV_VAL:
			checkStackElement(ps);
			ps--;
			checkStackElement(ps);
			ps++;
			for (i=0; i<MAX_VARGS && !isnan(ps->d); ) {
				ps--;
			}
			if (i == MAX_VARGS)
				return(-1);
			/* replace NaN marker with result */
			ps->d = 0;
			ps->s = NULL;
			break;

		case VARG_TERM:
			/* put a NaN on the value stack to mark the end of arguments */
			ps++;
			ps->s = NULL;
			pu = (DOUBLE_LONG *)&ps->d;
			pu->l[0] = pu->l[1] = 0xffffffff;
			break;

		case COND_IF:
			/*
			 * Recursively check all COND_IF forks:
			 * Take the condition-false path, call sCalcCheck() to check the
			 * condition-true path, giving instructions (forks_checked,
			 * dir_mask) that will bring it to this fork and cause it to take
			 * the condition-true path. 
			 */
			dir = (this_fork <= forks_checked) ? dir_mask&(1<<this_fork) : 0;
			if (this_fork == forks_checked) {
				if (dir == 0) {
					if (sCalcCheck(post_top, this_fork, dir_mask|(1<<this_fork)))
						return(-1);
				}
			} else if (this_fork > forks_checked) {
				/* New fork, so dir has been set to 0 */
				forks_checked++;
				if (sCalcCheck(post_top, this_fork, dir_mask|(1<<this_fork)))
					return(-1);
#if DEBUG
				if (sCalcPostfixDebug) strcat(debug_prefix, "F");
#endif
			}
			this_fork++;  /* assuming we do, in fact, encounter another fork */

			/* if false condition then skip true expression */
			checkStackElement(ps);
			if (ps->d == (double)dir) {
				/* skip to matching COND_ELSE */
				for (got_if=1; got_if>0 && *(post+1) != END_STACK; ++post) {
					switch(post[1]) {
					case LITERAL:	post+=8; break;
					case SLITERAL:	post++; while (post[1]) post++; break;
					case COND_IF:	got_if++; break;
					case COND_ELSE:	got_if--; break;
					case FETCH: case SFETCH: post++; break;
					}
				}
			}
			/* remove condition from stack top */
			--ps;
			break;

		case COND_ELSE:
			/* result, true condition is on stack so skip false condition  */
			/* skip to matching COND_END */
			for (got_if=1; got_if>0 && *(post+1) != END_STACK; ++post) {
				switch(post[1]) {
				case LITERAL:	post+=8; break;
				case SLITERAL:	post++; while (post[1]) post++; break;
				case COND_IF:	got_if++; break;
				case COND_END:	got_if--; break;
				case FETCH: case SFETCH: post++; break;
				}
			}
			break;

		case COND_END:
			break;

		default:
			break;
		}

		/* move ahead in postfix expression */
		++post;
	}

#if DEBUG
	if (ps != top) {
		if (sCalcPostfixDebug>=10) {
			printf("sCalcCheck: stack error: top=%p, ps=%p, top-ps=%d, got_if=%d\n",
				top, ps, top-ps, got_if);
			printf("sCalcCheck: stack error: &stack[0]=%p, stack[0].d=%f\n", &stack[0], stack[0].d);
		}
	}
	if (sCalcPostfixDebug) printf("sCalcCheck: normal exit\n");
#endif

	/* If we have a stack error, and it's not attributable to '?' without ':', complain */
	if ((ps != top) && ((top-ps) != got_if)) return(-1);
	return(0);
}



/*
 * FIND_ELEMENT
 *
 * find the pointer to an entry in the element table
 */
static int find_element(pbuffer, pelement, pno_bytes, parg)
 register char	*pbuffer;
 register struct expression_element	**pelement;
 register short	*pno_bytes, *parg;
 {
	*parg = 0;

 	/* compare the string to each element in the element table */
 	*pelement = &elements[0];
 	while ((*pelement)->element[0] != NULL){
 		if (strncasecmp(pbuffer,(*pelement)->element, strlen((*pelement)->element)) == 0){
 			*pno_bytes += strlen((*pelement)->element);
 			return(TRUE);
 		}
 		*pelement += 1;
 	}

	/* look for a variable reference */
	/* double variables: ["a" - "z"], numbered 1-26 */
	if (isalpha((int)*pbuffer)) {
		*pelement = &fetch_element; /* fetch means "variable reference" (fetch or store) */
		*parg = *pbuffer - (isupper((int)*pbuffer) ? 'A' : 'a');
		*pno_bytes += 1;
		/* string variables: ["aa" - "zz"], numbered 1-26 */
		if (pbuffer[1] == pbuffer[0]) {
			*pelement = &fetch_string_element;
			*pno_bytes += 1;
		}
 		return(TRUE);
	}
#if DEBUG
	if (sCalcPostfixDebug) printf("find_element: can't find '%s'\n", pbuffer);
#endif
 	return(FALSE);
 }
 
/*
 * GET_ELEMENT
 *
 * get an expression element
 */
static int get_element(pinfix, pelement, pno_bytes, parg)
register char	*pinfix;
register struct expression_element	**pelement;
register short	*pno_bytes, *parg;
{

	/* get the next expression element from the infix expression */
	if (*pinfix == NULL) return(END);
	*pno_bytes = 0;
	while (*pinfix == 0x20){
		*pno_bytes += 1;
		pinfix++;
	}
	if (*pinfix == NULL) return(END);
	if (!find_element(pinfix, pelement, pno_bytes, parg))
		return(UNKNOWN_ELEMENT);
#if DEBUG
	if (sCalcPostfixDebug > 5) printf("get_element: found element '%s', arg=%d\n", (*pelement)->element, *parg);
#endif
	return(FINE);

	
}

/*
 * sCalcPostFix
 *
 * convert an infix expression to a postfix expression
 */
long epicsShareAPI sCalcPostfix(char *pinfix, char *ppostfix, short *perror)
{
	short no_bytes, operand_needed, new_expression;
	struct expression_element stack[80], *pelement, *pstacktop;
	double constant;
	char c, in_stack_pri, in_coming_pri, code;
	char *pposthold, *ppostfixStart;
	short arg;

#if DEBUG
	if (sCalcPostfixDebug) printf("sCalcPostfix: entry\n");
#endif

	if (ppostfix == NULL) {
		printf("sCalcPostfix: Caller did not provide a postfix buffer.\n");
		return(-1);
	}

	ppostfixStart = ppostfix;

	*ppostfixStart = BAD_EXPRESSION;
	*(++ppostfix) = END_STACK;

	operand_needed = TRUE;
	new_expression = TRUE;
	*perror = 0;
	if (*pinfix == 0) return(0);
	pstacktop = &stack[0];

	/*** place the expression elements into postfix ***/

	while (get_element(pinfix, &pelement, &no_bytes, &arg) != END){
		pinfix += no_bytes;
		if (*ppostfixStart != USES_STRING) {
			switch (pelement->code) {
			case TO_STRING:
			case PRINTF:
			case SSCANF:
			case SLITERAL:
			case SUBRANGE:
			case REPLACE:
			case SFETCH:
			case A_SFETCH:
				*ppostfixStart = USES_STRING;
				break;
			default:
				break;
			}
		}

		switch (pelement->type){

	    case OPERAND:
			if (!operand_needed){
				*perror = 5;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operand to the expression */
			if (pelement->code == (char)FETCH) {
				/*
				 * Args A..L are required to exist, so we can code for an
				 * optimized fetch.  For args M..Z, we code a parameterized
				 * fetch; sCalcPerform() should check that the arg exists.
				 */
				if (arg < 12) {
					*ppostfix++ = FETCH_A + arg;
				} else {
					*ppostfix++ = FETCH;
					*ppostfix++ = arg;
				}
			} else {
				*ppostfix++ = pelement->code;
			}

			/* if this is a string variable reference, append variable number */
			if (pelement->code == (char)SFETCH) {
				*ppostfix++ = arg;
			}

			operand_needed = FALSE;
			new_expression = FALSE;
			break;

		case FLOAT_PT_CONST:
			if (!operand_needed){
				*perror = 5;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add constant to postfix expression */
			*ppostfix++ = pelement->code;
			pposthold = ppostfix;

			pinfix -= no_bytes;
			while (*pinfix == ' ') *ppostfix++ = *pinfix++;
			while (TRUE) {
				if ( ( *pinfix >= '0' && *pinfix <= '9' ) || *pinfix == '.' ) {
					*ppostfix++ = *pinfix;
					pinfix++;
				} else if ( *pinfix == 'E' || *pinfix == 'e' ) {
					*ppostfix++ = *pinfix;
					pinfix++;
						if (*pinfix == '+' || *pinfix == '-' ) {
							*ppostfix++ = *pinfix;
							pinfix++;
						}
				} else break;
			}
			*ppostfix++ = '\0';

			ppostfix = pposthold;
			if (sscanf(ppostfix,"%lg",&constant) != 1) {
				*ppostfix = '\0';
			} else {
				memcpy(ppostfix,(void *)&constant,8);
			}
			ppostfix+=8;

			operand_needed = FALSE;
			new_expression = FALSE;
			break;

		case STRING_CONST:
			if (!operand_needed){
				*perror = 5;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add string literal to the postfix expression */
			*ppostfix++ = pelement->code;
			c = pinfix[-1];
			while (*pinfix != c && *pinfix) *ppostfix++ = *pinfix++;
			*ppostfix++ = '\0';
			if (*pinfix) pinfix++;

			operand_needed = FALSE;
			new_expression = FALSE;
			break;

		case BINARY_OPERATOR:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators of higher or equal priority to	postfix expression */
			while ((pstacktop->in_stack_pri >= pelement->in_coming_pri)
					&& (pstacktop >= &stack[1])){
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* add new operator to stack */
			pstacktop++;
			*pstacktop = *pelement;

			operand_needed = TRUE;
			break;

		case UNARY_OPERATOR:
			if (!operand_needed){
				*perror = 5;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators of higher or equal priority to	postfix expression */
			while ((pstacktop->in_stack_pri >= pelement->in_coming_pri)
					&& (pstacktop >= &stack[1])){
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* add new operator to stack */
			pstacktop++;
			*pstacktop = *pelement;

			new_expression = FALSE;
			break;

		case VARG_FUNC:
			if (!operand_needed){
				*perror = 5;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators of higher or equal priority to	postfix expression */
			while ((pstacktop->in_stack_pri >= pelement->in_coming_pri)
					&& (pstacktop >= &stack[1])){
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* mark the end of the argument list */
			*ppostfix++ = VARG_TERM;

			/* add new operator to stack */
			pstacktop++;
			*pstacktop = *pelement;

			new_expression = FALSE;
			break;

		case MINUS_OPERATOR:
			if (operand_needed) {
				/* then assume minus was intended as a unary operator */
				in_coming_pri = UNARY_MINUS_I_C_P;
				in_stack_pri = UNARY_MINUS_I_S_P;
				code = UNARY_MINUS_CODE;
				new_expression = FALSE;
			} else {
				/* then assume minus was intended as a binary operator */
				in_coming_pri = BINARY_MINUS_I_C_P;
				in_stack_pri = BINARY_MINUS_I_S_P;
				code = BINARY_MINUS_CODE;
				operand_needed = TRUE;
			}

			/* add operators of higher or equal priority to	postfix expression */
			while ((pstacktop->in_stack_pri >= in_coming_pri)
					&& (pstacktop >= &stack[1])){
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* add new operator to stack */
			pstacktop++;
			*pstacktop = *pelement;
			pstacktop->in_stack_pri = in_stack_pri;
			pstacktop->code = code;

			break;

		case SEPARATOR:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators to postfix until open paren */
			while ((pstacktop->element[0] != '(') && (pstacktop->element[0] != '[')
					&& (pstacktop->element[0] != '{')) {
				if (pstacktop == &stack[1] || pstacktop == &stack[0]){
					*perror = 6;
					*ppostfixStart = BAD_EXPRESSION; return(-1);
				}
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}
			operand_needed = TRUE;
			break;

		case CLOSE_PAREN:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators to postfix until matching paren */
			while (pstacktop->element[0] != '(') {
				if (pstacktop == &stack[1] || pstacktop == &stack[0]) {
					*perror = 6;
					*ppostfixStart = BAD_EXPRESSION; return(-1);
				}
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}
			pstacktop--;	/* remove ( from stack */
			break;

		case CLOSE_BRACKET:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators to postfix until matching bracket */
			while (pstacktop->element[0] != '[') {
				if (pstacktop == &stack[1] || pstacktop == &stack[0]) {
					*perror = 6;
					*ppostfixStart = BAD_EXPRESSION; return(-1);
				}
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}
			/* add SUBRANGE operator to postfix */
			if (pstacktop == &stack[0]) {
				*perror = 6;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}
			*ppostfix++ = pstacktop->code;
			pstacktop--;
			break;

		case CLOSE_CURLY:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators to postfix until matching bracket */
			while (pstacktop->element[0] != '{') {
				if (pstacktop == &stack[1] || pstacktop == &stack[0]) {
					*perror = 6;
					*ppostfixStart = BAD_EXPRESSION; return(-1);
				}
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}
			/* add REPLACE operator to postfix */
			if (pstacktop == &stack[0]) {
				*perror = 6;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}
			*ppostfix++ = pstacktop->code;
			pstacktop--;
			break;

		case CONDITIONAL:
			if (operand_needed){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add operators of higher priority to postfix expression */
			while ((pstacktop->in_stack_pri > pelement->in_coming_pri)
					&& (pstacktop >= &stack[1])){
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* add new element to the postfix expression */
			*ppostfix++ = pelement->code;

			/* add : operator with COND_END code to stack */
			if (pelement->element[0] == ':'){
				pstacktop++;
				*pstacktop = *pelement;
				pstacktop->code = COND_END;
			}

			operand_needed = TRUE;
			break;

		case EXPR_TERM:
			if (operand_needed && !new_expression){
				*perror = 4;
				*ppostfixStart = BAD_EXPRESSION; return(-1);
			}

			/* add all operators on stack to postfix */
			while (pstacktop >= &stack[1]){
				if (pstacktop->element[0] == '('){
					*perror = 6;
					*ppostfixStart = BAD_EXPRESSION; return(-1);
				}
				*ppostfix++ = pstacktop->code;
				pstacktop--;
			}

			/* add new element to the postfix expression */
			*ppostfix++ = pelement->code;

			operand_needed = TRUE;
			new_expression = TRUE;
			break;

		default:
			*perror = 8;
			*ppostfixStart = BAD_EXPRESSION; return(-1);
		}
	}
	if (operand_needed){
		*perror = 4;
		*ppostfixStart = BAD_EXPRESSION; return(-1);
	}

	/* add all operators on stack to postfix */
	while (pstacktop >= &stack[1]){
		if (pstacktop->element[0] == '('){
			*perror = 6;
			*ppostfixStart = BAD_EXPRESSION; return(-1);
		}
		*ppostfix++ = pstacktop->code;
		pstacktop--;
	}
	*ppostfix++ = END_STACK;
	*ppostfix = '\0';

	if (ppostfixStart[1] == END_STACK)
		*ppostfixStart = BAD_EXPRESSION;
	else if (*ppostfixStart != USES_STRING)
		*ppostfixStart = NO_STRING;

#if DEBUG
	if (sCalcPostfixDebug) {
		printf("sCalcPostfix: buf-used=%d\n", (int)(1+ppostfix-ppostfixStart));
	}
#endif

	return(sCalcCheck(ppostfixStart, 0, 0));
}
