/* calcPostfix.h
 *      Author:          Bob Dalesio
 *      Date:            9-21-88
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
 * .01  01-11-89        lrd     add right and left shift
 * .02  02-01-89        lrd     add trig functions
 * .03  02-17-92        jba     add exp, CEIL, and FLOOR
 * .04  03-03-92        jba     added MAX, MIN, and comma
 * .05  03-06-92        jba     added multiple conditional expressions ?
 * .06  04-02-92        jba     added CONSTANT for floating pt constants in expression
 * .07  05-11-94        jba     added CONST_PI, CONST_D2R, and CONST_R2D
 */

#ifndef INCpostfixh
#define INCpostfixh

/*	defines for element table      */
#define		BAD_EXPRESSION	0
#define 	FETCH		1
#define		ACOS		2
#define		ASIN		3
#define		ATAN		4
#define		COS		5
#define		COSH		6
#define		SIN		7
#define		RIGHT_SHIFT	8
#define		LEFT_SHIFT	9
#define		SINH		10
#define		TAN		11
#define		TANH		12
#define		LOG_2		13
#define		COND_ELSE	14
#define		ABS_VAL		15
#define		UNARY_NEG	16
#define		SQU_RT		17
#define		EXP		18
#define		CEIL		19
#define		FLOOR		20
#define		LOG_10		21
#define		LOG_E		22
#define		RANDOM		23
#define		ADD		24
#define		SUB		25
#define		MULT		26
#define		DIV		27
#define		EXPON		28
#define		MODULO		29
#define		BIT_OR		30
#define		BIT_AND		31
#define		BIT_EXCL_OR	32
#define		GR_OR_EQ	33
#define		GR_THAN		34
#define		LESS_OR_EQ	35
#define		LESS_THAN	36
#define		NOT_EQ		37
#define		EQUAL		38
#define		REL_OR		39
#define		REL_AND		40
#define		REL_NOT		41
#define		BIT_NOT		42
#define		PAREN		43
#define		MAX		44
#define		MIN		45
#define		COMMA		46
#define		COND_IF		47
#define		COND_END	48
#define		LITERAL		49
#define		CONST_PI	50
#define		CONST_D2R	51
#define		CONST_R2D	52
#define		NINT		53
#define		ATAN2		54
#define		CONST_S2R	55
#define		CONST_R2S	56
#define		END_STACK	127

#endif /* INCpostfixh */
