/* initHooks.c	ioc initialization hooks */ 
/* share/src/db @(#)initHooks.c	1.5     7/11/94 */
/*
 *      Author:		Marty Kraimer
 *      Date:		06-01-91
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
 * .01  09-05-92	rcz	initial version
 * .02  09-10-92	rcz	changed return from void to long
 * .03  09-10-92	rcz	changed completely
 * .04  09-10-92	rcz	bug - moved call to setMasterTimeToSelf later
 *
 */


#include	<vxWorks.h>
#include	<initHooks.h>


/*
 * INITHOOKS
 *
 * called by iocInit at various points during initialization
 *
 */


/* If this function (initHooks) is loaded, iocInit calls this function
 * at certain defined points during IOC initialization */
void initHooks(initHookState state)
{
	switch (state) {
	case INITHOOKatBeginning :
		printf("INITHOOKatBeginning\n");
	    break;
	case INITHOOKafterGetResources :
		printf("INITHOOKafterGetResources\n");
	    break;
	case INITHOOKafterLogInit :
		printf("INITHOOKafterLogIni\n");
	    break;
	case INITHOOKafterCallbackInit :
		printf("INITHOOKafterCallbackInit\n");
	    break;
	case INITHOOKafterCaLinkInit :
		printf("INITHOOKafterCaLinkInit\n");
	    break;
	case INITHOOKafterInitDrvSup :
		printf("INITHOOKafterInitDrvSup\n");
	    break;
	case INITHOOKafterInitRecSup :
		printf("INITHOOKafterInitRecSup\n");
	    break;
	case INITHOOKafterInitDevSup :
		printf("INITHOOKafterInitDevSup\n");
	    break;
	case INITHOOKafterTS_init :
		printf("INITHOOKafterTS_init\n");
		/*
		 * restore fields that require init_record() to send a value
		 * down to the hardware.
		 */
#if 1
		printf("*** restoring from 'auto_positions.sav' (pass 0) ***\n");
		reboot_restore("auto_positions.sav", state, 0);
		printf("*** restoring from 'auto_settings.sav' (pass 0) ***\n");
		reboot_restore("auto_settings.sav", state, 0);
#endif
	    break;
	case INITHOOKafterInitDatabase :
		printf("INITHOOKafterInitDatabase\n");
		/*
		 * restore fields that init_record() would have overwritten with
		 * info from the dol (desired output location).
		 */ 
#if 1
		printf("*** restoring from 'auto_settings.sav' (pass 1) ***\n");
		reboot_restore("auto_settings.sav", state, 1);
#endif
	    break;
	case INITHOOKafterFinishDevSup :
		printf("INITHOOKafterFinishDevSup\n");
	    break;
	case INITHOOKafterScanInit :
		printf("INITHOOKafterScanInit\n");
	    break;
	case INITHOOKafterInterruptAccept :
		printf("INITHOOKafterInterruptAccept\n");
	    break;
	case INITHOOKafterInitialProcess :
		printf("INITHOOKafterInitialProcess\n");
	    break;
	case INITHOOKatEnd :
		printf("INITHOOKatEnd\n");
	    break;
	default:
		printf("INITHOOK unknown state\n");
	    break;
	}
	return;
}
