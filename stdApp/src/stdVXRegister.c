/* stdRegister.c */

#include <iocsh.h>
#include <epicsExport.h>

/* This is where any registration calls for vxWorks only code that are not in 
   their respective source code modules should be placed
 */
void stdVXRegister(void)
{
}

epicsExportRegistrar(stdVXRegister);


