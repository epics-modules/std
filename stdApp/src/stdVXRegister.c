/* stdRegister.c */

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "iocsh.h"
#include "epicsExport.h"

extern int fastPIDDebug;
static const iocshArg fastPIDDebugArg0 = { "debugLevel",iocshArgInt};
static const iocshArg * const fastPIDDebugArgs[1] = {&fastPIDDebugArg0};
static const iocshFuncDef fastPIDDebugFuncDef = {"setFastPIDDebug",1,fastPIDDebugArgs};
static void fastPIDDebugCallFunc(const iocshArgBuf *args)
{
    fastPIDDebug = (int)args[0].sval;
}

void stdRegister(void)
{
    iocshRegister(&fastPIDDebugFuncDef,fastPIDDebugCallFunc);
}

epicsExportRegistrar(stdRegister);
