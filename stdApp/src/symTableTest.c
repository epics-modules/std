/* symTableTest.c */

#include <string.h>
#include "epicsExport.h"
#include "epicsTypes.h"
#include "symTable.h"

static epicsInt8      testInt8=0;
static epicsInt16     testInt16=1;
static epicsInt32     testInt32=2;
static epicsUInt8     testUInt8=3;
static epicsUInt16    testUInt16=4;
static epicsEnum16    testEnum16=5;
static epicsUInt32    testUInt32=6;
static epicsFloat32   testFloat32=7;
static epicsFloat64   testFloat64=8;
static epicsString    testNewString;
static epicsOldString testOldString="This is an old EPICS string";
static char*          nstring="This is a new EPICS string";

void symTableTestRegister(void)
{
    testNewString.pString = nstring;
    testNewString.length  = strlen(testNewString.pString);
    addSymbol("testInt8",      &testInt8,      epicsInt8T);
    addSymbol("testInt16",     &testInt16,     epicsInt16T);
    addSymbol("testInt32",     &testInt32,     epicsInt32T);
    addSymbol("testUInt8",     &testUInt8,     epicsUInt8T);
    addSymbol("testUInt16",    &testUInt16,    epicsUInt16T);
    addSymbol("testEnum16",    &testEnum16,    epicsEnum16T);
    addSymbol("testUInt32",    &testUInt32,    epicsUInt32T);
    addSymbol("testFloat32",   &testFloat32,   epicsFloat32T);
    addSymbol("testFloat64",   &testFloat64,   epicsFloat64T);
    addSymbol("testNewString", &testNewString, epicsStringT);
    addSymbol("testOldString", testOldString,  epicsOldStringT);
}

epicsExportRegistrar(symTableTestRegister);
