#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ellLib.h>
#define epicsTypesGLOBAL 1
#include <epicsTypes.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsMutex.h>

#include "symTable.h"

static epicsMutexId mutexID=NULL;
static ELLLIST *symList=NULL;

typedef struct SYMNODE {
   struct SYMNODE *next;
   struct SYMNODE *previous;
   const char *name;
   void *address;
   epicsType type;
} SYMNODE;

int addSymbol(const char *name, void *address, epicsType type)
{
    SYMNODE *node;
    if (symList == NULL) {
        symList = (ELLLIST *)calloc(1, sizeof(ELLLIST));
        ellInit(symList);
    }
    if (mutexID == NULL) mutexID = epicsMutexCreate();
    if (name == NULL) {
        printf("setSymbol: NULL name pointer\n");
        return(-1);
    }
    if (address == NULL) {
        printf("setSymbol: NULL address\n");
        return(-1);
    }
    if (invalidEpicsType(type)) {
        printf("setSymbol: unsupported data type\n");
        return(-1);
    }
    epicsMutexLock(mutexID);
    node = calloc(1, sizeof(SYMNODE));
    node->name = name;
    node->address = address;
    node->type = type;
    ellAdd(symList, (ELLNODE *)node);
    epicsMutexUnlock(mutexID);
    return(0);
}

int setSymbol(const char *name, const char *pValue)
{
    SYMNODE *node;
    epicsInt32   ival=0;
    epicsUInt32  uval=0;
    epicsFloat64 dval=0.;
    char *pstring=NULL;
    char *endptr = NULL;
    int status = -1;
    if (symList == NULL) {
       printf("setSymbol: no symbol table\n");
       return(-1);
    }
    if (name == NULL) {
       printf("usage: setSymbol symbol value\n");
       return(-1);
    }
    if (pValue == NULL) {
       printf("usage: setSymbol symbol value\n");
       return(-1);
    }
    epicsMutexLock(mutexID);
    for (node=(SYMNODE *)ellFirst(symList); node!=NULL; node=(SYMNODE *)ellNext(node)) {
        if(strcmp(name, node->name) == 0) {
            /* Convert from string to number */
            switch(node->type) {
            case epicsInt8T:
            case epicsInt16T:
            case epicsInt32T:
               ival = strtol(pValue, &endptr, 0);
               break;
            case epicsUInt8T:
            case epicsUInt16T:
            case epicsEnum16T:
            case epicsUInt32T:
               uval = strtoul(pValue, &endptr, 0);
               break;
            case epicsFloat32T:
            case epicsFloat64T:
               dval = strtod(pValue, &endptr);
               break;
            case epicsStringT:
            case epicsOldStringT:
               pstring = calloc(strlen((char *)pValue)+1, 1);
               strcpy(pstring, (char *)pValue);
               break;
            }
            /* Check that a valid number was entered */
            if (endptr == pValue) {
               printf("setSymbol: invalid number\n");
               status = -2;
               break;
            }
            status = 0;
            /* Copy the value */
            switch(node->type) {
            case epicsInt8T:
                *(epicsInt8 *)node->address = ival;
                break;
            case epicsInt16T:
                *(epicsInt16 *)node->address = ival;
                break;
            case epicsInt32T:
                *(epicsInt32 *)node->address = ival;
                break;
            case epicsUInt8T:
                *(epicsUInt8 *)node->address = uval;
                break;
            case epicsUInt16T:
                *(epicsUInt16 *)node->address = uval;
                break;
            case epicsEnum16T:
                *(epicsEnum16 *)node->address = uval;
                break;
            case epicsUInt32T:
                *(epicsUInt32 *)node->address = uval;
                break;
            case epicsFloat32T:
                *(epicsFloat32 *)node->address = dval;
                break;
            case epicsFloat64T:
                *(epicsFloat64 *)node->address = dval;
                break;
            case epicsStringT:
                ((epicsString *)node->address)->pString = pstring;
                break;
            case epicsOldStringT:
                (char *)node->address = pstring;
                break;
            }
        }
    }
    if (status == -1) printf("setSymbol: no such symbol: %s\n", name);
    epicsMutexUnlock(mutexID);
    return(status);
}

int showSymbol(const char *pattern)
{
    SYMNODE *node;
    void *pval;
    const char *tname;
    epicsInt32   ival=0;
    epicsUInt32  uval=0;
    epicsFloat64 dval=0.;
    char *pstring=NULL;
    if (symList == NULL) return(-1);
 
    printf("%20s %10s %15s %12s\n", "Symbol","Address","Type","Value"); 
    printf("%20s %10s %15s %12s\n", "------","-------","----","-----"); 
    for (node=(SYMNODE *)ellFirst(symList); node!=NULL; node=(SYMNODE *)ellNext(node)) {
        if ((pattern != NULL) && (strstr(node->name, pattern) == NULL)) continue;
        printf("%20s %10p ", node->name, node->address);
        tname = epicsTypeNames[node->type];
        pval = node->address;
        /* Get the value */
        switch(node->type) {
        case epicsInt8T:
            ival = *(epicsInt8 *)node->address;
            break;
        case epicsInt16T:
            ival = *(epicsInt16 *)node->address;
            break;
        case epicsInt32T:
            ival = *(epicsInt32 *)node->address;
            break;
        case epicsUInt8T:
            uval = *(epicsUInt8 *)node->address;
            break;
        case epicsUInt16T:
            uval = *(epicsUInt16 *)node->address;
            break;
        case epicsEnum16T:
            uval = *(epicsEnum16 *)node->address;
            break;
        case epicsUInt32T:
            uval = *(epicsUInt32 *)node->address;
            break;
        case epicsFloat32T:
            dval = *(epicsFloat32 *)node->address;
            break;
        case epicsFloat64T:
            dval = *(epicsFloat64 *)node->address;
            break;
        case epicsStringT:
            pstring = ((epicsString *)node->address)->pString;
            break;
        case epicsOldStringT:
            pstring = (char *)node->address;
            break;
        }

        /* Print the string */
        switch(node->type) {
        case epicsInt8T:
        case epicsInt16T:
        case epicsInt32T:
           printf("%15s %12d (0x%x)\n", tname, ival, ival); 
           break;
        case epicsUInt8T:
        case epicsUInt16T:
        case epicsEnum16T:
        case epicsUInt32T:
           printf("%15s %12u (0x%x)\n", tname, uval, uval); 
           break;
        case epicsFloat32T:
        case epicsFloat64T:
           printf("%15s %12g\n", tname, dval);
           break;
        case epicsStringT:
        case epicsOldStringT:
           printf("%15s %s\n", tname, pstring);
           break;
        }
    }
    return(0);
}

static const iocshArg setSymbolArg0 = { "Symbol name",iocshArgString};
static const iocshArg setSymbolArg1 = { "Value",iocshArgString};
static const iocshArg * const setSymbolArgs[2] = {&setSymbolArg0, &setSymbolArg1};
static const iocshFuncDef setSymbolFuncDef = {"setSymbol",2,setSymbolArgs};
static void setSymbolCallFunc(const iocshArgBuf *args)
{
    setSymbol(args[0].sval, args[1].sval);
}

static const iocshArg showSymbolArg0 = { "Symbol name",iocshArgString};
static const iocshArg * const showSymbolArgs[1] = {&showSymbolArg0};
static const iocshFuncDef showSymbolFuncDef = {"showSymbol",1,showSymbolArgs};
static void showSymbolCallFunc(const iocshArgBuf *args)
{
    showSymbol(args[0].sval);
}

void symTableRegister(void)
{
    iocshRegister(&setSymbolFuncDef,setSymbolCallFunc);
    iocshRegister(&showSymbolFuncDef,showSymbolCallFunc);
}

epicsExportRegistrar(symTableRegister);

