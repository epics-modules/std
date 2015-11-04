#include <vxWorks.h>
#include <symLib.h>
#include <sysSymTbl.h>

#include <iocsh.h>
#include <epicsExport.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>

 /*
 * VxWorks 6 defines SYM_IS_TEXT in symbol.h, but earlier versions do not, so
 * define it if it is not already defined.
 */
#if !defined(SYM_IS_TEXT)
#define SYM_IS_TEXT(symType)	((symType) & SYM_TEXT)
#endif

/*
 * We are going to use symEach and need to return information about the number
 * and type of symbols found. The necessary header to use symEach doesn't allow
 * us to return that information as an output, so we'll need to use LOCAL vars.
 */
LOCAL unsigned int matchcnt;
LOCAL SYMBOL* matchsym;

LOCAL BOOL symTblFindPartial(char* name, int val, INT8 type, char* substr, UINT16 group, SYMBOL* pSymbol)
{
	if (name == NULL || substr == NULL)    { return FALSE; }
	
	int subStrLen = strlen(substr);
	int ntries = strlen(name) - subStrLen;
	
	for (; ntries >= 0; ntries -= 1)
	{
		if (strncmp(name, substr, subStrLen) == 0)
		{
			// We only want to match symbols that contain executable instructions
			if (SYM_IS_TEXT(pSymbol->type))
			{
				matchcnt += 1;
				matchsym = pSymbol;
				return TRUE;
			}
		}
	}
	
	return TRUE;
}

STATUS symPartialMatch(SYMTAB_ID symTblId, char* name, char** pValue, SYM_TYPE* pType)
{
	matchcnt = 0;
	matchsym = NULL;
	
	symEach( symTblId, (FUNCPTR) symTblFindPartial, (int) name);
	
	// Only proceed with a uniquely identified function
	if (matchcnt != 1)     { return ERROR; }
	
	if (pValue != NULL)    { *pValue = (char *) matchsym->value; }
	if (pType != NULL)     { *pType = matchsym->type; }
	
	return OK;
}

extern "C"
{	
	static const iocshArg vx_arg0       = {"functionName",    iocshArgString};
	static const iocshArg vx_arg1       = {"[arg1, ...]",     iocshArgArgv};
	
	static const iocshArg* vx_args[]    = {&vx_arg0, &vx_arg1};
	
	static const iocshFuncDef vx_func   = {"vxCall", 2, vx_args};
	
	static void call_vx_func(const iocshArgBuf* args)
	{
		// Maximum number of arguments vxshell supports is 10
		long send_args[10];
		
		for (int index = 0; (index < 10 && index < args[1].aval.ac); index += 1)
		{
			// Ignore any arguments that were left out
			if (args[1].aval.av[index + 1] == NULL)    { send_args[index] = 0; }
			else
			{
				// Default to sending a pointer to the string
				send_args[index] = (long) args[1].aval.av[index + 1];
				
				// Attempt to parse as a long
				char* end_parse;
				long val = strtol((char*) send_args[index], &end_parse, 0);
				
				// If end_parse is NIL, the parse was successful.
				if (*end_parse == '\0')    { send_args[index] = val; }
			}
		}
		
		char*    symbol_value;
		SYM_TYPE symbol_type;
		
		if (OK != symFindByName(sysSymTbl, args[0].sval, &symbol_value, &symbol_type) ||
			OK != symPartialMatch(sysSymTbl, args[0].sval, &symbol_value, &symbol_type))
		{
			printf("No unique symbol found.\n");
			return;
		}		
		
		FUNCPTR address = (FUNCPTR) symbol_value;
		
		int retval = (*address) (send_args[0], 
		                         send_args[1], 
		                         send_args[2], 
		                         send_args[3], 
		                         send_args[4], 
		                         send_args[5], 
		                         send_args[6], 
		                         send_args[7],
		                         send_args[8],
		                         send_args[9]);
		
		// Emulate the vxworks shell's printing of the return value
		printf("value = %d = 0x%x\n", retval, (unsigned int) retval);
	}
	
	static void vxcallRegistrar(void) { iocshRegister(&vx_func, call_vx_func); }
	
	epicsExportRegistrar(vxcallRegistrar);
}
