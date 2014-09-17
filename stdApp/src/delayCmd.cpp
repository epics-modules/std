#include <string>
#include <cstdlib>
#include <cstring>
#include <string.h>

#include "iocsh.h"
#include "epicsExport.h"
#include "initHooks.h"
#include "ellLib.h"
#include "cantProceed.h"
#include "dbDefs.h"

static ELLLIST allcmds = ELLLIST_INIT;
static bool registered = false;

struct cmdNode
{
	ELLNODE node;
	char* cmd;
};

static void runDelayed (initHookState state)
{
	if (state != initHookAfterIocRunning)    { return; }

	ELLNODE* current;

	for ( current = ellFirst(&allcmds); current; current = ellNext(current) )
	{
		struct cmdNode* node = CONTAINER(current, struct cmdNode, node);

		printf("%s\n", node->cmd);
		iocshCmd(node->cmd);
        
        free(node->cmd);
	}
    
    ellFree(&allcmds);
}


static int delayCmd (const char* cmd)
{
	if (! registered)
	{
		initHookRegister(&runDelayed);
		registered = true;
	}

	struct cmdNode* nextcmd;

	nextcmd = (struct cmdNode*) callocMustSucceed(1, sizeof(*nextcmd), "cmdnode");
	nextcmd->cmd = (char*) callocMustSucceed(strlen(cmd), sizeof(char), "cmdstring");

	strcpy(nextcmd->cmd, cmd);

	ellAdd(&allcmds, &nextcmd->node);

	return 0;
}


extern "C"
{
	static const iocshArg delay_arg            = { "commandString", iocshArgString };
	static const iocshArg* const delay_args[1] = { &delay_arg };
	static const iocshFuncDef delay_func       = {"delayCmd", 1, delay_args};


	static void call_delay_func(const iocshArgBuf* args) { delayCmd(args[0].sval); }
	static void delayCmdRegistrar(void)                  { iocshRegister(&delay_func, call_delay_func); }

	epicsExportRegistrar(delayCmdRegistrar);
}
