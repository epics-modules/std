/* getFilledBuckets.st				F.Lenkszus */
/* quick and dirty sequence program to load Bunch Clock generator 
until I get time to change the device support */

program getFillPat

/* defines */
#define NOTINJECTING 0
#define INJECTING 1
#define CAERROR 15
#define NOBEAM 0
#define STOREDBEAM 1


/* compile options flags */
 
/* Process Variables */

string message;
assign message to "{unit}BnchClkGenMessageSI";
 
short  clearMessage;
assign clearMessage to "{unit}BnchClkGenClearMessBO";
monitor clearMessage;

short loadPat;
assign loadPat to "{unit}BnchClkGenLoadPatBO";
monitor loadPat;

short  FilledBuckets[1296];
assign FilledBuckets to "Mt:S:FilledBucketsWF.VAL" ;

short  BunchPattern[1296];
assign BunchPattern to "Mt:S:FillPatternWF.VAL" ;

short LocalCopy[1296];
assign LocalCopy to "{unit}BnchClkGenListWF";

short BCGPattern;
assign BCGPattern to "{unit}BnchClkGenPatternWF.PROC";

short BCGBucketList;
assign BCGBucketList to "{unit}BnchClkGenBucketsWF.PROC";

short   NumBuckets;
assign  NumBuckets to "Mt:S:NumBucketsFilledAI" ;

short setBucket;
assign setBucket to "{unit}BnchClkGenSetBucketAO.VAL";

short clearAll;
assign clearAll to "{unit}BnchClkGenClearPatBO.VAL";

short autoLoad;
assign autoLoad to "{unit}BnchClkGenAutoLoadBO";
monitor autoLoad;

short srBeamStatus;
assign srBeamStatus to "S:BeamStatusMBBI.VAL";
monitor srBeamStatus;

short   srInjectStatus;
assign srInjectStatus to  "S:InjectingStatusMBBI";
monitor srInjectStatus;

/* variables */

%%#include <vxWorks.h>
%%#include <stddef.h>
%%#include <stdlib.h>
%%#include <stdio.h>
%%#include <string.h>
%%#include <math.h>
%%#include <taskLib.h>
%%#include <sysLib.h>

%% int getFilledBucketsDebug = 0;
%% int getFilledBucketsLDebug = 0;

int i;
int running = 0;

ss getFillPattern {

        state init {
                when (pvConnectCount() == pvChannelCount()) {
%%	                strcpy(message, "Waiting");
                        pvPut(message);
			running = 1;
		}state waitCommand

                when(delay(2.0)) {
			running = 0;
%%	                strcpy(message, "CA Error");
                        pvPut(message);
                } state init
        }

	state waitCommand {
                when(pvConnectCount() != pvChannelCount()) {
			running = 0;
                } state init
                when(clearMessage == 1) {
                        clearMessage = 0;
                        pvPut(clearMessage);
%%                      strcpy(message, "Message Cleared");
                        pvPut(message);
                } state waitCommand 
		when(loadPat == 1) {
			loadPat = 0;
			pvPut(loadPat);
%%			strcpy(message, "Loading Pattern");
			pvPut(message);
			clearAll = 1;
			pvPut(clearAll);	/* clear the pattern */
			pvGet(NumBuckets);
%%                        if(getFilledBucketsDebug) {	
%%				printf("NumBuckets = %d\n", NumBuckets);
%%			}
			/* get the pattern */
			pvGet(LocalCopy);
			for(i=0; i < NumBuckets; i++) {
				setBucket = LocalCopy[i];
				pvPut(setBucket);
%%				if(getFilledBucketsDebug) {
%%					printf("%d\t%d\n", i,  LocalCopy[i]);
%%				}
			}	
			/* update waveforms */
			BCGPattern = 1;
			pvPut(BCGPattern);
			BCGBucketList = 1;
			pvPut(BCGBucketList);
%%			strcpy(message, "Pattern Loaded");
			pvPut(message);
		} state waitCommand
	} 


}

ss autoLoad {
		state idle {
			when((running == 1) && (autoLoad == 1)) {
%%				if(getFilledBucketsLDebug) {
%%					printf("autoLoad: State NoBeam\n");
%%				}
			} state NoBeam
		}
		state NoBeam{
			when((running != 1) || (autoLoad != 1)) {
%%				if(getFilledBucketsLDebug) {
%%					printf("autoLoad: State idle\n");
%%				}
			} state idle
			when((srInjectStatus ==  NOTINJECTING) &&
				(srBeamStatus) == STOREDBEAM) {
				taskDelay(sysClkRateGet());  /* 1 sec delay */
				loadPat = 1;
				pvPut(loadPat);
%%				if(getFilledBucketsLDebug) {
%%				   printf("autoLoad: State Pattern Loaded\n");
%%				}
			} state PatternLoaded
		}
		state PatternLoaded {
		        when((running != 1) || (autoLoad != 1)) {
%%				if(getFilledBucketsLDebug) {
%%				   printf("autoLoad: State idle\n");
%%				}
                        } state idle
			when(srInjectStatus == INJECTING) {
%%				if(getFilledBucketsLDebug) {
%%				   printf("autoLoad: State NoBeam\n");
%%				}
			} state NoBeam
			when(srBeamStatus != STOREDBEAM) {
%%				if(getFilledBucketsLDebug) {
%%				   printf("autoLoad: State NoBeam\n");
%%				}
			} state NoBeam
		}


}
/* native C code */

%{

}%

