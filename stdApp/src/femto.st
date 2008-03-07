/*  femto - Femto Amplifier Control
    Calling seqeunce:
        seq &femto,"name=,P=,H=,F=G1=,G2=,G3=,NO="
 
    Where:
        P  = prefix of database and sequencer
        H  = hardware (i.e. fem01)
        F  = functionality (i.e. seq01)
        G1 = PV name for gain bit 0
        G2 = PV name for gain bit 1
        G3 = PV name for gain bit 2
        NE = PV name for noise ctl bit

 */
program femto("name=femto,P=,H=,F=,G1=,G2=,G3=,NO=")

option +d;
option -c;
option +r;
option -w;

%%#include <stdio.h>
%%#include <math.h>
#include "seqPVmacros.h"

#define MIN_GAIN      0
#define MAX_GAIN      15
#define UNUSED_GAIN   7

PV(int,t0,"{G1}",EvFlag);
PV(int,t1,"{G2}",EvFlag);
PV(int,t2,"{G3}",EvFlag);
PV(int,tx,"{NO}",EvFlag);
PV(double,gain,"{P}{H}{F}gain",NoMon);
PV(int,gainidx,"{P}{H}{F}gainidx",EvFlag);
PV(int,debug_flag,"{P}{H}{F}debug",EvFlag);

int curgain;
char msg[256];
char* SNLtaskName;

%%static int powers[] = {5,6,7,8,9,10,11,0,3,4,5,6,7,8,9,0};

ss femto
{

    state init
    {
        when()
        {
            SNLtaskName = macValueGet("name");

            pvGet(t0,SYNC);
            pvGet(t1,SYNC);
            pvGet(t2,SYNC);
            pvGet(tx,SYNC);

            if( (tx+t2+t1+t0) == 0 )
            {
                gainidx = 8;   /* most common undefined case 1e3 */
                DEBUG_PRINT(2,"gainidx = default (1e3 since all bits were OFF)");
            }
            else
            {
               gainidx = (tx << 3) | (t2 << 2) | (t1 << 1) | t0;
               sprintf(msg,"gainidx = %d",gainidx);
               DEBUG_PRINT(2,msg);
            }

            if( gainidx >= MAX_GAIN || gainidx == UNUSED_GAIN )
            {
                gainidx = 6;
                DEBUG_PRINT(4,"resetting gain to 1e11");
            }

            curgain = -1;
            PVPUT(gainidx,gainidx);
            PVPUT(gain,pow(10,powers[gainidx]));
            sprintf(msg,"gainidx = %d",gainidx);
            DEBUG_PRINT(2,msg);

            efClear(t0_mon);
            efClear(t1_mon);
            efClear(t2_mon);
            efClear(tx_mon);
            efClear(gainidx_mon);
            efClear(debug_flag_mon);

            DEBUG_PRINT(2,"init complete");
        } state changeGain

    }

    state updateGain
    {
        when()
        {
            gainidx = (tx << 3) | (t2 << 2) | (t1 << 1) | t0;
            sprintf(msg,"gainidx = %d",gainidx);
            DEBUG_PRINT(2,msg);

            if( gainidx >= MAX_GAIN || gainidx == UNUSED_GAIN )
            {
                gainidx = 6;
                DEBUG_PRINT(4,"resetting gain to 1e11");
            }

            curgain = gainidx;
            PVPUT(gainidx,gainidx);
            PVPUT(gain,pow(10,powers[gainidx]));
            sprintf(msg,"gainidx = %d",gainidx);
            DEBUG_PRINT(2,msg);

        } state idle

    }

    state changeGain
    {
        when( curgain == gainidx )
        {
            DEBUG_PRINT(2,"No gain change required");
        } state idle

        when( gainidx < MIN_GAIN )
        {
            PVPUT(gainidx,curgain);
            PVPUT(gain,pow(10,powers[curgain]));
            sprintf(msg,"Requested gain (%d) too low!",gainidx);
            DEBUG_PRINT(2,msg);
        } state idle

        when( gainidx > MAX_GAIN )
        {
            PVPUT(gainidx,curgain);
            PVPUT(gain,pow(10,powers[curgain]));
            sprintf(msg,"Requested gain (%d) too high!",gainidx);
            DEBUG_PRINT(2,msg);
        } state idle

        when( gainidx == UNUSED_GAIN || gainidx == MAX_GAIN )
        {
            PVPUT(gainidx,curgain);
            PVPUT(gain,pow(10,powers[curgain]));
            sprintf(msg,"Requested gain (%d) invalid!",gainidx);
            DEBUG_PRINT(2,msg);
        } state idle

        when()
        {
            sprintf(msg,"gain %d selected",gainidx);
            DEBUG_PRINT(2,msg);

            /* 1e5 V/A gain, high speed */
            if( gainidx == 0 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 0);
                PVPUT(t2, 0);
                PVPUT(tx, 0);
            }

            /* 1e6 V/A gain, high speed */
            if( gainidx == 1 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 0);
                PVPUT(t2, 0);
                PVPUT(tx, 0);
            }

            /* 1e7 V/A gain, high speed */
            if( gainidx == 2 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 1);
                PVPUT(t2, 0);
                PVPUT(tx, 0);
            }

            /* 1e8 V/A gain, high speed */
            if( gainidx == 3 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 1);
                PVPUT(t2, 0);
                PVPUT(tx, 0);
            }

            /* 1e9 V/A gain, high speed */
            if( gainidx == 4 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 0);
                PVPUT(t2, 1);
                PVPUT(tx, 0);
            }

            /* 1e10 V/A gain, high speed */
            if( gainidx == 5 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 0);
                PVPUT(t2, 1);
                PVPUT(tx, 0);
            }

            /* 1e11 V/A gain, high speed */
            if( gainidx == 6 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 1);
                PVPUT(t2, 1);
                PVPUT(tx, 0);
            }

            /* 1e3 V/A gain, low noise */
            if( gainidx == 8 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 0);
                PVPUT(t2, 0);
                PVPUT(tx, 1);
            }

            /* 1e4 V/A gain, low noise */
            if( gainidx == 9 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 0);
                PVPUT(t2, 0);
                PVPUT(tx, 1);
            }

            /* 1e5 V/A gain, low noise */
            if( gainidx == 10 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 1);
                PVPUT(t2, 0);
                PVPUT(tx, 1);
            }

            /* 1e6 V/A gain, low noise */
            if( gainidx == 11 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 1);
                PVPUT(t2, 0);
                PVPUT(tx, 1);
            }

            /* 1e7 V/A gain, low noise */
            if( gainidx == 12 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 0);
                PVPUT(t2, 1);
                PVPUT(tx, 1);
            }

            /* 1e8 V/A gain, low noise */
            if( gainidx == 13 )
            {
                PVPUT(t0, 1);
                PVPUT(t1, 0);
                PVPUT(t2, 1);
                PVPUT(tx, 1);
            }

            /* 1e9 V/A gain, low noise */
            if( gainidx == 14 )
            {
                PVPUT(t0, 0);
                PVPUT(t1, 1);
                PVPUT(t2, 1);
                PVPUT(tx, 1);
            }

            curgain = gainidx;
            PVPUT(gain,pow(10,powers[gainidx]));

            efClear(t0_mon);
            efClear(t1_mon);
            efClear(t2_mon);
            efClear(tx_mon);
            efClear(gainidx_mon);

        } state idle

    }

    state idle
    {
        when( efTestAndClear(debug_flag_mon) )
        {
            sprintf(msg,"Debug level changed to %d",debug_flag);
            DEBUG_PRINT(1,msg);
        } state idle

        when( efTestAndClear(tx_mon) || efTestAndClear(t2_mon) ||
              efTestAndClear(t1_mon) || efTestAndClear(t0_mon)
            )
        {
            DEBUG_PRINT(2,"bits changed, updating");
        } state updateGain

        when( efTestAndClear(gainidx_mon) )
        {
            DEBUG_PRINT(2,"gain change");
        } state changeGain

        when( delay(.1) ) {} state idle
    }

}
