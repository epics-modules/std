#include <module_types.h>

module_types()
{
ai_addrs[XY566SE]		= 0x6000;
ai_num_cards[XY566SE] = 0;

ai_addrs[XY566DI]		= 0x7000;
ai_num_cards[XY566DI] = 0;

ai_addrs[XY566DIL]		= 0xe000;
ai_num_cards[XY566DIL] = 0;

ai_addrs[VXI_AT5_AI]		= 0xc014;
ai_num_cards[VXI_AT5_AI] = 32;

ai_addrs[DVX2502]		= 0xff00;
ai_num_cards[DVX2502] = 0;


ao_addrs[VMI4100]		= 0x4100;
ao_num_cards[VMI4100] = 0;

ao_addrs[ZIO085]		= 0x0800;
ao_num_cards[ZIO085] = 0;

ao_addrs[VXI_AT5_AO]		= 0xc000;
ao_num_cards[VXI_AT5_AO] = 32;

bi_addrs[BB910]		= 0xb800;
bi_num_cards[BB910] = 0;

bi_addrs[XY210]		= 0xa000;
bi_num_cards[XY210] = 0;

bi_addrs[VXI_AT5_BI]	= 0xc000;
bi_num_cards[VXI_AT5_BI] = 32;

bi_addrs[HPE1368A_BI]	= 0xc000;
bi_num_cards[HPE1368A_BI] = 32;

bi_addrs[AT8_FP10S_BI]	= 0x0e00;
bi_num_cards[AT8_FP10S_BI] = 0;

bi_addrs[XY240_BI]	= 0xd000;
bi_num_cards[XY240_BI] = 2;


bo_addrs[BB902]		= 0xd800;
bo_num_cards[BB902] = 0;

bo_addrs[XY220]		= 0xc800;
bo_num_cards[XY220] = 0;

bo_addrs[VXI_AT5_BO]	= 0xc000;
bo_num_cards[VXI_AT5_BO] = 32;

bo_addrs[HPE1368A_BO]	= 0xc000;
bo_num_cards[HPE1368A_BO] = 32;

bo_addrs[AT8_FP10M_BO]	= 0x0c00;
bo_num_cards[AT8_FP10M_BO] = 0;

bo_addrs[XY240_BO]	= 0xd000;
bo_num_cards[XY240_BO] = 2;


sm_addrs[CM57_83E]	= 0x8000;
sm_num_cards[CM57_83E] = 0;

sm_addrs[OMS_6AXIS]	= 0xfc00;
sm_num_cards[OMS_6AXIS] = 8;

wf_addrs[XY566WF]	= 0x9000;
wf_armaddrs[XY566WF]	= 0x5400;
wf_num_cards[XY566WF] = 0;

wf_addrs[JGVTR1]	= 0xB000;
wf_armaddrs[JGVTR1]	= 0;
wf_num_cards[JGVTR1] = 0;

wf_addrs[COMET]		= 0xbc00;
wf_armaddrs[COMET]	= 0;
wf_num_cards[COMET] = 0;


tm_addrs[MZ8310]	= 0xf800;
tm_num_cards[MZ8310] = 4;

tm_addrs[VXI_AT5_TIME]	= 0xc000;
tm_num_cards[VXI_AT5_TIME] = 32;

AT830X_1_addrs		= 0x0400;
AT830X_1_num_cards	= 2;
AT830X_addrs		= 0xaa0000;
AT830X_num_cards	= 2;

xy010ScA16Base		= 0x0000;

EPICS_VXI_LA_COUNT	= 32;
EPICS_VXI_A24_BASE	= (char *) 0x900000;
EPICS_VXI_A24_SIZE	= 0x100000;
EPICS_VXI_A32_BASE	= (char *) 0x90000000;
EPICS_VXI_A32_SIZE	= 0x10000000;


AI566_VNUM	= 0xf8;
DVX_IVEC0	= 0xd0;
MD_INT_BASE	= 0xf0;
MZ8310_INT_VEC_BASE = 0xe8;
AB_VEC_BASE	= 0x60;
JGVTR1_INT_VEC	= 0xe0;
AT830X_1_IVEC0	= 0xd4;
AT830X_IVEC0	= 0xd6;
AT8FP_IVEC_BASE	= 0xa2;
AT8FPM_IVEC_BASE= 0xaa;

BB_SHORT_OFF	= 0x1800;
BB_IVEC_BASE	= 0xa0;
BB_IRQ_LEVEL	= 5;
PEP_BB_SHORT_OFF= 0x1c00;
PEP_BB_IVEC_BASE= 0xe8;

NIGPIB_SHORT_OFF	= 0x5000;
NIGPIB_IVEC_BASE	= 100;
NIGPIB_IRQ_LEVEL	= 5;

return(0);
}
