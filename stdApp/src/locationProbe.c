#include	<vxWorks.h>
#include	<vme.h>
#include	<vxLib.h>
#include	"devLib.h"



#define EPICSAddrTypeNoConvert        -1
int EPICStovxWorksAddrType[] 
                = {
                VME_AM_SUP_SHORT_IO,
                VME_AM_STD_SUP_DATA,
                VME_AM_EXT_SUP_DATA,
                EPICSAddrTypeNoConvert
	        };
#define SUCCESS 0

long locationProbe(
epicsAddressType	addrType,
char			*pLocation
)
{
	char	*pPhysical;
	int	s;

	/*
	 * every byte in the block must 
	 * map to a physical address
	 */
	if (EPICStovxWorksAddrType[addrType] == EPICSAddrTypeNoConvert)
	{
		pPhysical = pLocation;
	}
	else
	{
		s = sysBusToLocalAdrs(
             	        EPICStovxWorksAddrType[addrType],
			pLocation,
                        &pPhysical);
		if(s<0){
/* If the following symbol doesn't exist in the version of EPICS we're
 * running against, try S_dev_vxWorksAddrMapFail.
 */
			return S_dev_addrMapFail;
		}
	}


	{
		int8_t	*pChar;
		int8_t	byte;

		pChar = (int8_t *) pPhysical;
		if(devPtrAlignTest(pChar)){
			s = vxMemProbe(
				(char *) pChar,
				READ,
				sizeof(byte),
				(char *) &byte);
			if(s!=ERROR){
				return S_dev_addressOverlap;
			}			
		}
	}
	{
		int16_t	*pWord;
		int16_t	word;

		pWord = (int16_t *)pPhysical;
		if(devPtrAlignTest(pWord)){
			s = vxMemProbe(
				(char *)pWord,
				READ,
				sizeof(word),
				(char *) &word);
			if(s!=ERROR){
				return S_dev_addressOverlap;
			}			
		}
	}
	{
		int32_t	*pLongWord;
		int32_t	longWord;

		pLongWord = (int32_t *) pPhysical;
		if(devPtrAlignTest(pLongWord)){
			s = vxMemProbe(
				(char *)pLongWord,
				READ,
				sizeof(longWord),
				(char *)&longWord);
			if(s!=ERROR){
				return S_dev_addressOverlap;
			}			
		}
	}

	return SUCCESS;
}

