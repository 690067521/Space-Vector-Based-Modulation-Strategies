#include	"DSP28x_Project.h"
#define 	SST_ID                  0x00BF         /* SST Manufacturer's ID code   */
#define 	SST_39VF1601            0x234B         /*SST39VF1601 device code */


#define		FlashIdErr				1
#define		VerifyErr				2
#define		EraseErr				3
#define		TimeOutErr				4
#define		EraseOK					0			
#define		WriteOK					0
#define		NoErr					0

#define		SectorSize				0x800
#define		BlockSize				0x8000

unsigned  	int  *FlashStart = (unsigned  int *)0x100000;
unsigned  	int  *ExRamStart = (unsigned  int *)0x200000;

static 	Uint16 state = 0;
//=======================FLASH====================

static void CFIQueryExit(void)
{
	*(FlashStart + 0x5555) = 0xAA;
	*(FlashStart + 0x2AAA) = 0x55;
	*(FlashStart + 0x5555) = 0xf0;
	state &= 0xfc;	
}

static void SWPIDExit(void)
{
	*(FlashStart + 0x5555) = 0xf0;
	state &= 0xfc;
}

static void SWPIDEntry(void)
{
	if(state&1)
	{
		if(state&2)
			return;
		else
			CFIQueryExit();
	}

	*(FlashStart + 0x5555) = 0xAA;
	*(FlashStart + 0x2AAA) = 0x55;
	*(FlashStart + 0x5555) = 0x90;
	state |= 3;
}

Uint16  Read_ID(void)
{
	Uint16	SST_ID1,SST_ID2;
	SWPIDEntry();
	SST_ID1 = *(FlashStart + 0x0000) ;
	SST_ID2 = *(FlashStart + 0x0001) ;
	SWPIDExit();
	if((SST_ID1 == SST_ID) && (SST_ID2 == SST_39VF1601))
		return NoErr;
    else
	    return FlashIdErr;

}
Uint16	ChipErase(void)
{
	Uint16	temp;
	Uint32	i;

	if(state&1)
	{
		if(state&2)
			SWPIDExit();
		else
			CFIQueryExit();						
	}
	*(FlashStart + 0x5555) = 0xAA;
	*(FlashStart + 0x2AAA) = 0x55;
	*(FlashStart + 0x5555) = 0x80;
	*(FlashStart + 0x5555) = 0xAA;
	*(FlashStart + 0x2AAA) = 0x55;
	*(FlashStart + 0x5555) = 0x10;	

	while(1)
	{
		temp = *(FlashStart + 0x6666) & 0x40;
		if(temp != *(FlashStart + 0x6666) & 0x40)	
			continue;
		if( *(FlashStart + 0x8888) & 0x80)	
			break;						//D7 == 1
	}		

	for	(i=0;i<0x100000;i++)	
	{
		temp = *(FlashStart + i);
		if (temp != 0xFFFF)	
			return (EraseErr);
	}
	return  (EraseOK);											

}

void	WriteFlash(Uint32  addr,  Uint16	data)
{
	*(FlashStart + 0x5555) = 0xAA;
	*(FlashStart + 0x2AAA) = 0x55;
	*(FlashStart + 0x5555) = 0xA0;
	*(unsigned int *)addr = data;
	DELAY_US(10);
}


Uint16	ReadFlash(Uint32  addr)
{
	
	return (*(FlashStart + addr));
}

//=========================================NO MORE==============================

